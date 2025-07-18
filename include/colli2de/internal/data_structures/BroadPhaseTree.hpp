#pragma once

#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <colli2de/Ray.hpp>
#include <colli2de/internal/data_structures/DynamicBVH.hpp>
#include <colli2de/internal/geometry/AABB.hpp>

namespace c2d
{

template <typename IdType>
using RaycastInfo = typename DynamicBVH<IdType>::RaycastInfo;
using BroadPhaseTreeHandle = std::size_t;

constexpr int32_t INFINITE_RAY_CELL_SPAN = 50;

struct GridCell
{
    int32_t x;
    int32_t y;

    bool operator==(GridCell other) const { return x == other.x && y == other.y; }
};
struct HashGridCell
{
    size_t operator()(GridCell cell) const noexcept
    {
        return std::hash<int32_t>{}(cell.x) ^ (std::hash<int32_t>{}(cell.y) << 1);
    }
};

namespace
{

inline GridCell getCellFor(c2d::Vec2 position, int32_t cellSize)
{
    const int32_t posX = int32_t(position.x);
    const int32_t posY = int32_t(position.y);

    return { 
        .x = int32_t(position.x) - (position.x >= 0 ? posX % cellSize : (posX % cellSize) + cellSize),
        .y = int32_t(position.y) - (position.y >= 0 ? posY % cellSize : (posY % cellSize) + cellSize)
    };
}

inline void forEachCell(c2d::AABB aabb, int32_t cellSize, std::function<void(GridCell)> callback)
{
    const GridCell startCell = getCellFor(aabb.min, cellSize);
    const GridCell endCell = getCellFor(aabb.max, cellSize);

    for (int32_t y = startCell.y; y <= endCell.y; y += cellSize)
        for (int32_t x = startCell.x; x <= endCell.x; x += cellSize)
            callback(GridCell{x, y});
}

} // namespace

template<typename IdType>
class BroadPhaseTree
{
public:
    BroadPhaseTree(int32_t cellSize = 64) : cellSize(cellSize)
    {
        proxies.reserve(32);
    }

    // Add a new proxy, returns handle for later moves/removal
    BroadPhaseTreeHandle addProxy(IdType entityId, AABB aabb, BitMaskType categoryBits = 1, BitMaskType maskBits = ~0ull);
    void removeProxy(BroadPhaseTreeHandle handle);
    void moveProxy(BroadPhaseTreeHandle handle, AABB aabb);

    // AABB queries
    std::set<IdType> query(AABB queryAABB, BitMaskType maskBits = ~0ull) const;
    std::vector<std::set<IdType>> batchQuery(const std::vector<std::pair<AABB, BitMaskType>>& queries, size_t numThreads) const;
    std::set<IdType> sweepQuery(AABB startAABB, AABB endAABB, BitMaskType maskBits = ~0ull) const;
    std::vector<std::set<IdType>> batchSweepQuery(const std::vector<std::tuple<AABB, AABB, BitMaskType>>& queries, size_t numThreads) const;
    std::set<std::pair<IdType, IdType>> findAllCollisions() const;
    std::set<IdType> findAllCollisions(BroadPhaseTreeHandle handle) const;

    // Raycast queries
    std::optional<RaycastInfo<IdType>> firstHitRaycast(Ray ray, BitMaskType maskBits = ~0ull) const;
    std::set<RaycastInfo<IdType>> piercingRaycast(Ray ray, BitMaskType maskBits = ~0ull) const;
    std::optional<RaycastInfo<IdType>> firstHitRaycast(InfiniteRay ray, BitMaskType maskBits = ~0ull) const;
    std::set<RaycastInfo<IdType>> piercingRaycast(InfiniteRay ray, BitMaskType maskBits = ~0ull) const;

    // For debugging/statistics
    std::size_t size() const { return proxies.size() - freeList.size(); }
    void clear();

    // Optionally, support spatial partitioning (e.g. grid or region BVHs)
    // Each grid cell/region can have its own BVH
private:
    bool isValidHandle(BroadPhaseTreeHandle handle) const;

    struct Proxy
    {
        std::unordered_map<GridCell, NodeIndex, HashGridCell> bvhHandles;
        IdType entityId;
        AABB aabb;
        BitMaskType categoryBits;           // For collision filtering
        BitMaskType maskBits;               // For collision filtering
    };
    
    struct Region
    {
        DynamicBVH<IdType> bvh;
        std::unordered_set<BroadPhaseTreeHandle> proxies;
    };

    const int32_t cellSize;                       // Size of each grid cell for spatial partitioning
    std::vector<Proxy> proxies;                   // Indexed by BroadPhaseTreeHandle
    std::vector<BroadPhaseTreeHandle> freeList;   // For fast recycling
    std::unordered_map<GridCell, Region, HashGridCell> regions;
};



template<typename IdType>
BroadPhaseTreeHandle BroadPhaseTree<IdType>::addProxy(IdType entityId, AABB aabb, BitMaskType categoryBits, BitMaskType maskBits)
{
    BroadPhaseTreeHandle treeHandle;
    if (!freeList.empty())
    {
        treeHandle = freeList.back();
        freeList.pop_back();
    }
    else
    {
        treeHandle = static_cast<IdType>(proxies.size());
        proxies.emplace_back();
    }

    auto& proxy = proxies[treeHandle];
    proxy.entityId = entityId;
    proxy.aabb = aabb;
    proxy.categoryBits = categoryBits;
    proxy.maskBits = maskBits;

    forEachCell(aabb, cellSize, [&](GridCell cell)
    {
        auto& region = regions[cell];
        auto& bvh = region.bvh;

        // Region stores every proxy that belongs to it
        region.proxies.insert(treeHandle);

        // Proxy stores the BVH handle for each cell (region) it belongs to
        const auto bvhHandle = bvh.createProxy(aabb, entityId, categoryBits, maskBits);
        proxy.bvhHandles.emplace(cell, bvhHandle);
    });

    return treeHandle;
}

template<typename IdType>
void BroadPhaseTree<IdType>::removeProxy(BroadPhaseTreeHandle handle)
{
    assert(isValidHandle(handle));

    Proxy& proxy = proxies.at(handle);

    // Remove the proxy from all regions it belongs to
    for (const auto& [cell, bvhHandle] : proxy.bvhHandles)
    {
        auto& region = regions.at(cell);
        region.proxies.erase(handle);
        region.bvh.destroyProxy(bvhHandle);

        if (region.proxies.empty())
            regions.erase(cell);
    }
    
    proxy.bvhHandles.clear();
    freeList.push_back(handle);
}

template<typename IdType>
void BroadPhaseTree<IdType>::moveProxy(BroadPhaseTreeHandle handle, AABB aabb)
{
    assert(isValidHandle(handle));
    Proxy& proxy = proxies.at(handle);

    if (proxy.aabb.contains(aabb))
        return;

    const auto oldMinCell = getCellFor(proxy.aabb.min, cellSize);
    const auto oldMaxCell = getCellFor(proxy.aabb.max, cellSize);
    const auto newMinCell = getCellFor(aabb.min, cellSize);
    const auto newMaxCell = getCellFor(aabb.max, cellSize);

    proxy.aabb = aabb;
    const bool isSameCellRange = oldMinCell == newMinCell && oldMaxCell == newMaxCell;

    if (isSameCellRange)
    {
        for (const auto& [cell, bvhHandle] : proxy.bvhHandles)
            regions.at(cell).bvh.moveProxy(bvhHandle, aabb);
        return;
    }

    const auto addToCell = [this, handle, aabb](GridCell cell)
    {
        // Region stores every proxy that belongs to it
        auto& region = regions[cell];
        region.proxies.insert(handle);

        // Proxy stores the BVH handle for each cell (region) it belongs to
        auto& proxy = proxies.at(handle);
        const auto bvhHandle = region.bvh.createProxy(aabb, proxy.entityId, proxy.categoryBits, proxy.maskBits);
        proxy.bvhHandles.emplace(cell, bvhHandle);
    };
    const auto removeFromCell = [this, handle](GridCell cell)
    {
        // Remove the proxy from the region
        auto& region = regions.at(cell);
        region.proxies.erase(handle);

        // Destroy the proxy from the BVH in that region and remove the bvh handle
        auto& proxy = proxies.at(handle);
        const auto bvhHandleInRegion = proxy.bvhHandles.find(cell);
        assert(bvhHandleInRegion != proxy.bvhHandles.end());
        region.bvh.destroyProxy(bvhHandleInRegion->second);
        proxy.bvhHandles.erase(bvhHandleInRegion);

        if (region.proxies.empty())
            regions.erase(cell);
    };
    const auto moveSameCell = [this, handle, aabb](GridCell cell)
    {
        // If the proxy is still in the same cell, just update the BVH
        auto& region = regions.at(cell);
        const auto& proxy = proxies.at(handle);
        region.bvh.moveProxy(proxy.bvhHandles.at(cell), aabb);
    };

    const int32_t minX = std::min(oldMinCell.x, newMinCell.x);
    const int32_t minY = std::min(oldMinCell.y, newMinCell.y);
    const int32_t maxX = std::max(oldMaxCell.x, newMaxCell.x);
    const int32_t maxY = std::max(oldMaxCell.y, newMaxCell.y);

    for (int32_t x = minX; x <= maxX; x += cellSize)
        for (int32_t y = minY; y <= maxY; y += cellSize)
        {
            const GridCell cell{x, y};
            const bool isNotInOld = (x < oldMinCell.x || x > oldMaxCell.x || y < oldMinCell.y || y > oldMaxCell.y);
            const bool isNotInNew = (x < newMinCell.x || x > newMaxCell.x || y < newMinCell.y || y > newMaxCell.y);

            if (isNotInOld)
                addToCell(cell);
            else if (isNotInNew)
                removeFromCell(cell);
            else
                moveSameCell(cell);
        }
}

// AABB queries
template<typename IdType>
std::set<IdType> BroadPhaseTree<IdType>::query(AABB queryAABB, BitMaskType maskBits) const
{
    std::set<IdType> intersections;

    forEachCell(queryAABB, cellSize, [&](GridCell cell)
    {
        const auto regionIt = regions.find(cell);
        if (regionIt == regions.end())
            return; // No region here

        const auto& region = regionIt->second;
        region.bvh.query(queryAABB, intersections, maskBits);
    });

    return intersections;
}

template<typename IdType>
std::vector<std::set<IdType>> BroadPhaseTree<IdType>::batchQuery(const std::vector<std::pair<AABB, BitMaskType>>& queries, size_t numThreads) const
{
    if (queries.empty())
        return {};

    const size_t n = queries.size();
    std::vector<std::set<IdType>> results(n);

    constexpr size_t minQueriesPerThread = 5;
    numThreads = std::min(numThreads, (n - minQueriesPerThread) / minQueriesPerThread + 1);
    if (numThreads == 1)
    {
        for (size_t i = 0; i < n; ++i)
            results[i] = query(queries[i].first, queries[i].second);
        return results;
    }

    const size_t chunk = (n + numThreads - 1) / numThreads;
    std::vector<std::future<void>> futures;

    for (size_t t = 0; t < numThreads; ++t) {
        const size_t begin = t * chunk;
        const size_t end = std::min(n, begin + chunk);

        futures.push_back(std::async(std::launch::async, [this, &queries, &results, begin, end]() {
            for (size_t i = begin; i < end; ++i)
                results[i] = this->query(queries[i].first, queries[i].second);
        }));
    }

    for (auto& f : futures)
        f.get();

    return results;
}


template<typename IdType>
std::set<IdType> BroadPhaseTree<IdType>::sweepQuery(AABB startAABB, AABB endAABB, BitMaskType maskBits) const
{
    std::set<IdType> intersections;

    AABB combinedAABB = AABB::combine(startAABB, endAABB);

    forEachCell(combinedAABB, cellSize, [&](GridCell cell)
    {
        const auto regionIt = regions.find(cell);
        if (regionIt == regions.end())
            return; // No region here

        const auto& region = regionIt->second;
        region.bvh.sweepQuery(startAABB, endAABB, intersections, maskBits);
    });

    return intersections;
}

template<typename IdType>
std::vector<std::set<IdType>> BroadPhaseTree<IdType>::batchSweepQuery(
    const std::vector<std::tuple<AABB, AABB, BitMaskType>>& queries,
    size_t numThreads
) const
{
    if (queries.empty())
        return {};
    
    const size_t n = queries.size();
    std::vector<std::set<IdType>> results(n);

    constexpr size_t minQueriesPerThread = 2;
    numThreads = std::min(numThreads, (n - minQueriesPerThread) / minQueriesPerThread + 1);
    if (numThreads == 1)
    {
        for (size_t i = 0; i < n; ++i)
            results[i] = sweepQuery(std::get<0>(queries[i]), std::get<1>(queries[i]), std::get<2>(queries[i]));
        return results;
    }

    const size_t chunk = (n + numThreads - 1) / numThreads;
    std::vector<std::future<void>> futures;

    for (size_t t = 0; t < numThreads; ++t) {
        const size_t begin = t * chunk;
        const size_t end = std::min(n, begin + chunk);

        futures.push_back(std::async(std::launch::async, [this, &queries, &results, begin, end]() {
            for (size_t i = begin; i < end; ++i)
                results[i] = this->sweepQuery(std::get<0>(queries[i]), std::get<1>(queries[i]), std::get<2>(queries[i]));
        }));
    }

    for (auto& f : futures)
        f.get();

    return results;
}

template<typename IdType>
std::set<std::pair<IdType, IdType>> BroadPhaseTree<IdType>::findAllCollisions() const
{
    std::set<std::pair<IdType, IdType>> collisions;

    for (const auto& [cell, region] : regions)
        region.bvh.findAllCollisions(collisions);

    return collisions;
}

template<typename IdType>
std::set<IdType> BroadPhaseTree<IdType>::findAllCollisions(BroadPhaseTreeHandle handle) const
{
    assert(isValidHandle(handle));
    const Proxy& proxy = proxies.at(handle);
    return query(proxy.aabb, proxy.maskBits);
}

// Raycast queries
template<typename IdType>
std::optional<RaycastInfo<IdType>> BroadPhaseTree<IdType>::firstHitRaycast(Ray ray, BitMaskType maskBits) const
{
    const auto direction = ray.end - ray.start;
    const int32_t directionX = (direction.x > 0 ? cellSize : -cellSize);
    const int32_t directionY = (direction.y > 0 ? cellSize : -cellSize);

    const bool isParallelX = std::abs(direction.y) < 1e-6f;
    const bool isParallelY = std::abs(direction.x) < 1e-6f;

    const GridCell lastCell = getCellFor(ray.end, cellSize);
    auto currentCell = getCellFor(ray.start, cellSize);
    auto currentPoint = ray.start;

    while (currentCell.x * directionX <= lastCell.x * directionX &&
           currentCell.y * directionY <= lastCell.y * directionY)
    {
        if (const auto regionIt = regions.find(currentCell); regionIt != regions.end())
        {
            const auto& region = regionIt->second;
            const auto firstHit = region.bvh.firstHitRaycast(ray, maskBits);
            if (firstHit)
                return firstHit;
        }

        if (isParallelX)
        {
            currentCell.x += directionX;
            continue;
        }
        if (isParallelY)
        {
            currentCell.y += directionY;
            continue;
        }

        const auto nextCellX = currentCell.x + directionX;
        const auto nextCellY = currentCell.y + directionY;

        // Calculate the next intersection point with the grid lines
        const float timeX = (nextCellX - currentPoint.x) / direction.x;
        const float timeY = (nextCellY - currentPoint.y) / direction.y;

        if (timeX < timeY)
        {
            // Move to the next vertical grid line
            currentPoint.x = nextCellX;
            currentPoint.y += direction.y * timeX;
            currentCell.x += directionX;
        }
        else
        {
            // Move to the next horizontal grid line
            currentPoint.y = nextCellY;
            currentPoint.x += direction.x * timeY;
            currentCell.y += directionY;
        }
    }

    return std::nullopt;
}

template<typename IdType>
std::optional<RaycastInfo<IdType>> BroadPhaseTree<IdType>::firstHitRaycast(InfiniteRay ray, BitMaskType maskBits) const
{
    return firstHitRaycast(Ray{ray.start, ray.start + ray.direction.normalize() * cellSize * 50}, maskBits);
}

template<typename IdType>
std::set<RaycastInfo<IdType>> BroadPhaseTree<IdType>::piercingRaycast(Ray ray, BitMaskType maskBits) const
{
    const auto direction = ray.end - ray.start;
    const int32_t directionX = (direction.x > 0 ? cellSize : -cellSize);
    const int32_t directionY = (direction.y > 0 ? cellSize : -cellSize);

    const bool isParallelX = std::abs(direction.y) < 1e-6f;
    const bool isParallelY = std::abs(direction.x) < 1e-6f;

    const GridCell lastCell = getCellFor(ray.end, cellSize);
    auto currentCell = getCellFor(ray.start, cellSize);
    auto currentPoint = ray.start;

    std::set<RaycastInfo<IdType>> hits;

    while (currentCell.x * directionX <= lastCell.x * directionX &&
           currentCell.y * directionY <= lastCell.y * directionY)
    {
        if (const auto regionIt = regions.find(currentCell); regionIt != regions.end())
        {
            const auto& region = regionIt->second;
            region.bvh.piercingRaycast(ray, hits, maskBits);
        }

        if (isParallelX)
        {
            currentCell.x += directionX;
            continue;
        }
        if (isParallelY)
        {
            currentCell.y += directionY;
            continue;
        }

        const auto nextCellX = currentCell.x + directionX;
        const auto nextCellY = currentCell.y + directionY;

        // Calculate the next intersection point with the grid lines
        const float timeX = (nextCellX - currentPoint.x) / direction.x;
        const float timeY = (nextCellY - currentPoint.y) / direction.y;

        if (timeX < timeY)
        {
            // Move to the next vertical grid line
            currentPoint.x = nextCellX;
            currentPoint.y += direction.y * timeX;
            currentCell.x += directionX;
        }
        else
        {
            // Move to the next horizontal grid line
            currentPoint.y = nextCellY;
            currentPoint.x += direction.x * timeY;
            currentCell.y += directionY;
        }
    }

    return hits;
}

template<typename IdType>
std::set<RaycastInfo<IdType>> BroadPhaseTree<IdType>::piercingRaycast(InfiniteRay ray, BitMaskType maskBits) const
{
    return piercingRaycast(Ray{ray.start,
                               ray.start + ray.direction.normalize() * cellSize * INFINITE_RAY_CELL_SPAN},
                           maskBits);
}

template<typename IdType>
bool BroadPhaseTree<IdType>::isValidHandle(BroadPhaseTreeHandle handle) const
{
    return static_cast<size_t>(handle) < proxies.size();
}

template<typename IdType>
void BroadPhaseTree<IdType>::clear()
{
    proxies.clear();
    freeList.clear();
    regions.clear();
    regions.reserve(32);
}

} // namespace c2d
