#pragma once

#include <colli2de/Ray.hpp>
#include <set>
#include <unordered_map>
#include <vector>

#include "data_structures/DynamicBVH.hpp"
#include "geometry/AABB.hpp"

namespace 
{
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
    
    inline GridCell getCellFor(c2d::Vec2 position, int32_t cellSize)
    {
        return { int32_t(position.x) / cellSize, int32_t(position.y) / cellSize };
    }

    inline void forEachCell(c2d::AABB aabb, int32_t cellSize, std::function<void(GridCell)> callback)
    {
        const int32_t minX = int32_t(aabb.min.x) / cellSize;
        const int32_t minY = int32_t(aabb.min.y) / cellSize;
        const int32_t maxX = int32_t(aabb.max.x) / cellSize;
        const int32_t maxY = int32_t(aabb.max.y) / cellSize;

        for (int32_t x = minX; x <= maxX; ++x)
            for (int32_t y = minY; y <= maxY; ++y)
                callback(GridCell{x, y});
    }

    inline void forEachCellDiff(c2d::AABB oldAabb,
                                  c2d::AABB newAabb,
                                  int32_t cellSize,
                                  const std::function<void(GridCell)>& onCellAdded,
                                  const std::function<void(GridCell)>& onCellRemoved,
                                  const std::function<void(GridCell)>& onCellCommon)
    {
        const int32_t oldMinX = int32_t(oldAabb.min.x) / cellSize;
        const int32_t oldMinY = int32_t(oldAabb.min.y) / cellSize;
        const int32_t oldMaxX = int32_t(oldAabb.max.x) / cellSize;
        const int32_t oldMaxY = int32_t(oldAabb.max.y) / cellSize;

        const int32_t newMinX = int32_t(newAabb.min.x) / cellSize;
        const int32_t newMinY = int32_t(newAabb.min.y) / cellSize;
        const int32_t newMaxX = int32_t(newAabb.max.x) / cellSize;
        const int32_t newMaxY = int32_t(newAabb.max.y) / cellSize;

        const int32_t minX = std::min(oldMinX, newMinX);
        const int32_t minY = std::min(oldMinY, newMinY);
        const int32_t maxX = std::max(oldMaxX, newMaxX);
        const int32_t maxY = std::max(oldMaxY, newMaxY);
        
        for (int32_t x = minX; x <= maxX; ++x)
            for (int32_t y = minY; y <= maxY; ++y)
            {
                const GridCell cell{x, y};
                const bool isNotInOld = (x < oldMinX || x > oldMaxX || y < oldMinY || y > oldMaxY);
                const bool isNotInNew = (x < newMinX || x > newMaxX || y < newMinY || y > newMaxY);

                if (isNotInOld)
                    onCellAdded(cell);
                else if (isNotInNew)
                    onCellRemoved(cell);
                else
                    onCellCommon(cell);
            }
    }
}

namespace c2d
{

template <typename IdType>
using RaycastInfo = typename DynamicBVH<IdType>::RaycastInfo;
using BroadPhaseTreeHandle = std::size_t;

template<typename IdType>
class BroadPhaseTree
{
public:
    struct Proxy
    {
        std::unordered_map<GridCell, NodeIndex, HashGridCell> bvhHandles;
        IdType entityId;
        AABB aabb;
        BitMaskType categoryBits;           // For collision filtering
        BitMaskType maskBits;               // For collision filtering
    };

    BroadPhaseTree(int32_t cellSize = 64) : cellSize(cellSize)
    {
        proxies.reserve(32);
    }

    // Add a new proxy, returns handle for later moves/removal
    BroadPhaseTreeHandle addProxy(IdType entityId, AABB aabb, BitMaskType categoryBits = 1, BitMaskType maskBits = ~0ull);
    void removeProxy(BroadPhaseTreeHandle handle);
    void removeProxyFrom(BroadPhaseTreeHandle handle, GridCell cell);
    void moveProxy(BroadPhaseTreeHandle handle, AABB aabb);

    void setSleeping(BroadPhaseTreeHandle handle, bool sleeping);
    void updateSleeping();

    // AABB queries
    std::set<IdType> query(AABB queryAABB, BitMaskType maskBits = ~0ull) const;
    std::vector<std::set<IdType>> batchQuery(const std::vector<AABB>& queries, size_t numThreads, BitMaskType maskBits = ~0ull) const;
    std::set<std::pair<IdType, IdType>> findAllCollisions() const;

    // Raycast queries
    std::optional<RaycastInfo<IdType>> firstHitRaycast(Ray ray, BitMaskType maskBits = ~0ull) const;
    std::set<RaycastInfo<IdType>> piercingRaycast(Ray ray, BitMaskType maskBits = ~0ull) const;
    std::optional<RaycastInfo<IdType>> firstHitRaycast(InfiniteRay ray, BitMaskType maskBits = ~0ull) const;
    std::set<RaycastInfo<IdType>> piercingRaycast(InfiniteRay ray, BitMaskType maskBits = ~0ull) const;

    // For debugging/statistics
    std::size_t size() const { return proxies.size() - freeList.size(); }

    // Optionally, support spatial partitioning (e.g. grid or region BVHs)
    // Each grid cell/region can have its own BVH
private:
    bool isValidHandle(BroadPhaseTreeHandle handle) const;

    struct Region
    {
        DynamicBVH<IdType> bvh;
        std::set<BroadPhaseTreeHandle> proxies;
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
        proxy.bvhHandles[cell] = bvhHandle;
    });

    return treeHandle;
}

template<typename IdType>
void BroadPhaseTree<IdType>::removeProxy(BroadPhaseTreeHandle handle)
{
    assert(isValidHandle(handle));

    Proxy& proxy = proxies[handle];

    // Remove the proxy from all regions it belongs to
    for (const auto& [cell, bvhHandle] : proxy.bvhHandles)
    {
        auto& region = regions[cell];
        region.proxies.erase(handle);
        region.bvh.destroyProxy(bvhHandle);
    }
    
    proxy.bvhHandles.clear();
    freeList.push_back(handle);
}

template<typename IdType>
void BroadPhaseTree<IdType>::moveProxy(BroadPhaseTreeHandle handle, AABB aabb)
{
    assert(isValidHandle(handle));

    const auto addToCell = [this, handle, aabb](GridCell cell)
    {
        // Region stores every proxy that belongs to it
        auto& region = regions[cell];
        region.proxies.insert(handle);

        // Proxy stores the BVH handle for each cell (region) it belongs to
        auto& proxy = proxies[handle];
        const auto bvhHandle = region.bvh.createProxy(aabb, proxy.entityId, proxy.categoryBits, proxy.maskBits);
        proxy.bvhHandles.emplace(cell, bvhHandle);
    };
    const auto removeFromCell = [this, handle](GridCell cell)
    {
        // Remove the proxy from the region
        auto& region = regions[cell];
        region.proxies.erase(handle);

        // Destroy the proxy from the BVH in that region and remove the bvh handle
        auto& proxy = proxies[handle];
        const auto bvhHandleInRegion = proxy.bvhHandles.find(cell);
        assert(bvhHandleInRegion != proxy.bvhHandles.end());
        region.bvh.destroyProxy(bvhHandleInRegion->second);
        proxy.bvhHandles.erase(bvhHandleInRegion);
    };
    const auto moveSameCell = [this, handle, aabb](GridCell cell)
    {
        // If the proxy is still in the same cell, just update the BVH
        auto& region = regions[cell];
        auto& proxy = proxies[handle];
        region.bvh.moveProxy(proxy.bvhHandles[cell], aabb);
    };

    Proxy& proxy = proxies[handle];
    forEachCellDiff(proxy.aabb, aabb, cellSize, addToCell, removeFromCell, moveSameCell);
    proxy.aabb = aabb;
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
std::vector<std::set<IdType>> BroadPhaseTree<IdType>::batchQuery(const std::vector<AABB>& queries, size_t numThreads, BitMaskType maskBits) const
{
    const size_t n = queries.size();
    std::vector<std::set<IdType>> results(n);
    std::vector<std::future<void>> futures;

    numThreads = std::min(numThreads, n / 5 + 1); // Ensure at least one thread per 5 queries
    const size_t chunk = (n + numThreads - 1) / numThreads;

    for (size_t t = 0; t < numThreads; ++t) {
        const size_t begin = t * chunk;
        const size_t end = std::min(n, begin + chunk);

        futures.push_back(std::async(std::launch::async, [this, &queries, &results, begin, end, maskBits]() {
            for (size_t i = begin; i < end; ++i)
                results[i] = this->query(queries[i], maskBits);
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

// Raycast queries
template<typename IdType>
std::optional<RaycastInfo<IdType>> BroadPhaseTree<IdType>::firstHitRaycast(Ray ray, BitMaskType maskBits) const
{
    const auto direction = ray.p2 - ray.p1;
    const int32_t directionX = (direction.x > 0 ? cellSize : -cellSize);
    const int32_t directionY = (direction.y > 0 ? cellSize : -cellSize);

    const bool isParallelX = std::abs(direction.y) < 1e-6f;
    const bool isParallelY = std::abs(direction.x) < 1e-6f;

    const GridCell lastCell = getCellFor(ray.p2, cellSize);
    auto currentCell = getCellFor(ray.p1, cellSize);
    auto currentPoint = ray.p1;

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
    const auto direction = ray.p2 - ray.p1;
    const int32_t directionX = (direction.x > 0 ? cellSize : -cellSize);
    const int32_t directionY = (direction.y > 0 ? cellSize : -cellSize);

    const bool isParallelX = std::abs(direction.y) < 1e-6f;
    const bool isParallelY = std::abs(direction.x) < 1e-6f;

    const GridCell lastCell = getCellFor(ray.p2, cellSize);
    auto currentCell = getCellFor(ray.p1, cellSize);
    auto currentPoint = ray.p1;

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
    return handle >= 0 && static_cast<size_t>(handle) < proxies.size();
}

} // namespace c2d
