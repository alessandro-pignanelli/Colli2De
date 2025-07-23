#pragma once

#include <functional>
#include <set>
#include <map>
#include <vector>

#include <colli2de/Ray.hpp>
#include <colli2de/internal/data_structures/DynamicBVH.hpp>
#include <colli2de/internal/geometry/AABB.hpp>
#include <colli2de/internal/utils/Methods.hpp>

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
    bool operator!=(GridCell other) const { return !(*this == other); }
    bool operator<(GridCell other) const { return (x < other.x) || (x == other.x && y < other.y); }
    bool operator>(GridCell other) const { return (x > other.x) || (x == other.x && y > other.y); }
    bool operator<=(GridCell other) const { return !(*this > other); }
    bool operator>=(GridCell other) const { return !(*this < other); }
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
    BroadPhaseTree(int32_t cellSize = 120) : cellSize(cellSize)
    {
        proxies.reserve(32);
    }

    // Add a new proxy, returns handle for later moves/removal
    BroadPhaseTreeHandle addProxy(IdType entityId, AABB aabb, BitMaskType categoryBits = 1, BitMaskType maskBits = ~0ull);
    void removeProxy(BroadPhaseTreeHandle handle);
    void moveProxy(BroadPhaseTreeHandle handle, AABB aabb);

    // AABB queries
    void query(AABB queryAABB, std::vector<IdType>& intersections, BitMaskType maskBits = ~0ull) const;
    void batchQuery(const std::vector<std::pair<AABB, BitMaskType>>& queries,
                    const std::function<void(size_t, std::vector<IdType>)>& callback) const;
    void findAllCollisions(const std::function<void(IdType, IdType)>& callback) const;
    void findAllCollisions(const BroadPhaseTree<IdType>& other,
                           const std::function<void(IdType, IdType)>& callback) const;

    // Raycast queries
    std::optional<RaycastInfo<IdType>> firstHitRaycast(Ray ray, BitMaskType maskBits = ~0ull) const;
    void piercingRaycast(Ray ray, std::set<RaycastInfo<IdType>>& hits, BitMaskType maskBits = ~0ull) const;
    std::optional<RaycastInfo<IdType>> firstHitRaycast(InfiniteRay ray, BitMaskType maskBits = ~0ull) const;
    void piercingRaycast(InfiniteRay ray, std::set<RaycastInfo<IdType>>& hits, BitMaskType maskBits = ~0ull) const;

    // For debugging/statistics
    std::size_t size() const { return proxies.size() - proxiesFreeList.size(); }
    void clear();

    // Optionally, support spatial partitioning (e.g. grid or region BVHs)
    // Each grid cell/region can have its own BVH
private:
    bool isValidHandle(BroadPhaseTreeHandle handle) const;
    size_t getBVHIndexForCell(GridCell cell);

    struct Proxy
    {
        std::map<GridCell, NodeIndex> bvhHandles;
        IdType entityId;
        AABB aabb;
        BitMaskType categoryBits;    // For collision filtering
        BitMaskType maskBits;        // For collision filtering
    };

    const int32_t cellSize;                                     // Size of each grid cell for spatial partitioning
    std::vector<Proxy> proxies;                                 // Indexed by BroadPhaseTreeHandle
    std::vector<BroadPhaseTreeHandle> proxiesFreeList;          // For fast recycling
    std::vector<std::pair<GridCell, DynamicBVH<IdType>>> bvhs;  // Each grid cell has its own BVH
    std::map<GridCell, size_t> regions;                         // Maps grid cells to their BVH index
    std::vector<size_t> bvhFreeList;                            // For fast recycling of BVH indices
};



template<typename IdType>
BroadPhaseTreeHandle BroadPhaseTree<IdType>::addProxy(IdType entityId, AABB aabb, BitMaskType categoryBits, BitMaskType maskBits)
{
    BroadPhaseTreeHandle treeHandle;
    if (!proxiesFreeList.empty())
    {
        treeHandle = proxiesFreeList.back();
        proxiesFreeList.pop_back();
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
        size_t bvhIndex = getBVHIndexForCell(cell);
        DynamicBVH<IdType>& bvh = bvhs[bvhIndex].second;

        // Proxy stores the BVH handle for each cell (region) it belongs to
        const auto bvhHandle = bvh.addProxy(entityId, aabb, categoryBits, maskBits);
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
        size_t bvhIndex = regions.at(cell);
        auto& bvh = bvhs[bvhIndex].second;
        bvh.removeProxy(bvhHandle);

        if (bvh.size() == 0)
        {
            regions.erase(cell);
            bvhFreeList.push_back(bvhIndex);
        }
    }
    
    proxy.bvhHandles.clear();
    proxiesFreeList.push_back(handle);
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
        {
            size_t bvhIndex = regions.at(cell);
            bvhs[bvhIndex].second.moveProxy(bvhHandle, aabb);
        }
        return;
    }

    const auto addToCell = [this, handle, aabb](GridCell cell)
    {
        // Region stores every proxy that belongs to it
        size_t bvhIndex = getBVHIndexForCell(cell);
        auto& bvh = bvhs[bvhIndex].second;

        // Proxy stores the BVH handle for each cell (region) it belongs to
        auto& proxy = proxies.at(handle);
        const auto bvhHandle = bvh.addProxy(proxy.entityId, aabb, proxy.categoryBits, proxy.maskBits);
        proxy.bvhHandles.emplace(cell, bvhHandle);
    };
    const auto removeFromCell = [this, handle](GridCell cell)
    {
        // Remove the proxy from the region
        size_t bvhIndex = regions.at(cell);
        auto& bvh = bvhs[bvhIndex].second;

        // Destroy the proxy from the BVH in that region and remove the bvh handle
        auto& proxy = proxies.at(handle);
        const auto bvhHandleInRegion = proxy.bvhHandles.find(cell);
        assert(bvhHandleInRegion != proxy.bvhHandles.end());
        bvh.removeProxy(bvhHandleInRegion->second);
        proxy.bvhHandles.erase(bvhHandleInRegion);

        if (bvh.size() == 0)
        {
            regions.erase(cell);
            bvhFreeList.push_back(bvhIndex);
        }
    };
    const auto moveSameCell = [this, handle, aabb](GridCell cell)
    {
        // If the proxy is still in the same cell, just update the BVH
        size_t bvhIndex = regions.at(cell);
        auto& bvh = bvhs[bvhIndex].second;
        const auto& proxy = proxies.at(handle);
        bvh.moveProxy(proxy.bvhHandles.at(cell), aabb);
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
void BroadPhaseTree<IdType>::query(AABB queryAABB, std::vector<IdType>& intersections, BitMaskType maskBits) const
{
    forEachCell(queryAABB, cellSize, [&](GridCell cell)
    {
        const auto regionIt = regions.find(cell);
        if (regionIt == regions.end())
            return; // No region here

        const auto& bvh = bvhs[regionIt->second].second;
        bvh.query(queryAABB, intersections, maskBits);
    });
}

template<typename IdType>
void BroadPhaseTree<IdType>::batchQuery(const std::vector<std::pair<AABB, BitMaskType>>& queries,
                                        const std::function<void(size_t, std::vector<IdType>)>& callback) const
{
    C2D_PARALLEL_FOR(0, queries.size(), [&](size_t i)
    {
        std::vector<IdType> hits;
        query(queries[i].first, hits, queries[i].second);
        if (!hits.empty())
            callback(i, std::move(hits));
    });
}

template<typename IdType>
void BroadPhaseTree<IdType>::findAllCollisions(const std::function<void(IdType, IdType)>& callback) const
{
    for (size_t i = 0; i < bvhs.size(); i++)
    {
        const auto& bvh = bvhs[i].second;
        bvh.findAllCollisions(callback);
    }
}

template<typename IdType>
void BroadPhaseTree<IdType>::findAllCollisions(const BroadPhaseTree<IdType>& other,
                                               const std::function<void(IdType, IdType)>& callback) const
{
    for (size_t i = 0; i < bvhs.size(); ++i)
    {
        const auto& cell = bvhs[i].first;
        const auto& bvh = bvhs[i].second;

        // No region in the other tree
        const auto otherIt = other.regions.find(cell);
        if (otherIt == other.regions.end())
            continue;

        const auto& otherBvh = other.bvhs[otherIt->second].second;
        bvh.findAllCollisions(otherBvh, callback);
    }
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
            const auto& bvh = bvhs[regionIt->second].second;
            const auto firstHit = bvh.firstHitRaycast(ray, maskBits);
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
    return firstHitRaycast(Ray{ray.start, ray.start + ray.direction.normalize() * cellSize * INFINITE_RAY_CELL_SPAN},
                           maskBits);
}

template<typename IdType>
void BroadPhaseTree<IdType>::piercingRaycast(Ray ray,
                                             std::set<RaycastInfo<IdType>>& hits,
                                             BitMaskType maskBits) const
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
            const auto& bvh = bvhs[regionIt->second].second;
            bvh.piercingRaycast(ray, hits, maskBits);
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
}

template<typename IdType>
void BroadPhaseTree<IdType>::piercingRaycast(InfiniteRay ray,
                                             std::set<RaycastInfo<IdType>>& hits,
                                             BitMaskType maskBits) const
{
    piercingRaycast(Ray{ray.start, ray.start + ray.direction.normalize() * cellSize * INFINITE_RAY_CELL_SPAN},
                    hits, maskBits);
}

template<typename IdType>
bool BroadPhaseTree<IdType>::isValidHandle(BroadPhaseTreeHandle handle) const
{
    return static_cast<size_t>(handle) < proxies.size();
}

template<typename IdType>
size_t BroadPhaseTree<IdType>::getBVHIndexForCell(GridCell cell)
{
    const auto it = regions.find(cell);
    if (it != regions.end())
        return it->second;

    // If the cell does not exist, create a new BVH for it
    if (!bvhFreeList.empty())
    {
        size_t bvhIndex = bvhFreeList.back();
        bvhFreeList.pop_back();
        regions.emplace(cell, bvhIndex);
        bvhs[bvhIndex].first = cell;
        return bvhIndex;
    }
    else
    {
        size_t newBvhIndex = bvhs.size();
        bvhs.emplace_back(cell, DynamicBVH<IdType>());
        regions.emplace(cell, newBvhIndex);
        return newBvhIndex;
    }
}

template<typename IdType>
void BroadPhaseTree<IdType>::clear()
{
    proxies.clear();
    proxies.reserve(32);
    proxiesFreeList.clear();
    regions.clear();
    bvhs.clear();
    bvhFreeList.clear();
}

} // namespace c2d
