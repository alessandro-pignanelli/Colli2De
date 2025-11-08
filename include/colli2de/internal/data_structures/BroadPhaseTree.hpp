#pragma once

#include <colli2de/Ray.hpp>
#include <colli2de/internal/data_structures/DynamicBVH.hpp>
#include <colli2de/internal/geometry/AABB.hpp>
#include <colli2de/internal/utils/Debug.hpp>
#include <colli2de/internal/utils/Methods.hpp>

#include <functional>
#include <map>
#include <set>
#include <vector>

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

    bool operator==(GridCell other) const
    {
        return x == other.x && y == other.y;
    }

    bool operator!=(GridCell other) const
    {
        return !(*this == other);
    }

    bool operator<(GridCell other) const
    {
        return (x < other.x) || (x == other.x && y < other.y);
    }

    bool operator>(GridCell other) const
    {
        return (x > other.x) || (x == other.x && y > other.y);
    }

    bool operator<=(GridCell other) const
    {
        return !(*this > other);
    }

    bool operator>=(GridCell other) const
    {
        return !(*this < other);
    }

#ifdef C2D_USE_CEREAL
    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(x, y);
    }
#endif
};

namespace
{

inline GridCell getCellFor(c2d::Vec2 position, int32_t cellSize)
{
    const int32_t posX = int32_t(position.x);
    const int32_t posY = int32_t(position.y);

    return {.x = int32_t(position.x) - (position.x >= 0 ? posX % cellSize : (posX % cellSize) + cellSize),
            .y = int32_t(position.y) - (position.y >= 0 ? posY % cellSize : (posY % cellSize) + cellSize)};
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

template <typename IdType>
class BroadPhaseTree
{
  public:
    BroadPhaseTree(int32_t cellSize = 120) : cellSize(cellSize)
    {
        proxies.reserve(32);
    }

    // Add a new proxy, returns handle for later moves/removal
    BroadPhaseTreeHandle addProxy(IdType entityId,
                                  AABB aabb,
                                  BitMaskType categoryBits = 1,
                                  BitMaskType maskBits = ~0ull);
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

    void serialize(std::ostream& out) const;
    static BroadPhaseTree deserialize(std::istream& in);

#ifdef C2D_USE_CEREAL
    template <class Archive>
    void serialize(Archive& archive);
#endif

    // For debugging/statistics
    std::size_t size() const
    {
        return proxies.size() - proxiesFreeList.size();
    }

    void clear();

    bool operator==(const BroadPhaseTree& other) const;

    bool operator!=(const BroadPhaseTree& other) const
    {
        return !(*this == other);
    }

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
        BitMaskType categoryBits; // For collision filtering
        BitMaskType maskBits;     // For collision filtering

        void serialize(std::ostream& out) const;
        static Proxy deserialize(std::istream& in);

#ifdef C2D_USE_CEREAL
        template <class Archive>
        void serialize(Archive& archive);
#endif

        bool operator==(const Proxy& other) const;

        bool operator!=(const Proxy& other) const
        {
            return !(*this == other);
        }
    };

    int32_t cellSize;                                          // Size of each grid cell for spatial partitioning
    std::vector<Proxy> proxies;                                // Indexed by BroadPhaseTreeHandle
    std::vector<BroadPhaseTreeHandle> proxiesFreeList;         // For fast recycling
    std::vector<std::pair<GridCell, DynamicBVH<IdType>>> bvhs; // Each grid cell has its own BVH
    std::map<GridCell, size_t> regions;                        // Maps grid cells to their BVH index
    std::vector<size_t> bvhFreeList;                           // For fast recycling of BVH indices
};

template <typename IdType>
BroadPhaseTreeHandle BroadPhaseTree<IdType>::addProxy(IdType entityId,
                                                      AABB aabb,
                                                      BitMaskType categoryBits,
                                                      BitMaskType maskBits)
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

    forEachCell(aabb,
                cellSize,
                [&](GridCell cell)
                {
                    size_t bvhIndex = getBVHIndexForCell(cell);
                    DynamicBVH<IdType>& bvh = bvhs[bvhIndex].second;

                    // Proxy stores the BVH handle for each cell (region) it belongs to
                    const auto bvhHandle = bvh.addProxy(entityId, aabb, categoryBits, maskBits);
                    proxy.bvhHandles.emplace(cell, bvhHandle);
                });

    return treeHandle;
}

template <typename IdType>
void BroadPhaseTree<IdType>::removeProxy(BroadPhaseTreeHandle handle)
{
    DEBUG_ASSERT(isValidHandle(handle));

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

template <typename IdType>
void BroadPhaseTree<IdType>::moveProxy(BroadPhaseTreeHandle handle, AABB aabb)
{
    DEBUG_ASSERT(isValidHandle(handle));
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
        DEBUG_ASSERT(bvhHandleInRegion != proxy.bvhHandles.end());
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
template <typename IdType>
void BroadPhaseTree<IdType>::query(AABB queryAABB, std::vector<IdType>& intersections, BitMaskType maskBits) const
{
    forEachCell(queryAABB,
                cellSize,
                [&](GridCell cell)
                {
                    const auto regionIt = regions.find(cell);
                    if (regionIt == regions.end())
                        return; // No region here

                    const auto& bvh = bvhs[regionIt->second].second;
                    bvh.query(queryAABB, intersections, maskBits);
                });
}

template <typename IdType>
void BroadPhaseTree<IdType>::batchQuery(const std::vector<std::pair<AABB, BitMaskType>>& queries,
                                        const std::function<void(size_t, std::vector<IdType>)>& callback) const
{
    C2D_PARALLEL_FOR(0,
                     queries.size(),
                     [&](size_t i)
                     {
                         std::vector<IdType> hits;
                         query(queries[i].first, hits, queries[i].second);
                         if (!hits.empty())
                             callback(i, std::move(hits));
                     });
}

template <typename IdType>
void BroadPhaseTree<IdType>::findAllCollisions(const std::function<void(IdType, IdType)>& callback) const
{
    for (size_t i = 0; i < bvhs.size(); i++)
    {
        const auto& bvh = bvhs[i].second;
        bvh.findAllCollisions(callback);
    }
}

template <typename IdType>
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
template <typename IdType>
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

template <typename IdType>
std::optional<RaycastInfo<IdType>> BroadPhaseTree<IdType>::firstHitRaycast(InfiniteRay ray, BitMaskType maskBits) const
{
    return firstHitRaycast(Ray{ray.start, ray.start + ray.direction.normalize() * cellSize * INFINITE_RAY_CELL_SPAN},
                           maskBits);
}

template <typename IdType>
void BroadPhaseTree<IdType>::piercingRaycast(Ray ray, std::set<RaycastInfo<IdType>>& hits, BitMaskType maskBits) const
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

template <typename IdType>
void BroadPhaseTree<IdType>::piercingRaycast(InfiniteRay ray,
                                             std::set<RaycastInfo<IdType>>& hits,
                                             BitMaskType maskBits) const
{
    piercingRaycast(
        Ray{ray.start, ray.start + ray.direction.normalize() * cellSize * INFINITE_RAY_CELL_SPAN}, hits, maskBits);
}

template <typename IdType>
void BroadPhaseTree<IdType>::serialize(std::ostream& out) const
{
    Writer writer(out);

    writer(cellSize);

    // Write proxies
    size_t proxiesSize = proxies.size();
    writer(proxiesSize);

    for (const Proxy& proxy : proxies)
        proxy.serialize(out);

    // Write regions
    size_t regionsSize = regions.size();
    writer(regionsSize);
    for (const auto& [cell, bvhIndex] : regions)
    {
        writer(cell.x);
        writer(cell.y);
        writer(bvhIndex);
    }

    // Write BVHs
    size_t bvhsSize = bvhs.size();
    writer(bvhsSize);
    for (const auto& [cell, bvh] : bvhs)
    {
        writer(cell.x);
        writer(cell.y);
        bvh.serialize(out);
    }

    // Write proxies free list
    size_t proxiesFreeListSize = proxiesFreeList.size();
    writer(proxiesFreeListSize);
    for (const BroadPhaseTreeHandle& handle : proxiesFreeList)
        writer(handle);

    // Write BVH free list
    size_t bvhFreeListSize = bvhFreeList.size();
    writer(bvhFreeListSize);
    for (const size_t index : bvhFreeList)
        writer(index);
}

template <typename IdType>
BroadPhaseTree<IdType> BroadPhaseTree<IdType>::deserialize(std::istream& in)
{
    Reader reader(in);

    int32_t cellSize;
    reader(cellSize);

    BroadPhaseTree<IdType> tree(cellSize);

    // Read proxies
    size_t proxiesSize;
    reader(proxiesSize);

    tree.proxies.reserve(proxiesSize);
    for (size_t i = 0; i < proxiesSize; ++i)
        tree.proxies.push_back(Proxy::deserialize(in));

    // Read regions
    size_t regionsSize;
    reader(regionsSize);

    for (size_t i = 0; i < regionsSize; ++i)
    {
        GridCell cell;
        reader(cell.x);
        reader(cell.y);

        size_t bvhIndex;
        reader(bvhIndex);

        tree.regions.emplace(cell, bvhIndex);
    }

    // Read BVHs
    size_t bvhsSize;
    reader(bvhsSize);

    tree.bvhs.reserve(bvhsSize);
    for (size_t i = 0; i < bvhsSize; ++i)
    {
        GridCell cell;
        reader(cell.x);
        reader(cell.y);

        DynamicBVH<IdType> bvh = DynamicBVH<IdType>::deserialize(in);
        tree.bvhs.emplace_back(cell, std::move(bvh));
    }

    // Read proxies free list
    size_t proxiesFreeListSize;
    reader(proxiesFreeListSize);

    tree.proxiesFreeList.reserve(proxiesFreeListSize);
    for (size_t i = 0; i < proxiesFreeListSize; ++i)
    {
        BroadPhaseTreeHandle handle;
        reader(handle);
        tree.proxiesFreeList.push_back(handle);
    }

    // Read BVH free list
    size_t bvhFreeListSize;
    reader(bvhFreeListSize);

    tree.bvhFreeList.reserve(bvhFreeListSize);
    for (size_t i = 0; i < bvhFreeListSize; ++i)
    {
        size_t index;
        reader(index);
        tree.bvhFreeList.push_back(index);
    }

    return tree;
}

#ifdef C2D_USE_CEREAL
template <typename IdType>
template <class Archive>
void BroadPhaseTree<IdType>::serialize(Archive& archive)
{
    archive(cellSize);
    archive(proxies);
    archive(proxiesFreeList);
    archive(regions);
    archive(bvhs);
    archive(bvhFreeList);
}
#endif

template <typename IdType>
void BroadPhaseTree<IdType>::Proxy::serialize(std::ostream& out) const
{
    Writer writer(out);

    writer(entityId);
    writer(aabb.min.x);
    writer(aabb.min.y);
    writer(aabb.max.x);
    writer(aabb.max.y);
    writer(categoryBits);
    writer(maskBits);

    // Write bvhHandles size
    size_t bvhHandlesSize = bvhHandles.size();
    writer(bvhHandlesSize);

    // Write bvhHandles
    for (const auto& [cell, nodeIndex] : bvhHandles)
    {
        writer(cell.x);
        writer(cell.y);
        writer(nodeIndex);
    }
}

template <typename IdType>
typename BroadPhaseTree<IdType>::Proxy BroadPhaseTree<IdType>::Proxy::deserialize(std::istream& in)
{
    Reader reader(in);

    Proxy proxy;
    reader(proxy.entityId);
    reader(proxy.aabb.min.x);
    reader(proxy.aabb.min.y);
    reader(proxy.aabb.max.x);
    reader(proxy.aabb.max.y);
    reader(proxy.categoryBits);
    reader(proxy.maskBits);

    // Read bvhHandles
    size_t bvhHandlesSize;
    reader(bvhHandlesSize);

    for (size_t i = 0; i < bvhHandlesSize; ++i)
    {
        GridCell cell;
        reader(cell.x);
        reader(cell.y);

        NodeIndex nodeIndex;
        reader(nodeIndex);

        proxy.bvhHandles.emplace(cell, nodeIndex);
    }

    return proxy;
}

#ifdef C2D_USE_CEREAL
template <typename IdType>
template <class Archive>
void BroadPhaseTree<IdType>::Proxy::serialize(Archive& archive)
{
    archive(entityId, aabb, categoryBits, maskBits, bvhHandles);
}
#endif

template <typename IdType>
bool BroadPhaseTree<IdType>::isValidHandle(BroadPhaseTreeHandle handle) const
{
    return static_cast<size_t>(handle) < proxies.size();
}

template <typename IdType>
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

template <typename IdType>
void BroadPhaseTree<IdType>::clear()
{
    proxies.clear();
    proxies.reserve(32);
    proxiesFreeList.clear();
    regions.clear();
    bvhs.clear();
    bvhFreeList.clear();
}

template <typename IdType>
bool BroadPhaseTree<IdType>::operator==(const BroadPhaseTree& other) const
{
    if (cellSize != other.cellSize)
        return false;
    if (proxies != other.proxies)
        return false;
    if (proxiesFreeList != other.proxiesFreeList)
        return false;
    if (regions != other.regions)
        return false;
    if (bvhs != other.bvhs)
        return false;
    if (bvhFreeList != other.bvhFreeList)
        return false;

    return true;
}

template <typename IdType>
bool BroadPhaseTree<IdType>::Proxy::operator==(const Proxy& other) const
{
    if (bvhHandles != other.bvhHandles)
        return false;
    if (entityId != other.entityId)
        return false;
    if (aabb != other.aabb)
        return false;
    if (categoryBits != other.categoryBits)
        return false;
    if (maskBits != other.maskBits)
        return false;

    return true;
}

} // namespace c2d
