#pragma once

#include <colli2de/Ray.hpp>
#include <colli2de/internal/data_structures/DynamicBVH.hpp>
#include <colli2de/internal/geometry/AABB.hpp>
#include <colli2de/internal/utils/Debug.hpp>
#include <colli2de/internal/utils/Methods.hpp>
#include <colli2de/internal/utils/Serialization.hpp>

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
    template <IsCerealArchive Archive>
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

inline void forEachCell(c2d::AABB boundingBox, int32_t cellSize, std::function<void(GridCell)> callback)
{
    const GridCell startCell = getCellFor(boundingBox.min, cellSize);
    const GridCell endCell = getCellFor(boundingBox.max, cellSize);

    for (int32_t y = startCell.y; y <= endCell.y; y += cellSize)
        for (int32_t x = startCell.x; x <= endCell.x; x += cellSize)
            callback(GridCell{x, y});
}

} // namespace

template <typename IdType>
class BroadPhaseTree
{
  public:
    BroadPhaseTree(int32_t cellSize = 120) : mCellSize(cellSize)
    {
        mProxies.reserve(32);
    }

    // Add a new proxy, returns handle for later moves/removal
    BroadPhaseTreeHandle addProxy(IdType entityId,
                                  AABB boundingBox,
                                  BitMaskType categoryBits = 1,
                                  BitMaskType maskBits = ~0ull);
    void removeProxy(BroadPhaseTreeHandle handle);
    void moveProxy(BroadPhaseTreeHandle handle, AABB boundingBox);

    // AABB queries
    template <typename Allocator>
    void query(AABB queryBoundingBox,
               std::vector<IdType, Allocator>& intersections,
               BitMaskType maskBits = ~0ull) const;
    void batchQuery(const std::vector<std::pair<AABB, BitMaskType>>& queries,
                    const std::function<void(size_t, std::vector<IdType>&)>& callback) const;
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
    template <IsCerealArchive Archive>
    void serialize(Archive& archive);
#endif

    // For debugging/statistics
    std::size_t size() const
    {
        return mProxies.size() - mProxiesFreeList.size();
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
        std::map<GridCell, NodeIndex> mBvhHandles;
        IdType mEntityId;
        AABB mBoundingBox;
        BitMaskType mCategoryBits; // For collision filtering
        BitMaskType mMaskBits;     // For collision filtering

        void serialize(std::ostream& out) const;
        static Proxy deserialize(std::istream& in);

#ifdef C2D_USE_CEREAL
        template <IsCerealArchive Archive>
        void serialize(Archive& archive);
#endif

        bool operator==(const Proxy& other) const;

        bool operator!=(const Proxy& other) const
        {
            return !(*this == other);
        }
    };

    int32_t mCellSize;                                          // Size of each grid cell for spatial partitioning
    std::vector<Proxy> mProxies;                                // Indexed by BroadPhaseTreeHandle
    std::vector<BroadPhaseTreeHandle> mProxiesFreeList;         // For fast recycling
    std::vector<std::pair<GridCell, DynamicBVH<IdType>>> mBvhs; // Each grid cell has its own BVH
    std::map<GridCell, size_t> mRegions;                        // Maps grid cells to their BVH index
    std::vector<size_t> mBvhFreeList;                           // For fast recycling of BVH indices
};

template <typename IdType>
BroadPhaseTreeHandle BroadPhaseTree<IdType>::addProxy(IdType entityId,
                                                      AABB boundingBox,
                                                      BitMaskType categoryBits,
                                                      BitMaskType maskBits)
{
    BroadPhaseTreeHandle treeHandle;
    if (!mProxiesFreeList.empty())
    {
        treeHandle = mProxiesFreeList.back();
        mProxiesFreeList.pop_back();
    }
    else
    {
        treeHandle = static_cast<IdType>(mProxies.size());
        mProxies.emplace_back();
    }

    auto& proxy = mProxies[treeHandle];
    proxy.mEntityId = entityId;
    proxy.mBoundingBox = boundingBox;
    proxy.mCategoryBits = categoryBits;
    proxy.mMaskBits = maskBits;

    forEachCell(boundingBox,
                mCellSize,
                [&](GridCell cell)
                {
                    const size_t bvhIndex = getBVHIndexForCell(cell);
                    DynamicBVH<IdType>& bvh = mBvhs[bvhIndex].second;

                    // Proxy stores the BVH handle for each cell (region) it belongs to
                    const auto bvhHandle = bvh.addProxy(entityId, boundingBox, categoryBits, maskBits);
                    proxy.mBvhHandles.emplace(cell, bvhHandle);
                });

    return treeHandle;
}

template <typename IdType>
void BroadPhaseTree<IdType>::removeProxy(BroadPhaseTreeHandle handle)
{
    DEBUG_ASSERT(isValidHandle(handle));

    Proxy& proxy = mProxies.at(handle);

    // Remove the proxy from all regions it belongs to
    for (const auto& [cell, bvhHandle] : proxy.mBvhHandles)
    {
        const size_t bvhIndex = mRegions.at(cell);
        auto& bvh = mBvhs[bvhIndex].second;
        bvh.removeProxy(bvhHandle);

        if (bvh.size() == 0)
        {
            mRegions.erase(cell);
            mBvhFreeList.push_back(bvhIndex);
        }
    }

    proxy.mBvhHandles.clear();
    mProxiesFreeList.push_back(handle);
}

template <typename IdType>
void BroadPhaseTree<IdType>::moveProxy(BroadPhaseTreeHandle handle, AABB boundingBox)
{
    DEBUG_ASSERT(isValidHandle(handle));
    Proxy& proxy = mProxies.at(handle);

    if (proxy.mBoundingBox.contains(boundingBox))
        return;

    const auto oldMinCell = getCellFor(proxy.mBoundingBox.min, mCellSize);
    const auto oldMaxCell = getCellFor(proxy.mBoundingBox.max, mCellSize);
    const auto newMinCell = getCellFor(boundingBox.min, mCellSize);
    const auto newMaxCell = getCellFor(boundingBox.max, mCellSize);

    proxy.mBoundingBox = boundingBox;

    const bool isSameCellRange = oldMinCell == newMinCell && oldMaxCell == newMaxCell;
    if (isSameCellRange)
    {
        for (const auto& [cell, bvhHandle] : proxy.mBvhHandles)
        {
            size_t bvhIndex = mRegions.at(cell);
            mBvhs[bvhIndex].second.moveProxy(bvhHandle, boundingBox);
        }
        return;
    }

    const auto addToCell = [this, handle, boundingBox](GridCell cell)
    {
        // Region stores every proxy that belongs to it
        size_t bvhIndex = getBVHIndexForCell(cell);
        auto& bvh = mBvhs[bvhIndex].second;

        // Proxy stores the BVH handle for each cell (region) it belongs to
        auto& proxy = mProxies.at(handle);
        const auto bvhHandle = bvh.addProxy(proxy.mEntityId, boundingBox, proxy.mCategoryBits, proxy.mMaskBits);
        proxy.mBvhHandles.emplace(cell, bvhHandle);
    };
    const auto removeFromCell = [this, handle](GridCell cell)
    {
        // Remove the proxy from the region
        const size_t bvhIndex = mRegions.at(cell);
        auto& bvh = mBvhs[bvhIndex].second;

        // Destroy the proxy from the BVH in that region and remove the bvh handle
        auto& proxy = mProxies.at(handle);
        const auto bvhHandleInRegion = proxy.mBvhHandles.find(cell);
        DEBUG_ASSERT(bvhHandleInRegion != proxy.mBvhHandles.end());
        bvh.removeProxy(bvhHandleInRegion->second);
        proxy.mBvhHandles.erase(bvhHandleInRegion);

        if (bvh.size() == 0)
        {
            mRegions.erase(cell);
            mBvhFreeList.push_back(bvhIndex);
        }
    };
    const auto moveSameCell = [this, handle, boundingBox](GridCell cell)
    {
        // If the proxy is still in the same cell, just update the BVH
        const size_t bvhIndex = mRegions.at(cell);
        auto& bvh = mBvhs[bvhIndex].second;
        const auto& proxy = mProxies.at(handle);
        bvh.moveProxy(proxy.mBvhHandles.at(cell), boundingBox);
    };

    const int32_t minX = std::min(oldMinCell.x, newMinCell.x);
    const int32_t minY = std::min(oldMinCell.y, newMinCell.y);
    const int32_t maxX = std::max(oldMaxCell.x, newMaxCell.x);
    const int32_t maxY = std::max(oldMaxCell.y, newMaxCell.y);

    for (int32_t x = minX; x <= maxX; x += mCellSize)
        for (int32_t y = minY; y <= maxY; y += mCellSize)
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
template <typename Allocator>
void BroadPhaseTree<IdType>::query(AABB queryBoundingBox,
                                   std::vector<IdType, Allocator>& intersections,
                                   BitMaskType maskBits) const
{
    forEachCell(queryBoundingBox,
                mCellSize,
                [&](GridCell cell)
                {
                    const auto regionIt = mRegions.find(cell);
                    if (regionIt == mRegions.end())
                        return; // No region here

                    const auto& bvh = mBvhs[regionIt->second].second;
                    bvh.query(queryBoundingBox, intersections, maskBits);
                });
}

template <typename IdType>
void BroadPhaseTree<IdType>::batchQuery(const std::vector<std::pair<AABB, BitMaskType>>& queries,
                                        const std::function<void(size_t, std::vector<IdType>&)>& callback) const
{
    C2D_PARALLEL_FOR(0,
                     queries.size(),
                     [&](size_t i)
                     {
                         std::vector<IdType> hits;
                         query(queries[i].first, hits, queries[i].second);
                         if (!hits.empty())
                             callback(i, hits);
                     });
}

template <typename IdType>
void BroadPhaseTree<IdType>::findAllCollisions(const std::function<void(IdType, IdType)>& callback) const
{
    for (size_t i = 0; i < mBvhs.size(); i++)
    {
        const auto& bvh = mBvhs[i].second;
        bvh.findAllCollisions(callback);
    }
}

template <typename IdType>
void BroadPhaseTree<IdType>::findAllCollisions(const BroadPhaseTree<IdType>& other,
                                               const std::function<void(IdType, IdType)>& callback) const
{
    for (size_t i = 0; i < mBvhs.size(); ++i)
    {
        const auto& cell = mBvhs[i].first;
        const auto& bvh = mBvhs[i].second;

        // No region in the other tree
        const auto otherIt = other.mRegions.find(cell);
        if (otherIt == other.mRegions.end())
            continue;

        const auto& otherBvh = other.mBvhs[otherIt->second].second;
        bvh.findAllCollisions(otherBvh, callback);
    }
}

// Raycast queries
template <typename IdType>
std::optional<RaycastInfo<IdType>> BroadPhaseTree<IdType>::firstHitRaycast(Ray ray, BitMaskType maskBits) const
{
    const auto direction = ray.end - ray.start;
    const int32_t directionX = (direction.x > 0 ? mCellSize : -mCellSize);
    const int32_t directionY = (direction.y > 0 ? mCellSize : -mCellSize);

    const bool isParallelX = std::abs(direction.y) < 1e-6f;
    const bool isParallelY = std::abs(direction.x) < 1e-6f;

    const GridCell lastCell = getCellFor(ray.end, mCellSize);
    auto currentCell = getCellFor(ray.start, mCellSize);
    auto currentPoint = ray.start;

    while (currentCell.x * directionX <= lastCell.x * directionX &&
           currentCell.y * directionY <= lastCell.y * directionY)
    {
        if (const auto regionIt = mRegions.find(currentCell); regionIt != mRegions.end())
        {
            const auto& bvh = mBvhs[regionIt->second].second;
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
    return firstHitRaycast(Ray{ray.start, ray.start + ray.direction.normalize() * mCellSize * INFINITE_RAY_CELL_SPAN},
                           maskBits);
}

template <typename IdType>
void BroadPhaseTree<IdType>::piercingRaycast(Ray ray, std::set<RaycastInfo<IdType>>& hits, BitMaskType maskBits) const
{
    const auto direction = ray.end - ray.start;
    const int32_t directionX = (direction.x > 0 ? mCellSize : -mCellSize);
    const int32_t directionY = (direction.y > 0 ? mCellSize : -mCellSize);

    const bool isParallelX = std::abs(direction.y) < 1e-6f;
    const bool isParallelY = std::abs(direction.x) < 1e-6f;

    const GridCell lastCell = getCellFor(ray.end, mCellSize);
    auto currentCell = getCellFor(ray.start, mCellSize);
    auto currentPoint = ray.start;

    while (currentCell.x * directionX <= lastCell.x * directionX &&
           currentCell.y * directionY <= lastCell.y * directionY)
    {
        if (const auto regionIt = mRegions.find(currentCell); regionIt != mRegions.end())
        {
            const auto& bvh = mBvhs[regionIt->second].second;
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
        Ray{ray.start, ray.start + ray.direction.normalize() * mCellSize * INFINITE_RAY_CELL_SPAN}, hits, maskBits);
}

template <typename IdType>
void BroadPhaseTree<IdType>::serialize(std::ostream& out) const
{
    Writer writer(out);

    writer(mCellSize);

    // Write proxies
    size_t proxiesSize = mProxies.size();
    writer(proxiesSize);

    for (const Proxy& proxy : mProxies)
        proxy.serialize(out);

    // Write regions
    size_t regionsSize = mRegions.size();
    writer(regionsSize);
    for (const auto& [cell, bvhIndex] : mRegions)
    {
        writer(cell.x);
        writer(cell.y);
        writer(bvhIndex);
    }

    // Write BVHs
    size_t bvhsSize = mBvhs.size();
    writer(bvhsSize);
    for (const auto& [cell, bvh] : mBvhs)
    {
        writer(cell.x);
        writer(cell.y);
        bvh.serialize(out);
    }

    // Write proxies free list
    size_t proxiesFreeListSize = mProxiesFreeList.size();
    writer(proxiesFreeListSize);
    for (const BroadPhaseTreeHandle& handle : mProxiesFreeList)
        writer(handle);

    // Write BVH free list
    size_t bvhFreeListSize = mBvhFreeList.size();
    writer(bvhFreeListSize);
    for (const size_t index : mBvhFreeList)
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

    tree.mProxies.reserve(proxiesSize);
    for (size_t i = 0; i < proxiesSize; ++i)
        tree.mProxies.push_back(Proxy::deserialize(in));

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

        tree.mRegions.emplace(cell, bvhIndex);
    }

    // Read BVHs
    size_t bvhsSize;
    reader(bvhsSize);

    tree.mBvhs.reserve(bvhsSize);
    for (size_t i = 0; i < bvhsSize; ++i)
    {
        GridCell cell;
        reader(cell.x);
        reader(cell.y);

        DynamicBVH<IdType> bvh = DynamicBVH<IdType>::deserialize(in);
        tree.mBvhs.emplace_back(cell, std::move(bvh));
    }

    // Read proxies free list
    size_t proxiesFreeListSize;
    reader(proxiesFreeListSize);

    tree.mProxiesFreeList.reserve(proxiesFreeListSize);
    for (size_t i = 0; i < proxiesFreeListSize; ++i)
    {
        BroadPhaseTreeHandle handle;
        reader(handle);
        tree.mProxiesFreeList.push_back(handle);
    }

    // Read BVH free list
    size_t bvhFreeListSize;
    reader(bvhFreeListSize);

    tree.mBvhFreeList.reserve(bvhFreeListSize);
    for (size_t i = 0; i < bvhFreeListSize; ++i)
    {
        size_t index;
        reader(index);
        tree.mBvhFreeList.push_back(index);
    }

    return tree;
}

#ifdef C2D_USE_CEREAL
template <typename IdType>
template <IsCerealArchive Archive>
void BroadPhaseTree<IdType>::serialize(Archive& archive)
{
    archive(mCellSize);
    archive(mProxies);
    archive(mProxiesFreeList);
    archive(mRegions);
    archive(mBvhs);
    archive(mBvhFreeList);
}
#endif

template <typename IdType>
void BroadPhaseTree<IdType>::Proxy::serialize(std::ostream& out) const
{
    Writer writer(out);

    writer(mEntityId);
    writer(mBoundingBox.min.x);
    writer(mBoundingBox.min.y);
    writer(mBoundingBox.max.x);
    writer(mBoundingBox.max.y);
    writer(mCategoryBits);
    writer(mMaskBits);

    // Write bvhHandles size
    size_t bvhHandlesSize = mBvhHandles.size();
    writer(bvhHandlesSize);

    // Write bvhHandles
    for (const auto& [cell, nodeIndex] : mBvhHandles)
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
    reader(proxy.mEntityId);
    reader(proxy.mBoundingBox.min.x);
    reader(proxy.mBoundingBox.min.y);
    reader(proxy.mBoundingBox.max.x);
    reader(proxy.mBoundingBox.max.y);
    reader(proxy.mCategoryBits);
    reader(proxy.mMaskBits);

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

        proxy.mBvhHandles.emplace(cell, nodeIndex);
    }

    return proxy;
}

#ifdef C2D_USE_CEREAL
template <typename IdType>
template <IsCerealArchive Archive>
void BroadPhaseTree<IdType>::Proxy::serialize(Archive& archive)
{
    archive(mEntityId, mBoundingBox, mCategoryBits, mMaskBits, mBvhHandles);
}
#endif

template <typename IdType>
bool BroadPhaseTree<IdType>::isValidHandle(BroadPhaseTreeHandle handle) const
{
    return static_cast<size_t>(handle) < mProxies.size();
}

template <typename IdType>
size_t BroadPhaseTree<IdType>::getBVHIndexForCell(GridCell cell)
{
    const auto it = mRegions.find(cell);
    if (it != mRegions.end())
        return it->second;

    // If the cell does not exist, create a new BVH for it
    if (!mBvhFreeList.empty())
    {
        size_t bvhIndex = mBvhFreeList.back();
        mBvhFreeList.pop_back();
        mRegions.emplace(cell, bvhIndex);
        mBvhs[bvhIndex].first = cell;
        return bvhIndex;
    }
    else
    {
        size_t newBvhIndex = mBvhs.size();
        mBvhs.emplace_back(cell, DynamicBVH<IdType>());
        mRegions.emplace(cell, newBvhIndex);
        return newBvhIndex;
    }
}

template <typename IdType>
void BroadPhaseTree<IdType>::clear()
{
    mProxies.clear();
    mProxies.reserve(32);
    mProxiesFreeList.clear();
    mRegions.clear();
    mBvhs.clear();
    mBvhFreeList.clear();
}

template <typename IdType>
bool BroadPhaseTree<IdType>::operator==(const BroadPhaseTree& other) const
{
    if (mCellSize != other.mCellSize)
        return false;
    if (mProxies != other.mProxies)
        return false;
    if (mProxiesFreeList != other.mProxiesFreeList)
        return false;
    if (mRegions != other.mRegions)
        return false;
    if (mBvhs != other.mBvhs)
        return false;
    if (mBvhFreeList != other.mBvhFreeList)
        return false;

    return true;
}

template <typename IdType>
bool BroadPhaseTree<IdType>::Proxy::operator==(const Proxy& other) const
{
    if (mBvhHandles != other.mBvhHandles)
        return false;
    if (mEntityId != other.mEntityId)
        return false;
    if (mBoundingBox != other.mBoundingBox)
        return false;
    if (mCategoryBits != other.mCategoryBits)
        return false;
    if (mMaskBits != other.mMaskBits)
        return false;

    return true;
}

} // namespace c2d
