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

} // namespace c2d

#include <colli2de/internal/data_structures/BroadPhaseTree_impl.tpp>
