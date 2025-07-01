#pragma once

#include <cstdint>
#include <utility>
#include <utils/methods.hpp>
#include <vector>

#include "colli2de/Ray.hpp"
#include "geometry/AABB.hpp"

namespace
{
    template<typename T>
    void write_pod(std::ostream& out, const T& v)
    {
        out.write(reinterpret_cast<const char*>(&v), sizeof(T));
    
    }
    template<typename T>
    void read_pod(std::istream& in, T& v)
    {
        in.read(reinterpret_cast<char*>(&v), sizeof(T));
    }
}

namespace c2d
{

using BitMaskType = uint64_t;
using NodeIndex = int32_t;
static constexpr NodeIndex INVALID_NODE_INDEX = -99;

template<typename IdType>
struct BVHNode
{
    AABB aabb;
    BitMaskType categoryBits = 1;      // for collision filtering
    BitMaskType isHittingBits = ~0ull; // for collision filtering

    NodeIndex parentIndex = INVALID_NODE_INDEX;
    NodeIndex child1Index = INVALID_NODE_INDEX;
    NodeIndex child2Index = INVALID_NODE_INDEX;

    uint32_t height = -1;

    IdType id{}; // stored payload (instead of void*)

    bool isLeaf() const { return child1Index == -1; }
    bool matchesMask(BitMaskType mask) const { return (categoryBits & mask) != 0; }
};

template<typename IdType>
class DynamicBVH
{
public:
    class LeafConstIterator;
    static constexpr uint32_t initialCapacity = 64;
    using RaycastInfo = RaycastHit<IdType>;
    using const_iterator = LeafConstIterator;
    using value_type = BVHNode<IdType>;
    
    const float fatAABBMargin = 3.0f;

    DynamicBVH(float fatAABBMargin = 0.0f);

    NodeIndex createNode();
    void destroyNode(NodeIndex nodeId);
    
    // Inserts a new object {aabb, id} into the BVH and returns the index of the created node
    NodeIndex createProxy(AABB aabb, IdType id, BitMaskType categoryBits = 1, BitMaskType isHittingBits = ~0ull);
    void destroyProxy(NodeIndex leafIndex);
    // Moves the proxy to a new AABB, returns true if the tree structure has changed
    bool moveProxy(NodeIndex nodeIndex, AABB aabb, Vec2 displacement = Vec2{0.0f, 0.0f});

    void clear();
    uint32_t size() const { return nodeCount; }
    uint32_t capacity() const { return nodes.size(); }
    uint32_t proxies() const { return proxyCount; }
    
    NodeIndex getRootIndex() const { return rootIndex; }
    const BVHNode<IdType>& getNode(NodeIndex index) const { return nodes[index]; }

    std::vector<std::pair<IdType, AABB>> data() const;
    auto leavesView() const;
    LeafConstIterator begin() const { return LeafConstIterator(nodes, 0); }
    LeafConstIterator end() const { return LeafConstIterator(nodes, nodes.size()); }
    LeafConstIterator cbegin() const { return LeafConstIterator(nodes, 0); }
    LeafConstIterator cend() const { return LeafConstIterator(nodes, nodes.size()); }

    // AABB queries
    std::vector<IdType> query(AABB queryAABB, BitMaskType maskBits = ~0ull) const;
    std::vector<std::vector<IdType>> batchQuery(const std::vector<AABB>& queries, size_t numThreads, BitMaskType maskBits = ~0ull) const;

    // Finite raycast queries
    std::optional<IdType> firstHitRaycast(Ray ray, BitMaskType maskBits = ~0ull) const;
    std::optional<RaycastInfo> firstHitRaycastDetailed(Ray ray, BitMaskType maskBits = ~0ull) const;
    std::vector<IdType> piercingRaycast(Ray ray, BitMaskType maskBits = ~0ull) const;
    std::vector<RaycastInfo> piercingRaycastDetailed(Ray ray, BitMaskType maskBits = ~0ull) const;

    // Infinite raycast queries
    std::optional<IdType> firstHitRaycast(InfiniteRay ray, BitMaskType maskBits = ~0ull) const;
    std::optional<RaycastInfo> firstHitRaycastDetailed(InfiniteRay ray, BitMaskType maskBits = ~0ull) const;
    std::vector<IdType> piercingRaycast(InfiniteRay ray, BitMaskType maskBits = ~0ull) const;
    std::vector<RaycastInfo> piercingRaycastDetailed(InfiniteRay ray, BitMaskType maskBits = ~0ull) const;

    std::vector<std::pair<IdType, IdType>> findAllCollisions() const;

    void serialize(std::ostream& out) const;
    static DynamicBVH<IdType> deserialize(std::istream& in);

    bool operator==(const DynamicBVH<IdType>& other) const;

private:
    std::vector<BVHNode<IdType>> nodes;
    uint32_t nodeCount = 0;
    uint32_t proxyCount = 0;

    NodeIndex rootIndex = INVALID_NODE_INDEX;
    NodeIndex nextAvailableIndex = INVALID_NODE_INDEX;

    void doubleCapacity();
    void allocateNodes(uint32_t capacity);

    void insertLeaf(NodeIndex leaf);
    void removeLeaf(NodeIndex leafIndex);
    void collectLeaves(NodeIndex nodeIdx, std::vector<NodeIndex>& out) const;
    void findPairsBetween(NodeIndex nodeAIdx, NodeIndex nodeBIdx, std::vector<std::pair<IdType, IdType>>& out) const;
    void findAllCollisionsRecursive(NodeIndex nodeIdx, std::vector<std::pair<IdType, IdType>>& out) const;

    NodeIndex findBestSiblingIndex(AABB leaf) const;
    NodeIndex balance(NodeIndex index);

public:
    class LeafConstIterator {
    public:
        using iterator_category = std::forward_iterator_tag;
        using value_type = std::pair<IdType, AABB>;
        using difference_type = std::ptrdiff_t;
        using pointer = const std::pair<IdType, AABB>*;
        using reference = const std::pair<IdType, AABB>&;

        LeafConstIterator(const std::vector<BVHNode<IdType>>& nodes, uint32_t nodeIndex)
            : nodes(nodes), m_nodeIndex(nodeIndex)
        {
            if (m_nodeIndex < nodes.size() && !nodes[m_nodeIndex].isLeaf())
                next();
        }

        value_type operator*() const
        {
            cache = std::make_pair(nodes[m_nodeIndex].id, nodes[m_nodeIndex].aabb);
            return cache;
        }
        pointer operator->() const
        {
            cache = std::make_pair(nodes[m_nodeIndex].id, nodes[m_nodeIndex].aabb);
            return &cache;
        }

        LeafConstIterator& operator++()
        {
            next();
            return *this;
        }

        LeafConstIterator operator++(int)
        {
            LeafConstIterator tmp = *this;
            ++(*this);
            return tmp;
        }

        bool operator==(const LeafConstIterator& other) const { return m_nodeIndex == other.m_nodeIndex; }
        bool operator!=(const LeafConstIterator& other) const { return !(*this == other); }

    private:
        const std::vector<BVHNode<IdType>>& nodes;
        NodeIndex m_nodeIndex;
        mutable value_type cache;

        void next()
        {
            ++m_nodeIndex;
            while (m_nodeIndex < nodes.size() && !nodes[m_nodeIndex].isLeaf())
                ++m_nodeIndex;
        }
    };
};
static_assert(std::is_trivially_copyable_v<BVHNode<uint32_t>>, "BVHNode must be trivially copyable");

} // namespace c2d

#include "DynamicBVH_treeHandling.tpp"
#include "DynamicBVH_queries.tpp"
