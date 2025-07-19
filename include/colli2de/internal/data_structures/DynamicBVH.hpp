#pragma once

#include <algorithm>
#include <cstdint>
#include <execution>
#include <ranges>
#include <set>
#include <utility>
#include <vector>

#include <colli2de/Ray.hpp>
#include <colli2de/internal/geometry/AABB.hpp>
#include <colli2de/internal/utils/methods.hpp>

namespace
{
    constexpr float displacementFactor = 2.2f; // Factor to adjust the displacement for fattening AABBs

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
    
    const float fatAABBMargin = 0.0f;

    DynamicBVH(float fatAABBMargin = 0.0f);

    NodeIndex createNode();
    void destroyNode(NodeIndex nodeId);
    
    // Inserts a new object {aabb, id} into the BVH and returns the index of the created node
    NodeIndex createProxy(AABB aabb, IdType id, BitMaskType categoryBits = 1, BitMaskType isHittingBits = ~0ull);
    void destroyProxy(NodeIndex leafIndex);
    // Moves the proxy to a new AABB, returns true if the tree structure has changed
    bool moveProxy(NodeIndex nodeIndex, AABB aabb);

    void clear();
    size_t size() const { return nodeCount; }
    size_t capacity() const { return nodes.size(); }
    size_t proxies() const { return proxyCount; }
    
    NodeIndex getRootIndex() const { return rootIndex; }
    const BVHNode<IdType>& getNode(NodeIndex index) const { return nodes[index]; }

    std::vector<std::pair<IdType, AABB>> data() const;
    auto leavesView() const;
    LeafConstIterator begin() const { return LeafConstIterator(nodes, 0); }
    LeafConstIterator end() const { return LeafConstIterator(nodes, nodes.size()); }
    LeafConstIterator cbegin() const { return LeafConstIterator(nodes, 0); }
    LeafConstIterator cend() const { return LeafConstIterator(nodes, nodes.size()); }

    // AABB queries
    void query(AABB queryAABB, std::set<IdType>& intersections, BitMaskType maskBits = ~0ull) const;
    std::vector<std::set<IdType>> batchQuery(const std::vector<AABB>& queries,
                                             BitMaskType maskBits = ~0ull) const;
    void findAllCollisions(std::set<std::pair<IdType, IdType>>& collisions) const;

    // Finite raycast queries
    std::optional<RaycastInfo> firstHitRaycast(Ray ray, BitMaskType maskBits = ~0ull) const;
    void piercingRaycast(Ray ray, std::set<RaycastInfo>& hits, BitMaskType maskBits = ~0ull) const;

    // Infinite raycast queries
    std::optional<RaycastInfo> firstHitRaycast(InfiniteRay ray, BitMaskType maskBits = ~0ull) const;
    void piercingRaycast(InfiniteRay ray, std::set<RaycastInfo>& hits, BitMaskType maskBits = ~0ull) const;

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
    void allocateNodes(int32_t capacity);

    void insertLeaf(NodeIndex leaf);
    void removeLeaf(NodeIndex leafIndex);
    void collectLeaves(NodeIndex nodeIdx, std::vector<NodeIndex>& out) const;
    void findPairsBetween(NodeIndex nodeAIdx, NodeIndex nodeBIdx, std::set<std::pair<IdType, IdType>>& out) const;
    void findAllCollisionsRecursive(NodeIndex nodeIdx, std::set<std::pair<IdType, IdType>>& out) const;

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



template<typename IdType>
DynamicBVH<IdType>::DynamicBVH(float fatAABBMargin) : fatAABBMargin(fatAABBMargin)
{
    allocateNodes(initialCapacity);
}

template<typename IdType>
void DynamicBVH<IdType>::allocateNodes(int32_t capacity)
{
    nodes.resize(capacity);

    // Create a singly-linked free list
    // [0] -> [1] -> [2] -> ... -> [capacity - 1] -> null
    for (NodeIndex i = 0; i < capacity - 1; ++i)
        nodes[i].parentIndex = i + 1;
    nodes[capacity - 1].parentIndex = INVALID_NODE_INDEX;

    nextAvailableIndex = 0;
    nodeCount = 0;
}

template<typename IdType>
void DynamicBVH<IdType>::doubleCapacity()
{
    assert(nextAvailableIndex == INVALID_NODE_INDEX);

    const size_t currentCapacity = capacity();
    assert(nodeCount == static_cast<uint32_t>(currentCapacity));

    const size_t newCapacity = currentCapacity <= 4096 ? currentCapacity << 1 : currentCapacity << 1;
    nodes.resize(newCapacity);
    
    for (NodeIndex i = currentCapacity - 1; i < static_cast<int32_t>(newCapacity) - 1; ++i)
        nodes[i].parentIndex = i + 1;
    nodes[newCapacity - 1].parentIndex = INVALID_NODE_INDEX;
    
    nextAvailableIndex = currentCapacity;
}

template<typename IdType>
NodeIndex DynamicBVH<IdType>::createNode()
{
    if (nextAvailableIndex == INVALID_NODE_INDEX)
        doubleCapacity();

    const NodeIndex nodeId = nextAvailableIndex;
    BVHNode<IdType>& node = nodes[nodeId];

    nextAvailableIndex = node.parentIndex;
    node.parentIndex = INVALID_NODE_INDEX;
    node.child1Index = -1;
    node.child2Index = -1;
    node.height = 0;
    node.id = IdType{};

    ++nodeCount;
    return nodeId;
}

template<typename IdType>
void DynamicBVH<IdType>::destroyNode(NodeIndex nodeId)
{
    assert(0 <= nodeId && nodeId < static_cast<NodeIndex>(nodes.size()));
    BVHNode<IdType>& node = nodes[nodeId];

    node.parentIndex = nextAvailableIndex;
    node.height = -1;
    node.child1Index = INVALID_NODE_INDEX;
    node.child2Index = INVALID_NODE_INDEX;
    node.id = IdType{};

    nextAvailableIndex = nodeId;
    --nodeCount;
}

template<typename IdType>
NodeIndex DynamicBVH<IdType>::createProxy(AABB aabb, IdType id, BitMaskType categoryBits, BitMaskType isHittingBits)
{
    const NodeIndex nodeId = createNode();
    BVHNode<IdType>& node = nodes[nodeId];

    node.aabb = aabb.fattened(fatAABBMargin);
    node.id = id;
    node.categoryBits = categoryBits;
    node.isHittingBits = isHittingBits;

    ++proxyCount;
    insertLeaf(nodeId);
    return nodeId;
}

template<typename IdType>
void DynamicBVH<IdType>::destroyProxy(NodeIndex leafIndex)
{
    --proxyCount;
    removeLeaf(leafIndex);
    destroyNode(leafIndex);
}

template <typename IdType>
void DynamicBVH<IdType>::clear()
{
    nodes.clear();
    nodeCount = 0;
    rootIndex = INVALID_NODE_INDEX;
    nextAvailableIndex = INVALID_NODE_INDEX;
    allocateNodes(initialCapacity);
}

template<typename IdType>
bool DynamicBVH<IdType>::moveProxy(NodeIndex nodeIndex, AABB newAABB)
{
    // If the current fattened AABB contains the new AABB, no need to update the tree
    if (nodes[nodeIndex].aabb.contains(newAABB))
        return false;

    // Update the leaf AABB in place and propagate the change up the tree.
    // This avoids expensive remove/insert operations for large displacements.
    const Vec2 displacement = Vec2((newAABB.min.x - nodes[nodeIndex].aabb.min.x) * displacementFactor,
                                   (newAABB.min.y - nodes[nodeIndex].aabb.min.y) * displacementFactor);
    const AABB fatAabb = newAABB.fattened(fatAABBMargin, displacement);
    nodes[nodeIndex].aabb = fatAabb;

    NodeIndex currentIndex = nodes[nodeIndex].parentIndex;
    while (currentIndex != INVALID_NODE_INDEX)
    {
        const NodeIndex child1 = nodes[currentIndex].child1Index;
        const NodeIndex child2 = nodes[currentIndex].child2Index;

        const AABB newParentAABB = AABB::combine(nodes[child1].aabb, nodes[child2].aabb);
        if (newParentAABB == nodes[currentIndex].aabb)
            break;

        nodes[currentIndex].aabb = newParentAABB;
        currentIndex = nodes[currentIndex].parentIndex;
    }
    return true;
}

template<typename IdType>
void DynamicBVH<IdType>::insertLeaf(NodeIndex leafIndex)
{
    if (rootIndex == INVALID_NODE_INDEX)
    {
        rootIndex = leafIndex;
        nodes[rootIndex].parentIndex = INVALID_NODE_INDEX;
        return;
    }

    AABB leaf = nodes[leafIndex].aabb;

    const NodeIndex siblingIndex = findBestSiblingIndex(leaf);
    const NodeIndex oldParentIndex = nodes[siblingIndex].parentIndex;

    const NodeIndex newParentIndex = createNode();
    nodes[newParentIndex].parentIndex = oldParentIndex;
    nodes[newParentIndex].aabb = AABB::combine(leaf, nodes[siblingIndex].aabb);
    nodes[newParentIndex].height = nodes[siblingIndex].height + 1;
    nodes[newParentIndex].categoryBits = nodes[siblingIndex].categoryBits | nodes[leafIndex].categoryBits;

    if (oldParentIndex != INVALID_NODE_INDEX)
    {
        if (nodes[oldParentIndex].child1Index == siblingIndex)
            nodes[oldParentIndex].child1Index = newParentIndex;
        else
            nodes[oldParentIndex].child2Index = newParentIndex;

        nodes[newParentIndex].child1Index = siblingIndex;
        nodes[newParentIndex].child2Index = leafIndex;
        nodes[siblingIndex].parentIndex = newParentIndex;
        nodes[leafIndex].parentIndex = newParentIndex;
    }
    else
    {
        nodes[newParentIndex].child1Index = siblingIndex;
        nodes[newParentIndex].child2Index = leafIndex;
        nodes[siblingIndex].parentIndex = newParentIndex;
        nodes[leafIndex].parentIndex = newParentIndex;
        rootIndex = newParentIndex;
    }

    // Walk up the tree fixing heights and AABBs
    NodeIndex currentNodeIndex = nodes[leafIndex].parentIndex;
    while (currentNodeIndex != INVALID_NODE_INDEX)
    {
        currentNodeIndex = balance(currentNodeIndex);

        const NodeIndex child1Index = nodes[currentNodeIndex].child1Index;
        const NodeIndex child2Index = nodes[currentNodeIndex].child2Index;

        nodes[currentNodeIndex].aabb = AABB::combine(nodes[child1Index].aabb, nodes[child2Index].aabb);
        nodes[currentNodeIndex].height = 1 + std::max(nodes[child1Index].height, nodes[child2Index].height);
        nodes[currentNodeIndex].categoryBits = nodes[child1Index].categoryBits | nodes[child2Index].categoryBits;

        currentNodeIndex = nodes[currentNodeIndex].parentIndex;
    }
}

template<typename IdType>
void DynamicBVH<IdType>::removeLeaf(NodeIndex leafIndex)
{
    if (leafIndex == rootIndex)
    {
        // destroyNode(rootIndex);
        rootIndex = INVALID_NODE_INDEX;
        return;
    }

    const NodeIndex parentIndex = nodes[leafIndex].parentIndex;
    const NodeIndex grandParentIndex = nodes[parentIndex].parentIndex;
    const NodeIndex siblingIndex = (nodes[parentIndex].child1Index == leafIndex)
                                 ? nodes[parentIndex].child2Index
                                 : nodes[parentIndex].child1Index;

    // Connect sibling to grandparent
    if (grandParentIndex != INVALID_NODE_INDEX)
    {
        if (nodes[grandParentIndex].child1Index == parentIndex)
            nodes[grandParentIndex].child1Index = siblingIndex;
        else
            nodes[grandParentIndex].child2Index = siblingIndex;
        nodes[siblingIndex].parentIndex = grandParentIndex;
    }
    else
    {
        rootIndex = siblingIndex;
        nodes[siblingIndex].parentIndex = INVALID_NODE_INDEX;
    }

    // Recycle parent node
    destroyNode(parentIndex);

    // Walk up and fix heights and AABBs (including rebalancing)
    NodeIndex currentNodeIndex = grandParentIndex;
    while (currentNodeIndex != INVALID_NODE_INDEX)
    {
        currentNodeIndex = balance(currentNodeIndex);

        const NodeIndex child1 = nodes[currentNodeIndex].child1Index;
        const NodeIndex child2 = nodes[currentNodeIndex].child2Index;

        nodes[currentNodeIndex].aabb = AABB::combine(nodes[child1].aabb, nodes[child2].aabb);
        nodes[currentNodeIndex].height = 1 + std::max(nodes[child1].height, nodes[child2].height);
        nodes[currentNodeIndex].categoryBits = nodes[child1].categoryBits | nodes[child2].categoryBits;

        currentNodeIndex = nodes[currentNodeIndex].parentIndex;
    }
}

template<typename IdType>
NodeIndex DynamicBVH<IdType>::findBestSiblingIndex(AABB leaf) const
{
    NodeIndex index = rootIndex;

    // Find the best sibling for the new leaf
    while (!nodes[index].isLeaf())
    {
        const NodeIndex child1Index = nodes[index].child1Index;
        const NodeIndex child2Index = nodes[index].child2Index;

        const AABB combined = AABB::combine(leaf, nodes[index].aabb);
        const float cost = combined.perimeter();
        const float inheritedCost = 2.0f * cost;

        const auto child1LeafCombined = AABB::combine(leaf, nodes[child1Index].aabb);
        const float cost1 = nodes[child1Index].isLeaf()
            ? child1LeafCombined.perimeter() + inheritedCost
            : child1LeafCombined.perimeter() - nodes[child1Index].aabb.perimeter() + inheritedCost;

        const auto child2LeafCombined = AABB::combine(leaf, nodes[child2Index].aabb);
        const float cost2 = nodes[child2Index].isLeaf()
            ? child2LeafCombined.perimeter() + inheritedCost
            : child2LeafCombined.perimeter() - nodes[child2Index].aabb.perimeter() + inheritedCost;

        index = (cost1 < cost2) ? child1Index : child2Index;
    }

    return index;
}

template<typename IdType>
NodeIndex DynamicBVH<IdType>::balance(NodeIndex index)
{
    BVHNode<IdType>& node = nodes[index];

    // Can't balance a leaf
    // if (node.isLeaf() || node.height < 2)
    //     return index;

    const NodeIndex child1Index = node.child1Index;
    const NodeIndex child2Index = node.child2Index;
    const BVHNode<IdType>& child1 = nodes[child1Index];
    const BVHNode<IdType>& child2 = nodes[child2Index];

    const int32_t unbalanceBetweenChildren = child2.height - child1.height;

    NodeIndex movingUpChildIndex;
    NodeIndex grandChild1Index;
    NodeIndex grandChild2Index;
    bool isRightRotation;
    if (unbalanceBetweenChildren > 1)
    {
        // Rotate Child2 up
        grandChild1Index = child2.child1Index;
        grandChild2Index = child2.child2Index;
        movingUpChildIndex = child2Index;
        isRightRotation = true;
    }
    else if (unbalanceBetweenChildren < -1)
    {
        // Rotate Child1 up
        grandChild1Index = child1.child1Index;
        grandChild2Index = child1.child2Index;
        movingUpChildIndex = child1Index;
        isRightRotation = false;
    }
    else
    {
        // No rotation needed
        return index;
    }

    BVHNode<IdType>& movingUpChild = nodes[movingUpChildIndex];

    // Swap node and movingUpChild
    movingUpChild.child1Index = index;
    movingUpChild.parentIndex = node.parentIndex;
    node.parentIndex = movingUpChildIndex;

    if (movingUpChild.parentIndex != INVALID_NODE_INDEX)
    {
        BVHNode<IdType>& parent = nodes[movingUpChild.parentIndex];
        if (parent.child1Index == index)
            parent.child1Index = movingUpChildIndex;
        else
            parent.child2Index = movingUpChildIndex;
    }
    else
    {
        rootIndex = movingUpChildIndex;
    }

    // Pick the taller child for attachment
    NodeIndex attachIndex, remainIndex;
    if ((nodes[grandChild1Index].height > nodes[grandChild2Index].height))
    {
        attachIndex = grandChild1Index;
        remainIndex = grandChild2Index;
    }
    else
    {
        attachIndex = grandChild2Index;
        remainIndex = grandChild1Index;
    }

    if (isRightRotation)
    {
        movingUpChild.child2Index = attachIndex;
        node.child2Index = remainIndex;
    }
    else
    {
        movingUpChild.child2Index = attachIndex;
        node.child1Index = remainIndex;
    }

    nodes[attachIndex].parentIndex = movingUpChildIndex;
    nodes[remainIndex].parentIndex = index;

    // Recompute AABBs and heights
    if (isRightRotation)
    {
        node.aabb = AABB::combine(child1.aabb, nodes[remainIndex].aabb);
        movingUpChild.aabb = AABB::combine(node.aabb, nodes[attachIndex].aabb);

        node.height = 1 + std::max(child1.height, nodes[remainIndex].height);
        movingUpChild.height = 1 + std::max(node.height, nodes[attachIndex].height);

        node.categoryBits = child1.categoryBits | nodes[remainIndex].categoryBits;
        movingUpChild.categoryBits = node.categoryBits | nodes[attachIndex].categoryBits;
    }
    else
    {
        node.aabb = AABB::combine(child2.aabb, nodes[remainIndex].aabb);
        movingUpChild.aabb = AABB::combine(node.aabb, nodes[attachIndex].aabb);

        node.height = 1 + std::max(child2.height, nodes[remainIndex].height);
        movingUpChild.height = 1 + std::max(node.height, nodes[attachIndex].height);

        node.categoryBits = child2.categoryBits | nodes[remainIndex].categoryBits;
        movingUpChild.categoryBits = node.categoryBits | nodes[attachIndex].categoryBits;
    }

    return movingUpChildIndex;
}

template<typename IdType>
std::vector<std::pair<IdType, AABB>> DynamicBVH<IdType>::data() const
{
    if (rootIndex == INVALID_NODE_INDEX)
        return {};

    std::vector<std::pair<IdType, AABB>> result;

    for (uint32_t i = 0; i < nodeCount; ++i)
    {
        const auto& node = nodes[i];
        if (node.isLeaf())
            result.emplace_back(node.id, node.aabb);
    }

    return result;
}

template<typename IdType>
auto DynamicBVH<IdType>::leavesView() const
{
    return nodes
        | std::views::filter([](const auto& node) { return node.isLeaf(); })
        | std::views::transform([](const auto& node) { return std::pair<IdType, AABB>{node.id, node.aabb}; });
}

template<typename IdType>
void DynamicBVH<IdType>::query(AABB queryAABB, std::set<IdType>& intersections, BitMaskType maskBits) const
{
    std::vector<NodeIndex> stack;
    stack.push_back(rootIndex);

    while (!stack.empty())
    {
        NodeIndex nodeIndex = stack.back();
        stack.pop_back();
        const auto& node = nodes[nodeIndex];

        // Collision detected only if the node's categoryBits match the maskBits
        if (!node.matchesMask(maskBits))
            continue;
        // Skip the whole subtree (since it is contained in the node's AABB)
        if (!node.aabb.intersects(queryAABB))
            continue;

        if (node.isLeaf())
        {
            intersections.insert(node.id);
        }
        else
        {
            stack.push_back(node.child1Index);
            stack.push_back(node.child2Index);
        }
    }
}

template<typename IdType>
std::vector<std::set<IdType>> DynamicBVH<IdType>::batchQuery(const std::vector<AABB>& queries,
                                                             BitMaskType maskBits) const
{
    const size_t n = queries.size();
    std::vector<std::set<IdType>> results(n);
    std::ranges::iota_view indexes((size_t)0, n);

    std::for_each(std::execution::par_unseq, indexes.begin(), indexes.end(), [&](size_t i) {
        query(queries[i], results[i], maskBits);
    });

    return results;
}

template<typename IdType>
void DynamicBVH<IdType>::piercingRaycast(Ray ray,
                                         std::set<typename DynamicBVH<IdType>::RaycastInfo>& hits,
                                         BitMaskType maskBits) const
{
    if (rootIndex == INVALID_NODE_INDEX)
        return;

    std::vector<NodeIndex> stack;
    stack.push_back(rootIndex);

    while (!stack.empty())
    {
        NodeIndex nodeIndex = stack.back();
        stack.pop_back();
        const auto& node = nodes[nodeIndex];

        // Collision detected only if the node's categoryBits match the maskBits
        if (!node.matchesMask(maskBits))
            continue;

        const auto intersection = node.aabb.intersects(ray);
        if (!intersection)
            continue;

        if (node.isLeaf())
        {
            hits.insert(RaycastInfo::fromRay(node.id, ray, *intersection));
        }
        else
        {
            stack.push_back(node.child1Index);
            stack.push_back(node.child2Index);
        }
    }
}

template<typename IdType>
std::optional<typename DynamicBVH<IdType>::RaycastInfo> DynamicBVH<IdType>::firstHitRaycast(Ray ray,
                                                                                            BitMaskType maskBits) const
{
    if (rootIndex == INVALID_NODE_INDEX)
        return std::nullopt;

    std::optional<RaycastInfo> firstHit;
    float firstHitTime = std::numeric_limits<float>::max();

    std::vector<NodeIndex> stack;
    stack.push_back(rootIndex);

    while (!stack.empty())
    {
        NodeIndex nodeIndex = stack.back();
        stack.pop_back();
        const auto& node = nodes[nodeIndex];

        // Collision detected only if the node's categoryBits match the maskBits
        if (!node.matchesMask(maskBits))
            continue;

        const auto intersection = node.aabb.intersects(ray);
        if (!intersection)
            continue;

        if (node.isLeaf())
        {
            if (intersection->first < firstHitTime)
            {
                firstHitTime = intersection->first;
                firstHit = RaycastInfo::fromRay(node.id, ray, *intersection);
            }
        }
        else
        {
            stack.push_back(node.child1Index);
            stack.push_back(node.child2Index);
        }
    }

    return firstHit;
}

template<typename IdType>
void DynamicBVH<IdType>::piercingRaycast(InfiniteRay ray,
                                         std::set<typename DynamicBVH<IdType>::RaycastInfo>& hits,
                                         BitMaskType maskBits) const
{
    if (rootIndex == INVALID_NODE_INDEX)
        return;

    std::vector<NodeIndex> stack;
    stack.push_back(rootIndex);

    while (!stack.empty())
    {
        NodeIndex nodeIndex = stack.back();
        stack.pop_back();
        const auto& node = nodes[nodeIndex];

        // Collision detected only if the node's categoryBits match the maskBits
        if (!node.matchesMask(maskBits))
            continue;

        const auto intersection = node.aabb.intersects(ray);
        if (!intersection)
            continue;

        if (node.isLeaf())
        {
            hits.insert(RaycastInfo::fromRay(node.id, ray, *intersection));
        }
        else
        {
            stack.push_back(node.child1Index);
            stack.push_back(node.child2Index);
        }
    }
}

template<typename IdType>
std::optional<typename DynamicBVH<IdType>::RaycastInfo> DynamicBVH<IdType>::firstHitRaycast(InfiniteRay ray,
                                                                                            BitMaskType maskBits) const
{
    if (rootIndex == INVALID_NODE_INDEX)
        return std::nullopt;

    std::optional<RaycastInfo> firstHit;
    float firstHitTime = std::numeric_limits<float>::max();

    std::vector<NodeIndex> stack;
    stack.push_back(rootIndex);

    while (!stack.empty())
    {
        NodeIndex nodeIndex = stack.back();
        stack.pop_back();
        const auto& node = nodes[nodeIndex];

        // Collision detected only if the node's categoryBits match the maskBits
        if (!node.matchesMask(maskBits))
            continue;

        const auto intersection = node.aabb.intersects(ray);
        if (!intersection)
            continue;

        if (node.isLeaf())
        {
            if (intersection->first < firstHitTime)
            {
                firstHitTime = intersection->first;
                firstHit = RaycastInfo::fromRay(node.id, ray, *intersection);
            }
        }
        else
        {
            stack.push_back(node.child1Index);
            stack.push_back(node.child2Index);
        }
    }

    return firstHit;
}

template<typename IdType>
void DynamicBVH<IdType>::findAllCollisions(std::set<std::pair<IdType, IdType>>& out) const
{
    findAllCollisionsRecursive(rootIndex, out);
}

template<typename IdType>
void DynamicBVH<IdType>::collectLeaves(NodeIndex nodeIdx, std::vector<NodeIndex>& out) const
{
    const auto& node = nodes[nodeIdx];
    if (node.isLeaf())
    {
        out.push_back(nodeIdx);
        return;
    }

    collectLeaves(node.child1Index, out);
    collectLeaves(node.child2Index, out);
}

template<typename IdType>
void DynamicBVH<IdType>::findPairsBetween(NodeIndex nodeAIdx,
                                          NodeIndex nodeBIdx,
                                          std::set<std::pair<IdType, IdType>>& out) const
{
    if (nodeAIdx == INVALID_NODE_INDEX || nodeBIdx == INVALID_NODE_INDEX || nodeAIdx == nodeBIdx)
        return;

    const auto& nodeA = nodes[nodeAIdx];
    const auto& nodeB = nodes[nodeBIdx];

    // Check bitmask compatibility
    if (!nodeA.matchesMask(nodeB.categoryBits) || !nodeB.matchesMask(nodeA.categoryBits))
        return;
    if (!nodeA.aabb.intersects(nodeB.aabb))
        return;

    if (nodeA.isLeaf() && nodeB.isLeaf())
    {
        // Always store (min, max) to avoid (A, B) and (B, A) being different
        if (nodeA.id < nodeB.id)
            out.emplace(nodeA.id, nodeB.id);
        else
            out.emplace(nodeB.id, nodeA.id);
        return;
    }

    // Expand the node with greater height (heuristic)
    if (nodeA.isLeaf() || (!nodeB.isLeaf() && nodeB.height > nodeA.height))
    {
        findPairsBetween(nodeAIdx, nodeB.child1Index, out);
        findPairsBetween(nodeAIdx, nodeB.child2Index, out);
    }
    else
    {
        findPairsBetween(nodeA.child1Index, nodeBIdx, out);
        findPairsBetween(nodeA.child2Index, nodeBIdx, out);
    }
}

template<typename IdType>
void DynamicBVH<IdType>::findAllCollisionsRecursive(NodeIndex nodeIdx,
                                                    std::set<std::pair<IdType, IdType>>& out) const
{
    if (nodeIdx == INVALID_NODE_INDEX)
        return;

    const auto& node = nodes[nodeIdx];
    if (node.isLeaf())
        return;

    // Check all pairs between children
    findPairsBetween(node.child1Index, node.child2Index, out);

    // Check pairs within each child
    findAllCollisionsRecursive(node.child1Index, out);
    findAllCollisionsRecursive(node.child2Index, out);
}

template<typename IdType>
void DynamicBVH<IdType>::serialize(std::ostream& out) const
{
    write_pod(out, fatAABBMargin);
    write_pod(out, rootIndex);
    write_pod(out, nodeCount);
    write_pod(out, nextAvailableIndex);

    // Write nodes vector size
    uint64_t nodesSize = nodes.size();
    write_pod(out, nodesSize);

    // Write nodes
    for (const auto& node : nodes)
    {
        write_pod(out, node.aabb.min.x); write_pod(out, node.aabb.min.y);
        write_pod(out, node.aabb.max.x); write_pod(out, node.aabb.max.y);
        write_pod(out, node.parentIndex);
        write_pod(out, node.child1Index);
        write_pod(out, node.child2Index);
        write_pod(out, node.height);
        write_pod(out, node.id);
    }
}

template<typename IdType>
DynamicBVH<IdType> DynamicBVH<IdType>::deserialize(std::istream& in)
{
    float fatAABBMargin;
    read_pod(in, fatAABBMargin);
    DynamicBVH<IdType> bvh(fatAABBMargin);

    read_pod(in, bvh.rootIndex);
    read_pod(in, bvh.nodeCount);
    read_pod(in, bvh.nextAvailableIndex);

    uint64_t nodesSize;
    read_pod(in, nodesSize);
    bvh.nodes.resize(nodesSize);

    for (auto& node : bvh.nodes)
    {
        read_pod(in, node.aabb.min.x); read_pod(in, node.aabb.min.y);
        read_pod(in, node.aabb.max.x); read_pod(in, node.aabb.max.y);
        read_pod(in, node.parentIndex);
        read_pod(in, node.child1Index);
        read_pod(in, node.child2Index);
        read_pod(in, node.height);
        read_pod(in, node.id);
    }

    return bvh;
}

template<typename IdType>
bool DynamicBVH<IdType>::operator==(const DynamicBVH<IdType>& other) const
{
    if (!float_equals(fatAABBMargin, other.fatAABBMargin))
        return false;
    if (rootIndex != other.rootIndex)
        return false;
    if (nodeCount != other.nodeCount)
        return false;
    if (nextAvailableIndex != other.nextAvailableIndex)
        return false;
    if (nodes.size() != other.nodes.size())
        return false;

    for (size_t i = 0; i < nodes.size(); ++i)
    {
        const auto& a = nodes[i];
        const auto& b = other.nodes[i];

        if (a.aabb != b.aabb)
            return false;
        if (a.parentIndex != b.parentIndex)
            return false;
        if (a.child1Index != b.child1Index)
            return false;
        if (a.child2Index != b.child2Index)
            return false;
        if (a.height != b.height)
            return false;
        if (a.id != b.id)
            return false;
    }

    return true;
}

} // namespace c2d
