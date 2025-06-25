#pragma once

#include <algorithm>
#include <cstdint>
#include <future>
#include <ranges>
#include <stack>
#include <utility>
#include <utils/methods.hpp>
#include <vector>

#include "geometry/AABB.hpp"
#include "geometry/Ray.hpp"

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

using NodeIndex = int32_t;
static constexpr NodeIndex INVALID_NODE_INDEX = -99;

template<typename IdType>
struct BVHNode
{
    AABB aabb;

    NodeIndex parentIndex = INVALID_NODE_INDEX;
    NodeIndex child1Index = INVALID_NODE_INDEX;
    NodeIndex child2Index = INVALID_NODE_INDEX;

    uint32_t height = -1;

    IdType id{}; // stored payload (instead of void*)

    bool isLeaf() const { return child1Index == -1; }
};

template<typename IdType>
class DynamicBVH
{
public:
    class LeafConstIterator;
    static constexpr uint32_t initialCapacity = 16;
    using RaycastInfo = RaycastHit<IdType>;
    using const_iterator = LeafConstIterator;
    using value_type = BVHNode<IdType>;
    
    const float fatAABBMargin = 3.0f;

    DynamicBVH(float fatAABBMargin = 0.0f);

    NodeIndex createNode();
    void destroyNode(NodeIndex nodeId);
    
    // Inserts a new object {aabb, id} into the BVH and returns the index of the created node
    NodeIndex createProxy(AABB aabb, IdType id);
    void destroyProxy(NodeIndex leafIndex);
    // Moves the proxy to a new AABB, returns true if the tree structure has changed
    bool moveProxy(NodeIndex nodeIndex, AABB aabb, Vec2 displacement = Vec2{0.0f, 0.0f});

    void clear();
    uint32_t size() const { return nodeCount; }
    uint32_t capacity() const { return nodes.size(); }
    
    NodeIndex getRootIndex() const { return rootIndex; }
    const BVHNode<IdType>& getNode(NodeIndex index) const { return nodes[index]; }

    std::vector<std::pair<IdType, AABB>> data() const;
    auto leavesView() const;
    LeafConstIterator begin() const { return LeafConstIterator(nodes, 0); }
    LeafConstIterator end() const { return LeafConstIterator(nodes, nodes.size()); }
    LeafConstIterator cbegin() const { return LeafConstIterator(nodes, 0); }
    LeafConstIterator cend() const { return LeafConstIterator(nodes, nodes.size()); }

    // AABB queries
    std::vector<IdType> query(AABB queryAABB) const;
    std::vector<std::vector<IdType>> batchQuery(const std::vector<AABB>& queries, size_t numThreads) const;

    // Finite raycast queries
    std::optional<IdType> firstHitRaycast(Ray ray) const;
    std::optional<RaycastInfo> firstHitRaycastDetailed(Ray ray) const;
    std::vector<IdType> piercingRaycast(Ray ray) const;
    std::vector<RaycastInfo> piercingRaycastDetailed(Ray ray) const;

    // Infinite raycast queries
    std::optional<IdType> firstHitRaycast(InfiniteRay ray) const;
    std::optional<RaycastInfo> firstHitRaycastDetailed(InfiniteRay ray) const;
    std::vector<IdType> piercingRaycast(InfiniteRay ray) const;
    std::vector<RaycastInfo> piercingRaycastDetailed(InfiniteRay ray) const;

    std::vector<std::pair<IdType, IdType>> findBroadPhaseCollisions() const;

    void serialize(std::ostream& out) const;
    static DynamicBVH<IdType> deserialize(std::istream& in);

    bool operator==(const DynamicBVH<IdType>& other) const;

private:
    std::vector<BVHNode<IdType>> nodes;
    uint32_t nodeCount = 0;

    NodeIndex rootIndex = INVALID_NODE_INDEX;
    NodeIndex nextAvailableIndex = INVALID_NODE_INDEX;

    void doubleCapacity();
    void allocateNodes(uint32_t capacity);

    void insertLeaf(NodeIndex leaf);
    void removeLeaf(NodeIndex leafIndex);
    void collectLeaves(NodeIndex nodeIdx, std::vector<NodeIndex>& out) const;
    void findPairsBetween(NodeIndex nodeAIdx, NodeIndex nodeBIdx, std::vector<std::pair<IdType, IdType>>& out) const;
    void findBroadPhaseCollisionsRecursive(NodeIndex nodeIdx, std::vector<std::pair<IdType, IdType>>& out) const;

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
DynamicBVH<IdType>::DynamicBVH(float fatAABBMargin)
    : fatAABBMargin(fatAABBMargin)
{
    allocateNodes(initialCapacity);
}

template<typename IdType>
void DynamicBVH<IdType>::allocateNodes(uint32_t capacity)
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
    const uint32_t currentCapacity = capacity();
    assert(nodeCount == currentCapacity);

    const uint32_t newCapacity = currentCapacity << 1;

    nodes.resize(newCapacity);
    for (NodeIndex i = currentCapacity - 1; i < newCapacity - 1; ++i)
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
NodeIndex DynamicBVH<IdType>::createProxy(AABB aabb, IdType id)
{
    const NodeIndex nodeId = createNode();
    BVHNode<IdType>& node = nodes[nodeId];

    node.aabb = aabb.fattened(fatAABBMargin);
    node.id = id;
    node.height = 0;

    insertLeaf(nodeId);
    return nodeId;
}

template<typename IdType>
void DynamicBVH<IdType>::destroyProxy(NodeIndex leafIndex)
{
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
bool DynamicBVH<IdType>::moveProxy(NodeIndex nodeIndex, AABB newAABB, Vec2 displacement)
{
    // If the current fattened AABB contains the new AABB, no need to update the tree
    if (nodes[nodeIndex].aabb.contains(newAABB))
        return false;

    // Otherwise, remove and re-insert with a newly fattened AABB
    removeLeaf(nodeIndex);
    nodes[nodeIndex].aabb = newAABB.fattened(fatAABBMargin, displacement);
    insertLeaf(nodeIndex);
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

    NodeIndex siblingIndex = findBestSiblingIndex(leaf);
    NodeIndex oldParentIndex = nodes[siblingIndex].parentIndex;

    NodeIndex newParentIndex = createNode();
    nodes[newParentIndex].parentIndex = oldParentIndex;
    nodes[newParentIndex].aabb = AABB::combine(leaf, nodes[siblingIndex].aabb);
    nodes[newParentIndex].height = nodes[siblingIndex].height + 1;

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

        NodeIndex child1Index = nodes[currentNodeIndex].child1Index;
        NodeIndex child2Index = nodes[currentNodeIndex].child2Index;

        nodes[currentNodeIndex].aabb = AABB::combine(nodes[child1Index].aabb, nodes[child2Index].aabb);
        nodes[currentNodeIndex].height = 1 + std::max(nodes[child1Index].height, nodes[child2Index].height);

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

    NodeIndex parentIndex = nodes[leafIndex].parentIndex;
    NodeIndex grandParentIndex = nodes[parentIndex].parentIndex;
    NodeIndex siblingIndex = (nodes[parentIndex].child1Index == leafIndex)
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

        NodeIndex child1 = nodes[currentNodeIndex].child1Index;
        NodeIndex child2 = nodes[currentNodeIndex].child2Index;

        nodes[currentNodeIndex].aabb = AABB::combine(nodes[child1].aabb, nodes[child2].aabb);
        nodes[currentNodeIndex].height = 1 + std::max(nodes[child1].height, nodes[child2].height);

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
    if (node.isLeaf() || node.height < 2)
        return index;

    const NodeIndex child1Index = node.child1Index;
    const NodeIndex child2Index = node.child2Index;
    BVHNode<IdType>& child1 = nodes[child1Index];
    BVHNode<IdType>& child2 = nodes[child2Index];

    const NodeIndex balance = child2.height - child1.height;

    NodeIndex upIndex;
    NodeIndex sideIndex1 = INVALID_NODE_INDEX;
    NodeIndex sideIndex2 = INVALID_NODE_INDEX;
    bool isRightRotation;
    if (balance > 1)
    {
        // Rotate C up
        sideIndex1 = child2.child1Index;
        sideIndex2 = child2.child2Index;
        upIndex = child2Index;
        isRightRotation = true;
    }
    else if (balance < -1)
    {
        // Rotate B up
        sideIndex1 = child1.child1Index;
        sideIndex2 = child1.child2Index;
        upIndex = child1Index;
        isRightRotation = false;
    }
    else
    {
        // No rotation needed
        return index;
    }

    BVHNode<IdType>& up = nodes[upIndex];
    BVHNode<IdType>& side1 = nodes[sideIndex1];
    BVHNode<IdType>& side2 = nodes[sideIndex2];

    // Swap node and up
    up.child1Index = index;
    up.parentIndex = node.parentIndex;
    node.parentIndex = upIndex;

    if (up.parentIndex != INVALID_NODE_INDEX)
    {
        BVHNode<IdType>& parent = nodes[up.parentIndex];
        if (parent.child1Index == index)
            parent.child1Index = upIndex;
        else
            parent.child2Index = upIndex;
    }
    else
    {
        rootIndex = upIndex;
    }

    // Pick the taller child for attachment
    NodeIndex attachIndex = (side1.height > side2.height) ? sideIndex1 : sideIndex2;
    NodeIndex remainIndex = (side1.height > side2.height) ? sideIndex2 : sideIndex1;

    if (isRightRotation)
    {
        up.child2Index = attachIndex;
        node.child2Index = remainIndex;
    }
    else
    {
        up.child2Index = attachIndex;
        node.child1Index = remainIndex;
    }

    nodes[attachIndex].parentIndex = upIndex;
    nodes[remainIndex].parentIndex = index;

    // Recompute AABBs and heights
    if (isRightRotation)
    {
        node.aabb = AABB::combine(child1.aabb, nodes[remainIndex].aabb);
        up.aabb = AABB::combine(node.aabb, nodes[attachIndex].aabb);

        node.height = 1 + std::max(child1.height, nodes[remainIndex].height);
        up.height = 1 + std::max(node.height, nodes[attachIndex].height);
    }
    else
    {
        node.aabb = AABB::combine(child2.aabb, nodes[remainIndex].aabb);
        up.aabb = AABB::combine(node.aabb, nodes[attachIndex].aabb);

        node.height = 1 + std::max(child2.height, nodes[remainIndex].height);
        up.height = 1 + std::max(node.height, nodes[attachIndex].height);
    }

    return upIndex;
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
std::vector<IdType> DynamicBVH<IdType>::query(AABB queryAABB) const
{
    if (rootIndex == INVALID_NODE_INDEX)
        return {};

    std::vector<IdType> intersections;
    std::stack<NodeIndex> stack;
    stack.push(rootIndex);

    while (!stack.empty())
    {
        NodeIndex nodeIndex = stack.top();
        stack.pop();
        const auto& node = nodes[nodeIndex];

        // If the node's AABB does not intersect with the query AABB
        // Skip the whole subtree (since it is contained in the node's AABB)
        if (!node.aabb.intersects(queryAABB))
            continue;

        if (node.isLeaf())
        {
            intersections.push_back(node.id);
        }
        else
        {
            stack.push(node.child1Index);
            stack.push(node.child2Index);
        }
    }

    return intersections;
}

template<typename IdType>
std::vector<std::vector<IdType>> DynamicBVH<IdType>::batchQuery(const std::vector<AABB>& queries,
                                                                size_t numThreads) const
{
    size_t n = queries.size();
    std::vector<std::vector<IdType>> results(n);
    std::vector<std::future<void>> futures;

    size_t chunk = (n + numThreads - 1) / numThreads;

    for (size_t t = 0; t < numThreads; ++t) {
        size_t begin = t * chunk;
        size_t end = std::min(n, begin + chunk);

        futures.push_back(std::async(std::launch::async, [this, &queries, &results, begin, end]() {
            for (size_t i = begin; i < end; ++i)
                results[i] = this->query(queries[i]);
        }));
    }

    for (auto& f : futures)
        f.get();

    return results;
}

template<typename IdType>
std::vector<IdType> DynamicBVH<IdType>::piercingRaycast(Ray ray) const
{
    if (rootIndex == INVALID_NODE_INDEX)
        return {};

    std::vector<IdType> hits;
    std::stack<NodeIndex> stack;
    stack.push(rootIndex);

    while (!stack.empty())
    {
        NodeIndex nodeIndex = stack.top();
        stack.pop();
        const auto& node = nodes[nodeIndex];

        if (!node.aabb.intersects(ray))
            continue;

        if (node.isLeaf())
        {
            hits.push_back(node.id);
        }
        else
        {
            stack.push(node.child1Index);
            stack.push(node.child2Index);
        }
    }

    return hits;
}

template<typename IdType>
std::optional<IdType> DynamicBVH<IdType>::firstHitRaycast(Ray ray) const
{
    if (rootIndex == INVALID_NODE_INDEX)
        return std::nullopt;

    std::optional<IdType> closestId;
    float firstHitTime = std::numeric_limits<float>::max();

    std::stack<NodeIndex> stack;
    stack.push(rootIndex);

    while (!stack.empty())
    {
        NodeIndex nodeIndex = stack.top();
        stack.pop();
        const auto& node = nodes[nodeIndex];

        const auto intersection = node.aabb.intersects(ray);
        if (!intersection)
            continue;

        if (node.isLeaf())
        {
            if (intersection->first < firstHitTime)
            {
                firstHitTime = intersection->first;
                closestId = node.id;
            }
        }
        else
        {
            stack.push(node.child1Index);
            stack.push(node.child2Index);
        }
    }

    return closestId;
}

template<typename IdType>
std::vector<typename DynamicBVH<IdType>::RaycastInfo> DynamicBVH<IdType>::piercingRaycastDetailed(Ray ray) const
{
    if (rootIndex == INVALID_NODE_INDEX)
        return {};

    std::vector<RaycastInfo> hits;
    std::stack<NodeIndex> stack;
    stack.push(rootIndex);

    while (!stack.empty())
    {
        NodeIndex nodeIndex = stack.top();
        stack.pop();
        const auto& node = nodes[nodeIndex];

        const auto intersection = node.aabb.intersects(ray);
        if (!intersection)
            continue;

        if (node.isLeaf())
        {
            hits.push_back(RaycastInfo::fromRay(node.id, ray, *intersection));
        }
        else
        {
            stack.push(node.child1Index);
            stack.push(node.child2Index);
        }
    }

    std::sort(hits.begin(), hits.end(), [&](const RaycastInfo& a, const RaycastInfo& b)
    {
        return (a.entry - ray.p1).lengthSqr() < (b.entry - ray.p1).lengthSqr();
    });

    return hits;
}

template<typename IdType>
std::optional<typename DynamicBVH<IdType>::RaycastInfo> DynamicBVH<IdType>::firstHitRaycastDetailed(Ray ray) const
{
    if (rootIndex == INVALID_NODE_INDEX)
        return std::nullopt;

    std::optional<RaycastInfo> firstHit;
    float firstHitTime = std::numeric_limits<float>::max();

    std::stack<NodeIndex> stack;
    stack.push(rootIndex);

    while (!stack.empty())
    {
        NodeIndex nodeIndex = stack.top();
        stack.pop();
        const auto& node = nodes[nodeIndex];

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
            stack.push(node.child1Index);
            stack.push(node.child2Index);
        }
    }

    return firstHit;
}

template<typename IdType>
std::vector<IdType> DynamicBVH<IdType>::piercingRaycast(InfiniteRay ray) const
{
    if (rootIndex == INVALID_NODE_INDEX)
        return {};

    std::vector<IdType> hits;
    std::stack<NodeIndex> stack;
    stack.push(rootIndex);

    while (!stack.empty())
    {
        NodeIndex nodeIndex = stack.top();
        stack.pop();
        const auto& node = nodes[nodeIndex];

        if (!node.aabb.intersects(ray))
            continue;

        if (node.isLeaf())
        {
            hits.push_back(node.id);
        }
        else
        {
            stack.push(node.child1Index);
            stack.push(node.child2Index);
        }
    }

    return hits;
}

template<typename IdType>
std::optional<IdType> DynamicBVH<IdType>::firstHitRaycast(InfiniteRay ray) const
{
    if (rootIndex == INVALID_NODE_INDEX)
        return std::nullopt;

    std::optional<IdType> closestId;
    float firstHitTime = std::numeric_limits<float>::max();

    std::stack<NodeIndex> stack;
    stack.push(rootIndex);

    while (!stack.empty())
    {
        NodeIndex nodeIndex = stack.top();
        stack.pop();
        const auto& node = nodes[nodeIndex];

        const auto intersection = node.aabb.intersects(ray);
        if (!intersection)
            continue;

        if (node.isLeaf())
        {
            if (intersection->first < firstHitTime)
            {
                firstHitTime = intersection->first;
                closestId = node.id;
            }
        }
        else
        {
            stack.push(node.child1Index);
            stack.push(node.child2Index);
        }
    }

    return closestId;
}

template<typename IdType>
std::vector<typename DynamicBVH<IdType>::RaycastInfo> DynamicBVH<IdType>::piercingRaycastDetailed(InfiniteRay ray) const
{
    if (rootIndex == INVALID_NODE_INDEX)
        return {};

    std::vector<RaycastInfo> hits;
    std::stack<NodeIndex> stack;
    stack.push(rootIndex);

    while (!stack.empty())
    {
        NodeIndex nodeIndex = stack.top();
        stack.pop();
        const auto& node = nodes[nodeIndex];

        const auto intersection = node.aabb.intersects(ray);
        if (!intersection)
            continue;

        if (node.isLeaf())
        {
            hits.push_back(RaycastInfo::fromRay(node.id, ray, *intersection));
        }
        else
        {
            stack.push(node.child1Index);
            stack.push(node.child2Index);
        }
    }

    std::sort(hits.begin(), hits.end(), [&](const RaycastInfo& a, const RaycastInfo& b)
    {
        return (a.entry - ray.start).lengthSqr() < (b.entry - ray.start).lengthSqr();
    });

    return hits;
}

template<typename IdType>
std::optional<typename DynamicBVH<IdType>::RaycastInfo> DynamicBVH<IdType>::firstHitRaycastDetailed(InfiniteRay ray) const
{
    if (rootIndex == INVALID_NODE_INDEX)
        return std::nullopt;

    std::optional<RaycastInfo> firstHit;
    float firstHitTime = std::numeric_limits<float>::max();

    std::stack<NodeIndex> stack;
    stack.push(rootIndex);

    while (!stack.empty())
    {
        NodeIndex nodeIndex = stack.top();
        stack.pop();
        const auto& node = nodes[nodeIndex];

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
            stack.push(node.child1Index);
            stack.push(node.child2Index);
        }
    }

    return firstHit;
}

template<typename IdType>
std::vector<std::pair<IdType, IdType>> DynamicBVH<IdType>::findBroadPhaseCollisions() const
{
    if (rootIndex == INVALID_NODE_INDEX)
        return {};

    std::vector<std::pair<IdType, IdType>> pairs;
    findBroadPhaseCollisionsRecursive(rootIndex, pairs);
    return pairs;
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
                                          std::vector<std::pair<IdType, IdType>>& out) const
{
    if (nodeAIdx == INVALID_NODE_INDEX || nodeBIdx == INVALID_NODE_INDEX || nodeAIdx == nodeBIdx)
        return;

    const auto& nodeA = nodes[nodeAIdx];
    const auto& nodeB = nodes[nodeBIdx];

    if (!nodeA.aabb.intersects(nodeB.aabb))
        return;

    if (nodeA.isLeaf() && nodeB.isLeaf())
    {
        // Always store (min, max) to avoid (A, B) and (B, A) being different
        if (nodeA.id < nodeB.id)
            out.emplace_back(nodeA.id, nodeB.id);
        else
            out.emplace_back(nodeB.id, nodeA.id);
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
void DynamicBVH<IdType>::findBroadPhaseCollisionsRecursive(NodeIndex nodeIdx,
                                                           std::vector<std::pair<IdType, IdType>>& out) const
{
    if (nodeIdx == INVALID_NODE_INDEX)
        return;

    const auto& node = nodes[nodeIdx];
    if (node.isLeaf())
        return;

    // Check all pairs between children
    findPairsBetween(node.child1Index, node.child2Index, out);

    // Check pairs within each child
    findBroadPhaseCollisionsRecursive(node.child1Index, out);
    findBroadPhaseCollisionsRecursive(node.child2Index, out);
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
