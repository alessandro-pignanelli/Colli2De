#pragma once

#include <colli2de/Ray.hpp>
#include <colli2de/internal/geometry/AABB.hpp>
#include <colli2de/internal/utils/Debug.hpp>
#include <colli2de/internal/utils/Methods.hpp>
#include <colli2de/internal/utils/Serialization.hpp>

#include <algorithm>
#include <cstdint>
#include <functional>
#include <ranges>
#include <set>
#include <utility>
#include <vector>

namespace
{
constexpr float displacementFactor = 2.2f; // Factor to adjust the displacement for fattening AABBs
} // namespace

namespace c2d
{

using BitMaskType = uint64_t;
using NodeIndex = int32_t;
static constexpr NodeIndex INVALID_NODE_INDEX = -99;

template <typename IdType>
struct BVHNode
{
    AABB aabb;
    BitMaskType categoryBits = 1;      // for collision filtering
    BitMaskType isHittingBits = ~0ull; // for collision filtering

    NodeIndex parentIndex = INVALID_NODE_INDEX;
    NodeIndex child1Index = INVALID_NODE_INDEX;
    NodeIndex child2Index = INVALID_NODE_INDEX;

    int32_t height = -1;

    IdType id{}; // stored payload (instead of void*)

    bool isLeaf() const
    {
        return child1Index == -1;
    }

    bool matchesMask(BitMaskType mask) const
    {
        return (categoryBits & mask) != 0;
    }

    void serialize(std::ostream& out) const;
    static BVHNode deserialize(std::istream& in);

    bool operator==(const BVHNode& other) const;

    bool operator!=(const BVHNode& other) const
    {
        return !(*this == other);
    }
};

template <typename IdType>
class DynamicBVH
{
  public:
    class LeafConstIterator;
    static constexpr uint32_t initialCapacity = 64;
    using RaycastInfo = RaycastHit<IdType>;
    using const_iterator = LeafConstIterator;
    using value_type = BVHNode<IdType>;

    DynamicBVH(float fatAABBMargin = 0.0f);

    NodeIndex createNode();
    void destroyNode(NodeIndex nodeId);

    // Inserts a new object {aabb, id} into the BVH and returns the index of the created node
    NodeIndex addProxy(IdType id, AABB aabb, BitMaskType categoryBits = 1, BitMaskType isHittingBits = ~0ull);
    void removeProxy(NodeIndex leafIndex);
    // Moves the proxy to a new AABB, returns true if the tree structure has changed
    bool moveProxy(NodeIndex nodeIndex, AABB aabb);

    void clear();

    float getFatAABBMargin() const
    {
        return fatAABBMargin;
    }

    size_t size() const
    {
        return nodeCount;
    }

    size_t capacity() const
    {
        return nodes.size();
    }

    size_t proxies() const
    {
        return proxyCount;
    }

    NodeIndex getRootIndex() const
    {
        return rootIndex;
    }

    const BVHNode<IdType>& getNode(NodeIndex index) const
    {
        return nodes[index];
    }

    std::vector<std::pair<IdType, AABB>> data() const;
    auto leavesView() const;

    // AABB queries
    void query(AABB queryAABB, std::vector<IdType>& intersections, BitMaskType maskBits = ~0ull) const;
    void batchQuery(const std::vector<std::pair<AABB, BitMaskType>>& queries,
                    const std::function<void(size_t, std::vector<IdType>)>& callback) const;
    void findAllCollisions(const std::function<void(IdType, IdType)>& callback) const;
    void findAllCollisions(const DynamicBVH<IdType>& other, const std::function<void(IdType, IdType)>& callback) const;

    // Finite raycast queries
    std::optional<RaycastInfo> firstHitRaycast(Ray ray, BitMaskType maskBits = ~0ull) const;
    void piercingRaycast(Ray ray, std::set<RaycastInfo>& hits, BitMaskType maskBits = ~0ull) const;

    // Infinite raycast queries
    std::optional<RaycastInfo> firstHitRaycast(InfiniteRay ray, BitMaskType maskBits = ~0ull) const;
    void piercingRaycast(InfiniteRay ray, std::set<RaycastInfo>& hits, BitMaskType maskBits = ~0ull) const;

    void serialize(std::ostream& out) const;
    static DynamicBVH deserialize(std::istream& in);

    bool operator==(const DynamicBVH& other) const;

    bool operator!=(const DynamicBVH& other) const
    {
        return !(*this == other);
    }

  private:
    std::vector<BVHNode<IdType>> nodes;
    float fatAABBMargin = 0.0f;
    uint32_t nodeCount = 0;
    uint32_t proxyCount = 0;

    NodeIndex rootIndex = INVALID_NODE_INDEX;
    NodeIndex nextAvailableIndex = INVALID_NODE_INDEX;

    void doubleCapacity();
    void allocateNodes(int32_t capacity);

    void insertLeaf(NodeIndex leaf);
    void removeLeaf(NodeIndex leafIndex);
    void findPairsBetween(NodeIndex nodeAIdx,
                          NodeIndex nodeBIdx,
                          const std::function<void(IdType, IdType)>& callback) const;
    void findPairsBetween(const DynamicBVH<IdType>& other,
                          NodeIndex nodeAIdx,
                          NodeIndex nodeBIdx,
                          const std::function<void(IdType, IdType)>& callback) const;
    void findAllCollisionsRecursive(NodeIndex nodeIdx, const std::function<void(IdType, IdType)>& callback) const;

    NodeIndex findBestSiblingIndex(AABB leaf) const;
    NodeIndex balance(NodeIndex index);

  public:
    class LeafConstIterator
    {
      public:
        using iterator_category = std::forward_iterator_tag;
        using value_type = std::pair<IdType, AABB>;
        using difference_type = std::ptrdiff_t;
        using pointer = const std::pair<IdType, AABB>*;
        using reference = const std::pair<IdType, AABB>&;

        LeafConstIterator(const std::vector<BVHNode<IdType>>& nodes, uint32_t nodeIndex)
            : nodes(nodes),
              m_nodeIndex(nodeIndex)
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

        bool operator==(const LeafConstIterator& other) const
        {
            return m_nodeIndex == other.m_nodeIndex;
        }

        bool operator!=(const LeafConstIterator& other) const
        {
            return !(*this == other);
        }

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

    LeafConstIterator begin() const
    {
        return LeafConstIterator(nodes, 0);
    }

    LeafConstIterator end() const
    {
        return LeafConstIterator(nodes, nodes.size());
    }

    LeafConstIterator cbegin() const
    {
        return LeafConstIterator(nodes, 0);
    }

    LeafConstIterator cend() const
    {
        return LeafConstIterator(nodes, nodes.size());
    }
};

static_assert(std::is_trivially_copyable_v<BVHNode<uint32_t>>, "BVHNode must be trivially copyable");

template <typename IdType>
DynamicBVH<IdType>::DynamicBVH(float fatAABBMargin) : fatAABBMargin(fatAABBMargin)
{
    allocateNodes(initialCapacity);
}

template <typename IdType>
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

template <typename IdType>
void DynamicBVH<IdType>::doubleCapacity()
{
    DEBUG_ASSERT(nextAvailableIndex == INVALID_NODE_INDEX);

    const size_t currentCapacity = capacity();
    DEBUG_ASSERT(nodeCount == static_cast<uint32_t>(currentCapacity));

    const size_t newCapacity = currentCapacity <= 4096 ? currentCapacity << 1 : currentCapacity << 1;
    nodes.resize(newCapacity);

    for (NodeIndex i = currentCapacity - 1; i < static_cast<int32_t>(newCapacity) - 1; ++i)
        nodes[i].parentIndex = i + 1;
    nodes[newCapacity - 1].parentIndex = INVALID_NODE_INDEX;

    nextAvailableIndex = currentCapacity;
}

template <typename IdType>
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

template <typename IdType>
void DynamicBVH<IdType>::destroyNode(NodeIndex nodeId)
{
    DEBUG_ASSERT(0 <= nodeId && nodeId < static_cast<NodeIndex>(nodes.size()));
    BVHNode<IdType>& node = nodes[nodeId];

    node.parentIndex = nextAvailableIndex;
    node.height = -1;
    node.child1Index = INVALID_NODE_INDEX;
    node.child2Index = INVALID_NODE_INDEX;
    node.id = IdType{};

    nextAvailableIndex = nodeId;
    --nodeCount;
}

template <typename IdType>
NodeIndex DynamicBVH<IdType>::addProxy(IdType id, AABB aabb, BitMaskType categoryBits, BitMaskType isHittingBits)
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

template <typename IdType>
void DynamicBVH<IdType>::removeProxy(NodeIndex leafIndex)
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

template <typename IdType>
bool DynamicBVH<IdType>::moveProxy(NodeIndex nodeIndex, AABB newAABB)
{
    auto& node = nodes[nodeIndex];

    // If the current fattened AABB contains the new AABB, no need to update the
    // tree
    if (node.aabb.contains(newAABB))
        return false;

    // Update the leaf AABB in place and propagate the change up the tree.
    // This avoids expensive remove/insert operations for large displacements.
    const Vec2 displacement = Vec2((newAABB.min.x - node.aabb.min.x) * displacementFactor,
                                   (newAABB.min.y - node.aabb.min.y) * displacementFactor);
    const AABB fatAabb = newAABB.fattened(fatAABBMargin, displacement);
    node.aabb = fatAabb;

    NodeIndex currentIndex = node.parentIndex;
    while (currentIndex != INVALID_NODE_INDEX)
    {
        auto& parent = nodes[currentIndex];
        const NodeIndex child1 = parent.child1Index;
        const NodeIndex child2 = parent.child2Index;

        const AABB newParentAABB = AABB::combine(nodes[child1].aabb, nodes[child2].aabb);
        if (newParentAABB == parent.aabb)
            break;

        parent.aabb = newParentAABB;
        currentIndex = parent.parentIndex;
    }
    return true;
}

template <typename IdType>
void DynamicBVH<IdType>::insertLeaf(NodeIndex leafIndex)
{
    if (rootIndex == INVALID_NODE_INDEX)
    {
        rootIndex = leafIndex;
        nodes[rootIndex].parentIndex = INVALID_NODE_INDEX;
        return;
    }

    const NodeIndex siblingIndex = findBestSiblingIndex(nodes[leafIndex].aabb);
    const NodeIndex oldParentIndex = nodes[siblingIndex].parentIndex;
    const NodeIndex newParentIndex = createNode();

    auto& leaf = nodes[leafIndex];
    auto& sibling = nodes[siblingIndex];
    auto& newParent = nodes[newParentIndex];

    newParent.parentIndex = oldParentIndex;
    newParent.aabb = AABB::combine(leaf.aabb, sibling.aabb);
    newParent.height = sibling.height + 1;
    newParent.categoryBits = sibling.categoryBits | leaf.categoryBits;

    if (oldParentIndex != INVALID_NODE_INDEX)
    {
        auto& oldParent = nodes[oldParentIndex];
        if (oldParent.child1Index == siblingIndex)
            oldParent.child1Index = newParentIndex;
        else
            oldParent.child2Index = newParentIndex;

        newParent.child1Index = siblingIndex;
        newParent.child2Index = leafIndex;
        sibling.parentIndex = newParentIndex;
        leaf.parentIndex = newParentIndex;
    }
    else
    {
        newParent.child1Index = siblingIndex;
        newParent.child2Index = leafIndex;
        sibling.parentIndex = newParentIndex;
        leaf.parentIndex = newParentIndex;
        rootIndex = newParentIndex;
    }

    // Walk up the tree fixing heights and AABBs
    NodeIndex currentNodeIndex = leaf.parentIndex;
    while (currentNodeIndex != INVALID_NODE_INDEX)
    {
        currentNodeIndex = balance(currentNodeIndex);
        auto& currentNode = nodes[currentNodeIndex];

        const NodeIndex child1Index = currentNode.child1Index;
        const NodeIndex child2Index = currentNode.child2Index;
        auto& child1 = nodes[child1Index];
        auto& child2 = nodes[child2Index];

        currentNode.aabb = AABB::combine(child1.aabb, child2.aabb);
        currentNode.height = 1 + std::max(child1.height, child2.height);
        currentNode.categoryBits = child1.categoryBits | child2.categoryBits;

        currentNodeIndex = currentNode.parentIndex;
    }
}

template <typename IdType>
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
    const NodeIndex siblingIndex =
        (nodes[parentIndex].child1Index == leafIndex) ? nodes[parentIndex].child2Index : nodes[parentIndex].child1Index;

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
        auto& currentNode = nodes[currentNodeIndex];

        const NodeIndex child1Index = currentNode.child1Index;
        const NodeIndex child2Index = currentNode.child2Index;
        auto& child1 = nodes[child1Index];
        auto& child2 = nodes[child2Index];

        currentNode.aabb = AABB::combine(child1.aabb, child2.aabb);
        currentNode.height = 1 + std::max(child1.height, child2.height);
        currentNode.categoryBits = child1.categoryBits | child2.categoryBits;

        currentNodeIndex = currentNode.parentIndex;
    }
}

template <typename IdType>
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

template <typename IdType>
NodeIndex DynamicBVH<IdType>::balance(NodeIndex index)
{
    BVHNode<IdType>& node = nodes[index];

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

template <typename IdType>
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

template <typename IdType>
auto DynamicBVH<IdType>::leavesView() const
{
    return nodes | std::views::filter([](const auto& node) { return node.isLeaf(); }) |
           std::views::transform([](const auto& node) { return std::pair<IdType, AABB>{node.id, node.aabb}; });
}

template <typename IdType>
void DynamicBVH<IdType>::query(AABB queryAABB, std::vector<IdType>& intersections, BitMaskType maskBits) const
{
    if (rootIndex == INVALID_NODE_INDEX)
        return;

    std::vector<NodeIndex> stack = {rootIndex};

    while (!stack.empty())
    {
        const NodeIndex nodeIndex = stack.back();
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
            intersections.push_back(node.id);
        }
        else
        {
            stack.push_back(node.child1Index);
            stack.push_back(node.child2Index);
        }
    }
}

template <typename IdType>
void DynamicBVH<IdType>::batchQuery(const std::vector<std::pair<AABB, BitMaskType>>& queries,
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
void DynamicBVH<IdType>::piercingRaycast(Ray ray, std::set<RaycastInfo>& hits, BitMaskType maskBits) const
{
    if (rootIndex == INVALID_NODE_INDEX)
        return;

    std::vector<NodeIndex> stack = {rootIndex};

    while (!stack.empty())
    {
        const NodeIndex nodeIndex = stack.back();
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

template <typename IdType>
std::optional<typename DynamicBVH<IdType>::RaycastInfo> DynamicBVH<IdType>::firstHitRaycast(Ray ray,
                                                                                            BitMaskType maskBits) const
{
    if (rootIndex == INVALID_NODE_INDEX)
        return std::nullopt;

    std::optional<RaycastInfo> firstHit;
    float firstHitTime = std::numeric_limits<float>::max();

    std::vector<NodeIndex> stack = {rootIndex};

    while (!stack.empty())
    {
        const NodeIndex nodeIndex = stack.back();
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

template <typename IdType>
void DynamicBVH<IdType>::piercingRaycast(InfiniteRay ray, std::set<RaycastInfo>& hits, BitMaskType maskBits) const
{
    if (rootIndex == INVALID_NODE_INDEX)
        return;

    std::vector<NodeIndex> stack = {rootIndex};

    while (!stack.empty())
    {
        const NodeIndex nodeIndex = stack.back();
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

template <typename IdType>
std::optional<typename DynamicBVH<IdType>::RaycastInfo> DynamicBVH<IdType>::firstHitRaycast(InfiniteRay ray,
                                                                                            BitMaskType maskBits) const
{
    if (rootIndex == INVALID_NODE_INDEX)
        return std::nullopt;

    std::optional<RaycastInfo> firstHit;
    float firstHitTime = std::numeric_limits<float>::max();

    std::vector<NodeIndex> stack = {rootIndex};

    while (!stack.empty())
    {
        const NodeIndex nodeIndex = stack.back();
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

template <typename IdType>
void DynamicBVH<IdType>::findAllCollisions(const std::function<void(IdType, IdType)>& callback) const
{
    findAllCollisionsRecursive(rootIndex, callback);
}

template <typename IdType>
void DynamicBVH<IdType>::findAllCollisions(const DynamicBVH<IdType>& other,
                                           const std::function<void(IdType, IdType)>& callback) const
{
    findPairsBetween(other, rootIndex, other.rootIndex, callback);
}

template <typename IdType>
void DynamicBVH<IdType>::findPairsBetween(NodeIndex nodeAIdx,
                                          NodeIndex nodeBIdx,
                                          const std::function<void(IdType, IdType)>& callback) const
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
        callback(nodeA.id, nodeB.id);
        return;
    }

    // Expand the node with greater height (heuristic)
    if (nodeA.isLeaf() || (!nodeB.isLeaf() && nodeB.height > nodeA.height))
    {
        findPairsBetween(nodeAIdx, nodeB.child1Index, callback);
        findPairsBetween(nodeAIdx, nodeB.child2Index, callback);
    }
    else
    {
        findPairsBetween(nodeA.child1Index, nodeBIdx, callback);
        findPairsBetween(nodeA.child2Index, nodeBIdx, callback);
    }
}

template <typename IdType>
void DynamicBVH<IdType>::findPairsBetween(const DynamicBVH<IdType>& other,
                                          NodeIndex nodeAIdx,
                                          NodeIndex nodeBIdx,
                                          const std::function<void(IdType, IdType)>& callback) const
{
    if (nodeAIdx == INVALID_NODE_INDEX || nodeBIdx == INVALID_NODE_INDEX)
        return;

    const auto& nodeA = nodes[nodeAIdx];
    const auto& nodeB = other.nodes[nodeBIdx];

    if (!nodeA.matchesMask(nodeB.categoryBits) || !nodeB.matchesMask(nodeA.categoryBits))
        return;
    if (!nodeA.aabb.intersects(nodeB.aabb))
        return;

    if (nodeA.isLeaf() && nodeB.isLeaf())
    {
        callback(nodeA.id, nodeB.id);
        return;
    }

    if (nodeA.isLeaf() || (!nodeB.isLeaf() && nodeB.height > nodeA.height))
    {
        findPairsBetween(other, nodeAIdx, nodeB.child1Index, callback);
        findPairsBetween(other, nodeAIdx, nodeB.child2Index, callback);
    }
    else
    {
        findPairsBetween(other, nodeA.child1Index, nodeBIdx, callback);
        findPairsBetween(other, nodeA.child2Index, nodeBIdx, callback);
    }
}

template <typename IdType>
void DynamicBVH<IdType>::findAllCollisionsRecursive(NodeIndex nodeIdx,
                                                    const std::function<void(IdType, IdType)>& callback) const
{
    if (nodeIdx == INVALID_NODE_INDEX)
        return;

    const auto& node = nodes[nodeIdx];
    if (node.isLeaf())
        return;

    // Check all pairs between children
    findPairsBetween(node.child1Index, node.child2Index, callback);

    // Check pairs within each child
    findAllCollisionsRecursive(node.child1Index, callback);
    findAllCollisionsRecursive(node.child2Index, callback);
}

template <typename IdType>
void DynamicBVH<IdType>::serialize(std::ostream& out) const
{
    Writer writer(out);

    writer(fatAABBMargin);
    writer(rootIndex);
    writer(nodeCount);
    writer(proxyCount);
    writer(nextAvailableIndex);

    // Write nodes vector size
    const size_t nodesSize = nodes.size();
    writer(nodesSize);

    // Write nodes
    for (const auto& node : nodes)
        node.serialize(out);
}

template <typename IdType>
DynamicBVH<IdType> DynamicBVH<IdType>::deserialize(std::istream& in)
{
    Reader reader(in);

    float fatAABBMargin;
    reader(fatAABBMargin);

    DynamicBVH<IdType> bvh(fatAABBMargin);

    reader(bvh.rootIndex);
    reader(bvh.nodeCount);
    reader(bvh.proxyCount);
    reader(bvh.nextAvailableIndex);

    size_t nodesSize;
    reader(nodesSize);

    // Replace the preallocated storage created by the constructor
    bvh.nodes.clear();
    bvh.nodes.reserve(nodesSize);
    for (size_t i = 0; i < nodesSize; ++i)
        bvh.nodes.emplace_back(BVHNode<IdType>::deserialize(in));

    return bvh;
}

template <typename IdType>
bool DynamicBVH<IdType>::operator==(const DynamicBVH& other) const
{
    if (!float_equals(fatAABBMargin, other.fatAABBMargin))
        return false;
    if (nodeCount != other.nodeCount)
        return false;
    if (proxyCount != other.proxyCount)
        return false;
    if (rootIndex != other.rootIndex)
        return false;
    if (nextAvailableIndex != other.nextAvailableIndex)
        return false;
    if (nodes != other.nodes)
        return false;

    return true;
}

template <typename IdType>
void BVHNode<IdType>::serialize(std::ostream& out) const
{
    Writer writer(out);

    writer(aabb.min.x);
    writer(aabb.min.y);
    writer(aabb.max.x);
    writer(aabb.max.y);
    writer(categoryBits);
    writer(isHittingBits);
    writer(parentIndex);
    writer(child1Index);
    writer(child2Index);
    writer(height);
    writer(id);
}

template <typename IdType>
BVHNode<IdType> BVHNode<IdType>::deserialize(std::istream& in)
{
    Reader reader(in);
    BVHNode<IdType> node;

    reader(node.aabb.min.x);
    reader(node.aabb.min.y);
    reader(node.aabb.max.x);
    reader(node.aabb.max.y);
    reader(node.categoryBits);
    reader(node.isHittingBits);
    reader(node.parentIndex);
    reader(node.child1Index);
    reader(node.child2Index);
    reader(node.height);
    reader(node.id);

    return node;
}

template <typename IdType>
bool BVHNode<IdType>::operator==(const BVHNode& other) const
{
    if (aabb != other.aabb)
        return false;
    if (categoryBits != other.categoryBits)
        return false;
    if (isHittingBits != other.isHittingBits)
        return false;

    if (parentIndex != other.parentIndex)
        return false;
    if (child1Index != other.child1Index)
        return false;
    if (child2Index != other.child2Index)
        return false;

    if (height != other.height)
        return false;

    if (id != other.id)
        return false;

    return true;
}

} // namespace c2d
