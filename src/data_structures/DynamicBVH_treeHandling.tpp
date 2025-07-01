#pragma once

#include <algorithm>
#include <future>
#include <ranges>
#include <stack>

namespace c2d
{

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
    assert(nextAvailableIndex == INVALID_NODE_INDEX);

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
bool DynamicBVH<IdType>::moveProxy(NodeIndex nodeIndex, AABB newAABB, Vec2 displacement)
{
    // If the current fattened AABB contains the new AABB, no need to update the tree
    if (nodes[nodeIndex].aabb.contains(newAABB))
        return false;

    // Update the leaf AABB in place and propagate the change up the tree.
    // This avoids expensive remove/insert operations for large displacements.
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

}
