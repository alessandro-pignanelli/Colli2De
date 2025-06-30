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

    // Update the leaf AABB in place and propagate the change up the tree.
    // This avoids expensive remove/insert operations for large displacements.
    const AABB fatAabb = newAABB.fattened(fatAABBMargin, displacement);
    nodes[nodeIndex].aabb = fatAabb;

    NodeIndex currentIndex = nodes[nodeIndex].parentIndex;
    while (currentIndex != INVALID_NODE_INDEX)
    {
        NodeIndex child1 = nodes[currentIndex].child1Index;
        NodeIndex child2 = nodes[currentIndex].child2Index;

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

}
