#pragma once

#include <cstdint>
#include <stack>
#include <vector>

#include "geometry/AABB.hpp"

namespace c2d
{

using NodeId = int32_t;
using NodeIndex = int32_t;

template<typename IdType>
struct BVHNode
{
    AABB aabb;

    NodeIndex parentIndex = -1;
    NodeIndex child1Index = -1;
    NodeIndex child2Index = -1;

    NodeIndex height = -1;

    IdType id{}; // stored payload (instead of void*)

    bool isLeaf() const { return child1Index == -1; }
};

template<typename IdType>
class DynamicBVH
{

public:
    static constexpr uint32_t initialCapacity = 16;
    static constexpr float defaultAABBMargin = 3.0f; // Expansion for AABBs

    DynamicBVH(float fatAABBMargin = defaultAABBMargin);

    NodeId createNode();
    void destroyNode(NodeId nodeId);
    // Moves the proxy to a new AABB, returns true if the tree structure has changed
    bool moveProxy(NodeIndex nodeIndex, AABB aabb, Vec2 /*displacement*/);
    
    // Inserts a new object {aabb, id} into the BVH and returns the index of the created node
    NodeId createProxy(AABB aabb, IdType id);
    void destroyProxy(NodeIndex leafIndex);

    uint32_t size() const { return nodeCount; }
    uint32_t capacity() const { return nodes.size(); }
    
    NodeIndex getRootIndex() const { return rootIndex; }
    const BVHNode<IdType>& getNode(NodeIndex index) const { return nodes[index]; }

    template<typename Callback>
    void query(AABB queryAABB, Callback&& callback) const;

private:
    std::vector<BVHNode<IdType>> nodes;
    uint32_t nodeCount = 0;
    const float fatAABBMargin = 3.0f;

    NodeIndex rootIndex = -1;
    NodeIndex nextAvailableIndex = -1;

    void doubleCapacity();
    void allocateNodes(uint32_t capacity);

    void insertLeaf(NodeId leaf);
    void removeLeaf(NodeIndex leafIndex);

    NodeId findBestSiblingIndex(AABB leaf) const;
    NodeIndex balance(NodeIndex index);
};

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
    nodes[capacity - 1].parentIndex = -1;

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
    nodes[newCapacity - 1].parentIndex = -1;
    
    nextAvailableIndex = currentCapacity;
}

template<typename IdType>
NodeId DynamicBVH<IdType>::createNode()
{
    if (nextAvailableIndex == -1)
        doubleCapacity();

    const NodeId nodeId = nextAvailableIndex;
    BVHNode<IdType>& node = nodes[nodeId];

    nextAvailableIndex = node.parentIndex;
    node.parentIndex = -1;
    node.child1Index = -1;
    node.child2Index = -1;
    node.height = 0;
    node.id = IdType{};

    ++nodeCount;
    return nodeId;
}

template<typename IdType>
void DynamicBVH<IdType>::destroyNode(NodeId nodeId)
{
    assert(0 <= nodeId && nodeId < static_cast<NodeId>(nodes.size()));
    BVHNode<IdType>& node = nodes[nodeId];

    node.parentIndex = nextAvailableIndex;
    node.height = -1;
    node.child1Index = -1;
    node.child2Index = -1;
    node.id = IdType{};

    nextAvailableIndex = nodeId;
    --nodeCount;
}

template<typename IdType>
NodeId DynamicBVH<IdType>::createProxy(AABB aabb, IdType id)
{
    const NodeId nodeId = createNode();
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
    if (rootIndex == -1)
    {
        rootIndex = leafIndex;
        nodes[rootIndex].parentIndex = -1;
        return;
    }

    AABB leaf = nodes[leafIndex].aabb;

    NodeIndex siblingIndex = findBestSiblingIndex(leaf);
    NodeIndex oldParentIndex = nodes[siblingIndex].parentIndex;

    NodeIndex newParentIndex = createNode();
    nodes[newParentIndex].parentIndex = oldParentIndex;
    nodes[newParentIndex].aabb = AABB::combine(leaf, nodes[siblingIndex].aabb);
    nodes[newParentIndex].height = nodes[siblingIndex].height + 1;

    if (oldParentIndex != -1)
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
    while (currentNodeIndex != -1)
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
        rootIndex = -1;
        return;
    }

    NodeIndex parentIndex = nodes[leafIndex].parentIndex;
    NodeIndex grandParentIndex = nodes[parentIndex].parentIndex;
    NodeIndex siblingIndex = (nodes[parentIndex].child1Index == leafIndex)
                                 ? nodes[parentIndex].child2Index
                                 : nodes[parentIndex].child1Index;

    // Connect sibling to grandparent
    if (grandParentIndex != -1)
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
        nodes[siblingIndex].parentIndex = -1;
    }

    // Recycle parent node
    destroyNode(parentIndex);

    // Walk up and fix heights and AABBs (including rebalancing)
    NodeIndex currentNodeIndex = grandParentIndex;
    while (currentNodeIndex != -1)
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
    BVHNode<IdType>& A = nodes[index];

    // Can't balance a leaf
    if (A.isLeaf() || A.height < 2)
        return index;

    const NodeIndex child1Index = A.child1Index;
    const NodeIndex child2Index = A.child2Index;
    BVHNode<IdType>& B = nodes[child1Index];
    BVHNode<IdType>& C = nodes[child2Index];

    const NodeIndex balance = C.height - B.height;

    NodeIndex upIndex;
    NodeIndex sideIndex1 = -1;
    NodeIndex sideIndex2 = -1;
    bool isRightRotation;
    if (balance > 1)
    {
        // Rotate C up
        sideIndex1 = C.child1Index;
        sideIndex2 = C.child2Index;
        upIndex = child2Index;
        isRightRotation = true;
    }
    else if (balance < -1)
    {
        // Rotate B up
        sideIndex1 = B.child1Index;
        sideIndex2 = B.child2Index;
        upIndex = child1Index;
        isRightRotation = false;
    }
    else
    {
        // No rotation needed
        return index;
    }

    BVHNode<IdType>& Up = nodes[upIndex];
    BVHNode<IdType>& Side1 = nodes[sideIndex1];
    BVHNode<IdType>& Side2 = nodes[sideIndex2];

    // Swap A and Up
    Up.child1Index = index;
    Up.parentIndex = A.parentIndex;
    A.parentIndex = upIndex;

    if (Up.parentIndex != -1)
    {
        BVHNode<IdType>& parent = nodes[Up.parentIndex];
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
    NodeIndex attachIndex = (Side1.height > Side2.height) ? sideIndex1 : sideIndex2;
    NodeIndex remainIndex = (Side1.height > Side2.height) ? sideIndex2 : sideIndex1;

    if (isRightRotation)
    {
        Up.child2Index = attachIndex;
        A.child2Index = remainIndex;
    }
    else
    {
        Up.child2Index = attachIndex;
        A.child1Index = remainIndex;
    }

    nodes[attachIndex].parentIndex = upIndex;
    nodes[remainIndex].parentIndex = index;

    // Recompute AABBs and heights
    if (isRightRotation)
    {
        A.aabb = AABB::combine(B.aabb, nodes[remainIndex].aabb);
        Up.aabb = AABB::combine(A.aabb, nodes[attachIndex].aabb);

        A.height = 1 + std::max(B.height, nodes[remainIndex].height);
        Up.height = 1 + std::max(A.height, nodes[attachIndex].height);
    }
    else
    {
        A.aabb = AABB::combine(C.aabb, nodes[remainIndex].aabb);
        Up.aabb = AABB::combine(A.aabb, nodes[attachIndex].aabb);

        A.height = 1 + std::max(C.height, nodes[remainIndex].height);
        Up.height = 1 + std::max(A.height, nodes[attachIndex].height);
    }

    return upIndex;
}

template<typename IdType>
template<typename Callback>
void DynamicBVH<IdType>::query(AABB queryAABB, Callback&& callback) const
{
    if (rootIndex == -1)
        return;

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
            // Report the id
            callback(node.id);
        }
        else
        {
            stack.push(node.child1Index);
            stack.push(node.child2Index);
        }
    }
}

static_assert(std::is_trivially_copyable_v<BVHNode<uint32_t>>, "BVHNode must be trivially copyable");

} // namespace c2d
