#pragma once

#include <colli2de/Ray.hpp>
#include <colli2de/internal/geometry/AABB.hpp>
#include <colli2de/internal/utils/Debug.hpp>
#include <colli2de/internal/utils/Methods.hpp>
#include <colli2de/internal/utils/Serialization.hpp>

#include <algorithm>
#include <cstdint>
#include <functional>
#include <memory_resource>
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
    AABB mBoundingBox;
    BitMaskType mCategoryBits = 1;      // for collision filtering
    BitMaskType mIsHittingBits = ~0ull; // for collision filtering

    NodeIndex mParentIndex = INVALID_NODE_INDEX;
    NodeIndex mChild1Index = INVALID_NODE_INDEX;
    NodeIndex mChild2Index = INVALID_NODE_INDEX;

    int32_t mHeight = -1;

    IdType mId{}; // stored payload (instead of void*)

    bool isLeaf() const
    {
        return mChild1Index == -1;
    }

    bool matchesMask(BitMaskType mask) const
    {
        return (mCategoryBits & mask) != 0;
    }

    void serialize(std::ostream& out) const;
    static BVHNode deserialize(std::istream& in);

#ifdef C2D_USE_CEREAL
    template <IsCerealArchive Archive>
    void serialize(Archive& archive);
#endif

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

    // Inserts a new object {boundingBox, id} into the BVH and returns the index of the created node
    NodeIndex addProxy(IdType id, AABB boundingBox, BitMaskType categoryBits = 1, BitMaskType isHittingBits = ~0ull);
    void removeProxy(NodeIndex leafIndex);
    // Moves the proxy to a new AABB, returns true if the tree structure has changed
    bool moveProxy(NodeIndex nodeIndex, AABB boundingBox);

    void clear();

    float getFatAABBMargin() const
    {
        return mFatAABBMargin;
    }

    size_t size() const
    {
        return mNodeCount;
    }

    size_t capacity() const
    {
        return mNodes.size();
    }

    size_t proxies() const
    {
        return mProxyCount;
    }

    NodeIndex getRootIndex() const
    {
        return mRootIndex;
    }

    const BVHNode<IdType>& getNode(NodeIndex index) const
    {
        return mNodes[index];
    }

    std::vector<std::pair<IdType, AABB>> data() const;
    auto leavesView() const;

    // AABB queries
    template <typename Allocator>
    void query(AABB queryAABB, std::vector<IdType, Allocator>& intersections, BitMaskType maskBits = ~0ull) const;
    void batchQuery(std::pmr::monotonic_buffer_resource& poolResource,
                    const std::pmr::vector<std::pair<AABB, BitMaskType>>& queries,
                    const std::function<void(size_t, std::pmr::vector<IdType>&)>& callback) const;
    void batchQuery(const std::vector<std::pair<AABB, BitMaskType>>& queries,
                    const std::function<void(size_t, std::vector<IdType>&)>& callback) const;
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

#ifdef C2D_USE_CEREAL
    template <IsCerealArchive Archive>
    void serialize(Archive& archive);
#endif

    bool operator==(const DynamicBVH& other) const;

    bool operator!=(const DynamicBVH& other) const
    {
        return !(*this == other);
    }

  private:
    std::vector<BVHNode<IdType>> mNodes;
    float mFatAABBMargin = 0.0f;
    uint32_t mNodeCount = 0;
    uint32_t mProxyCount = 0;

    NodeIndex mRootIndex = INVALID_NODE_INDEX;
    NodeIndex mNextAvailableIndex = INVALID_NODE_INDEX;

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
            : mNodes(nodes),
              mNodeIndex(nodeIndex)
        {
            if (mNodeIndex < nodes.size() && !nodes[mNodeIndex].isLeaf())
                next();
        }

        value_type operator*() const
        {
            mCache = std::make_pair(mNodes[mNodeIndex].mId, mNodes[mNodeIndex].mBoundingBox);
            return mCache;
        }

        pointer operator->() const
        {
            mCache = std::make_pair(mNodes[mNodeIndex].mId, mNodes[mNodeIndex].mBoundingBox);
            return &mCache;
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
            return mNodeIndex == other.mNodeIndex;
        }

        bool operator!=(const LeafConstIterator& other) const
        {
            return !(*this == other);
        }

      private:
        const std::vector<BVHNode<IdType>>& mNodes;
        NodeIndex mNodeIndex;
        mutable value_type mCache;

        void next()
        {
            ++mNodeIndex;
            while (mNodeIndex < mNodes.size() && !mNodes[mNodeIndex].isLeaf())
                ++mNodeIndex;
        }
    };

    LeafConstIterator begin() const
    {
        return LeafConstIterator(mNodes, 0);
    }

    LeafConstIterator end() const
    {
        return LeafConstIterator(mNodes, mNodes.size());
    }

    LeafConstIterator cbegin() const
    {
        return LeafConstIterator(mNodes, 0);
    }

    LeafConstIterator cend() const
    {
        return LeafConstIterator(mNodes, mNodes.size());
    }
};

static_assert(std::is_trivially_copyable_v<BVHNode<uint32_t>>, "BVHNode must be trivially copyable");

template <typename IdType>
DynamicBVH<IdType>::DynamicBVH(float fatAABBMargin) : mFatAABBMargin(fatAABBMargin)
{
    allocateNodes(initialCapacity);
}

template <typename IdType>
void DynamicBVH<IdType>::allocateNodes(int32_t capacity)
{
    mNodes.resize(capacity);

    // Create a singly-linked free list
    // [0] -> [1] -> [2] -> ... -> [capacity - 1] -> null
    for (NodeIndex i = 0; i < capacity - 1; ++i)
        mNodes[i].mParentIndex = i + 1;
    mNodes[capacity - 1].mParentIndex = INVALID_NODE_INDEX;

    mNextAvailableIndex = 0;
    mNodeCount = 0;
}

template <typename IdType>
void DynamicBVH<IdType>::doubleCapacity()
{
    C2D_DEBUG_ASSERT(mNextAvailableIndex == INVALID_NODE_INDEX);

    const size_t currentCapacity = capacity();
    C2D_DEBUG_ASSERT(mNodeCount == static_cast<uint32_t>(currentCapacity));

    const size_t newCapacity = currentCapacity <= 4096 ? currentCapacity << 1 : currentCapacity << 1;
    mNodes.resize(newCapacity);

    for (NodeIndex i = currentCapacity - 1; i < static_cast<int32_t>(newCapacity) - 1; ++i)
        mNodes[i].mParentIndex = i + 1;
    mNodes[newCapacity - 1].mParentIndex = INVALID_NODE_INDEX;

    mNextAvailableIndex = currentCapacity;
}

template <typename IdType>
NodeIndex DynamicBVH<IdType>::createNode()
{
    if (mNextAvailableIndex == INVALID_NODE_INDEX)
        doubleCapacity();

    const NodeIndex nodeId = mNextAvailableIndex;
    BVHNode<IdType>& node = mNodes[nodeId];

    mNextAvailableIndex = node.mParentIndex;
    node.mParentIndex = INVALID_NODE_INDEX;
    node.mChild1Index = -1;
    node.mChild2Index = -1;
    node.mHeight = 0;
    node.mId = IdType{};

    ++mNodeCount;
    return nodeId;
}

template <typename IdType>
void DynamicBVH<IdType>::destroyNode(NodeIndex nodeId)
{
    C2D_DEBUG_ASSERT(0 <= nodeId && nodeId < static_cast<NodeIndex>(mNodes.size()));
    BVHNode<IdType>& node = mNodes[nodeId];

    node.mParentIndex = mNextAvailableIndex;
    node.mHeight = -1;
    node.mChild1Index = INVALID_NODE_INDEX;
    node.mChild2Index = INVALID_NODE_INDEX;
    node.mId = IdType{};

    mNextAvailableIndex = nodeId;
    --mNodeCount;
}

template <typename IdType>
NodeIndex DynamicBVH<IdType>::addProxy(IdType id, AABB boundingBox, BitMaskType categoryBits, BitMaskType isHittingBits)
{
    const NodeIndex nodeId = createNode();
    BVHNode<IdType>& node = mNodes[nodeId];

    node.mBoundingBox = boundingBox.fattened(mFatAABBMargin);
    node.mId = id;
    node.mCategoryBits = categoryBits;
    node.mIsHittingBits = isHittingBits;

    ++mProxyCount;
    insertLeaf(nodeId);
    return nodeId;
}

template <typename IdType>
void DynamicBVH<IdType>::removeProxy(NodeIndex leafIndex)
{
    --mProxyCount;
    removeLeaf(leafIndex);
    destroyNode(leafIndex);
}

template <typename IdType>
void DynamicBVH<IdType>::clear()
{
    mNodes.clear();
    mNodeCount = 0;
    mRootIndex = INVALID_NODE_INDEX;
    mNextAvailableIndex = INVALID_NODE_INDEX;
    allocateNodes(initialCapacity);
}

template <typename IdType>
bool DynamicBVH<IdType>::moveProxy(NodeIndex nodeIndex, AABB newAABB)
{
    auto& node = mNodes[nodeIndex];

    // If the current fattened AABB contains the new AABB, no need to update the
    // tree
    if (node.mBoundingBox.contains(newAABB))
        return false;

    // Update the leaf AABB in place and propagate the change up the tree.
    // This avoids expensive remove/insert operations for large displacements.
    const Vec2 displacement = Vec2((newAABB.min.x - node.mBoundingBox.min.x) * displacementFactor,
                                   (newAABB.min.y - node.mBoundingBox.min.y) * displacementFactor);
    const AABB fatAabb = newAABB.fattened(mFatAABBMargin, displacement);
    node.mBoundingBox = fatAabb;

    NodeIndex currentIndex = node.mParentIndex;
    while (currentIndex != INVALID_NODE_INDEX)
    {
        auto& parent = mNodes[currentIndex];
        const NodeIndex child1 = parent.mChild1Index;
        const NodeIndex child2 = parent.mChild2Index;

        const AABB newParentAABB = AABB::combine(mNodes[child1].mBoundingBox, mNodes[child2].mBoundingBox);
        if (newParentAABB == parent.mBoundingBox)
            break;

        parent.mBoundingBox = newParentAABB;
        currentIndex = parent.mParentIndex;
    }
    return true;
}

template <typename IdType>
void DynamicBVH<IdType>::insertLeaf(NodeIndex leafIndex)
{
    if (mRootIndex == INVALID_NODE_INDEX)
    {
        mRootIndex = leafIndex;
        mNodes[mRootIndex].mParentIndex = INVALID_NODE_INDEX;
        return;
    }

    const NodeIndex siblingIndex = findBestSiblingIndex(mNodes[leafIndex].mBoundingBox);
    const NodeIndex oldParentIndex = mNodes[siblingIndex].mParentIndex;
    const NodeIndex newParentIndex = createNode();

    auto& leaf = mNodes[leafIndex];
    auto& sibling = mNodes[siblingIndex];
    auto& newParent = mNodes[newParentIndex];

    newParent.mParentIndex = oldParentIndex;
    newParent.mBoundingBox = AABB::combine(leaf.mBoundingBox, sibling.mBoundingBox);
    newParent.mHeight = sibling.mHeight + 1;
    newParent.mCategoryBits = sibling.mCategoryBits | leaf.mCategoryBits;

    if (oldParentIndex != INVALID_NODE_INDEX)
    {
        auto& oldParent = mNodes[oldParentIndex];
        if (oldParent.mChild1Index == siblingIndex)
            oldParent.mChild1Index = newParentIndex;
        else
            oldParent.mChild2Index = newParentIndex;

        newParent.mChild1Index = siblingIndex;
        newParent.mChild2Index = leafIndex;
        sibling.mParentIndex = newParentIndex;
        leaf.mParentIndex = newParentIndex;
    }
    else
    {
        newParent.mChild1Index = siblingIndex;
        newParent.mChild2Index = leafIndex;
        sibling.mParentIndex = newParentIndex;
        leaf.mParentIndex = newParentIndex;
        mRootIndex = newParentIndex;
    }

    // Walk up the tree fixing heights and AABBs
    NodeIndex currentNodeIndex = leaf.mParentIndex;
    while (currentNodeIndex != INVALID_NODE_INDEX)
    {
        currentNodeIndex = balance(currentNodeIndex);
        auto& currentNode = mNodes[currentNodeIndex];

        const NodeIndex child1Index = currentNode.mChild1Index;
        const NodeIndex child2Index = currentNode.mChild2Index;
        auto& child1 = mNodes[child1Index];
        auto& child2 = mNodes[child2Index];

        currentNode.mBoundingBox = AABB::combine(child1.mBoundingBox, child2.mBoundingBox);
        currentNode.mHeight = 1 + std::max(child1.mHeight, child2.mHeight);
        currentNode.mCategoryBits = child1.mCategoryBits | child2.mCategoryBits;

        currentNodeIndex = currentNode.mParentIndex;
    }
}

template <typename IdType>
void DynamicBVH<IdType>::removeLeaf(NodeIndex leafIndex)
{
    if (leafIndex == mRootIndex)
    {
        // destroyNode(rootIndex);
        mRootIndex = INVALID_NODE_INDEX;
        return;
    }

    const NodeIndex parentIndex = mNodes[leafIndex].mParentIndex;
    const NodeIndex grandParentIndex = mNodes[parentIndex].mParentIndex;
    const NodeIndex siblingIndex = (mNodes[parentIndex].mChild1Index == leafIndex) ? mNodes[parentIndex].mChild2Index
                                                                                   : mNodes[parentIndex].mChild1Index;

    // Connect sibling to grandparent
    if (grandParentIndex != INVALID_NODE_INDEX)
    {
        if (mNodes[grandParentIndex].mChild1Index == parentIndex)
            mNodes[grandParentIndex].mChild1Index = siblingIndex;
        else
            mNodes[grandParentIndex].mChild2Index = siblingIndex;
        mNodes[siblingIndex].mParentIndex = grandParentIndex;
    }
    else
    {
        mRootIndex = siblingIndex;
        mNodes[siblingIndex].mParentIndex = INVALID_NODE_INDEX;
    }

    // Recycle parent node
    destroyNode(parentIndex);

    // Walk up and fix heights and AABBs (including rebalancing)
    NodeIndex currentNodeIndex = grandParentIndex;
    while (currentNodeIndex != INVALID_NODE_INDEX)
    {
        currentNodeIndex = balance(currentNodeIndex);
        auto& currentNode = mNodes[currentNodeIndex];

        const NodeIndex child1Index = currentNode.mChild1Index;
        const NodeIndex child2Index = currentNode.mChild2Index;
        auto& child1 = mNodes[child1Index];
        auto& child2 = mNodes[child2Index];

        currentNode.mBoundingBox = AABB::combine(child1.mBoundingBox, child2.mBoundingBox);
        currentNode.mHeight = 1 + std::max(child1.mHeight, child2.mHeight);
        currentNode.mCategoryBits = child1.mCategoryBits | child2.mCategoryBits;

        currentNodeIndex = currentNode.mParentIndex;
    }
}

template <typename IdType>
NodeIndex DynamicBVH<IdType>::findBestSiblingIndex(AABB leaf) const
{
    NodeIndex index = mRootIndex;

    // Find the best sibling for the new leaf
    while (!mNodes[index].isLeaf())
    {
        const NodeIndex child1Index = mNodes[index].mChild1Index;
        const NodeIndex child2Index = mNodes[index].mChild2Index;

        const AABB combined = AABB::combine(leaf, mNodes[index].mBoundingBox);
        const float cost = combined.perimeter();
        const float inheritedCost = 2.0f * cost;

        const auto child1LeafCombined = AABB::combine(leaf, mNodes[child1Index].mBoundingBox);
        const float cost1 =
            mNodes[child1Index].isLeaf()
                ? child1LeafCombined.perimeter() + inheritedCost
                : child1LeafCombined.perimeter() - mNodes[child1Index].mBoundingBox.perimeter() + inheritedCost;

        const auto child2LeafCombined = AABB::combine(leaf, mNodes[child2Index].mBoundingBox);
        const float cost2 =
            mNodes[child2Index].isLeaf()
                ? child2LeafCombined.perimeter() + inheritedCost
                : child2LeafCombined.perimeter() - mNodes[child2Index].mBoundingBox.perimeter() + inheritedCost;

        index = (cost1 < cost2) ? child1Index : child2Index;
    }

    return index;
}

template <typename IdType>
NodeIndex DynamicBVH<IdType>::balance(NodeIndex index)
{
    BVHNode<IdType>& node = mNodes[index];

    const NodeIndex child1Index = node.mChild1Index;
    const NodeIndex child2Index = node.mChild2Index;
    const BVHNode<IdType>& child1 = mNodes[child1Index];
    const BVHNode<IdType>& child2 = mNodes[child2Index];

    const int32_t unbalanceBetweenChildren = child2.mHeight - child1.mHeight;

    NodeIndex movingUpChildIndex;
    NodeIndex grandChild1Index;
    NodeIndex grandChild2Index;
    bool isRightRotation;
    if (unbalanceBetweenChildren > 1)
    {
        // Rotate Child2 up
        grandChild1Index = child2.mChild1Index;
        grandChild2Index = child2.mChild2Index;
        movingUpChildIndex = child2Index;
        isRightRotation = true;
    }
    else if (unbalanceBetweenChildren < -1)
    {
        // Rotate Child1 up
        grandChild1Index = child1.mChild1Index;
        grandChild2Index = child1.mChild2Index;
        movingUpChildIndex = child1Index;
        isRightRotation = false;
    }
    else
    {
        // No rotation needed
        return index;
    }

    BVHNode<IdType>& movingUpChild = mNodes[movingUpChildIndex];

    // Swap node and movingUpChild
    movingUpChild.mChild1Index = index;
    movingUpChild.mParentIndex = node.mParentIndex;
    node.mParentIndex = movingUpChildIndex;

    if (movingUpChild.mParentIndex != INVALID_NODE_INDEX)
    {
        BVHNode<IdType>& parent = mNodes[movingUpChild.mParentIndex];
        if (parent.mChild1Index == index)
            parent.mChild1Index = movingUpChildIndex;
        else
            parent.mChild2Index = movingUpChildIndex;
    }
    else
    {
        mRootIndex = movingUpChildIndex;
    }

    // Pick the taller child for attachment
    NodeIndex attachIndex, remainIndex;
    if ((mNodes[grandChild1Index].mHeight > mNodes[grandChild2Index].mHeight))
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
        movingUpChild.mChild2Index = attachIndex;
        node.mChild2Index = remainIndex;
    }
    else
    {
        movingUpChild.mChild2Index = attachIndex;
        node.mChild1Index = remainIndex;
    }

    mNodes[attachIndex].mParentIndex = movingUpChildIndex;
    mNodes[remainIndex].mParentIndex = index;

    // Recompute AABBs and heights
    if (isRightRotation)
    {
        node.mBoundingBox = AABB::combine(child1.mBoundingBox, mNodes[remainIndex].mBoundingBox);
        movingUpChild.mBoundingBox = AABB::combine(node.mBoundingBox, mNodes[attachIndex].mBoundingBox);

        node.mHeight = 1 + std::max(child1.mHeight, mNodes[remainIndex].mHeight);
        movingUpChild.mHeight = 1 + std::max(node.mHeight, mNodes[attachIndex].mHeight);

        node.mCategoryBits = child1.mCategoryBits | mNodes[remainIndex].mCategoryBits;
        movingUpChild.mCategoryBits = node.mCategoryBits | mNodes[attachIndex].mCategoryBits;
    }
    else
    {
        node.mBoundingBox = AABB::combine(child2.mBoundingBox, mNodes[remainIndex].mBoundingBox);
        movingUpChild.mBoundingBox = AABB::combine(node.mBoundingBox, mNodes[attachIndex].mBoundingBox);

        node.mHeight = 1 + std::max(child2.mHeight, mNodes[remainIndex].mHeight);
        movingUpChild.mHeight = 1 + std::max(node.mHeight, mNodes[attachIndex].mHeight);

        node.mCategoryBits = child2.mCategoryBits | mNodes[remainIndex].mCategoryBits;
        movingUpChild.mCategoryBits = node.mCategoryBits | mNodes[attachIndex].mCategoryBits;
    }

    return movingUpChildIndex;
}

template <typename IdType>
std::vector<std::pair<IdType, AABB>> DynamicBVH<IdType>::data() const
{
    if (mRootIndex == INVALID_NODE_INDEX)
        return {};

    std::vector<std::pair<IdType, AABB>> result;

    for (uint32_t i = 0; i < mNodeCount; ++i)
    {
        const auto& node = mNodes[i];
        if (node.isLeaf())
            result.emplace_back(node.mId, node.mBoundingBox);
    }

    return result;
}

template <typename IdType>
auto DynamicBVH<IdType>::leavesView() const
{
    return mNodes | std::views::filter([](const auto& node) { return node.isLeaf(); }) |
           std::views::transform([](const auto& node) { return std::pair<IdType, AABB>{node.mId, node.mBoundingBox}; });
}

template <typename IdType>
template <typename Allocator>
void DynamicBVH<IdType>::query(AABB queryAABB,
                               std::vector<IdType, Allocator>& intersections,
                               BitMaskType maskBits) const
{
    if (mRootIndex == INVALID_NODE_INDEX)
        return;

    std::vector<NodeIndex> stack = {mRootIndex};

    while (!stack.empty())
    {
        const NodeIndex nodeIndex = stack.back();
        stack.pop_back();
        const auto& node = mNodes[nodeIndex];

        // Collision detected only if the node's categoryBits match the maskBits
        if (!node.matchesMask(maskBits))
            continue;
        // Skip the whole subtree (since it is contained in the node's AABB)
        if (!node.mBoundingBox.intersects(queryAABB))
            continue;

        if (node.isLeaf())
        {
            intersections.push_back(node.mId);
        }
        else
        {
            stack.push_back(node.mChild1Index);
            stack.push_back(node.mChild2Index);
        }
    }
}

template <typename IdType>
void DynamicBVH<IdType>::batchQuery(std::pmr::monotonic_buffer_resource& poolResource,
                                    const std::pmr::vector<std::pair<AABB, BitMaskType>>& queries,
                                    const std::function<void(size_t, std::pmr::vector<IdType>&)>& callback) const
{
    C2D_PARALLEL_FOR(0,
                     queries.size(),
                     [&](size_t i)
                     {
                         std::pmr::vector<IdType> hits{&poolResource};
                         query(queries[i].first, hits, queries[i].second);
                         if (!hits.empty())
                             callback(i, hits);
                     });
}

template <typename IdType>
void DynamicBVH<IdType>::batchQuery(const std::vector<std::pair<AABB, BitMaskType>>& queries,
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
void DynamicBVH<IdType>::piercingRaycast(Ray ray, std::set<RaycastInfo>& hits, BitMaskType maskBits) const
{
    if (mRootIndex == INVALID_NODE_INDEX)
        return;

    std::vector<NodeIndex> stack = {mRootIndex};

    while (!stack.empty())
    {
        const NodeIndex nodeIndex = stack.back();
        stack.pop_back();
        const auto& node = mNodes[nodeIndex];

        // Collision detected only if the node's categoryBits match the maskBits
        if (!node.matchesMask(maskBits))
            continue;

        const auto intersection = node.mBoundingBox.intersects(ray);
        if (!intersection)
            continue;

        if (node.isLeaf())
        {
            hits.insert(RaycastInfo::fromRay(node.mId, ray, *intersection));
        }
        else
        {
            stack.push_back(node.mChild1Index);
            stack.push_back(node.mChild2Index);
        }
    }
}

template <typename IdType>
std::optional<typename DynamicBVH<IdType>::RaycastInfo> DynamicBVH<IdType>::firstHitRaycast(Ray ray,
                                                                                            BitMaskType maskBits) const
{
    if (mRootIndex == INVALID_NODE_INDEX)
        return std::nullopt;

    std::optional<RaycastInfo> firstHit;
    float firstHitTime = std::numeric_limits<float>::max();

    std::vector<NodeIndex> stack = {mRootIndex};

    while (!stack.empty())
    {
        const NodeIndex nodeIndex = stack.back();
        stack.pop_back();
        const auto& node = mNodes[nodeIndex];

        // Collision detected only if the node's categoryBits match the maskBits
        if (!node.matchesMask(maskBits))
            continue;

        const auto intersection = node.mBoundingBox.intersects(ray);
        if (!intersection)
            continue;

        if (node.isLeaf())
        {
            if (intersection->first < firstHitTime)
            {
                firstHitTime = intersection->first;
                firstHit = RaycastInfo::fromRay(node.mId, ray, *intersection);
            }
        }
        else
        {
            stack.push_back(node.mChild1Index);
            stack.push_back(node.mChild2Index);
        }
    }

    return firstHit;
}

template <typename IdType>
void DynamicBVH<IdType>::piercingRaycast(InfiniteRay ray, std::set<RaycastInfo>& hits, BitMaskType maskBits) const
{
    if (mRootIndex == INVALID_NODE_INDEX)
        return;

    std::vector<NodeIndex> stack = {mRootIndex};

    while (!stack.empty())
    {
        const NodeIndex nodeIndex = stack.back();
        stack.pop_back();
        const auto& node = mNodes[nodeIndex];

        // Collision detected only if the node's categoryBits match the maskBits
        if (!node.matchesMask(maskBits))
            continue;

        const auto intersection = node.mBoundingBox.intersects(ray);
        if (!intersection)
            continue;

        if (node.isLeaf())
        {
            hits.insert(RaycastInfo::fromRay(node.mId, ray, *intersection));
        }
        else
        {
            stack.push_back(node.mChild1Index);
            stack.push_back(node.mChild2Index);
        }
    }
}

template <typename IdType>
std::optional<typename DynamicBVH<IdType>::RaycastInfo> DynamicBVH<IdType>::firstHitRaycast(InfiniteRay ray,
                                                                                            BitMaskType maskBits) const
{
    if (mRootIndex == INVALID_NODE_INDEX)
        return std::nullopt;

    std::optional<RaycastInfo> firstHit;
    float firstHitTime = std::numeric_limits<float>::max();

    std::vector<NodeIndex> stack = {mRootIndex};

    while (!stack.empty())
    {
        const NodeIndex nodeIndex = stack.back();
        stack.pop_back();
        const auto& node = mNodes[nodeIndex];

        // Collision detected only if the node's categoryBits match the maskBits
        if (!node.matchesMask(maskBits))
            continue;

        const auto intersection = node.mBoundingBox.intersects(ray);
        if (!intersection)
            continue;

        if (node.isLeaf())
        {
            if (intersection->first < firstHitTime)
            {
                firstHitTime = intersection->first;
                firstHit = RaycastInfo::fromRay(node.mId, ray, *intersection);
            }
        }
        else
        {
            stack.push_back(node.mChild1Index);
            stack.push_back(node.mChild2Index);
        }
    }

    return firstHit;
}

template <typename IdType>
void DynamicBVH<IdType>::findAllCollisions(const std::function<void(IdType, IdType)>& callback) const
{
    findAllCollisionsRecursive(mRootIndex, callback);
}

template <typename IdType>
void DynamicBVH<IdType>::findAllCollisions(const DynamicBVH<IdType>& other,
                                           const std::function<void(IdType, IdType)>& callback) const
{
    findPairsBetween(other, mRootIndex, other.mRootIndex, callback);
}

template <typename IdType>
void DynamicBVH<IdType>::findPairsBetween(NodeIndex nodeAIdx,
                                          NodeIndex nodeBIdx,
                                          const std::function<void(IdType, IdType)>& callback) const
{
    if (nodeAIdx == INVALID_NODE_INDEX || nodeBIdx == INVALID_NODE_INDEX || nodeAIdx == nodeBIdx)
        return;

    const auto& nodeA = mNodes[nodeAIdx];
    const auto& nodeB = mNodes[nodeBIdx];

    // Check bitmask compatibility
    if (!nodeA.matchesMask(nodeB.mCategoryBits) || !nodeB.matchesMask(nodeA.mCategoryBits))
        return;
    if (!nodeA.mBoundingBox.intersects(nodeB.mBoundingBox))
        return;

    if (nodeA.isLeaf() && nodeB.isLeaf())
    {
        callback(nodeA.mId, nodeB.mId);
        return;
    }

    // Expand the node with greater height (heuristic)
    if (nodeA.isLeaf() || (!nodeB.isLeaf() && nodeB.mHeight > nodeA.mHeight))
    {
        findPairsBetween(nodeAIdx, nodeB.mChild1Index, callback);
        findPairsBetween(nodeAIdx, nodeB.mChild2Index, callback);
    }
    else
    {
        findPairsBetween(nodeA.mChild1Index, nodeBIdx, callback);
        findPairsBetween(nodeA.mChild2Index, nodeBIdx, callback);
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

    const auto& nodeA = mNodes[nodeAIdx];
    const auto& nodeB = other.mNodes[nodeBIdx];

    if (!nodeA.matchesMask(nodeB.mCategoryBits) || !nodeB.matchesMask(nodeA.mCategoryBits))
        return;
    if (!nodeA.mBoundingBox.intersects(nodeB.mBoundingBox))
        return;

    if (nodeA.isLeaf() && nodeB.isLeaf())
    {
        callback(nodeA.mId, nodeB.mId);
        return;
    }

    if (nodeA.isLeaf() || (!nodeB.isLeaf() && nodeB.mHeight > nodeA.mHeight))
    {
        findPairsBetween(other, nodeAIdx, nodeB.mChild1Index, callback);
        findPairsBetween(other, nodeAIdx, nodeB.mChild2Index, callback);
    }
    else
    {
        findPairsBetween(other, nodeA.mChild1Index, nodeBIdx, callback);
        findPairsBetween(other, nodeA.mChild2Index, nodeBIdx, callback);
    }
}

template <typename IdType>
void DynamicBVH<IdType>::findAllCollisionsRecursive(NodeIndex nodeIdx,
                                                    const std::function<void(IdType, IdType)>& callback) const
{
    if (nodeIdx == INVALID_NODE_INDEX)
        return;

    const auto& node = mNodes[nodeIdx];
    if (node.isLeaf())
        return;

    // Check all pairs between children
    findPairsBetween(node.mChild1Index, node.mChild2Index, callback);

    // Check pairs within each child
    findAllCollisionsRecursive(node.mChild1Index, callback);
    findAllCollisionsRecursive(node.mChild2Index, callback);
}

template <typename IdType>
void DynamicBVH<IdType>::serialize(std::ostream& out) const
{
    Writer writer(out);

    writer(mFatAABBMargin);
    writer(mRootIndex);
    writer(mNodeCount);
    writer(mProxyCount);
    writer(mNextAvailableIndex);

    // Write nodes vector size
    const size_t nodesSize = mNodes.size();
    writer(nodesSize);

    // Write nodes
    for (const auto& node : mNodes)
        node.serialize(out);
}

template <typename IdType>
DynamicBVH<IdType> DynamicBVH<IdType>::deserialize(std::istream& in)
{
    Reader reader(in);

    float fatAABBMargin;
    reader(fatAABBMargin);

    DynamicBVH<IdType> bvh(fatAABBMargin);

    reader(bvh.mRootIndex);
    reader(bvh.mNodeCount);
    reader(bvh.mProxyCount);
    reader(bvh.mNextAvailableIndex);

    size_t nodesSize;
    reader(nodesSize);

    // Replace the preallocated storage created by the constructor
    bvh.mNodes.clear();
    bvh.mNodes.reserve(nodesSize);
    for (size_t i = 0; i < nodesSize; ++i)
        bvh.mNodes.emplace_back(BVHNode<IdType>::deserialize(in));

    return bvh;
}

#ifdef C2D_USE_CEREAL
template <typename IdType>
template <IsCerealArchive Archive>
void DynamicBVH<IdType>::serialize(Archive& archive)
{
    archive(mFatAABBMargin);
    archive(mRootIndex);
    archive(mNodeCount);
    archive(mProxyCount);
    archive(mNextAvailableIndex);
    archive(mNodes);
}
#endif

template <typename IdType>
bool DynamicBVH<IdType>::operator==(const DynamicBVH& other) const
{
    if (!float_equals(mFatAABBMargin, other.mFatAABBMargin))
        return false;
    if (mNodeCount != other.mNodeCount)
        return false;
    if (mProxyCount != other.mProxyCount)
        return false;
    if (mRootIndex != other.mRootIndex)
        return false;
    if (mNextAvailableIndex != other.mNextAvailableIndex)
        return false;
    if (mNodes != other.mNodes)
        return false;

    return true;
}

template <typename IdType>
void BVHNode<IdType>::serialize(std::ostream& out) const
{
    Writer writer(out);

    writer(mBoundingBox.min.x);
    writer(mBoundingBox.min.y);
    writer(mBoundingBox.max.x);
    writer(mBoundingBox.max.y);
    writer(mCategoryBits);
    writer(mIsHittingBits);
    writer(mParentIndex);
    writer(mChild1Index);
    writer(mChild2Index);
    writer(mHeight);
    writer(mId);
}

template <typename IdType>
BVHNode<IdType> BVHNode<IdType>::deserialize(std::istream& in)
{
    Reader reader(in);
    BVHNode<IdType> node;

    reader(node.mBoundingBox);
    reader(node.mCategoryBits);
    reader(node.mIsHittingBits);
    reader(node.mParentIndex);
    reader(node.mChild1Index);
    reader(node.mChild2Index);
    reader(node.mHeight);
    reader(node.mId);

    return node;
}

#ifdef C2D_USE_CEREAL
template <typename IdType>
template <IsCerealArchive Archive>
void BVHNode<IdType>::serialize(Archive& archive)
{
    archive(mBoundingBox, mCategoryBits, mIsHittingBits, mParentIndex, mChild1Index, mChild2Index, mHeight, mId);
}
#endif

template <typename IdType>
bool BVHNode<IdType>::operator==(const BVHNode& other) const
{
    if (mBoundingBox != other.mBoundingBox)
        return false;
    if (mCategoryBits != other.mCategoryBits)
        return false;
    if (mIsHittingBits != other.mIsHittingBits)
        return false;

    if (mParentIndex != other.mParentIndex)
        return false;
    if (mChild1Index != other.mChild1Index)
        return false;
    if (mChild2Index != other.mChild2Index)
        return false;

    if (mHeight != other.mHeight)
        return false;

    if (mId != other.mId)
        return false;

    return true;
}

} // namespace c2d
