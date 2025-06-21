#include <algorithm>
#include <cstdint>
#include <iostream>
#include <set>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "Random.hpp"
#include "geometry/AABB.hpp"
#include "bvh/dynamicBVH.hpp"

using namespace c2d;
using namespace Catch;

TEST_CASE("Node allocation", "[DynamicBVH]")
{
    const auto initialCapacity = DynamicBVH<uint32_t>::initialCapacity;
    DynamicBVH<uint32_t> bvh;

    REQUIRE(bvh.size() == 0);
    REQUIRE(bvh.capacity() == initialCapacity);

    std::set<int32_t> allocatedIds;
    for (int i = 0; i < initialCapacity; ++i)
    {
        int32_t id = bvh.createNode();
        CHECK(id >= 0);
        CHECK(id <= bvh.capacity());
        CHECK(allocatedIds.find(id) == allocatedIds.end());
        allocatedIds.insert(id);
    }
    CHECK(bvh.size() == initialCapacity);
    CHECK(bvh.capacity() == initialCapacity);

    // Try to allocate more nodes than capacity
    int32_t id = bvh.createNode();
    CHECK(id >= 0);
    CHECK(bvh.capacity() == initialCapacity * 2); // Should double the capacity
    CHECK(bvh.size() == initialCapacity + 1);
    CHECK(allocatedIds.find(id) == allocatedIds.end());
}

TEST_CASE("Node deallocation", "[DynamicBVH]")
{
    const auto initialCapacity = DynamicBVH<uint32_t>::initialCapacity;
    DynamicBVH<uint32_t> bvh;

    std::set<int32_t> allocatedIds;
    for (int i = 0; i < initialCapacity; ++i)
    {
        int32_t id = bvh.createNode();
        allocatedIds.insert(id);
    }
    REQUIRE(bvh.size() == initialCapacity);
    REQUIRE(bvh.capacity() == initialCapacity);
    REQUIRE(allocatedIds.size() == initialCapacity);

    // Deallocate all nodes in random order
    std::vector<int32_t> ids(allocatedIds.begin(), allocatedIds.end());
    randomShuffle(ids.begin(), ids.end());

    for (int32_t id : ids)
    {
        bvh.destroyNode(id);
        allocatedIds.erase(id);
        CHECK(bvh.size() == allocatedIds.size());
    }
    REQUIRE(bvh.size() == 0);
    REQUIRE(allocatedIds.size() == 0);

    for (int i = 0; i < initialCapacity; ++i)
    {
        int32_t id = bvh.createNode();
        allocatedIds.insert(id);
        CHECK(id >= 0);
        CHECK(id <= initialCapacity);
    }
    CHECK(bvh.size() == initialCapacity);
    CHECK(bvh.capacity() == initialCapacity);
    CHECK(allocatedIds.size() == initialCapacity);
}

TEST_CASE("Create proxy inserts node into tree", "[DynamicBVH]")
{
    DynamicBVH<std::string> bvh;

    const std::string id1 = "proxy1";
    AABB aabb1(Vec2{0.0f, 0.0f}, Vec2{1.0f, 1.0f});
    int32_t index1 = bvh.createProxy(aabb1, id1);

    const std::string id2 = "proxy2";
    AABB aabb2(Vec2{2.0f, 2.0f}, Vec2{3.0f, 3.0f});
    int32_t index2 = bvh.createProxy(aabb2, id2);

    // Sanity: indices should be distinct
    REQUIRE(index1 != index2);

    // Root should be non-negative (composite node)
    REQUIRE(bvh.getRootIndex() != -1);

    // Root shouldn't be a leaf
    const auto& root = bvh.getNode(bvh.getRootIndex());
    REQUIRE_FALSE(root.isLeaf());

    // Root children should be the two inserted proxies
    const auto& child1 = bvh.getNode(root.child1Index);
    const auto& child2 = bvh.getNode(root.child2Index);

    REQUIRE((child1.id == id1 || child1.id == id2));
    REQUIRE((child2.id == id1 || child2.id == id2));

    // Verify AABB of root is the union
    AABB expected = AABB::combine(aabb1, aabb2).fattened(DynamicBVH<std::string>::defaultAABBMargin);
    REQUIRE(root.aabb.min.x == Approx(expected.min.x));
    REQUIRE(root.aabb.min.y == Approx(expected.min.y));
    REQUIRE(root.aabb.max.x == Approx(expected.max.x));
    REQUIRE(root.aabb.max.y == Approx(expected.max.y));
}

TEST_CASE("DynamicBVH balancing: height after sequential insertions", "[DynamicBVH][Balance]") 
{
    DynamicBVH<uint32_t> bvh;

    // Insert 100 proxies in a way that would normally skew the tree
    for (uint32_t i = 0; i < 100; ++i)
    {
        float f = static_cast<float>(i);
        AABB aabb{ Vec2{f, f}, Vec2{f + 1.0f, f + 1.0f} };
        bvh.createProxy(aabb, i);
    }

    // Check: height should be at most log2(100) * 2 ~= 13 (AVL-like)
    // Optimal height for 100 nodes is ceil(log2(100)) = 7, so we allow some overhead
    int height = bvh.getNode(bvh.getRootIndex()).height;
    REQUIRE(height <= 13);

    // Root should not be a leaf
    REQUIRE_FALSE(bvh.getNode(bvh.getRootIndex()).isLeaf());

    // All nodes in the tree are connected and each has a correct parent
    std::vector<NodeIndex> stack{bvh.getRootIndex()};
    int leafCount = 0;
    while (!stack.empty())
    {
        NodeIndex idx = stack.back();
        stack.pop_back();
        const auto& node = bvh.getNode(idx);

        if (node.isLeaf())
        {
            ++leafCount;
        }
        else
        {
            stack.push_back(node.child1Index);
            stack.push_back(node.child2Index);

            // Both children point back to this node as parent
            REQUIRE(bvh.getNode(node.child1Index).parentIndex == idx);
            REQUIRE(bvh.getNode(node.child2Index).parentIndex == idx);
        }
    }

    REQUIRE(leafCount == 100); // All proxies present
}

TEST_CASE("DynamicBVH balancing: tree remains valid after zigzag insertions", "[DynamicBVH][Balance]")
{
    DynamicBVH<uint32_t> bvh;
    // Insert left, right, left, right to force tree rotations
    for (uint32_t i = 0; i < 50; ++i)
    {
        float f = static_cast<float>(i % 2 == 0 ? i : 100 - i);
        AABB aabb{ Vec2{f, f}, Vec2{f + 1.0f, f + 1.0f} };
        bvh.createProxy(aabb, i);
    }

    // Check tree height is not degenerate
    int height = bvh.getNode(bvh.getRootIndex()).height;
    REQUIRE(height <= 12);

    // All proxies are present in leaves
    int leaves = 0;
    std::vector<NodeIndex> stack{bvh.getRootIndex()};
    while (!stack.empty())
    {
        NodeIndex idx = stack.back();
        stack.pop_back();
        const auto& node = bvh.getNode(idx);
        if (node.isLeaf())
            ++leaves;
        else
        {
            stack.push_back(node.child1Index);
            stack.push_back(node.child2Index);
        }
    }
    REQUIRE(leaves == 50);
}

TEST_CASE("DynamicBVH removes proxies and maintains balance", "[DynamicBVH][Remove]") 
{
    DynamicBVH<uint32_t> bvh;
    std::vector<NodeIndex> proxies;
    for (uint32_t i = 0; i < 20; ++i)
    {
        float f = static_cast<float>(i);
        proxies.push_back(bvh.createProxy({{f, f}, {f+1, f+1}}, i));
    }

    // Remove every other proxy
    for (size_t i = 0; i < proxies.size(); i += 2)
    {
        bvh.destroyProxy(proxies[i]);
    }

    // Check remaining leaves == 10
    int leaves = 0;
    std::vector<NodeIndex> stack{bvh.getRootIndex()};
    while (!stack.empty())
    {
        NodeIndex idx = stack.back();
        stack.pop_back();

        if (idx == -1)
            continue;

        const auto& node = bvh.getNode(idx);
        if (node.isLeaf())
            ++leaves;
        else
        {
            stack.push_back(node.child1Index);
            stack.push_back(node.child2Index);
        }
    }
    REQUIRE(leaves == 10);

    // Tree remains balanced
    int height = bvh.getNode(bvh.getRootIndex()).height;
    REQUIRE(height <= 5);
}

TEST_CASE("DynamicBVH uses fattened AABB for proxies", "[DynamicBVH][Fattened]") 
{
    const float margin = 3.0f;
    DynamicBVH<uint32_t> bvh(margin);

    AABB aabb{ Vec2{2.0f, 3.0f}, Vec2{5.0f, 7.0f} };
    NodeIndex idx = bvh.createProxy(aabb, 123);

    const auto& node = bvh.getNode(idx);
    REQUIRE(node.aabb.min.x == Approx(aabb.min.x - margin));
    REQUIRE(node.aabb.max.y == Approx(aabb.max.y + margin));
}

TEST_CASE("DynamicBVH proxy update skips tree change for small moves", "[DynamicBVH][Fattened][MoveProxy]") 
{
    const float margin = 3.0f;
    const Vec2 displacement{ 2.0f, 2.0f };
    DynamicBVH<uint32_t> bvh(margin);

    AABB original{ Vec2{0.0f, 0.0f}, Vec2{2.0f, 2.0f} };
    NodeId nodeId = bvh.createProxy(original, 42);
    std::cout << "Created proxy with id: " << nodeId << std::endl;

    // Small move: stays within fattened box
    AABB smallMove = original.move(Vec2{ margin, margin });
    bool treeChanged = bvh.moveProxy(nodeId, smallMove, Vec2{10.0f, 10.0f});
    CHECK_FALSE(treeChanged);
    std::cout << "Moved proxy with id: " << nodeId << std::endl;

    // Large move: outside fattened box triggers reinsertion
    AABB largeMove = smallMove.move(Vec2{0.01f, 0.0f });
    treeChanged = bvh.moveProxy(nodeId, largeMove, displacement);
    CHECK(treeChanged);
    std::cout << "Moved proxy with index: " << nodeId << std::endl;

    // Confirm that the node's AABB is now fattened in the +X direction
    const auto& node = bvh.getNode(nodeId);
    CHECK(node.aabb.min == largeMove.min - margin);
    CHECK(node.aabb.max == largeMove.max + margin + displacement);
}

TEST_CASE("DynamicBVH::query finds all overlapping proxies", "[DynamicBVH][Query]")
{
    const float margin = 0.1f;
    DynamicBVH<uint32_t> bvh(margin);
    
    // Query an area covering (2,2) to (4,4)
    AABB queryAABB{Vec2{2.0f, 2.0f}, Vec2{4.0f, 4.0f}};

    // Expect regions from {1,1} to {5,5} to overlap (due to margin)
    std::vector<AABB> expectedOverlaps;
    std::set<uint32_t> expectedIds;
    for (uint32_t i = 1; i < 5; i++)
        for (uint32_t j = 1; j < 5; j++)
        {
            float x = static_cast<float>(i);
            float y = static_cast<float>(j);
            expectedOverlaps.emplace_back(Vec2{ x, y },
                                          Vec2{ x + 1.0f, y + 1.0f });
        }

    // Insert a grid of proxies from (0,0) to (6,6)
    const uint32_t gridSize = 6;
    for (uint32_t i = 0; i < gridSize; ++i)
    {
        for (uint32_t j = 0; j < gridSize; ++j)
        {
            float x = static_cast<float>(i);
            float y = static_cast<float>(j);
            const Vec2 min(x, y);
            const Vec2 max(x + 1.0f, y + 1.0f);
            const AABB aabb{min, max};

            const bool isExpectedOverlap = std::find(
                    expectedOverlaps.begin(), expectedOverlaps.end(), aabb
                ) != expectedOverlaps.end();

            const auto customId = i * gridSize + j;
            NodeId nodeIndex = bvh.createProxy(aabb, customId);
            if (isExpectedOverlap)
                expectedIds.insert(customId);
        }
    }

    std::vector<uint32_t> foundIds;
    bvh.query(queryAABB, [&](uint32_t id) { foundIds.push_back(id); });

    // Expect overlaps:
    // - {(1, 1), (2, 2)}, {(1, 2), (2, 3)}, {(1, 3), (2, 4)}, {(1, 4), (2, 5)}
    // - {(2, 1), (3, 2)}, {(2, 2), (3, 3)}, {(2, 3), (3, 4)}, {(2, 4), (3, 5)}
    // - {(3, 1), (4, 2)}, {(3, 2), (4, 3)}, {(3, 3), (4, 4)}, {(3, 4), (4, 5)}
    // - {(4, 1), (5, 2)}, {(4, 2), (5, 3)}, {(4, 3), (5, 4)}, {(4, 4), (5, 5)}

    // Should find all 16 ids in the 3x3 area
    CHECK(foundIds.size() == expectedOverlaps.size());
    for (auto id : foundIds)
        CHECK(expectedIds.find(id) != expectedIds.end());

    // Query an area that does not overlap with any proxies
    foundIds.clear();
    AABB nonOverlappingQuery{Vec2{6.5f, 6.5f}, Vec2{7.5f, 7.5f}};
    bvh.query(nonOverlappingQuery, [&](uint32_t id) { foundIds.push_back(id); });

    // Should find no ids
    CHECK(foundIds.empty());

    // Query an area that overlaps with a single proxy
    foundIds.clear();
    AABB singleOverlapQuery{Vec2{3.2f, 3.2f}, Vec2{3.8f, 3.8f}};
    bvh.query(singleOverlapQuery, [&](uint32_t id) { foundIds.push_back(id); });

    // Should find exactly one id
    CHECK(foundIds.size() == 1);
    CHECK(foundIds[0] == 21); // The proxy with AABB{(3, 3), (4, 4)}

    // Query an area that overlaps with every proxy
    foundIds.clear();
    AABB fullOverlapQuery{Vec2{0.0f, 0.0f}, Vec2{6.0f, 6.0f}};
    bvh.query(fullOverlapQuery, [&](uint32_t id) { foundIds.push_back(id); });

    // Should find all 36 ids
    CHECK(foundIds.size() == gridSize * gridSize);
}
