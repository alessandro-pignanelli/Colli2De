#include <colli2de/Ray.hpp>
#include <cstdint>
#include <iostream>
#include <ranges>
#include <set>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "data_structures/DynamicBVH.hpp"
#include "geometry/AABB.hpp"
#include "utils/Random.hpp"

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
    constexpr float margin = 3.0f;
    DynamicBVH<std::string> bvh(margin);

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
    AABB expected = AABB::combine(aabb1, aabb2).fattened(margin);
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
    NodeIndex nodeId = bvh.createProxy(original, 42);

    // Small move: stays within fattened box
    AABB smallMove = original.move(Vec2{ margin, margin });
    bool treeChanged = bvh.moveProxy(nodeId, smallMove, Vec2{10.0f, 10.0f});
    CHECK_FALSE(treeChanged);

    // Large move: outside fattened box triggers reinsertion
    AABB largeMove = smallMove.move(Vec2{0.01f, 0.0f });
    treeChanged = bvh.moveProxy(nodeId, largeMove, displacement);
    CHECK(treeChanged);

    // Confirm that the node's AABB is now fattened in the +X direction
    const auto& node = bvh.getNode(nodeId);
    CHECK(node.aabb.min == largeMove.min - margin);
    CHECK(node.aabb.max == largeMove.max + margin + displacement);
}

TEST_CASE("DynamicBVH handles moving proxies with remove and reinsert", "[DynamicBVH][Advanced][Move]")
{
    constexpr float margin = 0.1f;
    DynamicBVH<uint32_t> bvh(0.1f);

    NodeIndex idx = bvh.createProxy({Vec2{0,0}, Vec2{1,1}}, 5);

    // Move proxy far away (should require removal and reinsertion)
    bool moved = bvh.moveProxy(idx, {Vec2{10,10}, Vec2{11,11}}, Vec2{10,10});
    REQUIRE(moved);

    // Query at old location should NOT find proxy
    AABB oldArea{Vec2{-1,-1}, Vec2{2,2}};
    std::set<uint32_t> hits;
    bvh.query(oldArea, hits);
    REQUIRE_FALSE(hits.count(5));

    // Query at new location should find proxy
    AABB newArea{Vec2{9,9}, Vec2{12,12}};
    hits.clear();
    bvh.query(newArea, hits);
    REQUIRE(hits.count(5));
}

TEST_CASE("DynamicBVH can serialize and deserialize correctly", "[DynamicBVH][Serialize][Deserialize]")
{
    // 1. Build and populate a BVH
    DynamicBVH<uint32_t> bvh(0.2f);
    std::vector<AABB> boxes;
    for (uint32_t i = 0; i < 100; ++i)
    {
        boxes.push_back(AABB{Vec2{float(i), float(i)}, Vec2{float(i+1), float(i+1)}});
        bvh.createProxy(boxes.back(), i);
    }

    // 2. Serialize to memory
    std::stringstream ss(std::ios::in | std::ios::out | std::ios::binary);
    bvh.serialize(ss);

    // 3. Deserialize into a new BVH
    auto bvh2 = DynamicBVH<uint32_t>::deserialize(ss);

    // 4. Check core fields
    CHECK(bvh2.fatAABBMargin == Approx(bvh.fatAABBMargin));
    CHECK(bvh2.capacity() == bvh.capacity());

    // 5. Query both and compare results
    AABB query{Vec2{20, 20}, Vec2{40, 40}};
    
    std::set<uint32_t> hits1, hits2;
    bvh.query(query, hits1);
    bvh2.query(query, hits2);

    CHECK(hits1 == hits2);

    // 6. Also test a raycast
    std::set<RaycastHit<uint32_t>> rayHits1, rayHits2;
    Ray ray{Vec2{19.5f, 19.5f}, Vec2{41.0f, 41.0f}};
    bvh.piercingRaycast(ray, rayHits1);
    bvh2.piercingRaycast(ray, rayHits2);
    CHECK(rayHits1 == rayHits2);

    // 7. Check that the tree structure is preserved
    CHECK(bvh == bvh2);
}

TEST_CASE("DynamicBVH iterator", "[DynamicBVH][Iterator]")
{
    DynamicBVH<uint32_t> bvh;
    std::set<uint32_t> ids;
    for (uint32_t i = 0; i < 10; ++i)
    {
        bvh.createProxy({Vec2{float(i), float(i)}, Vec2{float(i+1), float(i+1)}}, i);
        ids.insert(i);
    }

    for (const auto [id, aabb] : bvh)
    {
        CHECK(ids.find(id) != ids.end());
        ids.erase(id);

        CHECK(aabb.min.x == Approx(float(id)));
        CHECK(aabb.min.y == Approx(float(id)));
        CHECK(aabb.max.x == Approx(float(id + 1)));
        CHECK(aabb.max.y == Approx(float(id + 1)));
    }
    CHECK(ids.empty());
}

TEST_CASE("DynamicBVH data()", "[DynamicBVH][Data]")
{
    DynamicBVH<uint32_t> bvh;
    std::set<uint32_t> ids;
    for (uint32_t i = 0; i < 10; ++i)
    {
        bvh.createProxy({Vec2{float(i), float(i)}, Vec2{float(i+1), float(i+1)}}, i);
        ids.insert(i);
    }

    const auto data = bvh.data();
    CHECK(data.size() == 10);
    for (const auto& [id, aabb] : data)
    {
        CHECK(ids.find(id) != ids.end());
        ids.erase(id);

        CHECK(aabb.min.x == Approx(float(id)));
        CHECK(aabb.min.y == Approx(float(id)));
        CHECK(aabb.max.x == Approx(float(id + 1)));
        CHECK(aabb.max.y == Approx(float(id + 1)));
    }
}

TEST_CASE("DynamicBVH range view", "[DynamicBVH][RangeView]")
{
    DynamicBVH<uint32_t> bvh;
    std::set<uint32_t> ids;
    for (uint32_t i = 0; i < 10; ++i)
    {
        bvh.createProxy({Vec2{float(i), float(i)}, Vec2{float(i+1), float(i+1)}}, i);
        ids.insert(i);
    }

    for (auto&& [id, aabb] : bvh.leavesView())
    {
        CHECK(ids.find(id) != ids.end());
        ids.erase(id);

        CHECK(aabb.min.x == Approx(float(id)));
        CHECK(aabb.min.y == Approx(float(id)));
        CHECK(aabb.max.x == Approx(float(id + 1)));
        CHECK(aabb.max.y == Approx(float(id + 1)));
    }
    CHECK(ids.empty());
}
