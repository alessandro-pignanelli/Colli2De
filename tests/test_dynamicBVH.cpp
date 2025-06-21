#include <cstdint>
#include <set>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "Random.hpp"
#include "AABB.hpp"
#include "bvh/dynamicBVH.hpp"

using namespace c2d;

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
    AABB expected = AABB::combine(aabb1, aabb2);
    REQUIRE(root.aabb.min.x == Catch::Approx(expected.min.x));
    REQUIRE(root.aabb.min.y == Catch::Approx(expected.min.y));
    REQUIRE(root.aabb.max.x == Catch::Approx(expected.max.x));
    REQUIRE(root.aabb.max.y == Catch::Approx(expected.max.y));
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
