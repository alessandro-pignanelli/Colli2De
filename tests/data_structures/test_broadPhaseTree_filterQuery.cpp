#include <algorithm>
#include <cstdint>
#include <map>
#include <set>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "data_structures/BroadPhaseTree.hpp"
#include "geometry/AABB.hpp"

using namespace c2d;
using namespace Catch;
using BPTRaycastInfo = RaycastInfo<uint32_t>;

namespace
{
    std::vector<uint32_t> toVectorIds(const std::set<BPTRaycastInfo>& hits)
    {
        std::vector<uint32_t> ids;
        ids.reserve(hits.size());
        for (const auto& hit : hits)
            ids.push_back(hit.id);
        return ids;
    }
}

TEST_CASE("BroadPhaseTree | query with mask bits filters correctly", "[BroadPhaseTree][Query][BitsMask]")
{
    BroadPhaseTree<uint32_t> tree;

    // Insert proxies with distinct categories
    tree.addProxy(1, {Vec2{0,0}, Vec2{1,1}}, 0b0001); // Category 0
    tree.addProxy(2, {Vec2{1,0}, Vec2{2,1}}, 0b0010); // Category 1
    tree.addProxy(3, {Vec2{2,0}, Vec2{3,1}}, 0b0100); // Category 2
    tree.addProxy(4, {Vec2{3,0}, Vec2{4,1}}, 0b1000); // Category 3

    // Query AABB covering all
    AABB allAABB{Vec2{0,0}, Vec2{4,1}};

    // Query with mask that matches only category 2
    SECTION("BroadPhaseTree | Query with mask for category 2")
    {
        BitMaskType only2 = 0b0100;
        std::set<uint32_t> result;
        result = tree.query(allAABB, only2);
        CHECK(result.size() == 1);
        CHECK(result.count(3) == 1); // Should match only the proxy with category 2
    }

    // Query with mask that matches category 1 and 3
    SECTION("BroadPhaseTree | Query with mask for category 1 and 3")
    {
        BitMaskType oneAndThree = 0b1010;
        std::set<uint32_t> result;
        result = tree.query(allAABB, oneAndThree);
        CHECK(result.size() == 2);
        CHECK(result.count(2) == 1);
        CHECK(result.count(4) == 1);
    }

    // Query with mask that matches all
    SECTION("BroadPhaseTree | Query with mask for all categories")
    {
        BitMaskType all = 0b1111;
        std::set<uint32_t> result;
        result = tree.query(allAABB, all);
        CHECK(result.size() == 4);
        CHECK(result.count(1) == 1);
        CHECK(result.count(2) == 1);
        CHECK(result.count(3) == 1);
        CHECK(result.count(4) == 1);
    }
}

TEST_CASE("BroadPhaseTree | piercingRaycast finds intersected proxies", "[BroadPhaseTree][PiercingRaycast][BitsMask]")
{
    BroadPhaseTree<uint32_t> tree;

    // Insert a row of 10 AABBs at y = 0..1, x = 0..10, with unique categoryBits (bit i)
    for (uint32_t i = 0; i < 10; ++i)
        tree.addProxy(i, AABB{ Vec2{float(i), 0.0f}, Vec2{float(i + 1), 1.0f} }, 1ull << i);

    // Ray from (-1,0.5) to (11,0.5) passes through all AABBs
    Ray ray{ Vec2{-1.0f, 0.5f}, Vec2{11.0f, 0.5f} };

    SECTION("BroadPhaseTree | No mask - finds all")
    {
        std::set<BPTRaycastInfo> hits;
        hits = tree.piercingRaycast(ray);
        const auto foundIds = toVectorIds(hits);
        CHECK(foundIds.size() == 10);
        for (uint32_t i = 0; i < 10; ++i)
            CHECK(std::find(foundIds.begin(), foundIds.end(), i) != foundIds.end());
    }

    SECTION("BroadPhaseTree | Mask for category 2 only")
    {
        BitMaskType mask = 1ull << 2;
        std::set<BPTRaycastInfo> hits;
        hits = tree.piercingRaycast(ray, mask);
        REQUIRE(hits.size() == 1);
        CHECK(hits.begin()->id == 2);
    }

    SECTION("BroadPhaseTree | Mask for categories 4 and 7 only")
    {
        BitMaskType mask = (1ull << 4) | (1ull << 7);
        std::set<BPTRaycastInfo> hits;
        hits = tree.piercingRaycast(ray, mask);
        const auto foundIds = toVectorIds(hits);
        CHECK(foundIds.size() == 2);
        CHECK(std::find(foundIds.begin(), foundIds.end(), 4) != foundIds.end());
        CHECK(std::find(foundIds.begin(), foundIds.end(), 7) != foundIds.end());
    }
}

TEST_CASE("BroadPhaseTree | firstHitRaycast finds the nearest hit", "[BroadPhaseTree][FirstHitRaycast][BitsMask]")
{
    BroadPhaseTree<uint32_t> tree;

    tree.addProxy(1, {Vec2{0,0}, Vec2{1,1}}, 0b001); // cat 0
    tree.addProxy(2, {Vec2{2,0}, Vec2{3,1}}, 0b010); // cat 1
    tree.addProxy(3, {Vec2{4,0}, Vec2{5,1}}, 0b100); // cat 2

    Ray ray{Vec2{-1,0.5f}, Vec2{6,0.5f}};

    SECTION("BroadPhaseTree | No mask - hits the first AABB")
    {
        auto firstId = tree.firstHitRaycast(ray);
        REQUIRE(firstId);
        CHECK(firstId->id == 1);
    }

    SECTION("BroadPhaseTree | Mask for category 2 (only hits id 3)")
    {
        BitMaskType mask = 0b100;
        auto firstId = tree.firstHitRaycast(ray, mask);
        REQUIRE(firstId);
        CHECK(firstId->id == 3);
    }

    SECTION("BroadPhaseTree | Mask for category 1 (only hits id 2)")
    {
        BitMaskType mask = 0b010;
        auto firstId = tree.firstHitRaycast(ray, mask);
        REQUIRE(firstId);
        CHECK(firstId->id == 2);
    }

    SECTION("BroadPhaseTree | Mask for category 8 (no matches)")
    {
        BitMaskType mask = 0b1000;
        auto firstId = tree.firstHitRaycast(ray, mask);
        CHECK(!firstId);
    }
}

TEST_CASE("BroadPhaseTree | piercingRaycast finds all hits with entry/exit points", "[BroadPhaseTree][PiercingRaycast][BitsMask]")
{
    BroadPhaseTree<uint32_t> tree;

    tree.addProxy(10, {Vec2{0,0 }, Vec2{1,1 } }, 0b001); // cat 0
    tree.addProxy(20, {Vec2{2,0 }, Vec2{3,1 } }, 0b010); // cat 1
    tree.addProxy(30, {Vec2{4,0 }, Vec2{5,1 } }, 0b100); // cat 2

    Ray ray{Vec2{-1,0.5f}, Vec2{6,0.5f}};

    SECTION("BroadPhaseTree | No mask - finds all")
    {
        std::set<BPTRaycastInfo> hits;
        hits = tree.piercingRaycast(ray);
        const auto foundIds = toVectorIds(hits);
        CHECK(foundIds.size() == 3);
        CHECK(foundIds[0] == 10);
        CHECK(foundIds[1] == 20);
        CHECK(foundIds[2] == 30);
    }

    SECTION("BroadPhaseTree | Mask for category 2 (id 30 only)")
    {
        BitMaskType mask = 0b100;
        std::set<BPTRaycastInfo> hits;
        hits = tree.piercingRaycast(ray, mask);
        const auto foundIds = toVectorIds(hits);
        REQUIRE(foundIds.size() == 1);
        CHECK(foundIds[0] == 30);
    }

    SECTION("BroadPhaseTree | Mask for category 1 (id 20 only)")
    {
        BitMaskType mask = 0b010;
        std::set<BPTRaycastInfo> hits;
        hits = tree.piercingRaycast(ray, mask);
        const auto foundIds = toVectorIds(hits);
        REQUIRE(foundIds.size() == 1);
        CHECK(foundIds[0] == 20);
    }
}

TEST_CASE("BroadPhaseTree | firstHitRaycast returns id, entry, and exit for closest hit", "[BroadPhaseTree][FirstHitRaycast][BitsMask]")
{
    BroadPhaseTree<uint32_t> tree;

    tree.addProxy(101, {Vec2{1,1 }, Vec2{2,2 } }, 0b001); // cat 0
    tree.addProxy(102, {Vec2{3,1 }, Vec2{4,2 } }, 0b010); // cat 1
    tree.addProxy(103, {Vec2{5,1 }, Vec2{6,2 } }, 0b100); // cat 2

    Ray ray{Vec2{0,1.5f}, Vec2{5,1.5f}};

    SECTION("BroadPhaseTree | No mask - first hit is 101")
    {
        auto hit = tree.firstHitRaycast(ray);
        REQUIRE(hit);
        CHECK(hit->id == 101);
    }

    SECTION("BroadPhaseTree | Mask for category 2 (id 103)")
    {
        BitMaskType mask = 0b100;
        auto hit = tree.firstHitRaycast(ray, mask);
        REQUIRE(hit);
        CHECK(hit->id == 103);
    }

    SECTION("BroadPhaseTree | Mask for category 1 (id 102)")
    {
        BitMaskType mask = 0b010;
        auto hit = tree.firstHitRaycast(ray, mask);
        REQUIRE(hit);
        CHECK(hit->id == 102);
    }
}

TEST_CASE("BroadPhaseTree | piercingRaycast with infinite ray finds intersected proxies", "[BroadPhaseTree][PiercingRaycast][BitsMask]")
{
    BroadPhaseTree<uint32_t> tree;

    for (uint32_t i = 0; i < 10; ++i)
        tree.addProxy(i, AABB{ Vec2{float(i), 0.0f}, Vec2{float(i + 1), 1.0f} }, 1ull << i);

    InfiniteRay ray{ Vec2{-1.0f, 0.5f}, Vec2{1.0f, 0.0f} };

    SECTION("BroadPhaseTree | Mask for category 3 (id 3 only)")
    {
        BitMaskType mask = 1ull << 3;
        std::set<BPTRaycastInfo> hits;
        hits = tree.piercingRaycast(ray, mask);
        const auto foundIds = toVectorIds(hits);
        REQUIRE(foundIds.size() == 1);
        CHECK(foundIds[0] == 3);
    }

    SECTION("BroadPhaseTree | Mask for categories 2-5")
    {
        BitMaskType mask = (1ull << 2) | (1ull << 3) | (1ull << 4) | (1ull << 5);
        std::set<BPTRaycastInfo> hits;
        hits = tree.piercingRaycast(ray, mask);
        const auto foundIds = toVectorIds(hits);
        CHECK(foundIds.size() == 4);
        for (uint32_t i = 2; i <= 5; ++i)
            CHECK(std::find(foundIds.begin(), foundIds.end(), i) != foundIds.end());
    }
}
