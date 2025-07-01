#include <algorithm>
#include <cstdint>
#include <map>
#include <set>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "data_structures/DynamicBVH.hpp"
#include "geometry/AABB.hpp"

using namespace c2d;
using namespace Catch;
using RaycastInfo = DynamicBVH<uint32_t>::RaycastInfo;

TEST_CASE("DynamicBVH::query with mask bits filters correctly", "[DynamicBVH][Query][BitsMask]")
{
    DynamicBVH<uint32_t> bvh;

    // Insert proxies with distinct categories
    bvh.createProxy({Vec2{0,0}, Vec2{1,1}}, 1, 0b0001); // Category 0
    bvh.createProxy({Vec2{1,0}, Vec2{2,1}}, 2, 0b0010); // Category 1
    bvh.createProxy({Vec2{2,0}, Vec2{3,1}}, 3, 0b0100); // Category 2
    bvh.createProxy({Vec2{3,0}, Vec2{4,1}}, 4, 0b1000); // Category 3

    // Query AABB covering all
    AABB allAABB{Vec2{0,0}, Vec2{4,1}};

    // Query with mask that matches only category 2
    SECTION("Query with mask for category 2")
    {
        BitMaskType only2 = 0b0100;
        auto result = bvh.query(allAABB, only2);
        REQUIRE(result.size() == 1);
        CHECK(result[0] == 3); // Should match only the proxy with category 2
    }

    // Query with mask that matches category 1 and 3
    SECTION("Query with mask for category 1 and 3")
    {
        BitMaskType oneAndThree = 0b1010;
        auto result = bvh.query(allAABB, oneAndThree);
        REQUIRE(result.size() == 2);
        CHECK(std::find(result.begin(), result.end(), 2) != result.end());
        CHECK(std::find(result.begin(), result.end(), 4) != result.end());
    }

    // Query with mask that matches all
    SECTION("Query with mask for all categories")
    {
        BitMaskType all = 0b1111;
        auto result = bvh.query(allAABB, all);
        REQUIRE(result.size() == 4);
        CHECK(std::find(result.begin(), result.end(), 1) != result.end());
        CHECK(std::find(result.begin(), result.end(), 2) != result.end());
        CHECK(std::find(result.begin(), result.end(), 3) != result.end());
        CHECK(std::find(result.begin(), result.end(), 4) != result.end());
    }
}

TEST_CASE("DynamicBVH::piercingRaycast finds intersected proxies", "[DynamicBVH][PiercingRaycast][BitsMask]")
{
    const float margin = 0.0f;
    DynamicBVH<uint32_t> bvh(margin);

    // Insert a row of 10 AABBs at y = 0..1, x = 0..10, with unique categoryBits (bit i)
    for (uint32_t i = 0; i < 10; ++i)
        bvh.createProxy(AABB{ Vec2{float(i), 0.0f}, Vec2{float(i + 1), 1.0f} }, i, 1ull << i);

    // Ray from (-1,0.5) to (11,0.5) passes through all AABBs
    Ray ray{ Vec2{-1.0f, 0.5f}, Vec2{11.0f, 0.5f} };

    SECTION("No mask - finds all")
    {
        auto foundIds = bvh.piercingRaycast(ray);
        CHECK(foundIds.size() == 10);
        for (uint32_t i = 0; i < 10; ++i)
            CHECK(std::find(foundIds.begin(), foundIds.end(), i) != foundIds.end());
    }

    SECTION("Mask for category 2 only")
    {
        BitMaskType mask = 1ull << 2;
        auto foundIds = bvh.piercingRaycast(ray, mask);
        REQUIRE(foundIds.size() == 1);
        CHECK(foundIds[0] == 2);
    }

    SECTION("Mask for categories 4 and 7 only")
    {
        BitMaskType mask = (1ull << 4) | (1ull << 7);
        auto foundIds = bvh.piercingRaycast(ray, mask);
        CHECK(foundIds.size() == 2);
        CHECK(std::find(foundIds.begin(), foundIds.end(), 4) != foundIds.end());
        CHECK(std::find(foundIds.begin(), foundIds.end(), 7) != foundIds.end());
    }
}

TEST_CASE("DynamicBVH::firstHitRaycast finds the nearest hit", "[DynamicBVH][FirstHitRaycast][BitsMask]")
{
    const float margin = 0.0f;
    DynamicBVH<uint32_t> bvh(margin);

    bvh.createProxy({Vec2{0,0}, Vec2{1,1}}, 1, 0b001); // cat 0
    bvh.createProxy({Vec2{2,0}, Vec2{3,1}}, 2, 0b010); // cat 1
    bvh.createProxy({Vec2{4,0}, Vec2{5,1}}, 3, 0b100); // cat 2

    Ray ray{Vec2{-1,0.5f}, Vec2{6,0.5f}};

    SECTION("No mask - hits the first AABB")
    {
        auto firstId = bvh.firstHitRaycast(ray);
        REQUIRE(firstId);
        CHECK(*firstId == 1);
    }

    SECTION("Mask for category 2 (only hits id 3)")
    {
        BitMaskType mask = 0b100;
        auto firstId = bvh.firstHitRaycast(ray, mask);
        REQUIRE(firstId);
        CHECK(*firstId == 3);
    }

    SECTION("Mask for category 1 (only hits id 2)")
    {
        BitMaskType mask = 0b010;
        auto firstId = bvh.firstHitRaycast(ray, mask);
        REQUIRE(firstId);
        CHECK(*firstId == 2);
    }

    SECTION("Mask for category 8 (no matches)")
    {
        BitMaskType mask = 0b1000;
        auto firstId = bvh.firstHitRaycast(ray, mask);
        CHECK(!firstId);
    }
}

TEST_CASE("DynamicBVH::piercingRaycastDetailed finds all hits with entry/exit points", "[DynamicBVH][PiercingRaycast][BitsMask]")
{
    const float margin = 0.0f;
    DynamicBVH<uint32_t> bvh(margin);

    bvh.createProxy({Vec2{0,0 }, Vec2{1,1 } }, 10, 0b001); // cat 0
    bvh.createProxy({Vec2{2,0 }, Vec2{3,1 } }, 20, 0b010); // cat 1
    bvh.createProxy({Vec2{4,0 }, Vec2{5,1 } }, 30, 0b100); // cat 2

    Ray ray{Vec2{-1,0.5f}, Vec2{6,0.5f}};

    SECTION("No mask - finds all")
    {
        auto hits = bvh.piercingRaycastDetailed(ray);
        CHECK(hits.size() == 3);
        CHECK(hits[0].id == 10);
        CHECK(hits[1].id == 20);
        CHECK(hits[2].id == 30);
    }

    SECTION("Mask for category 2 (id 30 only)")
    {
        BitMaskType mask = 0b100;
        auto hits = bvh.piercingRaycastDetailed(ray, mask);
        REQUIRE(hits.size() == 1);
        CHECK(hits[0].id == 30);
    }

    SECTION("Mask for category 1 (id 20 only)")
    {
        BitMaskType mask = 0b010;
        auto hits = bvh.piercingRaycastDetailed(ray, mask);
        REQUIRE(hits.size() == 1);
        CHECK(hits[0].id == 20);
    }
}

TEST_CASE("DynamicBVH::firstHitRaycastDetailed returns id, entry, and exit for closest hit", "[DynamicBVH][FirstHitRaycastDetailed][BitsMask]")
{
    const float margin = 0.0f;
    DynamicBVH<uint32_t> bvh(margin);

    bvh.createProxy({Vec2{1,1 }, Vec2{2,2 } }, 101, 0b001); // cat 0
    bvh.createProxy({Vec2{3,1 }, Vec2{4,2 } }, 102, 0b010); // cat 1
    bvh.createProxy({Vec2{5,1 }, Vec2{6,2 } }, 103, 0b100); // cat 2

    Ray ray{Vec2{0,1.5f}, Vec2{5,1.5f}};

    SECTION("No mask - first hit is 101")
    {
        auto hit = bvh.firstHitRaycastDetailed(ray);
        REQUIRE(hit);
        CHECK(hit->id == 101);
    }

    SECTION("Mask for category 2 (id 103)")
    {
        BitMaskType mask = 0b100;
        auto hit = bvh.firstHitRaycastDetailed(ray, mask);
        REQUIRE(hit);
        CHECK(hit->id == 103);
    }

    SECTION("Mask for category 1 (id 102)")
    {
        BitMaskType mask = 0b010;
        auto hit = bvh.firstHitRaycastDetailed(ray, mask);
        REQUIRE(hit);
        CHECK(hit->id == 102);
    }
}

TEST_CASE("DynamicBVH::piercingRaycast with infinite ray finds intersected proxies", "[DynamicBVH][PiercingRaycast][BitsMask]")
{
    DynamicBVH<uint32_t> bvh(0.0f);

    for (uint32_t i = 0; i < 10; ++i)
        bvh.createProxy(AABB{ Vec2{float(i), 0.0f}, Vec2{float(i + 1), 1.0f} }, i, 1ull << i);

    InfiniteRay ray{ Vec2{-1.0f, 0.5f}, Vec2{1.0f, 0.0f} };

    SECTION("Mask for category 3 (id 3 only)")
    {
        BitMaskType mask = 1ull << 3;
        auto foundIds = bvh.piercingRaycast(ray, mask);
        REQUIRE(foundIds.size() == 1);
        CHECK(foundIds[0] == 3);
    }

    SECTION("Mask for categories 2-5")
    {
        BitMaskType mask = (1ull << 2) | (1ull << 3) | (1ull << 4) | (1ull << 5);
        auto foundIds = bvh.piercingRaycast(ray, mask);
        CHECK(foundIds.size() == 4);
        for (uint32_t i = 2; i <= 5; ++i)
            CHECK(std::find(foundIds.begin(), foundIds.end(), i) != foundIds.end());
    }
}
