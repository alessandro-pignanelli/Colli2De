#include <algorithm>
#include <cstdint>
#include <set>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <colli2de/internal/data_structures/DynamicBVH.hpp>
#include <colli2de/internal/geometry/AABB.hpp>

using namespace c2d;
using namespace Catch;
using RaycastInfo = DynamicBVH<uint32_t>::RaycastInfo;

namespace
{
    std::vector<uint32_t> toVectorIds(const std::set<RaycastInfo>& hits)
    {
        std::vector<uint32_t> ids;
        ids.reserve(hits.size());
        for (const auto& hit : hits)
            ids.push_back(hit.id);
        return ids;
    }
}

TEST_CASE("DynamicBVH | query with mask bits filters correctly", "[DynamicBVH][Query][BitsMask]")
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
    SECTION("DynamicBVH | Query with mask for category 2")
    {
        BitMaskType only2 = 0b0100;
        std::vector<uint32_t> result;
        bvh.query(allAABB, result, only2);
        CHECK(result.size() == 1);
        CHECK(std::find(result.begin(), result.end(), 3) != result.end()); // Should match only the proxy with category 2
    }

    // Query with mask that matches category 1 and 3
    SECTION("DynamicBVH | Query with mask for category 1 and 3")
    {
        BitMaskType oneAndThree = 0b1010;
        std::vector<uint32_t> result;
        bvh.query(allAABB, result, oneAndThree);
        CHECK(result.size() == 2);
        CHECK(std::find(result.begin(), result.end(), 2) != result.end());
        CHECK(std::find(result.begin(), result.end(), 4) != result.end());
    }

    // Query with mask that matches all
    SECTION("DynamicBVH | Query with mask for all categories")
    {
        BitMaskType all = 0b1111;
        std::vector<uint32_t> result;
        bvh.query(allAABB, result, all);
        CHECK(result.size() == 4);
        CHECK(std::find(result.begin(), result.end(), 1) != result.end());
        CHECK(std::find(result.begin(), result.end(), 2) != result.end());
        CHECK(std::find(result.begin(), result.end(), 3) != result.end());
        CHECK(std::find(result.begin(), result.end(), 4) != result.end());
    }
}

TEST_CASE("DynamicBVH | piercingRaycast finds intersected proxies", "[DynamicBVH][PiercingRaycast][BitsMask]")
{
    const float margin = 0.0f;
    DynamicBVH<uint32_t> bvh(margin);

    // Insert a row of 10 AABBs at y = 0..1, x = 0..10, with unique categoryBits (bit i)
    for (uint32_t i = 0; i < 10; ++i)
        bvh.createProxy(AABB{ Vec2{float(i), 0.0f}, Vec2{float(i + 1), 1.0f} }, i, 1ull << i);

    // Ray from (-1,0.5) to (11,0.5) passes through all AABBs
    Ray ray{ Vec2{-1.0f, 0.5f}, Vec2{11.0f, 0.5f} };

    SECTION("DynamicBVH | No mask - finds all")
    {
        std::set<RaycastInfo> hits;
        bvh.piercingRaycast(ray, hits);
        const auto foundIds = toVectorIds(hits);
        CHECK(foundIds.size() == 10);
        for (uint32_t i = 0; i < 10; ++i)
            CHECK(std::find(foundIds.begin(), foundIds.end(), i) != foundIds.end());
    }

    SECTION("DynamicBVH | Mask for category 2 only")
    {
        BitMaskType mask = 1ull << 2;
        std::set<RaycastInfo> hits;
        bvh.piercingRaycast(ray, hits, mask);
        REQUIRE(hits.size() == 1);
        CHECK(hits.begin()->id == 2);
    }

    SECTION("DynamicBVH | Mask for categories 4 and 7 only")
    {
        BitMaskType mask = (1ull << 4) | (1ull << 7);
        std::set<RaycastInfo> hits;
        bvh.piercingRaycast(ray, hits, mask);
        const auto foundIds = toVectorIds(hits);
        CHECK(foundIds.size() == 2);
        CHECK(std::find(foundIds.begin(), foundIds.end(), 4) != foundIds.end());
        CHECK(std::find(foundIds.begin(), foundIds.end(), 7) != foundIds.end());
    }
}

TEST_CASE("DynamicBVH | firstHitRaycast finds the nearest hit", "[DynamicBVH][FirstHitRaycast][BitsMask]")
{
    const float margin = 0.0f;
    DynamicBVH<uint32_t> bvh(margin);

    bvh.createProxy({Vec2{0,0}, Vec2{1,1}}, 1, 0b001); // cat 0
    bvh.createProxy({Vec2{2,0}, Vec2{3,1}}, 2, 0b010); // cat 1
    bvh.createProxy({Vec2{4,0}, Vec2{5,1}}, 3, 0b100); // cat 2

    Ray ray{Vec2{-1,0.5f}, Vec2{6,0.5f}};

    SECTION("DynamicBVH | No mask - hits the first AABB")
    {
        auto firstId = bvh.firstHitRaycast(ray);
        REQUIRE(firstId);
        CHECK(firstId->id == 1);
        CHECK(firstId->entry.x == Approx(0.0f));
        CHECK(firstId->exit.x == Approx(1.0f));
        CHECK(firstId->entry.y == Approx(0.5f));
        CHECK(firstId->exit.y == Approx(0.5f));
        CHECK(firstId->entryTime == Approx((0.0f - (-1.0f)) / 7.0f));
        CHECK(firstId->exitTime == Approx((1.0f - (-1.0f)) / 7.0f));
    }

    SECTION("DynamicBVH | Mask for category 2 (only hits id 3)")
    {
        BitMaskType mask = 0b100;
        auto firstId = bvh.firstHitRaycast(ray, mask);
        REQUIRE(firstId);
        CHECK(firstId->id == 3);
        CHECK(firstId->entry.x == Approx(4.0f));
        CHECK(firstId->exit.x == Approx(5.0f));
        CHECK(firstId->entry.y == Approx(0.5f));
        CHECK(firstId->exit.y == Approx(0.5f));
        CHECK(firstId->entryTime == Approx((4.0f - (-1.0f)) / 7.0f));
        CHECK(firstId->exitTime == Approx((5.0f - (-1.0f)) / 7.0f));
    }

    SECTION("DynamicBVH | Mask for category 1 (only hits id 2)")
    {
        BitMaskType mask = 0b010;
        auto firstId = bvh.firstHitRaycast(ray, mask);
        REQUIRE(firstId);
        CHECK(firstId->id == 2);
        CHECK(firstId->entry.x == Approx(2.0f));
        CHECK(firstId->exit.x == Approx(3.0f));
        CHECK(firstId->entry.y == Approx(0.5f));
        CHECK(firstId->exit.y == Approx(0.5f));
        CHECK(firstId->entryTime == Approx((2.0f - (-1.0f)) / 7.0f));
        CHECK(firstId->exitTime == Approx((3.0f - (-1.0f)) / 7.0f));
    }

    SECTION("DynamicBVH | Mask for category 8 (no matches)")
    {
        BitMaskType mask = 0b1000;
        auto firstId = bvh.firstHitRaycast(ray, mask);
        CHECK(!firstId);
    }
}

TEST_CASE("DynamicBVH | piercingRaycast finds all hits with entry/exit points", "[DynamicBVH][PiercingRaycast][BitsMask]")
{
    const float margin = 0.0f;
    DynamicBVH<uint32_t> bvh(margin);

    bvh.createProxy({Vec2{0,0 }, Vec2{1,1 } }, 10, 0b001); // cat 0
    bvh.createProxy({Vec2{2,0 }, Vec2{3,1 } }, 20, 0b010); // cat 1
    bvh.createProxy({Vec2{4,0 }, Vec2{5,1 } }, 30, 0b100); // cat 2

    Ray ray{Vec2{-1,0.5f}, Vec2{6,0.5f}};

    SECTION("DynamicBVH | No mask - finds all")
    {
        std::set<RaycastInfo> hits;
        bvh.piercingRaycast(ray, hits);
        const auto foundIds = toVectorIds(hits);
        CHECK(foundIds.size() == 3);
        CHECK(foundIds[0] == 10);
        CHECK(foundIds[1] == 20);
        CHECK(foundIds[2] == 30);
    }

    SECTION("DynamicBVH | Mask for category 2 (id 30 only)")
    {
        BitMaskType mask = 0b100;
        std::set<RaycastInfo> hits;
        bvh.piercingRaycast(ray, hits, mask);
        const auto foundIds = toVectorIds(hits);
        REQUIRE(foundIds.size() == 1);
        CHECK(foundIds[0] == 30);
    }

    SECTION("DynamicBVH | Mask for category 1 (id 20 only)")
    {
        BitMaskType mask = 0b010;
        std::set<RaycastInfo> hits;
        bvh.piercingRaycast(ray, hits, mask);
        const auto foundIds = toVectorIds(hits);
        REQUIRE(foundIds.size() == 1);
        CHECK(foundIds[0] == 20);
    }
}

TEST_CASE("DynamicBVH | firstHitRaycast returns id, entry, and exit for closest hit", "[DynamicBVH][FirstHitRaycast][BitsMask]")
{
    const float margin = 0.0f;
    DynamicBVH<uint32_t> bvh(margin);

    bvh.createProxy({Vec2{1,1 }, Vec2{2,2 } }, 101, 0b001); // cat 0
    bvh.createProxy({Vec2{3,1 }, Vec2{4,2 } }, 102, 0b010); // cat 1
    bvh.createProxy({Vec2{5,1 }, Vec2{6,2 } }, 103, 0b100); // cat 2

    Ray ray{Vec2{0,1.5f}, Vec2{5,1.5f}};

    SECTION("DynamicBVH | No mask - first hit is 101")
    {
        auto hit = bvh.firstHitRaycast(ray);
        REQUIRE(hit);
        CHECK(hit->id == 101);
        CHECK(hit->entry.x == Approx(1.0f));
        CHECK(hit->exit.x  == Approx(2.0f));
        CHECK(hit->entry.y == Approx(1.5f));
        CHECK(hit->exit.y  == Approx(1.5f));
        CHECK(hit->entryTime == Approx(1.0f / 5.0f));
        CHECK(hit->exitTime  == Approx(2.0f / 5.0f));
    }

    SECTION("DynamicBVH | Mask for category 2 (id 103)")
    {
        BitMaskType mask = 0b100;
        auto hit = bvh.firstHitRaycast(ray, mask);
        REQUIRE(hit);
        CHECK(hit->id == 103);
        CHECK(hit->entry.x == Approx(5.0f));
        CHECK(hit->exit.x  == Approx(5.0f));
        CHECK(hit->entry.y == Approx(1.5f));
        CHECK(hit->exit.y  == Approx(1.5f));
        CHECK(hit->entryTime == Approx(1.0f));
        CHECK(hit->exitTime  == Approx(1.0f));
    }

    SECTION("DynamicBVH | Mask for category 1 (id 102)")
    {
        BitMaskType mask = 0b010;
        auto hit = bvh.firstHitRaycast(ray, mask);
        REQUIRE(hit);
        CHECK(hit->id == 102);
        CHECK(hit->entry.x == Approx(3.0f));
        CHECK(hit->exit.x  == Approx(4.0f));
        CHECK(hit->entry.y == Approx(1.5f));
        CHECK(hit->exit.y  == Approx(1.5f));
        CHECK(hit->entryTime == Approx(3.0f / 5.0f));
        CHECK(hit->exitTime  == Approx(4.0f / 5.0f));
    }
}

TEST_CASE("DynamicBVH | piercingRaycast with infinite ray finds intersected proxies", "[DynamicBVH][PiercingRaycast][BitsMask]")
{
    DynamicBVH<uint32_t> bvh(0.0f);

    for (uint32_t i = 0; i < 10; ++i)
        bvh.createProxy(AABB{ Vec2{float(i), 0.0f}, Vec2{float(i + 1), 1.0f} }, i, 1ull << i);

    InfiniteRay ray{ Vec2{-1.0f, 0.5f}, Vec2{1.0f, 0.0f} };

    SECTION("DynamicBVH | Mask for category 3 (id 3 only)")
    {
        BitMaskType mask = 1ull << 3;
        std::set<RaycastInfo> hits;
        bvh.piercingRaycast(ray, hits, mask);
        const auto foundIds = toVectorIds(hits);
        REQUIRE(foundIds.size() == 1);
        CHECK(foundIds[0] == 3);
    }

    SECTION("DynamicBVH | Mask for categories 2-5")
    {
        BitMaskType mask = (1ull << 2) | (1ull << 3) | (1ull << 4) | (1ull << 5);
        std::set<RaycastInfo> hits;
        bvh.piercingRaycast(ray, hits, mask);
        const auto foundIds = toVectorIds(hits);
        CHECK(foundIds.size() == 4);
        for (uint32_t i = 2; i <= 5; ++i)
            CHECK(std::find(foundIds.begin(), foundIds.end(), i) != foundIds.end());
    }
}
