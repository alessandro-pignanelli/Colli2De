#include <algorithm>
#include <cstdint>
#include <set>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "bvh/dynamicBVH.hpp"
#include "geometry/AABB.hpp"

using namespace c2d;
using namespace Catch;

TEST_CASE("DynamicBVH::query finds all overlapping proxies", "[DynamicBVH][Query]")
{
    const float margin = 0.1f;
    DynamicBVH<uint32_t> bvh(margin);

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
            NodeIndex nodeIndex = bvh.createProxy(aabb, customId);
            if (isExpectedOverlap)
                expectedIds.insert(customId);
        }
    }

    std::vector<uint32_t> foundIds;
    
    // Query an area covering (2,2) to (4,4)
    AABB queryAABB{Vec2{2.0f, 2.0f}, Vec2{4.0f, 4.0f}};
    foundIds = bvh.query(queryAABB);

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
    AABB nonOverlappingQuery{Vec2{6.5f, 6.5f}, Vec2{7.5f, 7.5f}};
    foundIds = bvh.query(nonOverlappingQuery);

    // Should find no ids
    CHECK(foundIds.empty());

    // Query an area that overlaps with a single proxy
    AABB singleOverlapQuery{Vec2{3.2f, 3.2f}, Vec2{3.8f, 3.8f}};
    foundIds = bvh.query(singleOverlapQuery);

    // Should find exactly one id
    CHECK(foundIds.size() == 1);
    CHECK(foundIds[0] == 21); // The proxy with AABB{(3, 3), (4, 4)}

    // Query an area that overlaps with every proxy
    foundIds.clear();
    AABB fullOverlapQuery{Vec2{0.0f, 0.0f}, Vec2{6.0f, 6.0f}};
    foundIds = bvh.query(fullOverlapQuery);

    // Should find all 36 ids
    CHECK(foundIds.size() == gridSize * gridSize);
}

TEST_CASE("DynamicBVH::piercingRaycast finds intersected proxies", "[DynamicBVH][PiercingRaycast]")
{
    const float margin = 0.0f;
    DynamicBVH<uint32_t> bvh(0.1f);

    // Insert a row of 10 AABBs at y = 0..1, x = 0..10
    for (uint32_t i = 0; i < 10; ++i)
        bvh.createProxy(AABB{ Vec2{float(i), 0.0f }, Vec2{float(i + 1), 1.0f } }, i);

    // Ray from (-1,0.5) to (11,0.5) passes through all AABBs
    Ray ray{ Vec2{-1.0f, 0.5f }, Vec2{11.0f, 0.5f } };
    std::vector<uint32_t> foundIds;
    foundIds = bvh.piercingRaycast(ray);

    CHECK(foundIds.size() == 10);
    for (uint32_t i = 0; i < 10; ++i)
        CHECK(std::find(foundIds.begin(), foundIds.end(), i) != foundIds.end());

    // Ray that does not intersect any AABBs
    Ray nonIntersectingRay{ Vec2{12.0f, 0.5f }, Vec2{13.0f, 0.5f } };
    foundIds = bvh.piercingRaycast(nonIntersectingRay);
    CHECK(foundIds.empty());

    // Ray that intersects only two AABB
    Ray singleIntersectionRay{ Vec2{5.5f, 0.5f }, Vec2{6.5f, 0.5f } };
    foundIds = bvh.piercingRaycast(singleIntersectionRay);
    CHECK(foundIds.size() == 2);
    CHECK(std::find(foundIds.begin(), foundIds.end(), 5) != foundIds.end());
    CHECK(std::find(foundIds.begin(), foundIds.end(), 6) != foundIds.end());
}

TEST_CASE("DynamicBVH::firstHitRaycast finds the nearest hit", "[DynamicBVH][FirstHitRaycast]")
{
    const float margin = 0.0f;
    DynamicBVH<uint32_t> bvh(margin);

    // Insert 3 boxes at (0,0)-(1,1), (2,0)-(3,1), (4,0)-(5,1)
    bvh.createProxy({Vec2{0,0}, Vec2{1,1}}, 1);
    bvh.createProxy({Vec2{2,0}, Vec2{3,1}}, 2);
    bvh.createProxy({Vec2{4,0}, Vec2{5,1}}, 3);

    // Ray from (-1,0.5) to (6,0.5) should hit ID=1 first
    Ray ray{Vec2{-1,0.5f}, Vec2{6,0.5f}};
    auto firstId = bvh.firstHitRaycast(ray);
    REQUIRE(firstId);
    CHECK(*firstId == 1);

    // Ray that does not hit any boxes
    Ray noHitRay{Vec2{7,0.5f}, Vec2{8,0.5f}};
    firstId = bvh.firstHitRaycast(noHitRay);
    CHECK(!firstId);

    // Ray that hits only the second box
    Ray singleHitRay{Vec2{1.5f,0.5f}, Vec2{3.5f,0.5f}};
    firstId = bvh.firstHitRaycast(singleHitRay);
    REQUIRE(firstId);
    CHECK(*firstId == 2);
}

TEST_CASE("DynamicBVH::piercingRaycastDetailed finds all hits with entry/exit points", "[DynamicBVH][PiercingRaycast]")
{
    const float margin = 0.0f;
    DynamicBVH<uint32_t> bvh(margin);

    bvh.createProxy({Vec2{0,0 }, Vec2{1,1 } }, 10);
    bvh.createProxy({Vec2{2,0 }, Vec2{3,1 } }, 20);
    bvh.createProxy({Vec2{4,0 }, Vec2{5,1 } }, 30);

    // Ray passes through all three boxes
    Ray ray{Vec2{-1,0.5f}, Vec2{6,0.5f}};
    auto hits = bvh.piercingRaycastDetailed(ray);

    CHECK(hits.size() == 3);
    for (const auto& hit : hits) {
        if (hit.id == 10) {
            CHECK(hit.entry.x == Approx(0.0f));
            CHECK(hit.exit.x  == Approx(1.0f));
            CHECK(hit.entry.y == Approx(0.5f));
            CHECK(hit.exit.y  == Approx(0.5f));
        } else if (hit.id == 20) {
            CHECK(hit.entry.x == Approx(2.0f));
            CHECK(hit.exit.x  == Approx(3.0f));
            CHECK(hit.entry.y == Approx(0.5f));
            CHECK(hit.exit.y  == Approx(0.5f));
        } else if (hit.id == 30) {
            CHECK(hit.entry.x == Approx(4.0f));
            CHECK(hit.exit.x  == Approx(5.0f));
            CHECK(hit.entry.y == Approx(0.5f));
            CHECK(hit.exit.y  == Approx(0.5f));
        } else {
            FAIL("Unexpected hit ID");
        }
    }

    // Ray that does not hit any boxes
    Ray noHitRay{ Vec2{7,0.5f }, Vec2{8,0.5f } };
    hits = bvh.piercingRaycastDetailed(noHitRay);

    CHECK(hits.empty());

    // Ray that hits only the second box
    Ray singleHitRay{ Vec2{1.5f,0.5f }, Vec2{3.5f,0.5f } };
    hits = bvh.piercingRaycastDetailed(singleHitRay);

    CHECK(hits.size() == 1);
    CHECK(hits[0].id == 20);
    CHECK(hits[0].entry.x == Approx(2.0f));
    CHECK(hits[0].exit.x  == Approx(3.0f));
    CHECK(hits[0].entry.y == Approx(0.5f));
}

TEST_CASE("DynamicBVH::firstHitRaycastDetailed returns id, entry, and exit for closest hit", "[DynamicBVH][FirstHitRaycastDetailed]")
{
    const float margin = 0.0f;
    DynamicBVH<uint32_t> bvh(margin);

    bvh.createProxy({Vec2{1,1 }, Vec2{2,2 } }, 101);
    bvh.createProxy({Vec2{3,1 }, Vec2{4,2 } }, 102);
    bvh.createProxy({Vec2{5,1 }, Vec2{6,2 } }, 103);

    // Ray from (0, 1.5) to (5, 1.5) hits 101 first, then 102
    Ray ray{Vec2{0,1.5f}, Vec2{5,1.5f}};
    auto hit = bvh.firstHitRaycastDetailed(ray);

    REQUIRE(hit);
    CHECK(hit->id == 101);
    CHECK(hit->entry.x == Approx(1.0f));
    CHECK(hit->exit.x  == Approx(2.0f));
    CHECK(hit->entry.y == Approx(1.5f));
    CHECK(hit->exit.y  == Approx(1.5f));

    // Ray that does not hit any boxes
    Ray noHitRay{ Vec2{7,1.5f }, Vec2{8,1.5f } };
    hit = bvh.firstHitRaycastDetailed(noHitRay);
    CHECK(!hit); // Should return no hit

    // Ray that hits only the second box
    Ray singleHitRay{ Vec2{2.5f,1.5f }, Vec2{3.5f,1.5f } };
    hit = bvh.firstHitRaycastDetailed(singleHitRay);
    REQUIRE(hit);
    CHECK(hit->id == 102);
    CHECK(hit->entry.x == Approx(3.0f));
    CHECK(hit->exit.x  == Approx(3.5f)); // Ray does not fully exit the box
    CHECK(hit->entry.y == Approx(1.5f));
    CHECK(hit->exit.y  == Approx(1.5f));

    // Backwards ray that hits all boxes
    Ray backwardsRay{ Vec2{6,1.5f }, Vec2{0,1.5f } };
    hit = bvh.firstHitRaycastDetailed(backwardsRay);
    REQUIRE(hit);
    CHECK(hit->id == 103);
    CHECK(hit->entry.x == Approx(6.0f));
    CHECK(hit->exit.x  == Approx(5.0f));
    CHECK(hit->entry.y == Approx(1.5f));
    CHECK(hit->exit.y  == Approx(1.5f));
}

TEST_CASE("DynamicBVH: findBroadPhaseCollisions finds correct pairs", "[DynamicBVH][BroadPhaseCollisions]")
{
    DynamicBVH<uint32_t> bvh;

    // Four AABBs:
    // 0: (0,0)-(2,2)
    // 1: (1,1)-(3,3)   (overlaps with 0)
    // 2: (4,4)-(5,5)   (no overlap)
    // 3: (1.5,1.5)-(2.5,2.5) (overlaps with 0 and 1)
    bvh.createProxy({Vec2{0,0}, Vec2{2,2}}, 0);
    bvh.createProxy({Vec2{1,1}, Vec2{3,3}}, 1);
    bvh.createProxy({Vec2{4,4}, Vec2{5,5}}, 2);
    bvh.createProxy({Vec2{1.5f,1.5f}, Vec2{2.5f,2.5f}}, 3);

    const auto pairs = bvh.findBroadPhaseCollisions();

    // Build a set for easy checking
    std::set<std::pair<uint32_t, uint32_t>> found;
    for (auto p : pairs)
    {
        // Order doesn't matter (but avoid duplicate pairs)
        if (p.first > p.second)
            std::swap(p.first, p.second);
        found.insert(p);

        std::cout << "Found pair: (" << p.first << ", " << p.second << ")\n";
    }

    // Should find (0,1), (0,3), (1,3)
    std::set<std::pair<uint32_t, uint32_t>> expected {
        {0,1}, {0,3}, {1,3}
    };

    REQUIRE(found.size() == expected.size());
    for (const auto& e : expected)
    {
        CHECK(found.find(e) != found.end());
    }
}

