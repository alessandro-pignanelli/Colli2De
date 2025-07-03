#include <algorithm>
#include <colli2de/Ray.hpp>
#include <cstdint>
#include <set>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "data_structures/BroadPhaseTree.hpp"
#include "geometry/AABB.hpp"
#include "utils/Print.hpp"

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

TEST_CASE("BroadPhaseTree::query finds all overlapping proxies", "[BroadPhaseTree][Query]")
{
    BroadPhaseTree<uint32_t> tree;

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
            tree.addProxy(customId, aabb);
            if (isExpectedOverlap)
                expectedIds.insert(customId);
        }
    }

    std::set<uint32_t> foundIds;

    // Query an area covering (2,2) to (4,4)
    AABB queryAABB{Vec2{2.0f, 2.0f}, Vec2{4.0f, 4.0f}};
    foundIds = tree.query(queryAABB);

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
    foundIds.clear();
    foundIds = tree.query(nonOverlappingQuery);

    // Should find no ids
    CHECK(foundIds.empty());

    // Query an area that overlaps with a single proxy
    AABB singleOverlapQuery{Vec2{3.2f, 3.2f}, Vec2{3.8f, 3.8f}};
    foundIds.clear();
    foundIds = tree.query(singleOverlapQuery);

    // Should find exactly one id
    CHECK(foundIds.size() == 1);
    CHECK(foundIds.count(21) == 1); // The proxy with AABB{(3, 3), (4, 4)}

    // Query an area that overlaps with every proxy
    AABB fullOverlapQuery{Vec2{0.0f, 0.0f}, Vec2{6.0f, 6.0f}};
    foundIds.clear();
    foundIds = tree.query(fullOverlapQuery);

    // Should find all 36 ids
    CHECK(foundIds.size() == gridSize * gridSize);
}

TEST_CASE("BroadPhaseTree::piercingRaycast finds intersected proxies", "[BroadPhaseTree][PiercingRaycast]")
{
    BroadPhaseTree<uint32_t> tree;

    // Insert a row of 10 AABBs at y = 0..1, x = 0..10
    for (uint32_t i = 0; i < 10; ++i)
        tree.addProxy(i, AABB{ Vec2{float(i), 0.0f }, Vec2{float(i + 1), 1.0f } });

    // Ray from (-1,0.5) to (11,0.5) passes through all AABBs
    Ray ray{ Vec2{-1.0f, 0.5f }, Vec2{11.0f, 0.5f } };
    std::vector<uint32_t> foundIds;
    std::set<BPTRaycastInfo> hits;
    hits = tree.piercingRaycast(ray);
    foundIds = toVectorIds(hits);

    CHECK(foundIds.size() == 10);
    for (uint32_t i = 0; i < 10; ++i)
        CHECK(std::find(foundIds.begin(), foundIds.end(), i) != foundIds.end());

    // Ray that does not intersect any AABBs
    Ray nonIntersectingRay{ Vec2{12.0f, 0.5f }, Vec2{13.0f, 0.5f } };
    hits.clear();
    hits = tree.piercingRaycast(nonIntersectingRay);
    CHECK(hits.empty());

    // Ray that intersects only two AABB
    Ray singleIntersectionRay{ Vec2{5.5f, 0.5f }, Vec2{6.5f, 0.5f } };
    hits.clear();
    hits = tree.piercingRaycast(singleIntersectionRay);
    foundIds = toVectorIds(hits);
    CHECK(hits.size() == 2);
    CHECK(std::find(foundIds.begin(), foundIds.end(), 5) != foundIds.end());
    CHECK(std::find(foundIds.begin(), foundIds.end(), 6) != foundIds.end());
}

TEST_CASE("BroadPhaseTree::firstHitRaycast finds the nearest hit", "[BroadPhaseTree][FirstHitRaycast]")
{
    BroadPhaseTree<uint32_t> tree;

    // Insert 3 boxes at (0,0)-(1,1), (2,0)-(3,1), (4,0)-(5,1)
    tree.addProxy(1, {Vec2{0,0}, Vec2{1,1}});
    tree.addProxy(2, {Vec2{2,0}, Vec2{3,1}});
    tree.addProxy(3, {Vec2{4,0}, Vec2{5,1}});

    // Ray from (-1,0.5) to (6,0.5) should hit ID=1 first
    Ray ray{Vec2{-1,0.5f}, Vec2{6,0.5f}};
    auto firstHit = tree.firstHitRaycast(ray);
    REQUIRE(firstHit);
    CHECK(firstHit->id == 1);

    // Ray that does not hit any boxes
    Ray noHitRay{Vec2{7,0.5f}, Vec2{8,0.5f}};
    firstHit = tree.firstHitRaycast(noHitRay);
    CHECK(!firstHit);

    // Ray that hits only the second box
    Ray singleHitRay{Vec2{1.5f,0.5f}, Vec2{3.5f,0.5f}};
    firstHit = tree.firstHitRaycast(singleHitRay);
    REQUIRE(firstHit);
    CHECK(firstHit->id == 2);
}

TEST_CASE("BroadPhaseTree::piercingRaycast finds all hits with entry/exit points", "[BroadPhaseTree][PiercingRaycast]")
{
    BroadPhaseTree<uint32_t> tree;

    tree.addProxy(10, {Vec2{0,0 }, Vec2{1,1 } });
    tree.addProxy(20, {Vec2{2,0 }, Vec2{3,1 } });
    tree.addProxy(30, {Vec2{4,0 }, Vec2{5,1 } });

    // Ray passes through all three boxes
    Ray ray{Vec2{-1,0.5f}, Vec2{6,0.5f}};
    std::set<BPTRaycastInfo> hits;
    hits = tree.piercingRaycast(ray);

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
    hits.clear();
    hits = tree.piercingRaycast(noHitRay);

    CHECK(hits.empty());

    // Ray that hits only the second box
    Ray singleHitRay{ Vec2{1.5f,0.5f }, Vec2{3.5f,0.5f } };
    hits.clear();
    hits = tree.piercingRaycast(singleHitRay);

    CHECK(hits.size() == 1);
    CHECK(hits.begin()->id == 20);
    CHECK(hits.begin()->entry.x == Approx(2.0f));
    CHECK(hits.begin()->exit.x  == Approx(3.0f));
    CHECK(hits.begin()->entry.y == Approx(0.5f));
}

TEST_CASE("BroadPhaseTree::firstHitRaycast returns id, entry, and exit for closest hit", "[BroadPhaseTree][FirstHitRaycast]")
{    BroadPhaseTree<uint32_t> tree;

    tree.addProxy(101, {Vec2{1,1 }, Vec2{2,2 } });
    tree.addProxy(102, {Vec2{3,1 }, Vec2{4,2 } });
    tree.addProxy(103, {Vec2{5,1 }, Vec2{6,2 } });

    // Ray from (0, 1.5) to (5, 1.5) hits 101 first, then 102
    Ray ray{Vec2{0,1.5f}, Vec2{5,1.5f}};
    auto hit = tree.firstHitRaycast(ray);

    REQUIRE(hit);
    CHECK(hit->id == 101);
    CHECK(hit->entry.x == Approx(1.0f));
    CHECK(hit->exit.x  == Approx(2.0f));
    CHECK(hit->entry.y == Approx(1.5f));
    CHECK(hit->exit.y  == Approx(1.5f));

    // Ray that does not hit any boxes
    Ray noHitRay{ Vec2{7,1.5f }, Vec2{8,1.5f } };
    hit = tree.firstHitRaycast(noHitRay);
    CHECK(!hit); // Should return no hit

    // Ray that hits only the second box
    Ray singleHitRay{ Vec2{2.5f,1.5f }, Vec2{3.5f,1.5f } };
    hit = tree.firstHitRaycast(singleHitRay);
    REQUIRE(hit);
    CHECK(hit->id == 102);
    CHECK(hit->entry.x == Approx(3.0f));
    CHECK(hit->exit.x  == Approx(3.5f)); // Ray does not fully exit the box
    CHECK(hit->entry.y == Approx(1.5f));
    CHECK(hit->exit.y  == Approx(1.5f));

    // Backwards ray that hits all boxes
    Ray backwardsRay{ Vec2{6,1.5f }, Vec2{0,1.5f } };
    hit = tree.firstHitRaycast(backwardsRay);
    REQUIRE(hit);
    CHECK(hit->id == 103);
    CHECK(hit->entry.x == Approx(6.0f));
    CHECK(hit->exit.x  == Approx(5.0f));
    CHECK(hit->entry.y == Approx(1.5f));
    CHECK(hit->exit.y  == Approx(1.5f));
}

TEST_CASE("BroadPhaseTree::piercingRaycast with infinite ray finds intersected proxies", "[BroadPhaseTree][PiercingRaycast]")
{    BroadPhaseTree<uint32_t> tree;

    // Insert a row of 10 AABBs at y = 0..1, x = 0..10
    for (uint32_t i = 0; i < 10; ++i)
        tree.addProxy(i, AABB{ Vec2{float(i), 0.0f}, Vec2{float(i + 1), 1.0f} });

    // Ray from (-1,0.5) to (+inf,0.5) passes through all AABBs
    InfiniteRay ray{ Vec2{-1.0f, 0.5f}, Vec2{1.0f, 0.0f} };
    std::vector<uint32_t> foundIds;
    std::set<BPTRaycastInfo> hits;
    hits = tree.piercingRaycast(ray);
    foundIds = toVectorIds(hits);

    CHECK(foundIds.size() == 10);
    for (uint32_t i = 0; i < 10; ++i)
        CHECK(std::find(foundIds.begin(), foundIds.end(), i) != foundIds.end());

    // Ray that does not intersect any AABBs
    InfiniteRay nonIntersectingRay{ Vec2{12.0f, 0.5f}, Vec2{1.0f, 0.0f} };
    hits.clear();
    hits = tree.piercingRaycast(nonIntersectingRay);
    foundIds = toVectorIds(hits);
    CHECK(foundIds.empty());

    // Ray that intersects only half AABB
    InfiniteRay halfIntersectionsRay{ Vec2{5.5f, 0.5f}, Vec2{1.0f, 0.0f} };
    hits.clear();
    hits = tree.piercingRaycast(halfIntersectionsRay);
    foundIds = toVectorIds(hits);
    CHECK(foundIds.size() == 5);
    for (uint32_t i = 5; i < 10; ++i)
        CHECK(std::find(foundIds.begin(), foundIds.end(), i) != foundIds.end());

    // Ray that intersects only one AABB
    InfiniteRay oneIntersectionRay{ Vec2{9.5f, 0.5f}, Vec2{1.0f, 0.0f} };
    hits.clear();
    hits = tree.piercingRaycast(oneIntersectionRay);
    foundIds = toVectorIds(hits);
    REQUIRE(foundIds.size() == 1);
    CHECK(foundIds[0] == 9); // The last AABB at (9,0) to (10,1)

    // Backwards ray that hits all boxes
    InfiniteRay backwardsRay{ Vec2{11.0f, 0.5f}, Vec2{-1.0f, 0.0f} };
    hits.clear();
    hits = tree.piercingRaycast(backwardsRay);
    foundIds = toVectorIds(hits);
    CHECK(foundIds.size() == 10);
    for (uint32_t i = 0; i < 10; ++i)
        CHECK(std::find(foundIds.begin(), foundIds.end(), i) != foundIds.end());
}

TEST_CASE("BroadPhaseTree::firstHitRaycast with infinite ray finds the nearest hit", "[BroadPhaseTree][FirstHitRaycast]")
{    BroadPhaseTree<uint32_t> tree;

    // Insert a row of 10 AABBs at y = 0..1, x = 0..10
    for (uint32_t i = 0; i < 10; ++i)
        tree.addProxy(i, AABB{ Vec2{float(i), 0.0f}, Vec2{float(i + 1), 1.0f} });

    // Ray from (-1,0.5) to (+inf,0.5) passes through all AABBs
    InfiniteRay ray{ Vec2{-1.0f, 0.5f}, Vec2{1.0f, 0.0f} };
    std::optional<BPTRaycastInfo> firstHit;
    firstHit = tree.firstHitRaycast(ray);

    REQUIRE(firstHit);
    CHECK(firstHit->id == 0); // The first AABB at (0,0) to (1,1)

    // Ray that does not intersect any AABBs
    InfiniteRay nonIntersectingRay{ Vec2{12.0f, 0.5f}, Vec2{1.0f, 0.0f} };
    firstHit = tree.firstHitRaycast(nonIntersectingRay);
    CHECK(!firstHit); // Should return no hit

    // Ray that intersects only half AABB
    InfiniteRay halfIntersectionsRay{ Vec2{5.5f, 0.5f}, Vec2{1.0f, 0.0f} };
    firstHit = tree.firstHitRaycast(halfIntersectionsRay);
    REQUIRE(firstHit);
    CHECK(firstHit->id == 5); // The first AABB at (5,0) to (6,1)

    // Ray that intersects only one AABB
    InfiniteRay oneIntersectionRay{ Vec2{9.5f, 0.5f}, Vec2{1.0f, 0.0f} };
    firstHit = tree.firstHitRaycast(oneIntersectionRay);
    REQUIRE(firstHit);
    CHECK(firstHit->id == 9); // The last AABB at (9,0) to (10,1)

    // Backwards ray that hits all boxes
    InfiniteRay backwardsRay{ Vec2{11.0f, 0.5f}, Vec2{-1.0f, 0.0f} };
    firstHit = tree.firstHitRaycast(backwardsRay);
    REQUIRE(firstHit);
    CHECK(firstHit->id == 9); // The last AABB at (9,0) to (10,1)
}

TEST_CASE("BroadPhaseTree::piercingRaycast with infinite ray finds all hits with entry/exit points", "[BroadPhaseTree][PiercingRaycast]")
{    BroadPhaseTree<uint32_t> tree;

    // Insert a row of 10 AABBs at y = 0..1, x = 0..10
    for (uint32_t i = 0; i < 10; ++i)
        tree.addProxy(i, AABB{ Vec2{float(i), 0.0f}, Vec2{float(i + 1), 1.0f} });

    // Ray from (-1,0.5) to (+inf,0.5) passes through all AABBs
    InfiniteRay ray{ Vec2{-1.0f, 0.5f}, Vec2{1.0f, 0.0f} };
    std::set<BPTRaycastInfo> hits;
    hits = tree.piercingRaycast(ray);

    REQUIRE(hits.size() == 10);
    for (const auto& hit : hits)
    {
        CHECK(hit.id < 10);
        CHECK(hit.entry.x == Approx(static_cast<float>(hit.id)));
        CHECK(hit.exit.x  == Approx(static_cast<float>(hit.id + 1)));
        CHECK(hit.entry.y == Approx(0.5f));
        CHECK(hit.exit.y  == Approx(0.5f));
    }

    // Ray that does not intersect any AABBs
    InfiniteRay nonIntersectingRay{ Vec2{12.0f, 0.5f}, Vec2{1.0f, 0.0f} };
    hits.clear();
    hits = tree.piercingRaycast(nonIntersectingRay);
    CHECK(hits.empty());

    // Ray that intersects only half AABB
    InfiniteRay halfIntersectionsRay{ Vec2{5.5f, 0.5f}, Vec2{1.0f, 0.0f} };
    hits.clear();
    hits = tree.piercingRaycast(halfIntersectionsRay);
    REQUIRE(hits.size() == 5);
    for (const auto& hit : hits)
    {
        CHECK(hit.id >= 5);
        CHECK(hit.id < 10);
        CHECK(hit.entry.x == Approx(std::max(static_cast<float>(hit.id), halfIntersectionsRay.start.x)));
        CHECK(hit.exit.x  == Approx(static_cast<float>(hit.id + 1)));
        CHECK(hit.entry.y == Approx(0.5f));
        CHECK(hit.exit.y  == Approx(0.5f));
    }

    // Ray that intersects only one AABB
    InfiniteRay oneIntersectionRay{ Vec2{9.5f, 0.5f}, Vec2{1.0f, 0.0f} };
    hits.clear();
    hits = tree.piercingRaycast(oneIntersectionRay);
    REQUIRE(hits.size() == 1);
    CHECK(hits.begin()->id == 9); // The last AABB at (9,0) to (10,1)
    CHECK(hits.begin()->entry.x == Approx(9.5f));
    CHECK(hits.begin()->exit.x  == Approx(10.0f));
    CHECK(hits.begin()->entry.y == Approx(0.5f));
    CHECK(hits.begin()->exit.y  == Approx(0.5f));

    // Backwards ray that hits all boxes
    InfiniteRay backwardsRay{ Vec2{11.0f, 0.5f}, Vec2{-1.0f, 0.0f} };
    hits.clear();
    hits = tree.piercingRaycast(backwardsRay);
    REQUIRE(hits.size() == 10);

    // Check that all hits are in reverse order
    auto lastHit = hits.begin();
    for (const auto& hit : hits)
    {
        CHECK(hit.id >= 0);
        CHECK(hit.id < 10);
        CHECK(hit.entry.x == Approx(static_cast<float>(hit.id + 1)));
        CHECK(hit.exit.x  == Approx(static_cast<float>(hit.id)));
        CHECK(hit.entry.y == Approx(0.5f));
        CHECK(hit.exit.y  == Approx(0.5f));

        CHECK(hit.id <= lastHit->id);
        CHECK(hit.entryTime >= lastHit->entryTime);
    }
}

TEST_CASE("BroadPhaseTree::firstHitRaycast with infinite ray returns id, entry, and exit for closest hit", "[BroadPhaseTree][FirstHitRaycast]")
{    BroadPhaseTree<uint32_t> tree;

    // Insert a row of 10 AABBs at y = 0..1, x = 0..10
    for (uint32_t i = 0; i < 10; ++i)
        tree.addProxy(i, AABB{ Vec2{float(i), 0.0f}, Vec2{float(i + 1), 1.0f} });

    // Ray from (-1,0.5) to (+inf,0.5) passes through all AABBs
    InfiniteRay ray{ Vec2{-1.0f, 0.5f}, Vec2{1.0f, 0.0f} };
    std::optional<BPTRaycastInfo> hit;
    hit = tree.firstHitRaycast(ray);

    REQUIRE(hit);
    CHECK(hit->id == 0); // The first AABB at (0,0) to (1,1)
    CHECK(hit->entry.x == Approx(0.0f));
    CHECK(hit->entry.y == Approx(0.5f));
    CHECK(hit->exit.x  == Approx(1.0f));
    CHECK(hit->exit.y  == Approx(0.5f));

    // Ray that does not intersect any AABBs
    InfiniteRay nonIntersectingRay{ Vec2{12.0f, 0.5f}, Vec2{1.0f, 0.0f} };
    hit = tree.firstHitRaycast(nonIntersectingRay);
    CHECK(!hit); // Should return no hit

    // Ray that intersects only half AABB
    InfiniteRay halfIntersectionsRay{ Vec2{5.5f, 0.5f}, Vec2{1.0f, 0.0f} };
    hit = tree.firstHitRaycast(halfIntersectionsRay);
    REQUIRE(hit);
    CHECK(hit->id == 5); // The first AABB at (5,0) to (6,1)
    CHECK(hit->entry.x == Approx(5.5f));
    CHECK(hit->entry.y == Approx(0.5f));
    CHECK(hit->exit.x  == Approx(6.0f));
    CHECK(hit->exit.y  == Approx(0.5f));

    // Ray that intersects only one AABB
    InfiniteRay oneIntersectionRay{ Vec2{9.5f, 0.5f}, Vec2{1.0f, 0.0f} };
    hit = tree.firstHitRaycast(oneIntersectionRay);
    REQUIRE(hit);
    CHECK(hit->id == 9); // The last AABB at (9,0) to (10,1)
    CHECK(hit->entry.x == Approx(9.5f));
    CHECK(hit->entry.y == Approx(0.5f));
    CHECK(hit->exit.x  == Approx(10.0f));
    CHECK(hit->exit.y  == Approx(0.5f));

    // Backwards ray that hits all boxes
    InfiniteRay backwardsRay{ Vec2{11.0f, 0.5f}, Vec2{-1.0f, 0.0f} };
    hit = tree.firstHitRaycast(backwardsRay);
    REQUIRE(hit);
    CHECK(hit->id == 9); // The last AABB at (9,0) to (10,1)
    CHECK(hit->entry.x == Approx(10.0f));
    CHECK(hit->entry.y == Approx(0.5f));
    CHECK(hit->exit.x  == Approx(9.0f));
    CHECK(hit->exit.y  == Approx(0.5f));
}

TEST_CASE("BroadPhaseTree: findAllCollisions finds correct pairs", "[BroadPhaseTree][BroadPhaseCollisions]")
{
    BroadPhaseTree<uint32_t> tree;

    // Four AABBs:
    // 0: (0,0)-(2,2)
    // 1: (1,1)-(3,3)   (overlaps with 0)
    // 2: (4,4)-(5,5)   (no overlap)
    // 3: (1.5,1.5)-(2.5,2.5) (overlaps with 0 and 1)
    tree.addProxy(0, {Vec2{0,0}, Vec2{2,2}});
    tree.addProxy(1, {Vec2{1,1}, Vec2{3,3}});
    tree.addProxy(2, {Vec2{4,4}, Vec2{5,5}});
    tree.addProxy(3, {Vec2{1.5f,1.5f}, Vec2{2.5f,2.5f}});

    std::set<std::pair<uint32_t, uint32_t>> pairs = tree.findAllCollisions();

    // Should find (0,1), (0,3), (1,3)
    std::set<std::pair<uint32_t, uint32_t>> expected {
        {0,1}, {0,3}, {1,3}
    };

    REQUIRE(pairs.size() == expected.size());
    for (const auto& e : expected)
    {
        CHECK(pairs.find(e) != pairs.end());
    }
}

TEST_CASE("BroadPhaseTree batchQuery with multiple threads", "[BroadPhaseTree][BatchQuery]")
{
    BroadPhaseTree<uint32_t> tree;
    for (uint32_t i = 0; i < 1000; ++i)
        tree.addProxy(i, {Vec2{float(i), float(i)}, Vec2{float(i+1), float(i+1)}});

    std::vector<AABB> queries;
    std::vector<std::set<uint32_t>> expectedResults;
    for (uint32_t i = 0; i < 20000; ++i)
    {
        queries.push_back({Vec2{float(i * 10), float(i * 10)}, Vec2{float((i + 1) * 10), float((i + 1) * 10)}});
        std::set<uint32_t> hits;
        hits = tree.query(queries.back());
        expectedResults.push_back(std::move(hits));
    }

    auto results = tree.batchQuery(queries, 8);
    CHECK(results.size() == expectedResults.size());

    for (size_t i = 0; i < results.size(); ++i)
    {
        const auto& query = queries[i];
        const auto& hits = results[i];
        const auto& expected = expectedResults[i];

        CHECK(hits == expected);
    }
}
