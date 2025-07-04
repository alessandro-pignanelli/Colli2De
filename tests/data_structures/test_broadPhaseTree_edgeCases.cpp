#include <algorithm>
#include <cstdint>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "data_structures/BroadPhaseTree.hpp"
#include "geometry/AABB.hpp"

using namespace c2d;
using namespace Catch;

TEST_CASE("BroadPhaseTree | query handles degenerate and thin AABBs", "[BroadPhaseTree][Advanced][Degenerate]")
{
    constexpr float margin = 0.0f;
    BroadPhaseTree<uint32_t> tree;

    // Zero-width box
    tree.addProxy(1, {Vec2{5,5}, Vec2{5,10}});
    // Zero-height box
    tree.addProxy(2, {Vec2{1,3}, Vec2{4,3}});
    // Normal box
    tree.addProxy(3, {Vec2{2,2}, Vec2{4,4}});

    // Query covering all
    AABB query{Vec2{0,0}, Vec2{6,11}};
    std::set<uint32_t> hits = tree.query(query);

    REQUIRE(hits.size() == 3);
    REQUIRE(hits.count(1) == 1);
    REQUIRE(hits.count(2) == 1);
    REQUIRE(hits.count(3) == 1);
}

TEST_CASE("BroadPhaseTree | raycastFirstHit detects grazing rays", "[BroadPhaseTree][Advanced][RayGrazing]")
{
    BroadPhaseTree<uint32_t> tree;
    tree.addProxy(1, {Vec2{2,2}, Vec2{4,4}});

    // Ray just touches the lower-left corner
    Ray ray1{Vec2{0,0}, Vec2{2,2}};
    auto hit1 = tree.firstHitRaycast(ray1);
    REQUIRE(hit1.has_value());
    REQUIRE(hit1->id == 1);

    // Ray along the bottom edge
    Ray ray2{Vec2{2,2}, Vec2{4,2}};
    auto hit2 = tree.firstHitRaycast(ray2);
    REQUIRE(hit2.has_value());
    REQUIRE(hit2->id == 1);
}
