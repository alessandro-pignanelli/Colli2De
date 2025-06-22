#include <algorithm>
#include <cstdint>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "bvh/dynamicBVH.hpp"
#include "geometry/AABB.hpp"

using namespace c2d;
using namespace Catch;

TEST_CASE("DynamicBVH::query handles degenerate and thin AABBs", "[DynamicBVH][Advanced][Degenerate]")
{
    constexpr float margin = 0.0f;
    DynamicBVH<uint32_t> bvh(margin);

    // Zero-width box
    bvh.createProxy({Vec2{5,5}, Vec2{5,10}}, 1);
    // Zero-height box
    bvh.createProxy({Vec2{1,3}, Vec2{4,3}}, 2);
    // Normal box
    bvh.createProxy({Vec2{2,2}, Vec2{4,4}}, 3);

    // Query covering all
    AABB query{Vec2{0,0}, Vec2{6,11}};
    std::vector<uint32_t> hits;
    hits = bvh.query(query);

    REQUIRE(hits.size() == 3);
    REQUIRE(std::find(hits.begin(), hits.end(), 1) != hits.end());
    REQUIRE(std::find(hits.begin(), hits.end(), 2) != hits.end());
    REQUIRE(std::find(hits.begin(), hits.end(), 3) != hits.end());
}

TEST_CASE("DynamicBVH::raycastFirstHit detects grazing rays", "[DynamicBVH][Advanced][RayGrazing]")
{
    DynamicBVH<uint32_t> bvh;
    bvh.createProxy({Vec2{2,2}, Vec2{4,4}}, 1);

    // Ray just touches the lower-left corner
    Ray ray1{Vec2{0,0}, Vec2{2,2}};
    auto hit1 = bvh.firstHitRaycast(ray1);
    REQUIRE(hit1.has_value());
    REQUIRE(hit1.value() == 1);

    // Ray along the bottom edge
    Ray ray2{Vec2{2,2}, Vec2{4,2}};
    auto hit2 = bvh.firstHitRaycast(ray2);
    REQUIRE(hit2.has_value());
    REQUIRE(hit2.value() == 1);
}
