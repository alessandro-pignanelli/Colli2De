#include <algorithm>
#include <cstdint>
#include <set>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "Random.hpp"
#include "AABB.hpp"

using namespace c2d;

TEST_CASE("AABB Combine", "[AABB]")
{
    // Test overlapping AABBs
    AABB aabb1 {Vec2{0, 0}, Vec2{1, 1}};
    AABB aabb2 {Vec2{0.5f, 0.5f}, Vec2{1.5f, 1.5f}};
    const auto center = aabb1.center();
    AABB combined = AABB::combine(aabb1, aabb2);
    
    CHECK(combined.min == aabb1.min);
    CHECK(combined.max == aabb2.max);

    // Test AABBs with an edge in common
    AABB aabb3 {Vec2{-1, -1}, Vec2{0, 0}};
    AABB aabb4 {Vec2{0, 0}, Vec2{1, 1}};
    combined = AABB::combine(aabb3, aabb4);

    CHECK(combined.min == aabb3.min);
    CHECK(combined.max == aabb4.max);

    // Test AABBs that do not overlap
    AABB aabb5 {Vec2{2, 2}, Vec2{3, 3}};
    AABB aabb6 {Vec2{6, 6}, Vec2{7, 7}};
    combined = AABB::combine(aabb5, aabb6);
    CHECK(combined.min == aabb5.min);
    CHECK(combined.max == aabb6.max);
}

TEST_CASE("AABB Perimeter", "[AABB]")
{
    AABB aabb {Vec2{0, 0}, Vec2{2, 3}};
    float perimeter = aabb.perimeter();
    CHECK(perimeter == Catch::Approx(10.0f));

    aabb = AABB {Vec2{-1, -1}, Vec2{1, 2}};
    perimeter = aabb.perimeter();
    CHECK(perimeter == Catch::Approx(10.0f));

    aabb = AABB {Vec2{3, 12}, Vec2{4, 20}};
    perimeter = aabb.perimeter();
    CHECK(perimeter == Catch::Approx(18.0f));
}
