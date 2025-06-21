#include <algorithm>
#include <cstdint>
#include <set>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "Random.hpp"
#include "geometry/AABB.hpp"

using namespace c2d;
using namespace Catch;

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
    CHECK(perimeter == Approx(10.0f));

    aabb = AABB {Vec2{-1, -1}, Vec2{1, 2}};
    perimeter = aabb.perimeter();
    CHECK(perimeter == Approx(10.0f));

    aabb = AABB {Vec2{3, 12}, Vec2{4, 20}};
    perimeter = aabb.perimeter();
    CHECK(perimeter == Approx(18.0f));
}

TEST_CASE("AABB::fattened expands both min and max by margin", "[AABB][Fattened]") 
{
    AABB aabb{ Vec2{1.0f, 2.0f}, Vec2{4.0f, 6.0f} };
    float margin = 0.5f;
    AABB fat = aabb.fattened(margin);

    REQUIRE(fat.min.x == Approx(0.5f));
    REQUIRE(fat.min.y == Approx(1.5f));
    REQUIRE(fat.max.x == Approx(4.5f));
    REQUIRE(fat.max.y == Approx(6.5f));
}

TEST_CASE("AABB::contains checks complete containment", "[AABB][Fattened][Contains]") 
{
    // Outer is a fattened box, inner is original
    AABB outer{ Vec2{0.0f, 0.0f}, Vec2{5.0f, 5.0f} };
    AABB inner{ Vec2{1.0f, 1.0f}, Vec2{3.0f, 3.0f} };

    REQUIRE(outer.contains(inner));
    REQUIRE_FALSE(inner.contains(outer));

    // Identical AABBs: should contain
    REQUIRE(inner.contains(inner));

    // Slightly outside
    AABB justOutside{ Vec2{0.0f, 0.0f}, Vec2{5.01f, 5.0f} };
    REQUIRE_FALSE(outer.contains(justOutside));
}

TEST_CASE("AABB::fatten with displacement", "[AABB][Fattened][Displacement]")
{
    const float margin = 3.0f;
    AABB aabb{Vec2{1.0f, 2.0f }, Vec2{4.0f, 6.0f } };

    // No displacement
    Vec2 d5{0.0f, 0.0f };
    AABB fat5 = aabb.fattened(margin, d5);
    CHECK(fat5.min.x == aabb.min.x - margin);
    CHECK(fat5.max.x == aabb.max.x + margin);
    CHECK(fat5.min.y == aabb.min.y - margin);
    CHECK(fat5.max.y == aabb.max.y + margin);

    // Rightward displacement
    Vec2 d1{2.0f, 0.0f };
    AABB fat1 = aabb.fattened(margin, d1);
    // min x stays (1.0 - 0.5), max x is 4.0 + 2.0 + 0.5
    CHECK(fat1.min.x == aabb.min.x - margin);
    CHECK(fat1.max.x == aabb.max.x + margin + d1.x);
    CHECK(fat1.min.y == aabb.min.y - margin);
    CHECK(fat1.max.y == aabb.max.y + margin);

    // Leftward displacement
    Vec2 d2{-2.0f, 0.0f };
    AABB fat2 = aabb.fattened(margin, d2);
    // min x is 1.0 - 2.0 - 0.5 = -1.5, max x is 4.0 + 0.5 = 4.5
    CHECK(fat2.min.x == aabb.min.x - margin + d2.x);
    CHECK(fat2.max.x == aabb.max.x + margin);
    CHECK(fat2.min.y == aabb.min.y - margin);
    CHECK(fat2.max.y == aabb.max.y + margin);

    // Upward displacement
    Vec2 d3{0.0f, 1.5f };
    AABB fat3 = aabb.fattened(margin, d3);
    CHECK(fat3.min.x == aabb.min.x - margin);
    CHECK(fat3.max.x == aabb.max.x + margin);
    CHECK(fat3.min.y == aabb.min.y - margin);
    CHECK(fat3.max.y == aabb.max.y + margin + d3.y);

    // Downward displacement
    Vec2 d4{0.0f, -1.5f };
    AABB fat4 = aabb.fattened(margin, d4);
    CHECK(fat4.min.x == aabb.min.x - margin);
    CHECK(fat4.max.x == aabb.max.x + margin);
    CHECK(fat4.min.y == aabb.min.y - margin + d4.y);
    CHECK(fat4.max.y == aabb.max.y + margin);

    // Mixed displacement
    Vec2 d6{1.0f, -2.0f };
    AABB fat6 = aabb.fattened(margin, d6);
    CHECK(fat6.min.x == aabb.min.x - margin);
    CHECK(fat6.max.x == aabb.max.x + margin + d6.x);
    CHECK(fat6.min.y == aabb.min.y - margin + d6.y);
    CHECK(fat6.max.y == aabb.max.y + margin);
}
