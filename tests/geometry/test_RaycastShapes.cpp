#include <colli2de/internal/geometry/AABB.hpp>
#include <colli2de/internal/geometry/RaycastShapes.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using namespace c2d;
using namespace Catch;

TEST_CASE("Circle raycast basic hits", "[Raycast][Circle]")
{
    Circle circle{Vec2{1.0f, 1.0f}, 1.0f};
    Ray ray{Vec2{-2.0f, 1.0f}, Vec2{4.0f, 1.0f}};

    const auto result = raycast(circle, Transform{}, ray);
    REQUIRE(result);
    auto [entry, exit] = *result;
    CHECK(entry == Approx(1.0f / 3.0f));
    CHECK(exit == Approx(2.0f / 3.0f));
}

TEST_CASE("Circle raycast starting inside", "[Raycast][Circle]")
{
    Circle circle{Vec2{0.0f, 0.0f}, 1.0f};
    Ray ray{Vec2{0.0f, 0.0f}, Vec2{2.0f, 0.0f}};

    const auto result = raycast(circle, Transform{}, ray);
    REQUIRE(result);
    auto [entry, exit] = *result;
    CHECK(entry == Approx(0.0f));
    CHECK(exit == Approx(0.5f));
}

TEST_CASE("Circle infinite ray miss", "[Raycast][Circle]")
{
    Circle circle{Vec2{0.0f, 0.0f}, 1.0f};
    InfiniteRay ray{Vec2{0.0f, 2.0f}, Vec2{1.0f, 0.0f}};
    REQUIRE_FALSE(raycast(circle, Transform{}, ray));
}

TEST_CASE("Segment finite ray hit", "[Raycast][Segment]")
{
    Segment seg{Vec2{1.0f, 1.0f}, Vec2{3.0f, 1.0f}};
    Ray ray{Vec2{2.0f, 0.0f}, Vec2{2.0f, 2.0f}};

    const auto result = raycast(seg, Transform{}, ray);
    REQUIRE(result);
    CHECK(result->first == Approx(0.5f));
    CHECK(result->second == Approx(0.5f));
}

TEST_CASE("Segment parallel miss", "[Raycast][Segment]")
{
    Segment seg{Vec2{0.0f, 1.0f}, Vec2{3.0f, 1.0f}};
    Ray ray{Vec2{0.0f, 2.0f}, Vec2{3.0f, 2.0f}};
    REQUIRE_FALSE(raycast(seg, Transform{}, ray));
}

TEST_CASE("Polygon raycast through square", "[Raycast][Polygon]")
{
    Polygon poly = makeRectangle(Vec2{1.0f, 1.0f}, 1.0f, 1.0f);
    Ray ray{Vec2{-1.0f, 1.0f}, Vec2{3.0f, 1.0f}};

    const auto result = raycast(poly, Transform{}, ray);
    REQUIRE(result);
    auto [entry, exit] = *result;
    CHECK(entry == Approx(0.25f));
    CHECK(exit == Approx(0.75f));
}

TEST_CASE("Polygon infinite ray starting inside", "[Raycast][Polygon]")
{
    std::array<Vec2, 3> verts{Vec2{0, 0}, Vec2{2, 0}, Vec2{1, 2}};
    Polygon tri{verts, 3};
    InfiniteRay ray{Vec2{1.0f, 1.0f}, Vec2{0.0f, 1.0f}};

    const auto result = raycast(tri, Transform{}, ray);
    REQUIRE(result);
    CHECK(result->first == Approx(0.0f));
    CHECK(result->second > 0.0f);
}

TEST_CASE("Capsule raycast through body", "[Raycast][Capsule]")
{
    Capsule cap{Vec2{0.0f, 0.0f}, Vec2{4.0f, 0.0f}, 1.0f};
    Ray ray{Vec2{-2.0f, 0.0f}, Vec2{6.0f, 0.0f}};

    const auto result = raycast(cap, Transform{}, ray);
    REQUIRE(result);
    auto [entry, exit] = *result;
    CHECK(entry == Approx(0.125f));
    CHECK(exit == Approx(0.875f));
}

TEST_CASE("Capsule infinite ray miss", "[Raycast][Capsule]")
{
    Capsule cap{Vec2{0.0f, 0.0f}, Vec2{4.0f, 0.0f}, 1.0f};
    InfiniteRay ray{Vec2{0.0f, 3.0f}, Vec2{1.0f, 0.0f}};
    REQUIRE_FALSE(raycast(cap, Transform{}, ray));
}
