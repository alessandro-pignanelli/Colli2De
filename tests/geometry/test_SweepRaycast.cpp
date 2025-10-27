#include <colli2de/internal/collision/Collision.hpp>
#include <colli2de/internal/geometry/RaycastShapes.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using namespace c2d;
using namespace Catch;

TEST_CASE("Circle sweep against ray hits", "[Sweep][Raycast][Circle]")
{
    Circle circle{{0.0f, 0.0f}, 1.0f};
    Transform startTransform{Vec2{-2.0f, 0.0f}};
    Transform endTransform{Vec2{2.0f, 0.0f}};
    Ray ray{{0.0f, -2.0f}, {0.0f, 2.0f}};

    const auto result = sweep(circle, startTransform, endTransform, ray);
    REQUIRE(result);
    CHECK(result->first == Approx(0.5f));
    CHECK(result->second == Approx(0.5f));
}

TEST_CASE("Circle sweep against ray miss", "[Sweep][Raycast][Circle]")
{
    Circle circle{{0.0f, 0.0f}, 1.0f};
    Transform startTransform{Vec2{-2.0f, 4.0f}};
    Transform endTransform{Vec2{2.0f, 4.0f}};
    Ray ray{{0.0f, -2.0f}, {0.0f, 2.0f}};

    CHECK_FALSE(sweep(circle, startTransform, endTransform, ray));
}

TEST_CASE("Capsule sweep against infinite ray", "[Sweep][Raycast][Capsule]")
{
    Capsule capsule{Vec2{0.0f, -1.0f}, Vec2{0.0f, 1.0f}, 0.5f};
    Transform startTransform{Vec2{-2.0f, 0.0f}};
    Transform endTransform{Vec2{2.0f, 0.0f}};
    InfiniteRay ray{{0.0f, -2.0f}, {0.0f, 1.0f}};

    const auto result = sweep(capsule, startTransform, endTransform, ray);
    REQUIRE(result);
    CHECK(result->second >= result->first);
}
