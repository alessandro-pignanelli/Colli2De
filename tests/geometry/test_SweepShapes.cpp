#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "collision/Collision.hpp"

using namespace c2d;
using namespace Catch;

TEST_CASE("Circle sweep hits stationary circle", "[Sweep][Circle]")
{
    Circle moving{ Vec2{ -2.0f, 0.0f }, 1.0f };
    Circle target{ Vec2{ 1.0f, 0.0f }, 1.0f };
    const Vec2 translation{ 4.0f, 0.0f };

    const auto result = sweep(moving, Transform{}, Transform{translation, 0}, target, Transform{});
    REQUIRE(result);
    CHECK(result->fraction == Approx(0.25f));
    REQUIRE(result->manifold.pointCount == 1);
    CHECK(result->manifold.normal.x == Approx(1.0f).margin(0.001f));
    CHECK(result->manifold.normal.y == Approx(0.0f).margin(0.001f));
    CHECK(result->manifold.points[0].point == Vec2{ 0.0f, 0.0f });
    CHECK(result->manifold.points[0].separation == Approx(0.0f).margin(0.001f));
}

TEST_CASE("Circle sweep starting overlapped", "[Sweep][Circle]")
{
    Circle moving{ Vec2{ 0.0f, 0.0f }, 1.0f };
    Circle target{ Vec2{ 1.0f, 0.0f }, 1.0f };
    const Vec2 translation{ 2.0f, 0.0f };

    const auto result = sweep(moving, Transform{}, Transform{translation, 0}, target, Transform{});
    REQUIRE(result);
    CHECK(result->fraction == Approx(0.0f));
    REQUIRE(result->manifold.pointCount == 1);
    CHECK(result->manifold.normal.x == Approx(1.0f).margin(0.001f));
    CHECK(result->manifold.normal.y == Approx(0.0f).margin(0.001f));
    CHECK(result->manifold.points[0].separation < 0.0f);
}

TEST_CASE("Circle sweep misses target", "[Sweep][Circle]")
{
    Circle moving{ Vec2{ -2.0f, 0.0f }, 1.0f };
    Circle target{ Vec2{ 1.0f, 0.0f }, 1.0f };
    const Vec2 translation{ -3.0f, 0.0f };

    const auto result = sweep(moving, Transform{}, Transform{translation, 0}, target, Transform{});
    CHECK_FALSE(result);
}

TEST_CASE("Circle sweep vertical motion hits target", "[Sweep][Circle]")
{
    Circle moving{ Vec2{ 0.0f, -2.0f }, 1.0f };
    Circle target{ Vec2{ 0.0f, 1.0f }, 1.0f };
    const Vec2 translation{ 0.0f, 4.0f };

    const auto result = sweep(moving, Transform{}, Transform{translation, 0}, target, Transform{});
    REQUIRE(result);
    CHECK(result->fraction == Approx(0.25f));
    REQUIRE(result->manifold.pointCount == 1);
    CHECK(result->manifold.normal.x == Approx(0.0f).margin(0.001f));
    CHECK(result->manifold.normal.y == Approx(1.0f).margin(0.001f));
    CHECK(result->manifold.points[0].point == Vec2{ 0.0f, 0.0f });
    CHECK(result->manifold.points[0].separation == Approx(0.0f).margin(0.001f));
}

TEST_CASE("Circle sweep diagonal motion hits target", "[Sweep][Circle]")
{
    Circle moving{ Vec2{ -3.0f, -3.0f }, 1.0f };
    Circle target{ Vec2{ 0.0f, 0.0f }, 1.0f };
    const Vec2 translation{ 4.0f, 4.0f };

    const auto result = sweep(moving, Transform{}, Transform{translation, 0}, target, Transform{});
    REQUIRE(result);
    CHECK(result->fraction == Approx(0.3964f).margin(0.001f));
    REQUIRE(result->manifold.pointCount == 1);
    CHECK(result->manifold.normal.x == Approx(std::sqrt(0.5f)).margin(0.001f));
    CHECK(result->manifold.normal.y == Approx(std::sqrt(0.5f)).margin(0.001f));
    CHECK(result->manifold.points[0].separation == Approx(0.0f).margin(0.001f));
}

TEST_CASE("Circle sweep with zero translation and separation", "[Sweep][Circle]")
{
    Circle moving{ Vec2{ -2.0f, 0.0f }, 1.0f };
    Circle target{ Vec2{ 1.0f, 0.0f }, 1.0f };
    const Vec2 translation{ 0.0f, 0.0f };

    const auto result = sweep(moving, Transform{}, Transform{translation, 0}, target, Transform{});
    CHECK_FALSE(result);
}

TEST_CASE("Circle sweep rotates while moving", "[Sweep][Circle]")
{
    Circle moving{ Vec2{ -2.0f, 0.0f }, 1.0f };
    Circle target{ Vec2{ 1.0f, 0.0f }, 1.0f };
    const Vec2 translation{ 4.0f, 0.0f };
    const Rotation endRotation{ static_cast<float>(PI) };

    const auto result = sweep(moving, Transform{},  Transform{translation, endRotation}, target, Transform{});
    REQUIRE(result);
    CHECK(result->fraction == Approx(0.25f));
    REQUIRE(result->manifold.pointCount == 1);
    CHECK(result->manifold.normal.x == Approx(std::sqrt(0.5f)).margin(0.001f));
    CHECK(result->manifold.normal.y == Approx(std::sqrt(0.5f)).margin(0.001f));
}

TEST_CASE("Circle sweep zero translation while overlapping", "[Sweep][Circle]")
{
    Circle moving{ Vec2{ 0.0f, 0.0f }, 1.0f };
    Circle target{ Vec2{ 1.0f, 0.0f }, 1.0f };
    const Vec2 translation{ 0.0f, 0.0f };

    const auto result = sweep(moving, Transform{}, Transform{translation, 0}, target, Transform{});
    REQUIRE(result);
    CHECK(result->fraction == Approx(0.0f));
    REQUIRE(result->manifold.pointCount == 1);
    CHECK(result->manifold.normal.x == Approx(1.0f).margin(0.001f));
    CHECK(result->manifold.normal.y == Approx(0.0f).margin(0.001f));
    CHECK(result->manifold.points[0].separation < 0.0f);
}

TEST_CASE("Circle sweep hits capsule", "[Sweep][Circle][Capsule]")
{
    Circle moving{ Vec2{ 0.0f, 0.0f }, 0.5f };
    Capsule target{ Vec2{ 2.0f, -1.0f }, Vec2{ 2.0f, 1.0f }, 0.5f };
    const Vec2 translation{ 2.0f, 0.0f };

    const auto result = sweep(moving, Transform{}, Transform{translation, 0}, target, Transform{});
    REQUIRE(result);
    CHECK(result->fraction == Approx(0.5f));
    REQUIRE(result->manifold.pointCount == 1);
    CHECK(result->manifold.normal.x == Approx(1.0f).margin(0.001f));
    CHECK(result->manifold.normal.y == Approx(0.0f).margin(0.001f));
    CHECK(result->manifold.points[0].point == Vec2{ 1.5f, 0.0f });
    CHECK(result->manifold.points[0].separation == Approx(0.0f).margin(0.001f));
}

TEST_CASE("Capsule sweep hits circle", "[Sweep][Circle][Capsule]")
{
    Capsule moving{ Vec2{ -2.0f, -1.0f }, Vec2{ -2.0f, 1.0f }, 0.5f };
    Circle target{ Vec2{ 1.0f, 0.0f }, 0.5f };
    const Vec2 translation{ 4.0f, 0.0f };

    const auto result = sweep(moving, Transform{}, Transform{translation, 0}, target, Transform{});
    REQUIRE(result);
    CHECK(result->fraction == Approx(0.5f));
    REQUIRE(result->manifold.pointCount == 1);
    CHECK(result->manifold.normal.x == Approx(1.0f).margin(0.001f));
    CHECK(result->manifold.normal.y == Approx(0.0f).margin(0.001f));
    CHECK(result->manifold.points[0].point == Vec2{ 0.5f, 0.0f });
    CHECK(result->manifold.points[0].separation == Approx(0.0f).margin(0.001f));
}

TEST_CASE("Circle sweep hits rectangle", "[Sweep][Circle][Polygon]")
{
    Circle moving{ Vec2{ -2.0f, 0.0f }, 0.5f };
    Polygon target = makeRectangle(Vec2{ 0.0f, 0.0f }, 1.0f, 1.0f);
    const Vec2 translation{ 3.0f, 0.0f };

    const auto result = sweep(moving, Transform{}, Transform{translation, 0}, target, Transform{});
    REQUIRE(result);
    CHECK(result->fraction == Approx(1.0f / 6.0f).margin(0.001f));
    REQUIRE(result->manifold.pointCount == 1);
    CHECK(result->manifold.normal.x == Approx(1.0f).margin(0.001f));
    CHECK(result->manifold.normal.y == Approx(0.0f).margin(0.001f));
    CHECK(result->manifold.points[0].point.x == Approx(-1.0f).margin(0.001f));
    CHECK(result->manifold.points[0].point.y == Approx(0.0f).margin(0.001f));
    CHECK(result->manifold.points[0].separation == Approx(0.0f).margin(0.001f));
}

