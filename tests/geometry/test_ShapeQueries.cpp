#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <colli2de/Shapes.hpp>
#include <colli2de/internal/geometry/ShapesUtils.hpp>

using namespace c2d;
using namespace Catch;

TEST_CASE("Point containment for circle", "[ShapeQueries][Contains]")
{
    Circle circle{ Vec2{0.0f, 0.0f}, 1.0f };
    CHECK(containsPoint(circle, Transform{}, Vec2{0.5f, 0.0f}));
    CHECK_FALSE(containsPoint(circle, Transform{}, Vec2{2.0f, 0.0f}));
}

TEST_CASE("Point containment for capsule", "[ShapeQueries][Contains]")
{
    Capsule capsule{ Vec2{-1.0f, 0.0f}, Vec2{1.0f, 0.0f}, 0.5f };
    CHECK(containsPoint(capsule, Transform{}, Vec2{0.0f, 0.0f}));
    CHECK_FALSE(containsPoint(capsule, Transform{}, Vec2{2.0f, 0.0f}));
}

TEST_CASE("Point containment for segment", "[ShapeQueries][Contains]")
{
    Segment seg{ Vec2{0.0f, 0.0f}, Vec2{2.0f, 0.0f} };
    CHECK(containsPoint(seg, Transform{}, Vec2{1.0f, 0.0f}));
    CHECK_FALSE(containsPoint(seg, Transform{}, Vec2{1.0f, 0.1f}));
}

TEST_CASE("Point containment for polygon", "[ShapeQueries][Contains]")
{
    Polygon poly = makeRectangle(Vec2{0.0f, 0.0f}, 1.0f, 1.0f);
    CHECK(containsPoint(poly, Transform{}, Vec2{0.5f, 0.0f}));
    CHECK_FALSE(containsPoint(poly, Transform{}, Vec2{2.0f, 0.0f}));
}

TEST_CASE("Closest point on segment", "[ShapeQueries][Closest]")
{
    Segment seg{ Vec2{0.0f, 0.0f}, Vec2{2.0f, 0.0f} };
    const Vec2 result = closestPoint(seg, Transform{}, Vec2{1.0f, 1.0f});
    CHECK(result == Vec2{1.0f, 0.0f});
}

TEST_CASE("Distance between circles", "[ShapeQueries][Distance]")
{
    Circle a{ Vec2{0.0f, 0.0f}, 1.0f };
    Circle b{ Vec2{4.0f, 0.0f}, 1.0f };
    const float dist = distance(a, Transform{}, b, Transform{});
    CHECK(dist == Approx(2.0f));
}

TEST_CASE("Distance between circle and capsule", "[ShapeQueries][Distance]")
{
    Circle circle{ Vec2{0.0f, 0.0f}, 1.0f };
    Capsule capsule{ Vec2{4.0f, 0.0f}, Vec2{6.0f, 0.0f}, 0.5f };
    const float dist = distance(circle, Transform{}, capsule, Transform{});
    CHECK(dist == Approx(2.5f));
}

TEST_CASE("Distance between circle and segment", "[ShapeQueries][Distance]")
{
    Circle circle{ Vec2{0.0f, 0.0f}, 1.0f };
    Segment seg{ Vec2{4.0f, 0.0f}, Vec2{6.0f, 0.0f} };
    const float dist = distance(circle, Transform{}, seg, Transform{});
    CHECK(dist == Approx(3.0f));
}

TEST_CASE("Distance between circle and polygon", "[ShapeQueries][Distance]")
{
    Circle circle{ Vec2{0.0f, 0.0f}, 1.0f };
    Polygon poly = makeRectangle(Vec2{4.0f, 0.0f}, 1.0f, 1.0f);
    const float dist = distance(circle, Transform{}, poly, Transform{});
    CHECK(dist == Approx(2.0f));
}

TEST_CASE("Distance between capsules", "[ShapeQueries][Distance]")
{
    Capsule a{ Vec2{0.0f, 0.0f}, Vec2{2.0f, 0.0f}, 0.5f };
    Capsule b{ Vec2{4.0f, 0.0f}, Vec2{6.0f, 0.0f}, 0.5f };
    const float dist = distance(a, Transform{}, b, Transform{});
    CHECK(dist == Approx(1.0f));
}

TEST_CASE("Distance between capsule and segment", "[ShapeQueries][Distance]")
{
    Capsule capsule{ Vec2{0.0f, 0.0f}, Vec2{2.0f, 0.0f}, 0.5f };
    Segment seg{ Vec2{4.0f, 0.0f}, Vec2{6.0f, 0.0f} };
    const float dist = distance(capsule, Transform{}, seg, Transform{});
    CHECK(dist == Approx(1.5f));
}

TEST_CASE("Distance between capsule and polygon", "[ShapeQueries][Distance]")
{
    Capsule capsule{ Vec2{0.0f, 0.0f}, Vec2{2.0f, 0.0f}, 0.5f };
    Polygon poly = makeRectangle(Vec2{4.5f, 0.0f}, 1.0f, 1.0f);
    const float dist = distance(capsule, Transform{}, poly, Transform{});
    CHECK(dist == Approx(1.0f));
}

TEST_CASE("Distance between segments", "[ShapeQueries][Distance]")
{
    Segment a{ Vec2{0.0f, 0.0f}, Vec2{2.0f, 0.0f} };
    Segment b{ Vec2{4.0f, 0.0f}, Vec2{6.0f, 0.0f} };
    const float dist = distance(a, Transform{}, b, Transform{});
    CHECK(dist == Approx(2.0f));
}

TEST_CASE("Distance between segment and polygon", "[ShapeQueries][Distance]")
{
    Segment seg{ Vec2{0.0f, 0.0f}, Vec2{2.0f, 0.0f} };
    Polygon poly = makeRectangle(Vec2{4.5f, 0.0f}, 1.0f, 1.0f);
    const float dist = distance(seg, Transform{}, poly, Transform{});
    CHECK(dist == Approx(1.5f));
}

TEST_CASE("Distance between polygons", "[ShapeQueries][Distance]")
{
    Polygon a = makeRectangle(Vec2{0.0f, 0.0f}, 1.0f, 1.0f);
    Polygon b = makeRectangle(Vec2{3.0f, 0.0f}, 1.0f, 1.0f);
    const float dist = distance(a, Transform{}, b, Transform{});
    CHECK(dist == Approx(1.0f));
}

TEST_CASE("Distance between circle and rectangle", "[ShapeQueries][Distance]")
{
    Circle c{ Vec2{0.0f, 0.0f}, 0.5f };
    Polygon poly = makeRectangle(Vec2{3.0f, 0.0f}, 1.0f, 1.0f);
    const float dist = distance(c, Transform{}, poly, Transform{});
    CHECK(dist == Approx(1.5f));
}

