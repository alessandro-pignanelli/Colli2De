#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#if __has_include(<print>)
    #include <print>
    #define print(...) std::print(__VA_ARGS__)
    #define println(...) std::println(__VA_ARGS__)
#else
    #define print(...) ((void)0)
    #define println(...) ((void)0)
#endif

#include "collision/Collision.hpp"
#include "colli2de/Shapes.hpp"
#include "geometry/Transformations.hpp"

using namespace c2d;
using namespace Catch;

template<typename ShapeA, typename ShapeB>
static Manifold collideSymmetric(const ShapeA& shapeA,
                                 const Transform& transformA,
                                 const ShapeB& shapeB,
                                 const Transform& transformB)
{
    Manifold manifoldAB = collide(shapeA, transformA, shapeB, transformB);
    Manifold manifoldBA = collide(shapeB, transformB, shapeA, transformA);
    manifoldBA.reverse();

    REQUIRE(manifoldAB.pointCount == manifoldBA.pointCount);
    if (!(manifoldAB.normal == manifoldBA.normal))
    {
        CHECK(manifoldAB.normal == -manifoldBA.normal);
    }
    for (uint8_t i = 0; i < manifoldAB.pointCount; ++i)
    {
        CHECK(manifoldAB.points[i].point == manifoldBA.points[i].point);
        CHECK(manifoldAB.points[i].anchorA == manifoldBA.points[i].anchorA);
        CHECK(manifoldAB.points[i].anchorB == manifoldBA.points[i].anchorB);
        CHECK(manifoldAB.points[i].separation == Approx(manifoldBA.points[i].separation).margin(0.0001f));
    }

    return manifoldAB;
}

TEST_CASE("Circle vs Circle collision", "[collision][circle]")
{
    Circle circleA{Vec2{0.0f, 0.0f}, 1.0f};
    Transform t{Vec2{0.0f, 0.0f}, 0.0f};

    SECTION("No contact")
    {
        Circle circleB{Vec2{3.0f, 0.0f}, 1.0f};
        Manifold m = collideSymmetric(circleA, t, circleB, t);
        CHECK(m.pointCount == 0);
    }

    SECTION("Tangent on right")
    {
        Circle circleB{Vec2{2.0f, 0.0f}, 1.0f};
        Manifold m = collideSymmetric(circleA, t, circleB, t);
        REQUIRE(m.pointCount == 1);
        CHECK(std::abs(m.normal.x) == Approx(1.0f).margin(0.001f));
        CHECK(m.normal.y == Approx(0.0f).margin(0.001f));
        CHECK(m.points[0].separation == Approx(0.0f).margin(0.001f));
    }

    SECTION("Tangent on top")
    {
        Circle circleB{Vec2{0.0f, 2.0f}, 1.0f};
        Manifold m = collideSymmetric(circleA, t, circleB, t);
        REQUIRE(m.pointCount == 1);
        CHECK(m.normal.x == Approx(0.0f).margin(0.001f));
        CHECK(m.normal.y == Approx(1.0f).margin(0.001f));
    }

    SECTION("Overlapping")
    {
        Circle circleB{Vec2{1.5f, 0.0f}, 1.0f};
        Manifold m = collideSymmetric(circleA, t, circleB, t);
        REQUIRE(m.pointCount == 1);
        CHECK(m.points[0].separation < 0.0f);
    }
}

TEST_CASE("Capsule vs Circle collision", "[collision][capsule][circle]")
{
    Capsule capsule{Vec2{-1.0f, 0.0f}, Vec2{1.0f, 0.0f}, 0.5f};
    Transform t{Vec2{0.0f, 0.0f}, 0.0f};

    SECTION("No contact")
    {
        Circle circle{Vec2{0.0f, 3.0f}, 0.5f};
        Manifold m = collideSymmetric(capsule, t, circle, t);
        CHECK(m.pointCount == 0);
    }

    SECTION("Tangent on side")
    {
        Circle circle{Vec2{0.0f, 1.0f}, 0.5f};
        Manifold m = collideSymmetric(capsule, t, circle, t);
        REQUIRE(m.pointCount == 1);
        CHECK(m.normal.x == Approx(0.0f).margin(0.001f));
        CHECK(m.normal.y == Approx(1.0f).margin(0.001f));
    }

    SECTION("Tangent on capsule end")
    {
        Circle circle{Vec2{-1.0f, -1.0f}, 0.5f};
        Manifold m = collideSymmetric(capsule, t, circle, t);
        REQUIRE(m.pointCount == 1);
        CHECK(m.normal.x == Approx(0.0f).margin(0.001f));
        CHECK(m.normal.y == Approx(-1.0f).margin(0.001f));
    }

    SECTION("Deep overlap")
    {
        Circle circle{Vec2{0.0f, 0.0f}, 0.8f};
        Manifold m = collideSymmetric(capsule, t, circle, t);
        REQUIRE(m.pointCount == 1);
        CHECK(m.points[0].separation < -0.5f);
    }
}

TEST_CASE("Capsule vs Capsule collision", "[collision][capsule]")
{
    Capsule capA{Vec2{-1.0f, 0.0f}, Vec2{1.0f, 0.0f}, 0.5f};
    Transform t{Vec2{0.0f, 0.0f}, 0.0f};

    SECTION("No contact")
    {
        Capsule capB{Vec2{4.0f, 0.0f}, Vec2{6.0f, 0.0f}, 0.5f};
        Manifold m = collideSymmetric(capA, t, capB, t);
        CHECK(m.pointCount == 0);
    }

    SECTION("Tangent side by side")
    {
        Capsule capB{Vec2{2.0f, 0.0f}, Vec2{4.0f, 0.0f}, 0.5f};
        Manifold m = collideSymmetric(capA, t, capB, t);
        REQUIRE(m.pointCount == 1);
        CHECK(std::abs(m.normal.x) == Approx(1.0f).margin(0.001f));
        CHECK(m.normal.y == Approx(0.0f).margin(0.001f));
    }

    SECTION("Tangent end to end")
    {
        Capsule capB{Vec2{0.0f, 1.0f}, Vec2{0.0f, 3.0f}, 0.5f};
        Manifold m = collideSymmetric(capA, t, capB, t);
        REQUIRE(m.pointCount == 1);
        CHECK(m.normal.x == Approx(0.0f).margin(0.001f));
        CHECK(m.normal.y == Approx(1.0f).margin(0.001f));
    }

    SECTION("Cross overlap")
    {
        Capsule capB{Vec2{0.0f, -1.0f}, Vec2{0.0f, 1.0f}, 0.5f};
        Manifold m = collideSymmetric(capA, t, capB, t);
        REQUIRE(m.pointCount == 1);
        CHECK(m.points[0].separation < 0.0f);
    }
}

TEST_CASE("Segment vs Segment collision", "[collision][segment]")
{
    Segment segA{Vec2{-1.0f, 0.0f}, Vec2{1.0f, 0.0f}};
    Transform t{Vec2{0.0f, 0.0f}, 0.0f};

    SECTION("No contact")
    {
        Segment segB{Vec2{3.0f, 0.0f}, Vec2{5.0f, 0.0f}};
        Manifold m = collideSymmetric(segA, t, segB, t);
        CHECK(m.pointCount == 0);
    }

    SECTION("Endpoint touching")
    {
        Segment segB{Vec2{1.0f, 0.0f}, Vec2{3.0f, 0.0f}};
        Manifold m = collideSymmetric(segA, t, segB, t);
        REQUIRE(m.pointCount == 1);
        CHECK(m.points[0].point == Vec2{1.0f, 0.0f});
    }

    SECTION("Crossing")
    {
        Segment segB{Vec2{0.0f, -1.0f}, Vec2{0.0f, 1.0f}};
        Manifold m = collideSymmetric(segA, t, segB, t);
        REQUIRE(m.pointCount == 1);
        CHECK(m.points[0].point == Vec2{0.0f, 0.0f});
    }

    SECTION("Overlapping")
    {
        Segment segB{Vec2{-0.5f, 0.0f}, Vec2{0.5f, 0.0f}};
        Manifold m = collideSymmetric(segA, t, segB, t);
        REQUIRE(m.pointCount == 1);
        CHECK(m.points[0].separation == Approx(0.0f).margin(0.001f));
    }
}

TEST_CASE("Circle vs Polygon collision", "[collision][circle][polygon]")
{
    Polygon poly = makeRectangle(Vec2{0.0f, 0.0f}, 1.0f, 1.0f);
    Transform t{Vec2{0.0f, 0.0f}, 0.0f};

    SECTION("No contact")
    {
        Circle circle{Vec2{0.0f, 3.0f}, 0.5f};
        Manifold m = collideSymmetric(poly, t, circle, t);
        CHECK(m.pointCount == 0);
    }

    SECTION("Tangent on top")
    {
        Circle circle{Vec2{0.0f, 1.5f}, 0.5f};
        Manifold m = collideSymmetric(poly, t, circle, t);
        REQUIRE(m.pointCount == 1);
        CHECK(m.normal.x == Approx(0.0f).margin(0.001f));
        CHECK(m.normal.y == Approx(1.0f).margin(0.001f));
    }

    SECTION("Tangent on right")
    {
        Circle circle{Vec2{1.5f, 0.0f}, 0.5f};
        Manifold m = collideSymmetric(poly, t, circle, t);
        REQUIRE(m.pointCount == 1);
        CHECK(std::abs(m.normal.x) == Approx(1.0f).margin(0.001f));
        CHECK(m.normal.y == Approx(0.0f).margin(0.001f));
    }

    SECTION("Inside polygon")
    {
        Circle circle{Vec2{0.0f, 0.0f}, 0.5f};
        Manifold m = collideSymmetric(poly, t, circle, t);
        REQUIRE(m.pointCount == 1);
        CHECK(m.points[0].separation < 0.0f);
    }
}

TEST_CASE("Circle vs Segment collision", "[collision][circle][segment]")
{
    Segment seg{Vec2{-1.0f, 0.0f}, Vec2{1.0f, 0.0f}};
    Transform t{Vec2{0.0f, 0.0f}, 0.0f};

    SECTION("No contact")
    {
        Circle circle{Vec2{0.0f, 2.0f}, 0.5f};
        Manifold m = collideSymmetric(circle, t, seg, t);
        CHECK(m.pointCount == 0);
    }

    SECTION("Tangent at center")
    {
        Circle circle{Vec2{0.0f, 0.5f}, 0.5f};
        Manifold m = collideSymmetric(circle, t, seg, t);
        REQUIRE(m.pointCount == 1);
        CHECK(m.normal.x == Approx(0.0f).margin(0.001f));
        CHECK(m.normal.y == Approx(-1.0f).margin(0.001f));
    }

    SECTION("Tangent at endpoint")
    {
        Circle circle{Vec2{1.5f, 0.0f}, 0.5f};
        Manifold m = collideSymmetric(circle, t, seg, t);
        REQUIRE(m.pointCount == 1);
        CHECK(m.points[0].separation == Approx(0.0f).margin(0.01f));
    }

    SECTION("Overlap")
    {
        Circle circle{Vec2{0.0f, 0.0f}, 0.75f};
        Manifold m = collideSymmetric(circle, t, seg, t);
        REQUIRE(m.pointCount == 1);
        CHECK(m.points[0].separation < 0.0f);
    }
}

TEST_CASE("Capsule vs Polygon collision", "[collision][capsule][polygon]")
{
    Polygon poly = makeRectangle(Vec2{0.0f, 0.0f}, 1.0f, 1.0f);
    Transform t{Vec2{0.0f, 0.0f}, 0.0f};

    SECTION("No contact")
    {
        Capsule cap{Vec2{0.0f, 3.0f}, Vec2{0.0f, 4.0f}, 0.5f};
        Manifold m = collideSymmetric(poly, t, cap, t);
        CHECK(m.pointCount == 0);
    }

    SECTION("Tangent on top")
    {
        Capsule cap{Vec2{0.0f, 1.5f}, Vec2{0.0f, 2.5f}, 0.5f};
        Manifold m = collideSymmetric(poly, t, cap, t);
        REQUIRE(m.pointCount == 1);
        CHECK(m.normal.x == Approx(0.0f).margin(0.001f));
        CHECK(m.normal.y == Approx(1.0f).margin(0.001f));
    }

    SECTION("Tangent on right")
    {
        Capsule cap{Vec2{1.5f, 0.0f}, Vec2{2.5f, 0.0f}, 0.5f};
        Manifold m = collideSymmetric(poly, t, cap, t);
        REQUIRE(m.pointCount == 1);
        CHECK(std::abs(m.normal.x) == Approx(1.0f).margin(0.001f));
        CHECK(m.normal.y == Approx(0.0f).margin(0.001f));
    }

    SECTION("Intersecting")
    {
        Capsule cap{Vec2{0.0f, -1.0f}, Vec2{0.0f, 1.0f}, 0.5f};
        Manifold m = collideSymmetric(poly, t, cap, t);
        REQUIRE(m.pointCount == 1);
        CHECK(m.points[0].separation < 0.0f);
    }
}

TEST_CASE("Capsule vs Segment collision", "[collision][capsule][segment]")
{
    Capsule cap{Vec2{-1.0f, 0.0f}, Vec2{1.0f, 0.0f}, 0.5f};
    Transform t{Vec2{0.0f, 0.0f}, 0.0f};

    SECTION("No contact")
    {
        Segment seg{Vec2{3.0f, -2.0f}, Vec2{3.0f, 2.0f}};
        Manifold m = collideSymmetric(cap, t, seg, t);
        CHECK(m.pointCount == 0);
    }

    SECTION("Tangent on side")
    {
        Segment seg{Vec2{1.5f, -2.0f}, Vec2{1.5f, 2.0f}};
        Manifold m = collideSymmetric(cap, t, seg, t);
        REQUIRE(m.pointCount == 1);
    }

    SECTION("Tangent on end")
    {
        Segment seg{Vec2{0.0f, 0.5f}, Vec2{0.0f, 2.5f}};
        Manifold m = collideSymmetric(cap, t, seg, t);
        REQUIRE(m.pointCount == 1);
    }

    SECTION("Overlap")
    {
        Segment seg{Vec2{0.0f, -2.0f}, Vec2{0.0f, 2.0f}};
        Manifold m = collideSymmetric(cap, t, seg, t);
        REQUIRE(m.pointCount == 1);
        CHECK(m.points[0].separation < 0.0f);
    }
}

TEST_CASE("Polygon vs Segment collision", "[collision][polygon][segment]")
{
    Polygon poly = makeRectangle(Vec2{0.0f, 0.0f}, 1.0f, 1.0f);
    Transform t{Vec2{0.0f, 0.0f}, 0.0f};

    SECTION("No contact")
    {
        Segment seg{Vec2{-2.0f, 3.0f}, Vec2{2.0f, 3.0f}};
        Manifold m = collideSymmetric(poly, t, seg, t);
        CHECK(m.pointCount == 0);
    }

    SECTION("Tangent on top")
    {
        Segment seg{Vec2{-2.0f, 1.0f}, Vec2{2.0f, 1.0f}};
        Manifold m = collideSymmetric(poly, t, seg, t);
        REQUIRE(m.pointCount == 1);
    }

    SECTION("Tangent on right")
    {
        Segment seg{Vec2{1.0f, -2.0f}, Vec2{1.0f, 2.0f}};
        Manifold m = collideSymmetric(poly, t, seg, t);
        REQUIRE(m.pointCount == 1);
        CHECK(std::abs(m.normal.x) == Approx(1.0f).margin(0.001f));
        CHECK(m.normal.y == Approx(0.0f).margin(0.001f));
    }

    SECTION("Cross through")
    {
        Segment seg{Vec2{0.0f, -2.0f}, Vec2{0.0f, 2.0f}};
        Manifold m = collideSymmetric(poly, t, seg, t);
        REQUIRE(m.pointCount == 1);
        CHECK(m.points[0].separation <= Approx(0.0f).margin(0.01f));
    }
}

TEST_CASE("Polygon vs Polygon collision", "[collision][polygon]")
{
    Polygon polyA = makeRectangle(Vec2{0.0f, 0.0f}, 1.0f, 1.0f);
    Transform t{Vec2{0.0f, 0.0f}, 0.0f};

    SECTION("No contact")
    {
        Polygon polyB = makeRectangle(Vec2{3.0f, 0.0f}, 1.0f, 1.0f);
        Manifold m = collideSymmetric(polyA, t, polyB, t);
        CHECK(m.pointCount == 0);
    }

    SECTION("Tangent on right")
    {
        Polygon polyB = makeRectangle(Vec2{1.999f, 0.0f}, 1.0f, 1.0f);
        Manifold m = collideSymmetric(polyA, t, polyB, t);
        REQUIRE(m.pointCount == 1);
    }

    SECTION("Tangent on top")
    {
        Polygon polyB = makeRectangle(Vec2{0.0f, 1.999f}, 1.0f, 1.0f);
        Manifold m = collideSymmetric(polyA, t, polyB, t);
        REQUIRE(m.pointCount == 1);
    }

    SECTION("Overlapping")
    {
        Polygon polyB = makeRectangle(Vec2{0.5f, 0.5f}, 1.0f, 1.0f);
        Manifold m = collideSymmetric(polyA, t, polyB, t);
        REQUIRE(m.pointCount == 1);
        CHECK(m.points[0].separation < 0.0f);
    }
}
