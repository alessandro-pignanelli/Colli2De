#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#if __has_include(<print>)
    #include <print>
    #define print(...) std::print(__VA_ARGS__)
    #define println(...) std::println(__VA_ARGS__)
#else
    #define print(...) ((void)0) // Does nothing
    #define println(...) ((void)0) // Does nothing
#endif

#include "collision/Collision.hpp"
#include "colli2de/Shapes.hpp"
#include "geometry/Transformations.hpp"

using namespace c2d;
using namespace Catch;

TEST_CASE("Circle vs Circle collision", "[collision][circle]")
{
    Circle a{Vec2{0.0f, 0.0f}, 1.0f};
    Circle b{Vec2{1.5f, 0.0f}, 1.0f};
    Transform ta{Vec2{0.0f, 0.0f}, 0.0f};
    Transform tb{Vec2{0.0f, 0.0f}, 0.0f};

    SECTION("Overlapping circles")
    {
        Manifold m = collide(a, ta, b, tb);
        REQUIRE(m.pointCount == 1);
        REQUIRE(m.points[0].separation < 0.0f);
        REQUIRE(m.normal == Vec2{1.0f, 0.0f});
    }

    SECTION("Non-overlapping circles")
    {
        Circle c{Vec2{5.0f, 0.0f}, 1.0f};
        Manifold m2 = collide(a, ta, c, tb);
        REQUIRE(m2.pointCount == 0);
    }
}

TEST_CASE("Capsule vs Circle collision", "[collision][capsule][circle]")
{
    Capsule capsule{Vec2{-1.0f, 0.0f}, Vec2{1.0f, 0.0f}, 0.5f};
    Transform tcap{Vec2{0.0f, 0.0f}, 0.0f};

    SECTION("Touching at the top")
    {
        Circle circle{Vec2{0.0f, 1.0f}, 0.5f};
        Transform tc{Vec2{0.0f, 0.0f}, 0.0f};

        Manifold m = collide(capsule, tcap, circle, tc);
        REQUIRE(m.pointCount == 1);
        
        CHECK(m.points[0].separation < 1e-4f);
        CHECK(m.points[0].point.x == Approx(0.0f));
        CHECK(m.points[0].point.y == Approx(0.5f));
        CHECK(m.points[0].anchorA.x == Approx(0.0f));
        CHECK(m.points[0].anchorA.y == Approx(0.5f));
        CHECK(m.points[0].anchorB.x == Approx(0.0f));
        CHECK(m.points[0].anchorB.y == Approx(0.5f));
        
        CHECK(m.normal.x == Approx(0.0f));
        CHECK(m.normal.y == Approx(1.0f));
    }

    SECTION("Far apart")
    {
        Circle circle{Vec2{0.0f, 3.0f}, 0.5f};
        Transform tc{Vec2{0.0f, 0.0f}, 0.0f};

        Manifold m = collide(capsule, tcap, circle, tc);
        CHECK(m.pointCount == 0);
    }

    SECTION("Touching capsule endpoint")
    {
        Circle circle{Vec2{-1.0f, -1.0f}, 0.5f};
        Transform tc{Vec2{0.0f, 0.0f}, 0.0f};

        Manifold m = collide(capsule, tcap, circle, tc);
        REQUIRE(m.pointCount == 1);
        
        CHECK(m.points[0].separation < 1e-4f);
        CHECK(m.points[0].point.x == Approx(-1.0f));
        CHECK(m.points[0].point.y == Approx(-0.5f));
        CHECK(m.points[0].anchorA.x == Approx(-1.0f));
        CHECK(m.points[0].anchorA.y == Approx(-0.5f));
        CHECK(m.points[0].anchorB.x == Approx(-1.0f));
        CHECK(m.points[0].anchorB.y == Approx(-0.5f));

        CHECK(m.normal.x == Approx(0.0f));
        CHECK(m.normal.y == Approx(-1.0f));
    }

    SECTION("Deep overlap")
    {
        Circle circle{Vec2{0.0f, 0.0f}, 0.8f};
        Transform tc{Vec2{0.0f, 0.0f}, 0.0f};

        Manifold m = collide(capsule, tcap, circle, tc);
        REQUIRE(m.pointCount == 1);

        CHECK(m.points[0].separation < -1.0f);
    }
}
