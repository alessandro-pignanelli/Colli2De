#include <catch2/catch_test_macros.hpp>
#include "collision/Collision.hpp"
#include "colli2de/Shapes.hpp"
#include "geometry/Transformations.hpp"

using namespace c2d;
using namespace Catch;

TEST_CASE("isColliding Circle vs Circle", "[collision][isColliding]")
{
    Circle a{Vec2{0.0f, 0.0f}, 1.0f};
    Transform t{Vec2{0.0f, 0.0f}, 0.0f};
    SECTION("No contact")
    {
        Circle b{Vec2{3.0f, 0.0f}, 1.0f};
        CHECK_FALSE(isColliding(a, t, b, t));
    }
    SECTION("Overlapping")
    {
        Circle b{Vec2{1.0f, 0.0f}, 1.0f};
        CHECK(isColliding(a, t, b, t));
    }
}

TEST_CASE("isColliding Capsule vs Circle", "[collision][isColliding]")
{
    Capsule cap{Vec2{-1.0f, 0.0f}, Vec2{1.0f, 0.0f}, 0.5f};
    Transform t{Vec2{0.0f, 0.0f}, 0.0f};
    SECTION("No contact")
    {
        Circle c{Vec2{0.0f, 3.0f}, 0.5f};
        CHECK_FALSE(isColliding(cap, t, c, t));
    }
    SECTION("Overlap")
    {
        Circle c{Vec2{0.0f, 0.5f}, 0.5f};
        CHECK(isColliding(cap, t, c, t));
    }
}

TEST_CASE("isColliding Capsule vs Capsule", "[collision][isColliding]")
{
    Capsule a{Vec2{-1.0f, 0.0f}, Vec2{1.0f, 0.0f}, 0.5f};
    Capsule b{Vec2{-1.0f, 2.0f}, Vec2{1.0f, 2.0f}, 0.5f};
    Transform t{Vec2{0.0f,0.0f},0.0f};
    CHECK_FALSE(isColliding(a,t,b,t));
    b.center1 = Vec2{-0.5f,0.0f};
    b.center2 = Vec2{0.5f,0.0f};
    CHECK(isColliding(a,t,b,t));
}

TEST_CASE("isColliding Segment vs Segment", "[collision][isColliding]")
{
    Segment s1{Vec2{-1.0f,0.0f}, Vec2{1.0f,0.0f}};
    Transform t{Vec2{0.0f,0.0f},0.0f};
    SECTION("Separated")
    {
        Segment s2{Vec2{0.0f,2.0f}, Vec2{2.0f,2.0f}};
        CHECK_FALSE(isColliding(s1,t,s2,t));
    }
    SECTION("Crossing")
    {
        Segment s2{Vec2{0.0f,-1.0f}, Vec2{0.0f,1.0f}};
        CHECK(isColliding(s1,t,s2,t));
    }
}

TEST_CASE("isColliding Circle vs Segment", "[collision][isColliding]")
{
    Segment s{Vec2{-1.0f,0.0f}, Vec2{1.0f,0.0f}};
    Transform t{Vec2{0.0f,0.0f},0.0f};
    SECTION("No contact")
    {
        Circle c{Vec2{0.0f,2.0f},0.5f};
        CHECK_FALSE(isColliding(c,t,s,t));
    }
    SECTION("Overlap")
    {
        Circle c{Vec2{0.0f,0.5f},0.5f};
        CHECK(isColliding(c,t,s,t));
    }
}

TEST_CASE("isColliding Polygon vs Polygon", "[collision][isColliding]")
{
    Polygon a = makeRectangle(Vec2{0.0f,0.0f},1.0f,1.0f);
    Polygon b = makeRectangle(Vec2{3.0f,0.0f},1.0f,1.0f);
    Transform t{Vec2{0.0f,0.0f},0.0f};
    CHECK_FALSE(isColliding(a,t,b,t));
    b = makeRectangle(Vec2{1.9f,0.0f},1.0f,1.0f);
    CHECK(isColliding(a,t,b,t));
}

TEST_CASE("isColliding Polygon vs Circle", "[collision][isColliding]")
{
    Polygon p = makeRectangle(Vec2{0.0f,0.0f},1.0f,1.0f);
    Transform t{Vec2{0.0f,0.0f},0.0f};
    SECTION("No contact")
    {
        Circle c{Vec2{0.0f,3.0f},0.5f};
        CHECK_FALSE(isColliding(p,t,c,t));
    }
    SECTION("Inside")
    {
        Circle c{Vec2{0.0f,0.0f},0.5f};
        CHECK(isColliding(p,t,c,t));
    }
}

TEST_CASE("isColliding Capsule vs Polygon", "[collision][isColliding]")
{
    Polygon p = makeRectangle(Vec2{0.0f,0.0f},1.0f,1.0f);
    Capsule cap{Vec2{0.0f,-1.0f}, Vec2{0.0f,1.0f}, 0.5f};
    Transform t{Vec2{0.0f,0.0f},0.0f};
    CHECK(isColliding(p,t,cap,t));
    Capsule capFar{Vec2{0.0f,3.0f}, Vec2{0.0f,4.0f},0.5f};
    CHECK_FALSE(isColliding(p,t,capFar,t));
}

TEST_CASE("isColliding Polygon vs Segment", "[collision][isColliding]")
{
    Polygon p = makeRectangle(Vec2{0.0f,0.0f},1.0f,1.0f);
    Segment seg{Vec2{-2.0f,3.0f}, Vec2{2.0f,3.0f}};
    Transform t{Vec2{0.0f,0.0f},0.0f};
    CHECK_FALSE(isColliding(p,t,seg,t));
    Segment seg2{Vec2{0.0f,-2.0f}, Vec2{0.0f,2.0f}};
    CHECK(isColliding(p,t,seg2,t));
}
