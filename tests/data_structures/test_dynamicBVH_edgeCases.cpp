#include <colli2de/internal/data_structures/DynamicBVH.hpp>
#include <colli2de/internal/geometry/AABB.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cstdint>

using namespace c2d;
using namespace Catch;

TEST_CASE("DynamicBVH | query handles degenerate and thin AABBs", "[DynamicBVH][Advanced][Degenerate]")
{
    constexpr float margin = 0.0f;
    DynamicBVH<uint32_t> bvh(margin);

    // Zero-width box
    bvh.addProxy(1, {Vec2{5, 5}, Vec2{5, 10}});
    // Zero-height box
    bvh.addProxy(2, {Vec2{1, 3}, Vec2{4, 3}});
    // Normal box
    bvh.addProxy(3, {Vec2{2, 2}, Vec2{4, 4}});

    // Query covering all
    AABB query{Vec2{0, 0}, Vec2{6, 11}};
    std::pmr::vector<uint32_t> hits;
    bvh.query(query, hits);

    REQUIRE(hits.size() == 3);
    REQUIRE(std::find(hits.begin(), hits.end(), 1) != hits.end());
    REQUIRE(std::find(hits.begin(), hits.end(), 2) != hits.end());
    REQUIRE(std::find(hits.begin(), hits.end(), 3) != hits.end());
}

TEST_CASE("DynamicBVH | raycastFirstHit detects grazing rays", "[DynamicBVH][Advanced][RayGrazing]")
{
    DynamicBVH<uint32_t> bvh;
    bvh.addProxy(1, {Vec2{2, 2}, Vec2{4, 4}});

    // Ray just touches the lower-left corner
    Ray ray1{Vec2{0, 0}, Vec2{2, 2}};
    auto hit1 = bvh.firstHitRaycast(ray1);
    REQUIRE(hit1.has_value());
    REQUIRE(hit1->id == 1);
    CHECK(hit1->entry.x == Approx(2.0f));
    CHECK(hit1->entry.y == Approx(2.0f));
    CHECK(hit1->exit.x == Approx(2.0f));
    CHECK(hit1->exit.y == Approx(2.0f));
    CHECK(hit1->entryTime == Approx(1.0f));
    CHECK(hit1->exitTime == Approx(1.0f));

    // Ray along the bottom edge
    Ray ray2{Vec2{2, 2}, Vec2{4, 2}};
    auto hit2 = bvh.firstHitRaycast(ray2);
    REQUIRE(hit2.has_value());
    REQUIRE(hit2->id == 1);
    CHECK(hit2->entry.x == Approx(2.0f));
    CHECK(hit2->entry.y == Approx(2.0f));
    CHECK(hit2->exit.x == Approx(4.0f));
    CHECK(hit2->exit.y == Approx(2.0f));
    CHECK(hit2->entryTime == Approx(0.0f).margin(0.001f));
    CHECK(hit2->exitTime == Approx(1.0f));
}
