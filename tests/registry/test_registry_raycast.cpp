#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <colli2de/Registry.hpp>

using namespace c2d;
using Catch::Approx;

TEST_CASE("Registry rayCast hits nearest entity", "[Registry][Raycast]")
{
    Registry<int> reg;
    reg.createEntity(1, BodyType::Static, Transform({0.0f, 0.0f}));
    reg.addShape(1, Circle{ {0.0f, 0.0f}, 1.0f });
    reg.createEntity(2, BodyType::Static, Transform({3.0f, 0.0f}));
    reg.addShape(2, Circle{ {0.0f, 0.0f}, 1.0f });

    Ray ray{ {-2.0f, 0.0f}, {1.9f, 0.0f} };

    auto hits = reg.rayCast(ray);
    REQUIRE(hits.size() == 1);
    CHECK(hits.begin()->id.first == 1);
    CHECK(hits.begin()->entry.x == Approx(-1.0f));
    CHECK(hits.begin()->exit.x == Approx(1.0f));
    CHECK(hits.begin()->entryTime == Approx(( -1.0f - (-2.0f) ) / 3.9f));
    CHECK(hits.begin()->exitTime == Approx(( 1.0f - (-2.0f) ) / 3.9f));

    auto hit = reg.firstHitRayCast(ray);
    REQUIRE(hit.has_value());
    CHECK(hit->id.first == 1);
    CHECK(hit->entry.x == Approx(-1.0f));
    CHECK(hit->exit.x == Approx(1.0f));
    CHECK(hit->entryTime == Approx(( -1.0f - (-2.0f) ) / 3.9f));
    CHECK(hit->exitTime == Approx(( 1.0f - (-2.0f) ) / 3.9f));

    Ray ray2{ {4.0f, 0.0f}, {-2.0f, 0.0f} };

    hits = reg.rayCast(ray2);
    REQUIRE(hits.size() == 2);
    CHECK(hits.begin()->id.first == 2);
    CHECK(hits.begin()->entry.x == Approx(4.0f));
    CHECK(hits.begin()->exit.x  == Approx(2.0f));
    CHECK(hits.begin()->entryTime == Approx(0.0f));
    CHECK(hits.begin()->exitTime  == Approx((4.0f - 2.0f) / 6.0f));
    CHECK(std::next(hits.begin())->id.first == 1);

    hit = reg.firstHitRayCast(ray2);
    REQUIRE(hit.has_value());
    CHECK(hit->id.first == 2);
    CHECK(hit->entry.x == Approx(4.0f));
    CHECK(hit->exit.x  == Approx(2.0f));
    CHECK(hit->entryTime == Approx(0.0f));
    CHECK(hit->exitTime  == Approx((4.0f - 2.0f) / 6.0f));
}

TEST_CASE("Registry bullet rayCast detects swept collision", "[Registry][Raycast][Bullet]")
{
    Registry<int> registry;
    registry.createEntity(1, BodyType::Bullet, Transform({-2.0f, 0.0f}));
    registry.addShape(1, Circle{{0.0f, 0.0f}, 1.0f});
    registry.moveEntity(1, Transform({4.0f, 0.0f}));

    Ray ray{{0.0f, -2.0f}, {0.0f, 2.0f}};

    auto hits = registry.rayCast(ray);
    REQUIRE(hits.size() == 1);
    const auto& hitInfo = *hits.begin();
    CHECK(hitInfo.id.first == 1);
    CHECK(hitInfo.entryTime == Approx(0.5f));
    CHECK(hitInfo.exitTime == Approx(0.5f));
    CHECK(hitInfo.entry.x == Approx(0.0f));
    CHECK(hitInfo.exit.x == Approx(0.0f));

    const auto firstHit = registry.firstHitRayCast(ray);
    REQUIRE(firstHit.has_value());
    CHECK(firstHit->id.first == 1);
    CHECK(firstHit->entryTime == Approx(0.5f));
    CHECK(firstHit->exitTime == Approx(0.5f));
}

TEST_CASE("Registry bullet rayCast miss when path does not cross ray", "[Registry][Raycast][Bullet]")
{
    Registry<int> registry;
    registry.createEntity(1, BodyType::Bullet, Transform({-2.0f, 0.0f}));
    registry.addShape(1, Circle{{0.0f, 0.0f}, 1.0f});
    registry.moveEntity(1, Transform({4.0f, 0.0f}));

    Ray ray{{5.0f, -2.0f}, {5.0f, 2.0f}};
    const auto hits = registry.rayCast(ray);
    CHECK(hits.empty());
    CHECK_FALSE(registry.firstHitRayCast(ray));
}

