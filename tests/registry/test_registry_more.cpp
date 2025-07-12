#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <colli2de/Registry.hpp>

using namespace c2d;
using Catch::Approx;

TEST_CASE("Registry basic lifecycle", "[Registry]")
{
    SECTION("Create and remove entities")
    {
        Registry<int> registry;
        registry.createEntity(1, BodyType::Static, Transform({0.0f, 0.0f}));
        registry.addShape(1, Circle{{0.0f,0.0f}, 1.0f});

        registry.createEntity(2, BodyType::Dynamic, Transform({3.0f, 0.0f}));
        registry.addShape(2, Circle{{0.0f,0.0f}, 1.0f});

        CHECK_FALSE(registry.areColliding(1,2));
        registry.teleportEntity(2, Transform({1.5f, 0.0f}));
        CHECK(registry.areColliding(1,2));

        registry.removeEntity(2);
        CHECK(registry.getCollidingPairs().empty());
    }

    SECTION("Shape id reuse after removal")
    {
        Registry<int> registry;
        registry.createEntity(1, BodyType::Static, Transform({0.0f,0.0f}));
        ShapeId id1 = registry.addShape(1, Circle{{0.0f,0.0f},1.0f});

        registry.createEntity(2, BodyType::Static, Transform({5.0f,0.0f}));
        ShapeId id2 = registry.addShape(2, Circle{{0.0f,0.0f},1.0f});
        registry.removeEntity(2);

        registry.createEntity(3, BodyType::Static, Transform({10.0f,0.0f}));
        ShapeId id3 = registry.addShape(3, Circle{{0.0f,0.0f},1.0f});
        CHECK(id3 == id2);
        CHECK(id1 != id3);
    }
}

TEST_CASE("Registry movement and teleport", "[Registry]")
{
    SECTION("Teleport dynamic updates collisions")
    {
        Registry<int> registry;
        registry.createEntity(1, BodyType::Dynamic, Transform({0.0f,0.0f}));
        registry.addShape(1, Circle{{0.0f,0.0f},1.0f});
        registry.createEntity(2, BodyType::Static, Transform({3.0f,0.0f}));
        registry.addShape(2, Circle{{0.0f,0.0f},1.0f});
        CHECK_FALSE(registry.areColliding(1,2));
        registry.teleportEntity(1, Transform({2.0f,0.0f}));
        CHECK(registry.areColliding(1,2));
    }

    SECTION("Move bullet performs sweep")
    {
        Registry<int> registry;
        registry.createEntity(1, BodyType::Static, Transform({0.0f,0.0f}));
        registry.addShape(1, Circle{{0.0f,0.0f},1.0f});
        registry.createEntity(2, BodyType::Bullet, Transform({-3.0f,0.0f}));
        registry.addShape(2, Circle{{0.0f,0.0f},1.0f});
        CHECK_NOTHROW(registry.moveEntity(2, Transform({3.5f,0.0f})));
        const auto pairs = registry.getCollidingPairs();
        (void)pairs;
    }
}

TEST_CASE("Registry collision queries", "[Registry]")
{
    SECTION("Category and mask filtering")
    {
        Registry<int> registry;
        registry.createEntity(1, BodyType::Dynamic, Transform({0.0f,0.0f}));
        registry.addShape(1, Circle{{0.0f,0.0f},1.0f}, 1, 1);
        registry.createEntity(2, BodyType::Dynamic, Transform({0.5f,0.0f}));
        registry.addShape(2, Circle{{0.0f,0.0f},1.0f}, 2, 2);
        CHECK(registry.getCollidingPairs().empty());
        registry.createEntity(3, BodyType::Dynamic, Transform({0.5f,0.0f}));
        registry.addShape(3, Circle{{0.0f,0.0f},1.0f}, 1, 3);
        const auto collisions = registry.getCollisions(3);
        CHECK(collisions.size() >= 1);
    }

    SECTION("Bullet collisions queried")
    {
        Registry<int> registry;
        registry.createEntity(1, BodyType::Static, Transform({0.0f,0.0f}));
        registry.addShape(1, Circle{{0.0f,0.0f},1.0f});
        registry.createEntity(2, BodyType::Bullet, Transform({-1.5f,0.0f}));
        registry.addShape(2, Circle{{0.0f,0.0f},1.0f});
        registry.moveEntity(2, Transform({2.0f,0.0f}));
        const auto collisions = registry.getCollisions(2);
        CHECK_FALSE(collisions.empty());
    }
}

TEST_CASE("Registry ray casting", "[Registry]")
{
    SECTION("Finite ray with mask")
    {
        Registry<int> registry;
        registry.createEntity(1, BodyType::Static, Transform({0.0f,0.0f}));
        registry.addShape(1, Circle{{0.0f,0.0f},1.0f}, 1, 1);
        registry.createEntity(2, BodyType::Static, Transform({4.0f,0.0f}));
        registry.addShape(2, Circle{{0.0f,0.0f},1.0f}, 2, 2);
        Ray ray{{-2.0f,0.0f},{5.0f,0.0f}};
        auto hits = registry.rayCast(ray, 1);
        REQUIRE(hits.size() == 1);
        CHECK(hits.begin()->id.first == 1);
        CHECK(hits.begin()->entry.x == Approx(-1.0f));
        CHECK(hits.begin()->exit.x  == Approx(1.0f));
        CHECK(hits.begin()->entryTime == Approx(( -1.0f - (-2.0f) ) / 7.0f));
        CHECK(hits.begin()->exitTime  == Approx(( 1.0f - (-2.0f) ) / 7.0f));
        auto hit = registry.firstHitRayCast(ray, 2);
        REQUIRE(hit.has_value());
        CHECK(hit->id.first == 2);
        CHECK(hit->entry.x == Approx(3.0f));
        CHECK(hit->exit.x  == Approx(5.0f));
        CHECK(hit->entryTime == Approx(( 3.0f - (-2.0f) ) / 7.0f));
        CHECK(hit->exitTime  == Approx(( 5.0f - (-2.0f) ) / 7.0f));
    }

    SECTION("Infinite ray hits both")
    {
        Registry<int> registry;
        registry.createEntity(1, BodyType::Static, Transform({0.0f,0.0f}));
        registry.addShape(1, Circle{{0.0f,0.0f},1.0f});
        registry.createEntity(2, BodyType::Static, Transform({4.0f,0.0f}));
        registry.addShape(2, Circle{{0.0f,0.0f},1.0f});
        InfiniteRay ray{{-1.0f,0.0f},{1.0f,0.0f}};
        auto hits = registry.rayCast(ray);
        REQUIRE(hits.size() == 2);
        auto first = hits.begin();
        CHECK(first->id.first == 1);
        CHECK(first->entry.x == Approx(-1.0f));
        CHECK(first->exit.x  == Approx(1.0f));
        CHECK(first->entryTime == Approx(0.0f));
        CHECK(first->exitTime  == Approx(2.0f));
        CHECK(std::next(first)->id.first == 2);
        CHECK(std::next(first)->entry.x == Approx(3.0f));
        CHECK(std::next(first)->exit.x  == Approx(5.0f));
        CHECK(std::next(first)->entryTime == Approx(4.0f));
        CHECK(std::next(first)->exitTime  == Approx(6.0f));
        auto hit = registry.firstHitRayCast(ray);
        REQUIRE(hit.has_value());
        CHECK(hit->id.first == 1);
        CHECK(hit->entry.x == Approx(-1.0f));
        CHECK(hit->exit.x  == Approx(1.0f));
        CHECK(hit->entryTime == Approx(0.0f));
        CHECK(hit->exitTime  == Approx(2.0f));
    }
}

TEST_CASE("Registry collision checks", "[Registry]")
{
    SECTION("Static entities never collide")
    {
        Registry<int> registry;
        registry.createEntity(1, BodyType::Static, Transform({0.0f,0.0f}));
        registry.addShape(1, Circle{{0.0f,0.0f},1.0f});
        registry.createEntity(2, BodyType::Static, Transform({0.5f,0.0f}));
        registry.addShape(2, Circle{{0.0f,0.0f},1.0f});
        CHECK_FALSE(registry.areColliding(1,2));
    }

    SECTION("Bullet vs bullet sweep")
    {
        Registry<int> registry;
        registry.createEntity(1, BodyType::Bullet, Transform({0.0f,0.0f}));
        registry.addShape(1, Circle{{0.0f,0.0f},1.0f});
        registry.createEntity(2, BodyType::Bullet, Transform({-3.0f,0.0f}));
        registry.addShape(2, Circle{{0.0f,0.0f},1.0f});
        registry.moveEntity(2, Transform({6.0f,0.0f}));
        CHECK(registry.areColliding(1,2));
    }
}
