#include <colli2de/Shapes.hpp>
#include <map>
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <colli2de/Registry.hpp>
#include "utils/Print.hpp"

using namespace c2d;
using Catch::Approx;

TEST_CASE("Registry entity creation and shape management", "[Registry][Basic]")
{
    Registry<int> reg;
    std::map<int, ShapeId> shapeIds;

    // SECTION("Create and add shapes to an entity, non-colliding")
    {
        reg.createEntity(1, BodyType::Static, Transform({0.0f, 0.0f}));
        shapeIds[1] = reg.addShape(1, Circle{ {0.0f, 0.0f}, 1.0f });
        reg.createEntity(2, BodyType::Dynamic, Transform({3.0f, 0.0f}));
        shapeIds[2] = reg.addShape(2, Circle{ {0.0f, 0.0f}, 1.0f });

        CHECK_FALSE(reg.areColliding(1, 2));
    }

    // SECTION("Move entity and check collision")
    {
        reg.teleportEntity(2, Transform({1.0f, 0.0f}));
        CHECK(reg.areColliding(1, 2));
    }

    // SECTION("Add more entities and get all colliding pairs")
    {
        // Colliding with 1 and 2
        reg.createEntity(3, BodyType::Dynamic, Transform({2.0f, 0.0f}));
        shapeIds[3] = reg.addShape(3, Circle{ {0.0f, 0.0f}, 1.0f });

        // Colliding with 2
        reg.createEntity(4, BodyType::Dynamic, Transform({1.0f, 2.0f}));
        shapeIds[4] = reg.addShape(4, Circle{ {0.0f, 0.0f}, 1.0f });

        // Not colliding
        reg.createEntity(5, BodyType::Dynamic, Transform({1.0f, 5.0f}));
        reg.addShape(5, Circle{ {0.0f, 0.0f}, 1.0f });

        const std::vector<std::pair<int, int>> expectedCollisions = {
            {1, 2}, {1, 3}, {2, 3}, {2, 4}
        };
        
        auto collisions = reg.getCollidingPairs();
        REQUIRE(collisions.size() == expectedCollisions.size());
        
        for (const auto& pair : expectedCollisions)
        {
            const auto findCollision = [&pair](const Registry<int>::EntityCollision& collision)
            {
                return collision.entityA == pair.first && collision.entityB == pair.second ||
                       collision.entityA == pair.second && collision.entityB == pair.first;
            };
            CHECK(std::find_if(collisions.begin(), collisions.end(), findCollision) != collisions.end());
        }
    }

    // SECTION("Set inactive shapes and check collisions")
    {
        reg.setShapeActive(shapeIds[2], false);
        CHECK(reg.getCollidingPairs().size() == 1); // Only {1, 3} should collide now
        CHECK(reg.areColliding(1, 3));
        CHECK_FALSE(reg.areColliding(1, 2));
        CHECK_FALSE(reg.areColliding(2, 3));
        CHECK_FALSE(reg.areColliding(2, 4));

        reg.setShapeActive(shapeIds[2]);
        CHECK(reg.getCollidingPairs().size() == 4); // All pairs should collide again
        CHECK(reg.areColliding(1, 2));
        CHECK(reg.areColliding(1, 3));
        CHECK(reg.areColliding(2, 3));
        CHECK(reg.areColliding(2, 4));
    }

    // SECTION("Remove shapes and check collisions")
    {
        reg.removeEntity(2);
        CHECK(reg.getCollidingPairs().size() == 1); // Only {1, 3} should collide now
        
        reg.removeEntity(1);
        CHECK(reg.getCollidingPairs().empty());
    }
}

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

    SECTION("All body types collisions")
    {
        Registry<int> registry;

        registry.createEntity(1, BodyType::Static, Transform({0.0f,0.0f}));
        const auto shape1 = registry.addShape(1, Circle{{0.0f,0.0f},1.0f});
        registry.createEntity(2, BodyType::Dynamic, Transform({0.5f,0.0f}));
        const auto shape2 = registry.addShape(2, Circle{{0.0f,0.0f},1.0f});
        registry.createEntity(3, BodyType::Bullet, Transform({1.0f,0.0f}));
        const auto shape3 = registry.addShape(3, Circle{{0.0f,0.0f},1.0f});

        CHECK(registry.getCollidingPairs().size() == 3); // All pairs should collide
        CHECK(registry.areColliding(1, 2));
        CHECK(registry.areColliding(1, 3));
        CHECK(registry.areColliding(2, 3));

        registry.teleportEntity(3, Translation({-3.0f,0.0f}));
        
        CHECK(registry.getCollidingPairs().size() == 3); // All pairs should collide
        CHECK(registry.areColliding(1, 2));
        CHECK(registry.areColliding(1, 3));
        CHECK(registry.areColliding(2, 3));
        
        registry.teleportEntity(2, Translation({3.0f,0.0f}));
        registry.moveEntity(3, Translation{});
        
        CHECK(registry.getCollidingPairs().size() == 0);
        CHECK_FALSE(registry.areColliding(1, 2));
        CHECK_FALSE(registry.areColliding(1, 3));
        CHECK_FALSE(registry.areColliding(2, 3));
        
        registry.teleportEntity(2, Translation({2.05f,0.0f}));

        CHECK(registry.getCollidingPairs().size() == 0);
        CHECK_FALSE(registry.areColliding(1, 2));
        CHECK_FALSE(registry.areColliding(1, 3));
        CHECK_FALSE(registry.areColliding(2, 3));
        
        registry.teleportEntity(2, Translation({1.95f,0.0f}));

        CHECK(registry.getCollidingPairs().size() == 1); // Only 1 and 2 should collide
        CHECK(registry.areColliding(1, 2));
        CHECK_FALSE(registry.areColliding(1, 3));
        CHECK_FALSE(registry.areColliding(2, 3));
    }

    SECTION("Static and Dynamic rectangle entities collide")
    {
        Registry<int> registry;

        registry.createEntity(1, BodyType::Static, Transform({0.0f, 0.0f}));
        const auto shape = makeRectangle({0.0f, 0.0f}, 10.0f, 5.0f);
        registry.addShape(1, shape);

        registry.createEntity(2, BodyType::Dynamic, Transform({0.0f, 19.5f}));
        const auto shape2 = makeRectangle({0.0f, 0.0f}, 2.0f, 5.0f);
        registry.addShape(2, shape2);

        CHECK(registry.getCollidingPairs().size() == 0);
        CHECK_FALSE(registry.areColliding(1, 2));

        for (int i = 0; i < 9; i++)
        {
            registry.moveEntity(2, Transform({0.0f, -1.0f}));
            CHECK(registry.getCollidingPairs().size() == 0);
            CHECK_FALSE(registry.areColliding(1, 2));
        }
        
        registry.moveEntity(2, Transform({0.0f, -1.0f}));
        CHECK(registry.getCollidingPairs().size() == 1);
        CHECK(registry.areColliding(1, 2));
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

    SECTION("Bullet vs bullet queries")
    {
        Registry<int> registry;
        registry.createEntity(1, BodyType::Bullet, Transform({0.0f,0.0f}));
        const auto shapeId1 = registry.addShape(1, Circle{{0.0f,0.0f},1.0f});
        registry.createEntity(2, BodyType::Bullet, Transform({5.0f,0.0f}));
        const auto shapeId2 = registry.addShape(2, Circle{{0.0f,0.0f},1.0f});

        // Switch places. Without sweep they would not collide
        registry.teleportEntity(1, Translation({5.0f,0.0f}));
        registry.teleportEntity(2, Translation({0.0f,0.0f}));

        const auto pairs = registry.getCollidingPairs();
        REQUIRE(pairs.size() == 1);
        const auto& collision = pairs[0];
        CHECK(((collision.entityA == 1 && collision.entityB == 2) ||
               (collision.entityA == 2 && collision.entityB == 1)));
        CHECK((collision.shapeA == shapeId1 || collision.shapeA == shapeId2));
        CHECK((collision.shapeB == shapeId1 || collision.shapeB == shapeId2));
        CHECK(collision.manifold.pointCount == 1);
        CHECK(collision.manifold.points[0].point.x == Approx(2.5f));
        CHECK(collision.manifold.points[0].point.y == Approx(0.0f));
        CHECK(std::abs(collision.manifold.normal.x) == Approx(1.0f));
        CHECK(collision.manifold.normal.y == Approx(0.0f));

        const auto collisions1 = registry.getCollisions(1);
        REQUIRE(collisions1.size() == 1);
        CHECK(collisions1[0].entityA == 1);
        CHECK(collisions1[0].entityB == 2);

        const auto collisions2 = registry.getCollisions(2);
        REQUIRE(collisions2.size() == 1);
        CHECK(collisions2[0].entityA == 2);
        CHECK(collisions2[0].entityB == 1);
    }
}
