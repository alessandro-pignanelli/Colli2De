#include <catch2/catch_test_macros.hpp>

#include "colli2de/Registry.hpp"
#include "utils/Print.hpp"

using namespace c2d;

TEST_CASE("Registry entity creation and shape management", "[Registry][Basic]")
{
    Registry<int> reg;

    // SECTION("Create and add shapes to an entity, non-colliding")
    {
        reg.createEntity(1, BodyType::Static, Transform({0.0f, 0.0f}));
        reg.addShape(1, Circle{ {0.0f, 0.0f}, 1.0f });
        reg.createEntity(2, BodyType::Dynamic, Transform({3.0f, 0.0f}));
        reg.addShape(2, Circle{ {0.0f, 0.0f}, 1.0f });
        
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
        reg.addShape(3, Circle{ {0.0f, 0.0f}, 1.0f });

        // Colliding with 2
        reg.createEntity(4, BodyType::Dynamic, Transform({1.0f, 2.0f}));
        reg.addShape(4, Circle{ {0.0f, 0.0f}, 1.0f });

        // Not colliding
        reg.createEntity(5, BodyType::Dynamic, Transform({1.0f, 5.0f}));
        reg.addShape(5, Circle{ {0.0f, 0.0f}, 1.0f });

        const std::vector<std::pair<int, int>> expectedCollisions = {
            {1, 2}, {1, 3}, {2, 3}, {2, 4}
        };
        
        auto collisions = reg.getCollidingPairs();
        println("Collisions found: {}", collisions.size());
        for (const auto& collision : collisions)
        {
            println("Checking collision between {} and {}", collision.entityA, collision.entityB);
        }

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

    // SECTION("Remove shapes and check collisions")
    {
        reg.removeEntity(2);
        CHECK(reg.getCollidingPairs().size() == 1); // Only {1, 3} should collide now
        
        reg.removeEntity(1);
        CHECK(reg.getCollidingPairs().empty());
    }
}
