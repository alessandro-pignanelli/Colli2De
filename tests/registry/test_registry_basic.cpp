#include <catch2/catch_test_macros.hpp>
#include "colli2de/Registry.hpp"

using namespace c2d;

TEST_CASE("Registry entity creation and shape management", "[Registry]")
{
    Registry<int> reg;
    reg.createEntity(1, BodyType::Static, Transform({0.0f, 0.0f}));
    reg.addShape(1, Circle{ {0.0f, 0.0f}, 1.0f });
    reg.createEntity(2, BodyType::Dynamic, Transform({3.0f, 0.0f}));
    reg.addShape(2, Circle{ {0.0f, 0.0f}, 1.0f });

    CHECK_FALSE(reg.areColliding(1, 2));

    reg.teleportEntity(2, Transform({1.0f, 0.0f}));
    CHECK(reg.areColliding(1, 2));

    auto collisions = reg.getCollidingPairs();
    REQUIRE(collisions.size() == 1);
    CHECK(((collisions[0].entityA == 1 && collisions[0].entityB == 2) || (collisions[0].entityA == 2 && collisions[0].entityB == 1)));

    collisions = reg.getCollisions(1);
    REQUIRE(collisions.size() == 1);
    CHECK(collisions[0].entityB == 2);

    reg.removeEntity(2);
    CHECK(reg.getCollidingPairs().empty());
}
