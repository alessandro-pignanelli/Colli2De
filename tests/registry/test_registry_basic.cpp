#include <catch2/catch_test_macros.hpp>
#include "colli2de/Registry.hpp"

using namespace c2d;

TEST_CASE("Registry entity creation and shape management", "[Registry]")
{
    Registry<int> reg;
    reg.createEntity(1, BodyType::Dynamic, {0.0f, 0.0f});
    reg.addShape(1, Circle{ {0.0f, 0.0f}, 1.0f });
    reg.createEntity(2, BodyType::Static, {3.0f, 0.0f});
    reg.addShape(2, Circle{ {0.0f, 0.0f}, 1.0f });

    CHECK_FALSE(reg.areColliding(1, 2));

    reg.setPosition(2, {1.0f, 0.0f}, Rotation{});
    CHECK(reg.areColliding(1, 2));

    auto pairs = reg.getCollidingPairs();
    REQUIRE(pairs.size() == 1);
    CHECK(((pairs[0].first == 1 && pairs[0].second == 2) || (pairs[0].first == 2 && pairs[0].second == 1)));

    auto ids = reg.getCollisions(1);
    REQUIRE(ids.size() == 1);
    CHECK(ids[0] == 2);

    reg.removeEntity(2);
    CHECK(reg.getCollidingPairs().empty());
}
