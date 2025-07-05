#include <catch2/catch_test_macros.hpp>
#include "colli2de/Registry.hpp"

using namespace c2d;

TEST_CASE("Registry moveEntity updates collision status", "[Registry][Movement]")
{
    Registry<int> reg;
    reg.createEntity(1, BodyType::Dynamic, {0.0f, 0.0f});
    reg.addShape(1, Circle{{0.0f,0.0f}, 1.0f});
    reg.createEntity(2, BodyType::Static, {3.0f, 0.0f});
    reg.addShape(2, Circle{{0.0f,0.0f}, 1.0f});

    CHECK_FALSE(reg.areColliding(1,2));
    reg.moveEntity(1, {2.0f, 0.0f}, Rotation{});
    CHECK(reg.areColliding(1,2));
}
