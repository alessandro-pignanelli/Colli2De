#include <catch2/catch_test_macros.hpp>
#include "colli2de/Registry.hpp"

using namespace c2d;

TEST_CASE("Registry rayCast hits nearest entity", "[Registry][Raycast]")
{
    Registry<int> reg;
    reg.createEntity(1, BodyType::Static, {0.0f, 0.0f});
    reg.addShape(1, Circle{ {0.0f, 0.0f}, 1.0f });
    reg.createEntity(2, BodyType::Static, {3.0f, 0.0f});
    reg.addShape(2, Circle{ {0.0f, 0.0f}, 1.0f });

    Ray ray{ {-2.0f, 0.0f}, {2.0f, 0.0f} };
    auto hit = reg.rayCast(ray);
    REQUIRE(hit);
    CHECK(hit->id == 1);

    Ray ray2{ {4.0f, 0.0f}, {-2.0f, 0.0f} };
    hit = reg.rayCast(ray2);
    REQUIRE(hit);
    CHECK(hit->id == 2);
}
