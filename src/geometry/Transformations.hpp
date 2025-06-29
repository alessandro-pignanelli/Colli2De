#pragma once

#include <cmath>

#include "colli2de/Vec2.hpp"

namespace c2d
{

struct Rotation
{
    float sin = 0.0f;
    float cos = 1.0f;

    constexpr Rotation() = default;
    constexpr explicit Rotation(float angleRadians)
        : sin(std::sin(angleRadians)), cos(std::cos(angleRadians)) {}

    // Rotate a vector
    constexpr Vec2 apply(Vec2 vec) const
    {
        return Vec2{ cos * vec.x - sin * vec.y, sin * vec.x + cos * vec.y };
    }

    constexpr Vec2 inverse(Vec2 vec) const
    {
        return Vec2{ cos * vec.x + sin * vec.y, -sin * vec.x + cos * vec.y };
    }
};

struct Transform
{
    Vec2 translation{};
    Rotation rotation{};

    constexpr Transform() = default;
    constexpr Transform(Vec2 translation, float angleRadians)
        : translation(translation), rotation(angleRadians) {}

    // Transform a vector (rotate then translate)
    constexpr Vec2 apply(Vec2 vec) const
    {
        return rotation.apply(vec) + translation;
    }

    constexpr Vec2 toLocal(Vec2 vec) const
    {
        return rotation.inverse(vec - translation);
    }
};

} // namespace c2d