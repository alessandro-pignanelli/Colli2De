#pragma once

#include <colli2de/Vec2.hpp>
#include <colli2de/internal/utils/Methods.hpp>

#include <cmath>

namespace c2d
{

struct Rotation
{
  private:
    constexpr Rotation(float angleRadians, float sin, float cos) : angleRadians(angleRadians), sin(sin), cos(cos) {}

  public:
    float angleRadians = 0.0f;
    float sin = 0.0f;
    float cos = 1.0f;

    constexpr Rotation() = default;

    constexpr Rotation(float angleRadians)
        : angleRadians(angleRadians),
          sin(std::sin(angleRadians)),
          cos(std::cos(angleRadians))
    {
    }

    constexpr Rotation(float sin, float cos) : angleRadians(std::atan2(sin, cos)), sin(sin), cos(cos) {}

    // Rotate a vector
    constexpr Vec2 apply(Vec2 vec) const
    {
        return Vec2{cos * vec.x - sin * vec.y, sin * vec.x + cos * vec.y};
    }

    constexpr Rotation inverse() const
    {
        return Rotation{-angleRadians, -sin, cos};
    }

    constexpr Vec2 inverse(Vec2 vec) const
    {
        return Vec2{cos * vec.x + sin * vec.y, -sin * vec.x + cos * vec.y};
    }

    constexpr Rotation operator+(const Rotation& other) const
    {
        return Rotation{
            angleRadians + other.angleRadians, sin * other.cos + cos * other.sin, cos * other.cos - sin * other.sin};
    }

    constexpr Rotation operator-(const Rotation& other) const
    {
        return Rotation{
            angleRadians - other.angleRadians, sin * other.cos - cos * other.sin, cos * other.cos + sin * other.sin};
    }

    constexpr Rotation operator+(float angle) const
    {
        return Rotation{angleRadians + angle};
    }

    constexpr Rotation operator-(float angle) const
    {
        return Rotation{angleRadians - angle};
    }

    constexpr Rotation& operator+=(const Rotation& other)
    {
        angleRadians += other.angleRadians;
        sin = sin * other.cos + cos * other.sin;
        cos = cos * other.cos - sin * other.sin;
        return *this;
    }

    constexpr Rotation& operator-=(const Rotation& other)
    {
        angleRadians -= other.angleRadians;
        sin = sin * other.cos - cos * other.sin;
        cos = cos * other.cos + sin * other.sin;
        return *this;
    }

    constexpr Rotation& operator+=(float angle)
    {
        angleRadians += angle;
        sin = std::sin(angleRadians);
        cos = std::cos(angleRadians);
        return *this;
    }

    constexpr Rotation& operator-=(float angle)
    {
        angleRadians -= angle;
        sin = std::sin(angleRadians);
        cos = std::cos(angleRadians);
        return *this;
    }

    constexpr bool operator==(const Rotation& other) const
    {
        return float_equals(angleRadians, other.angleRadians);
    }

    constexpr bool operator!=(const Rotation& other) const
    {
        return !(*this == other);
    }
};

using Translation = Vec2;

struct Transform
{
    Translation translation{};
    Rotation rotation{};

    constexpr Transform() = default;

    constexpr Transform(Translation translation) : translation(translation), rotation() {}

    constexpr Transform(Translation translation, float angleRadians) : translation(translation), rotation(angleRadians)
    {
    }

    constexpr Transform(Translation translation, Rotation rotation) : translation(translation), rotation(rotation) {}

    // Transform a vector (rotate then translate)
    constexpr Vec2 apply(Vec2 vec) const
    {
        return rotation.apply(vec) + translation;
    }

    constexpr Vec2 toLocal(Vec2 vec) const
    {
        return rotation.inverse(vec - translation);
    }

    constexpr Transform operator+(const Transform& other) const
    {
        return Transform{translation + other.translation, rotation + other.rotation};
    }

    constexpr Transform operator-(const Transform& other) const
    {
        return Transform{translation - other.translation, rotation - other.rotation};
    }

    constexpr Transform operator+(float scalar) const
    {
        return Transform{translation + scalar, rotation + scalar};
    }

    constexpr Transform operator-(float scalar) const
    {
        return Transform{translation - scalar, rotation - scalar};
    }

    constexpr Transform operator*(float scalar) const
    {
        return Transform{translation * scalar, rotation.angleRadians * scalar};
    }

    constexpr Transform operator/(float scalar) const
    {
        return Transform{translation / scalar, rotation.angleRadians / scalar};
    }

    constexpr Transform& operator+=(const Transform& other)
    {
        translation += other.translation;
        rotation += other.rotation;
        return *this;
    }

    constexpr Transform& operator-=(const Transform& other)
    {
        translation -= other.translation;
        rotation -= other.rotation;
        return *this;
    }

    constexpr Transform& operator+=(float scalar)
    {
        translation += scalar;
        rotation += scalar;
        return *this;
    }

    constexpr Transform& operator-=(float scalar)
    {
        translation -= scalar;
        rotation -= scalar;
        return *this;
    }

    constexpr Transform& operator*=(float scalar)
    {
        translation *= scalar;
        rotation = rotation.angleRadians * scalar;
        return *this;
    }

    constexpr Transform& operator/=(float scalar)
    {
        translation /= scalar;
        rotation = rotation.angleRadians / scalar;
        return *this;
    }

    constexpr bool operator==(const Transform& other) const
    {
        return translation == other.translation && rotation == other.rotation;
    }

    constexpr bool operator!=(const Transform& other) const
    {
        return !(*this == other);
    }
};

} // namespace c2d