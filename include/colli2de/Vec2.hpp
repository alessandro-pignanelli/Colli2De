#pragma once

#include <colli2de/internal/utils/Methods.hpp>

#include <cassert>
#include <cmath>
#include <format>
#include <iostream>
#include <string>

namespace c2d
{

class Vec2
{
  public:
    float x;
    float y;

    constexpr Vec2() : Vec2(0, 0) {}

    constexpr Vec2(float x, float y) : x(x), y(y) {}

    constexpr Vec2(const Vec2& other) = default;
    constexpr Vec2& operator=(const Vec2& other) = default;
    constexpr Vec2(Vec2&& other) noexcept = default;
    constexpr Vec2& operator=(Vec2&& other) noexcept = default;

    std::string toString() const;

    constexpr float length() const;
    constexpr float inverseLength() const;
    constexpr float lengthSqr() const;

    constexpr Vec2 interpolate(Vec2 other, float percentage) const;
    constexpr float dot(Vec2 other) const;
    constexpr float cross(Vec2 other) const;

    constexpr Vec2 normalize() const;

    static constexpr Vec2 normalized(float x, float y)
    {
        const float inverseLen = 1.0f / std::sqrt(x * x + y * y);
        return Vec2(x * inverseLen, y * inverseLen);
    }

    constexpr float getBigger() const;
    constexpr Vec2 abs() const;

    constexpr Vec2& operator+=(Vec2 other);
    constexpr Vec2& operator-=(Vec2 other);
    constexpr Vec2& operator*=(Vec2 other);
    constexpr Vec2& operator/=(Vec2 other);
    constexpr Vec2& operator+=(float value);
    constexpr Vec2& operator-=(float value);
    constexpr Vec2& operator*=(float value);
    constexpr Vec2& operator/=(float value);

    constexpr Vec2 operator+(Vec2 other) const;
    constexpr Vec2 operator-(Vec2 other) const;
    constexpr Vec2 operator*(Vec2 other) const;
    constexpr Vec2 operator/(Vec2 other) const;
    constexpr Vec2 operator+(float value) const;
    constexpr Vec2 operator-(float value) const;
    constexpr Vec2 operator*(float value) const;
    constexpr Vec2 operator/(float value) const;
    constexpr friend Vec2 operator+(float value, Vec2 vec);
    constexpr friend Vec2 operator-(float value, Vec2 vec);
    constexpr friend Vec2 operator*(float value, Vec2 vec);
    constexpr friend Vec2 operator/(float value, Vec2 vec);

    constexpr Vec2 operator-() const;

    constexpr float operator[](int index) const
    {
        assert(index == 0 || index == 1);
        return index == 0 ? x : y;
    }

    constexpr bool operator==(Vec2 other) const;
    constexpr bool operator!=(Vec2 other) const;
    constexpr bool operator<(Vec2 other) const;
    constexpr bool operator>(Vec2 other) const;

    friend std::ostream& operator<<(std::ostream& os, const Vec2& vec)
    {
        return os << vec.toString();
    }
};

inline std::string Vec2::toString() const
{
    return std::format("Vec2({}, {})", x, y);
}

constexpr float Vec2::length() const
{
    return std::sqrt(x * x + y * y);
}

constexpr float Vec2::inverseLength() const
{
    return 1.0f / std::sqrt(x * x + y * y);
}

constexpr float Vec2::lengthSqr() const
{
    return x * x + y * y;
}

constexpr Vec2 Vec2::interpolate(Vec2 other, float percentage) const
{
    return Vec2(x + (other.x - x) * percentage, y + (other.y - y) * percentage);
}

constexpr float Vec2::dot(Vec2 other) const
{
    return x * other.x + y * other.y;
}

constexpr Vec2 Vec2::normalize() const
{
    const float multiplier = inverseLength();
    return Vec2(x * multiplier, y * multiplier);
}

constexpr float Vec2::getBigger() const
{
    return x > y ? x : y;
}

constexpr Vec2 Vec2::abs() const
{
    return Vec2(std::abs(x), std::abs(y));
}

// Arithmetic compound assignment
constexpr Vec2& Vec2::operator+=(Vec2 other)
{
    x += other.x;
    y += other.y;
    return *this;
}

constexpr Vec2& Vec2::operator-=(Vec2 other)
{
    x -= other.x;
    y -= other.y;
    return *this;
}

constexpr Vec2& Vec2::operator*=(Vec2 other)
{
    x *= other.x;
    y *= other.y;
    return *this;
}

constexpr Vec2& Vec2::operator/=(Vec2 other)
{
    x /= other.x;
    y /= other.y;
    return *this;
}

// Scalar compound assignment
constexpr Vec2& Vec2::operator+=(float value)
{
    x += value;
    y += value;
    return *this;
}

constexpr Vec2& Vec2::operator-=(float value)
{
    x -= value;
    y -= value;
    return *this;
}

constexpr Vec2& Vec2::operator*=(float value)
{
    x *= value;
    y *= value;
    return *this;
}

constexpr Vec2& Vec2::operator/=(float value)
{
    x /= value;
    y /= value;
    return *this;
}

constexpr Vec2 Vec2::operator+(Vec2 other) const
{
    Vec2 result = *this;
    result += other;
    return result;
}

constexpr Vec2 Vec2::operator-(Vec2 other) const
{
    Vec2 result = *this;
    result -= other;
    return result;
}

constexpr Vec2 Vec2::operator*(Vec2 other) const
{
    Vec2 result = *this;
    result *= other;
    return result;
}

constexpr Vec2 Vec2::operator/(Vec2 other) const
{
    Vec2 result = *this;
    result /= other;
    return result;
}

constexpr Vec2 Vec2::operator+(float value) const
{
    Vec2 result = *this;
    result += value;
    return result;
}

constexpr Vec2 Vec2::operator-(float value) const
{
    Vec2 result = *this;
    result -= value;
    return result;
}

constexpr Vec2 Vec2::operator*(float value) const
{
    Vec2 result = *this;
    result *= value;
    return result;
}

constexpr Vec2 Vec2::operator/(float value) const
{
    Vec2 result = *this;
    result /= value;
    return result;
}

constexpr Vec2 operator+(float value, Vec2 vec)
{
    return Vec2(vec.x + value, vec.y + value);
}

constexpr Vec2 operator-(float value, Vec2 vec)
{
    return Vec2(value - vec.x, value - vec.y);
}

constexpr Vec2 operator*(float value, Vec2 vec)
{
    return Vec2(vec.x * value, vec.y * value);
}

constexpr Vec2 operator/(float value, Vec2 vec)
{
    return Vec2(value / vec.x, value / vec.y);
}

constexpr Vec2 Vec2::operator-() const
{
    return Vec2(-x, -y);
}

constexpr bool Vec2::operator==(Vec2 other) const
{
    return float_equals(x, other.x) && float_equals(y, other.y);
}

constexpr bool Vec2::operator!=(Vec2 other) const
{
    return !(*this == other);
}

constexpr bool Vec2::operator<(Vec2 other) const
{
    if (x < other.x)
        return true;
    if (x > other.x)
        return false;
    return y < other.y;
}

constexpr bool Vec2::operator>(Vec2 other) const
{
    if (x > other.x)
        return true;
    if (x < other.x)
        return false;
    return y > other.y;
}

constexpr float Vec2::cross(Vec2 other) const
{
    return x * other.y - y * other.x;
}

static_assert(std::is_trivially_copyable_v<Vec2>, "Vec2 must be trivially copyable");

} // namespace c2d

// Specialization for std::formatter to allow formatted output of Vec2
template <>
struct std::formatter<c2d::Vec2> : std::formatter<std::string>
{
    template <typename FormatContext>
    auto format(c2d::Vec2 vec, FormatContext& ctx) const
    {
        return std::formatter<std::string>::format(vec.toString(), ctx);
    }
};
