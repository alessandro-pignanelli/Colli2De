#pragma once

#include <colli2de/Ray.hpp>
#include <colli2de/Vec2.hpp>

#include <format>
#include <optional>
#include <string>

namespace c2d
{

struct AABB
{
    Vec2 min;
    Vec2 max;

    AABB() = default;

    AABB(Vec2 min, Vec2 max) : min{min}, max{max} {}

    static AABB fromCenter(Vec2 center, Vec2 halfSize);

    // Returns the smallest AABB that contains both a and b
    static AABB combine(AABB a, AABB b);

    Vec2 center() const;
    Vec2 size() const;

    bool contains(Vec2 point) const;
    bool contains(AABB other) const;
    bool intersects(AABB other) const;
    // Returns the intersection times of the ray with this AABB
    std::optional<std::pair<float, float>> intersects(Ray ray) const;
    std::optional<std::pair<float, float>> intersects(InfiniteRay ray) const;

    void fatten(float margin);
    void fatten(Vec2 displacement);
    void fatten(float margin, Vec2 displacement);

    AABB fattened(float margin) const;
    AABB fattened(Vec2 displacement) const;
    AABB fattened(float margin, Vec2 displacement) const;

    void translate(Vec2 direction);
    AABB translated(Vec2 direction) const;

    void expandToInclude(Vec2 point);
    AABB expandedToInclude(Vec2 point) const;

    float perimeter() const;

    std::string toString() const;

    AABB operator+(Vec2 offset) const;
    AABB operator-(Vec2 offset) const;
    AABB& operator+=(Vec2 offset);
    AABB& operator-=(Vec2 offset);

    Vec2 operator+(AABB other) const;
    Vec2 operator-(AABB other) const;

    bool operator==(const AABB& other) const
    {
        return min == other.min && max == other.max;
    }

    bool operator!=(const AABB& other) const
    {
        return !(*this == other);
    }
};

static_assert(std::is_trivially_copyable_v<AABB>, "AABB must be trivially copyable");

} // namespace c2d

// Specialization for std::formatter to allow formatted output of AABB
template <> struct std::formatter<c2d::AABB> : std::formatter<std::string>
{
    template <typename FormatContext> auto format(c2d::AABB aabb, FormatContext& ctx) const
    {
        return std::formatter<std::string>::format(aabb.toString(), ctx);
    }
};
