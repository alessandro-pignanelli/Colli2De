#pragma once

#include <array>
#include <cstdint>
#include <cstddef>
#include <cassert>

#include "colli2de/Vec2.hpp"
#include "colli2de/Constants.hpp"
#include "geometry/AABB.hpp"

namespace c2d
{

enum class ShapeType : uint8_t
{
    Circle,
    Capsule,
    Segment,
    Polygon
};

struct Circle
{
    Vec2 center{};   // Local-space center
    float radius{1}; // Must be >= 0

    constexpr Circle() = default;
    constexpr Circle(Vec2 center, float radius) : center(center), radius(radius) {}

    constexpr ShapeType getType() const
    {
        return ShapeType::Circle;
    }

    constexpr AABB aabb() const
    {
        const auto halfSize = Vec2{radius, radius};
        return AABB{center - halfSize, center + halfSize};
    }
};


// Capsule (for segments with round ends)
struct Capsule
{
    Vec2 center1{};      // Center of end 1
    Vec2 center2{};      // Center of end 2
    float radius{1};     // Radius of the two semi-circles at the ends
    // Height = (center2.y - center1.y) + 2 * radius
    // Width = (center2.x - center1.x) + 2 * radius

    constexpr Capsule() = default;
    constexpr Capsule(Vec2 center1, Vec2 center2, float radius) : center1(center1), center2(center2), radius(radius) {}

    constexpr ShapeType getType() const
    {
        return ShapeType::Capsule;
    }

    constexpr AABB aabb() const
    {
        const auto halfSize = Vec2{radius, radius};
        const auto minX = std::min(center1.x, center2.x) - radius;
        const auto maxX = std::max(center1.x, center2.x) + radius;
        const auto minY = std::min(center1.y, center2.y) - radius;
        const auto maxY = std::max(center1.y, center2.y) + radius;
        return AABB{Vec2{minX, minY}, Vec2{maxX, maxY}};
    }
};


// Segment (line, two-sided, zero radius)
struct Segment
{
    Vec2 start{};
    Vec2 end{};

    constexpr Segment() = default;
    constexpr Segment(Vec2 start, Vec2 end) : start(start), end(end) {}

    constexpr ShapeType getType() const
    {
        return ShapeType::Segment;
    }

    constexpr AABB aabb() const
    {
        const auto minX = std::min(start.x, end.x);
        const auto maxX = std::max(start.x, end.x);
        const auto minY = std::min(start.y, end.y);
        const auto maxY = std::max(start.y, end.y);
        return AABB{Vec2{minX, minY}, Vec2{maxX, maxY}};
    }
};


struct Polygon
{
    std::array<Vec2, MAX_POLYGON_VERTICES> vertices{};
    std::array<Vec2, MAX_POLYGON_VERTICES> normals{}; // Outward edge normals (optional, for SAT)
    uint8_t count{0}; // Number of vertices actually used
    float radius{0}; // Rounding radius (for Minkowski sum)

    constexpr Polygon() = default;

    // Polygon with N vertices (radius optional)
    template<std::size_t N>
    constexpr Polygon(const std::array<Vec2, N>& verts, uint8_t count, float radius = 0)
        requires(N <= MAX_POLYGON_VERTICES) : count(count), radius(radius)
    {
        for (uint8_t i = 0; i < count; ++i)
            vertices[i] = verts[i];
    }

    constexpr ShapeType getType() const
    {
        return ShapeType::Polygon;
    }

    constexpr AABB aabb() const
    {
        Vec2 min = vertices[0];
        Vec2 max = vertices[0];
        for (uint8_t i = 1; i < count; ++i)
        {
            min = std::min(min, vertices[i]);
            max = std::max(max, vertices[i]);
        }
        return AABB{min - Vec2{radius, radius}, max + Vec2{radius, radius}};
    }
};

Polygon makeRectangle(Vec2 center, float halfWidth, float halfHeight, float angle = 0.0f, float radius = 0.0f);
Polygon makeTriangle(Vec2 v0, Vec2 v1, Vec2 v2, float radius = 0.0f);
Polygon makeRegularPolygon(Vec2 center, float radius, uint8_t n, float angle = 0.0f, float rounding = 0.0f);
Polygon makePolygon(const std::initializer_list<Vec2>& points, float radius = 0.0f);

} // namespace c2d
