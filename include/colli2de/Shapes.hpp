#pragma once

#include <colli2de/Constants.hpp>
#include <colli2de/Vec2.hpp>

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <variant>

namespace c2d
{

enum class ShapeType : uint8_t
{
    Circle,
    Capsule,
    Segment,
    Polygon
};

struct Circle;
struct Capsule;
struct Segment;
struct Polygon;

template <typename ShapeType>
concept IsShape = std::is_same_v<ShapeType, Circle> || std::is_same_v<ShapeType, Capsule> ||
                  std::is_same_v<ShapeType, Segment> || std::is_same_v<ShapeType, Polygon>;

using ShapeVariant = std::variant<Circle, Capsule, Segment, Polygon>;

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
};

// Capsule (for segments with round ends)
struct Capsule
{
    Vec2 center1{};  // Center of end 1
    Vec2 center2{};  // Center of end 2
    float radius{1}; // Radius of the two semi-circles at the ends
    // Height = (center2.y - center1.y) + 2 * radius
    // Width = (center2.x - center1.x) + 2 * radius

    constexpr Capsule() = default;

    constexpr Capsule(Vec2 center1, Vec2 center2, float radius) : center1(center1), center2(center2), radius(radius) {}

    constexpr ShapeType getType() const
    {
        return ShapeType::Capsule;
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
};

struct Polygon
{
    std::array<Vec2, MAX_POLYGON_VERTICES> vertices{};
    std::array<Vec2, MAX_POLYGON_VERTICES> normals{}; // Outward edge normals (for SAT)
    uint8_t count{0};                                 // Number of vertices actually used

    constexpr Polygon() = default;

    // Polygon with N vertices
    template <std::size_t N>
    constexpr Polygon(const std::array<Vec2, N>& verts, uint8_t count) requires(N <= MAX_POLYGON_VERTICES)
        : count(count)
    {
        for (uint8_t i = 0; i < count; ++i)
            vertices[i] = verts[i];
        computeNormals();
    }

    constexpr ShapeType getType() const
    {
        return ShapeType::Polygon;
    }

    constexpr void computeNormals()
    {
        const auto lastEdgeIndex = count - 1;
        for (uint8_t i = 0; i < lastEdgeIndex; ++i)
        {
            const Vec2 edge = vertices[(i + 1)] - vertices[i];
            normals[i] = Vec2::normalized(edge.y, -edge.x); // Right-hand normal (CCW)
        }
        {
            const Vec2 edge = vertices[0] - vertices[lastEdgeIndex];
            normals[lastEdgeIndex] = Vec2::normalized(edge.y, -edge.x); // Right-hand normal (CCW)
        }
    }
};

Polygon makeRectangle(Vec2 center, float halfWidth, float halfHeight, float angle = 0.0f);
Polygon makeTriangle(Vec2 v0, Vec2 v1, Vec2 v2);
Polygon makeRegularPolygon(uint8_t n, Vec2 center, float radius, float rotationAngle = 0.0f);
Polygon makePolygon(const std::initializer_list<Vec2>& points);

} // namespace c2d
