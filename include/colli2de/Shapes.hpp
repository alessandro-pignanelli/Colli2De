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

constexpr uint8_t shapeTypesCount = 4;
using Shape0 = Circle;
using Shape1 = Capsule;
using Shape2 = Segment;
using Shape3 = Polygon;
using ShapeVariant = std::variant<Shape0, Shape1, Shape2, Shape3>;

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

    bool operator==(const Circle& other) const
    {
        return center == other.center && float_equals(radius, other.radius);
    }

    bool operator!=(const Circle& other) const
    {
        return !(*this == other);
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

    bool operator==(const Capsule& other) const
    {
        return center1 == other.center1 && center2 == other.center2 && float_equals(radius, other.radius);
    }

    bool operator!=(const Capsule& other) const
    {
        return !(*this == other);
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

    bool operator==(const Segment& other) const
    {
        return start == other.start && end == other.end;
    }

    bool operator!=(const Segment& other) const
    {
        return !(*this == other);
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

    bool operator==(const Polygon& other) const
    {
        if (count != other.count)
            return false;
        for (uint8_t i = 0; i < count; ++i)
        {
            if (vertices[i] != other.vertices[i])
                return false;
            if (normals[i] != other.normals[i])
                return false;
        }
        return true;
    }

    bool operator!=(const Polygon& other) const
    {
        return !(*this == other);
    }
};

Polygon makeRectangle(Vec2 center, float halfWidth, float halfHeight, float angle = 0.0f);
Polygon makeTriangle(Vec2 v0, Vec2 v1, Vec2 v2);
Polygon makeRegularPolygon(uint8_t n, Vec2 center, float radius, float rotationAngle = 0.0f);
Polygon makePolygon(const std::initializer_list<Vec2>& points);

} // namespace c2d

template <>
struct std::formatter<c2d::ShapeType> : std::formatter<std::string>
{
    template <typename FormatContext>
    auto format(const c2d::ShapeType& shapeType, FormatContext& ctx) const
    {
        switch (shapeType)
        {
        case c2d::ShapeType::Circle:
            return std::formatter<std::string>::format("Circle", ctx);
        case c2d::ShapeType::Capsule:
            return std::formatter<std::string>::format("Capsule", ctx);
        case c2d::ShapeType::Segment:
            return std::formatter<std::string>::format("Segment", ctx);
        case c2d::ShapeType::Polygon:
            return std::formatter<std::string>::format("Polygon", ctx);
        default:
            return std::formatter<std::string>::format("Unknown", ctx);
        }
    }
};

template <>
struct std::formatter<c2d::ShapeVariant> : std::formatter<std::string>
{
    template <typename FormatContext>
    auto format(const c2d::ShapeVariant& shapeVariant, FormatContext& ctx) const
    {
        return std::visit(
            [&](const auto& shapeConcrete)
            {
                return std::formatter<std::string>::format(
                    std::format("ShapeVariant(type={}, data={})", shapeConcrete.getType(), shapeConcrete), ctx);
            },
            shapeVariant);
    }
};

template <>
struct std::formatter<c2d::Circle> : std::formatter<std::string>
{
    template <typename FormatContext>
    auto format(const c2d::Circle& circle, FormatContext& ctx) const
    {
        return std::formatter<std::string>::format(
            std::format("Circle(center={}, radius={})", circle.center, circle.radius), ctx);
    }
};

template <>
struct std::formatter<c2d::Capsule> : std::formatter<std::string>
{
    template <typename FormatContext>
    auto format(const c2d::Capsule& capsule, FormatContext& ctx) const
    {
        return std::formatter<std::string>::format(
            std::format("Capsule(center1={}, center2={}, radius={})", capsule.center1, capsule.center2, capsule.radius),
            ctx);
    }
};

template <>
struct std::formatter<c2d::Segment> : std::formatter<std::string>
{
    template <typename FormatContext>
    auto format(const c2d::Segment& segment, FormatContext& ctx) const
    {
        return std::formatter<std::string>::format(std::format("Segment(start={}, end={})", segment.start, segment.end),
                                                   ctx);
    }
};

template <>
struct std::formatter<c2d::Polygon> : std::formatter<std::string>
{
    template <typename FormatContext>
    auto format(const c2d::Polygon& polygon, FormatContext& ctx) const
    {
        std::string vertsStr = "[";
        for (uint8_t i = 0; i < polygon.count; ++i)
        {
            vertsStr += polygon.vertices[i].toString();
            if (i < polygon.count - 1)
                vertsStr += ", ";
        }
        vertsStr += "]";

        return std::formatter<std::string>::format(
            std::format("Polygon(count={}, vertices={})", polygon.count, vertsStr), ctx);
    }
};
