#pragma once

#include <colli2de/Vec2.hpp>

#include <algorithm>
#include <cstdint>

namespace c2d
{

// Maximum points per manifold in 2D
constexpr std::size_t MaxManifoldPoints = 2;

// Single contact point in a manifold
struct ManifoldPoint
{
    Vec2 point;       // Contact point in absolute coordinates
    Vec2 anchorA;     // Contact point relative to shape A origin
    Vec2 anchorB;     // Contact point relative to shape B origin
    float separation; // Negative if overlapping, positive if separated

    // Constructors
    constexpr ManifoldPoint() = default;

    std::string toString() const
    {
        return std::format("ManifoldPoint(point={}, anchorA={}, anchorB={}, separation={})",
                           point.toString(),
                           anchorA.toString(),
                           anchorB.toString(),
                           separation);
    }
};

struct Manifold
{
    Vec2 normal;                             // Direction of collision, from shape A to shape B
    ManifoldPoint points[MaxManifoldPoints]; // Up to 2 contact points
    uint8_t pointCount{0};                   // Actual number of contact points (0, 1, or 2)

    constexpr Manifold() = default;

    constexpr void reverse()
    {
        normal = -normal;
        for (uint8_t i = 0; i < pointCount; ++i)
        {
            std::swap(points[i].anchorA, points[i].anchorB);
        }
    }

    bool isColliding() const
    {
        return pointCount > 0;
    }

    std::string toString() const
    {
        std::string result = "Manifold(normal=" + normal.toString() + ", points=[";
        for (uint8_t i = 0; i < pointCount; ++i)
        {
            result += points[i].toString();
            if (i < pointCount - 1)
                result += ", ";
        }
        result += "], pointCount=" + std::to_string(pointCount) + ")";
        return result;
    }
};

struct SweepManifold
{
    float fraction{};
    Manifold manifold{};
};

} // namespace c2d

template <>
struct std::formatter<c2d::ManifoldPoint> : std::formatter<std::string>
{
    template <typename FormatContext>
    auto format(const c2d::ManifoldPoint& point, FormatContext& ctx) const
    {
        return std::formatter<std::string>::format(point.toString(), ctx);
    }
};

template <>
struct std::formatter<c2d::Manifold> : std::formatter<std::string>
{
    template <typename FormatContext>
    auto format(const c2d::Manifold& manifold, FormatContext& ctx) const
    {
        return std::formatter<std::string>::format(manifold.toString(), ctx);
    }
};

template <>
struct std::formatter<c2d::SweepManifold> : std::formatter<std::string>
{
    template <typename FormatContext>
    auto format(const c2d::SweepManifold& sweepManifold, FormatContext& ctx) const
    {
        return std::formatter<std::string>::format(
            std::format("SweepManifold(fraction={}, manifold={})", sweepManifold.fraction, sweepManifold.manifold),
            ctx);
    }
};
