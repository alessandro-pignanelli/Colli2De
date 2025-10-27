#include <colli2de/Shapes.hpp>

namespace c2d
{

// --- Create a rectangle (box) centered at `center`, with given half extents and rotation ---
Polygon makeRectangle(Vec2 center, float halfWidth, float halfHeight, float angle)
{
    assert(halfWidth > 0.0f && halfHeight > 0.0f);

    Polygon poly;
    poly.count = 4;

    const float cosA = std::cos(angle);
    const float sinA = std::sin(angle);

    // Local corners
    const std::array<Vec2, 4> corners = {Vec2{-halfWidth, -halfHeight},
                                         Vec2{halfWidth, -halfHeight},
                                         Vec2{halfWidth, halfHeight},
                                         Vec2{-halfWidth, halfHeight}};

    // Transform corners to world
    for (uint8_t i = 0; i < 4; ++i)
    {
        const float x = corners[i].x * cosA - corners[i].y * sinA + center.x;
        const float y = corners[i].x * sinA + corners[i].y * cosA + center.y;
        poly.vertices[i] = Vec2{x, y};
    }

    poly.computeNormals();

    return poly;
}

// --- Create a triangle from three points ---
Polygon makeTriangle(Vec2 v0, Vec2 v1, Vec2 v2)
{
    Polygon poly;
    poly.count = 3;
    poly.vertices[0] = v0;
    poly.vertices[1] = v1;
    poly.vertices[2] = v2;

    poly.computeNormals();

    return poly;
}

// --- Create a regular convex n-gon (centered at `center`, given radius, n >= 3) ---
Polygon makeRegularPolygon(uint8_t n, Vec2 center, float radius, float rotationAngle)
{
    assert(n >= 3 && n <= MAX_POLYGON_VERTICES);

    Polygon poly;
    poly.count = n;

    const float step = 2.0f * PI / float(n);
    for (uint8_t i = 0; i < n; ++i)
    {
        const float theta = rotationAngle + step * i;
        poly.vertices[i] = center + radius * Vec2{std::cos(theta), std::sin(theta)};
    }

    poly.computeNormals();

    return poly;
}

// --- Convex hull from a list of points (Graham scan or Andrew's monotone chain, stubbed here) ---
// TODO: Implement a real convex hull for robustness!
Polygon makePolygon(const std::initializer_list<Vec2>& points)
{
    assert(points.size() >= 3 && points.size() <= MAX_POLYGON_VERTICES);

    Polygon poly;
    // TODO: Implement robust convex hull algorithm, for now just copy as-is
    poly.count = static_cast<uint8_t>(points.size());

    std::copy(points.begin(), points.end(), poly.vertices.begin());

    poly.computeNormals();

    return poly;
}

} // namespace c2d
