#pragma once

#include "colli2de/Shapes.hpp"
#include "collision/Manifold.hpp"
#include "geometry/Transformations.hpp"

namespace c2d
{

// --- Core collision manifold solvers ---
// All functions assume local shape definitions and a world transform for each shape.

// Circle vs Circle
Manifold collide(const Circle& circleA,
                 const Transform& transformA,
                 const Circle& circleB,
                 const Transform& transformB);

// Capsule vs Capsule
Manifold collide(const Capsule& capsuleA,
                 const Transform& transformA,
                 const Capsule& capsuleB,
                 const Transform& transformB);

// Polygon vs Polygon
Manifold collide(const Polygon& polygonA,
                 const Transform& transformA,
                 const Polygon& polygonB,
                 const Transform& transformB);

// Segment vs Segment
Manifold collide(const Segment& segmentA,
                 const Transform& transformA,
                 const Segment& segmentB,
                 const Transform& transformB);

// Circle vs Capsule
Manifold collide(const Capsule& capsule,
                 const Transform& transformA,
                 const Circle& circle,
                 const Transform& transformB);
Manifold collide(const Circle& circle,
                 const Transform& transformA,
                 const Capsule& capsule,
                 const Transform& transformB);

// Circle vs Polygon
Manifold collide(const Polygon& polygon,
                 const Transform& transformA,
                 const Circle& circle,
                 const Transform& transformB);
Manifold collide(const Circle& circle,
                 const Transform& transformA,
                 const Polygon& polygon,
                 const Transform& transformB);

// Circle vs Segment
Manifold collide(const Circle& circle,
                 const Transform& transformA,
                 const Segment& segment,
                 const Transform& transformB);
Manifold collide(const Segment& segment,
                 const Transform& transformA,
                 const Circle& circle,
                 const Transform& transformB);

// Capsule vs Polygon
Manifold collide(const Capsule& capsule,
                 const Transform& transformA,
                 const Polygon& polygon,
                 const Transform& transformB);
Manifold collide(const Polygon& polygon,
                 const Transform& transformA,
                 const Capsule& capsule,
                 const Transform& transformB);

// Capsule vs Segment
Manifold collide(const Capsule& capsule,
                 const Transform& transformA,
                 const Segment& segment,
                 const Transform& transformB);
Manifold collide(const Segment& segment,
                 const Transform& transformA,
                 const Capsule& capsule,
                 const Transform& transformB);

// Polygon vs Segment
Manifold collide(const Polygon& polygon,
                 const Transform& transformA,
                 const Segment& segment,
                 const Transform& transformB);
Manifold collide(const Segment& segment,
                 const Transform& transformA,
                 const Polygon& polygon,
                 const Transform& transformB);

} // namespace c2d
