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
                 Transform transformA,
                 const Circle& circleB,
                 Transform transformB);

// Capsule vs Capsule
Manifold collide(const Capsule& capsuleA,
                 Transform transformA,
                 const Capsule& capsuleB,
                 Transform transformB);

// Polygon vs Polygon
Manifold collide(const Polygon& polygonA,
                 Transform transformA,
                 const Polygon& polygonB,
                 Transform transformB);

// Segment vs Segment
Manifold collide(const Segment& segmentA,
                 Transform transformA,
                 const Segment& segmentB,
                 Transform transformB);

// Circle vs Capsule
Manifold collide(const Capsule& capsule,
                 Transform transformA,
                 const Circle& circle,
                 Transform transformB);
Manifold collide(const Circle& circle,
                 Transform transformA,
                 const Capsule& capsule,
                 Transform transformB);

// Circle vs Polygon
Manifold collide(const Polygon& polygon,
                 Transform transformA,
                 const Circle& circle,
                 Transform transformB);
Manifold collide(const Circle& circle,
                 Transform transformA,
                 const Polygon& polygon,
                 Transform transformB);

// Circle vs Segment
Manifold collide(const Circle& circle,
                 Transform transformA,
                 const Segment& segment,
                 Transform transformB);
Manifold collide(const Segment& segment,
                 Transform transformA,
                 const Circle& circle,
                 Transform transformB);

// Capsule vs Polygon
Manifold collide(const Capsule& capsule,
                 Transform transformA,
                 const Polygon& polygon,
                 Transform transformB);
Manifold collide(const Polygon& polygon,
                 Transform transformA,
                 const Capsule& capsule,
                 Transform transformB);

// Capsule vs Segment
Manifold collide(const Capsule& capsule,
                 Transform transformA,
                 const Segment& segment,
                 Transform transformB);
Manifold collide(const Segment& segment,
                 Transform transformA,
                 const Capsule& capsule,
                 Transform transformB);

// Polygon vs Segment
Manifold collide(const Polygon& polygon,
                 Transform transformA,
                 const Segment& segment,
                 Transform transformB);
Manifold collide(const Segment& segment,
                 Transform transformA,
                 const Polygon& polygon,
                 Transform transformB);

} // namespace c2d
