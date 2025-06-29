#pragma once

#include <optional>

#include "colli2de/Shapes.hpp"
#include "colli2de/Vec2.hpp"
#include "geometry/Transformations.hpp"
#include "collision/Collision.hpp"
#include <cmath>

namespace c2d
{

struct SweepHit
{
    float fraction{};
    Manifold manifold{};
};

template <typename ShapeA, typename ShapeB>
std::optional<SweepHit> sweep(const ShapeA& movingShape,
                              Transform startTransform,
                              Vec2 translation,
                              Rotation endRotation,
                              const ShapeB& targetShape,
                              Transform targetTransform)
{
    const float startAngle = std::atan2(startTransform.rotation.sin, startTransform.rotation.cos);
    const float endAngle = std::atan2(endRotation.sin, endRotation.cos);

    auto manifoldAt = [&](float fraction) -> Manifold
    {
        Transform currentTransform;
        currentTransform.translation = startTransform.translation + translation * fraction;
        const float currentAngle = startAngle + (endAngle - startAngle) * fraction;
        currentTransform.rotation = Rotation{currentAngle};
        return collide(movingShape, currentTransform, targetShape, targetTransform);
    };

    Manifold initialManifold = manifoldAt(0.0f);
    if (initialManifold.pointCount > 0)
        return SweepHit{0.0f, initialManifold};

    const int32_t coarseSteps = 8;
    const int32_t refinementSteps = 10;
    float fractionLower = 0.0f;
    float fractionUpper = 1.0f;
    Manifold hitManifold{};

    for (int32_t stepIndex = 1; stepIndex <= coarseSteps; ++stepIndex)
    {
        const float testFraction = static_cast<float>(stepIndex) / static_cast<float>(coarseSteps);
        Manifold testManifold = manifoldAt(testFraction);
        if (testManifold.pointCount > 0)
        {
            fractionLower = static_cast<float>(stepIndex - 1) / static_cast<float>(coarseSteps);
            fractionUpper = testFraction;
            hitManifold = testManifold;
            break;
        }

        if (stepIndex == coarseSteps)
            return std::nullopt;
    }

    for (int32_t iteration = 0; iteration < refinementSteps; ++iteration)
    {
        const float midFraction = 0.5f * (fractionLower + fractionUpper);
        Manifold midManifold = manifoldAt(midFraction);
        if (midManifold.pointCount > 0)
        {
            fractionUpper = midFraction;
            hitManifold = midManifold;
        }
        else
        {
            fractionLower = midFraction;
        }
    }

    return SweepHit{fractionUpper, hitManifold};
}

} // namespace c2d

