#pragma once

#include <optional>
#include <utility>

#include "colli2de/Ray.hpp"
#include "colli2de/Shapes.hpp"
#include "geometry/Transformations.hpp"

namespace c2d
{

std::optional<std::pair<float, float>> raycast(const Circle& circle,
                                               Transform transform,
                                               Ray ray);

std::optional<std::pair<float, float>> raycast(const Circle& circle,
                                               Transform transform,
                                               InfiniteRay ray);

std::optional<std::pair<float, float>> raycast(const Capsule& capsule,
                                               Transform transform,
                                               Ray ray);

std::optional<std::pair<float, float>> raycast(const Capsule& capsule,
                                               Transform transform,
                                               InfiniteRay ray);

std::optional<std::pair<float, float>> raycast(const Segment& segment,
                                               Transform transform,
                                               Ray ray);

std::optional<std::pair<float, float>> raycast(const Segment& segment,
                                               Transform transform,
                                               InfiniteRay ray);

std::optional<std::pair<float, float>> raycast(const Polygon& polygon,
                                               Transform transform,
                                               Ray ray);

std::optional<std::pair<float, float>> raycast(const Polygon& polygon,
                                               Transform transform,
                                               InfiniteRay ray);

} // namespace c2d
