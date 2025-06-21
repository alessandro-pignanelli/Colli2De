#pragma once

#include "colli2de/Vec2.hpp"

namespace c2d
{

struct Ray
{
    Vec2 p1;
    Vec2 p2;
};

template <typename IdType>
struct RaycastHit
{
    IdType id;
    Vec2 entry;
    Vec2 exit;

    static RaycastHit<IdType> fromRay(IdType id, Ray ray, float entryTime, float exitTime)
    {
        return RaycastHit<IdType>
        {
            id,
            ray.p1 + (ray.p2 - ray.p1) * entryTime,
            ray.p1 + (ray.p2 - ray.p1) * exitTime
        };
    }
    static RaycastHit<IdType> fromRay(IdType id, Ray ray, std::pair<float, float> intersectionTimes)
    {
        return RaycastHit<IdType>
        {
            id,
            ray.p1 + (ray.p2 - ray.p1) * intersectionTimes.first,
            ray.p1 + (ray.p2 - ray.p1) * intersectionTimes.second
        };
    }
};

}