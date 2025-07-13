#pragma once

#include <colli2de/Vec2.hpp>

namespace c2d
{

struct Ray
{
    Vec2 start;
    Vec2 end;
};

struct InfiniteRay
{
    Vec2 start;
    Vec2 direction;
};

template <typename RayType>
concept IsRay = std::is_same_v<RayType, Ray> || std::is_same_v<RayType, InfiniteRay>;

template <typename IdType>
struct RaycastHit
{
    IdType id;
    Vec2 entry;
    Vec2 exit;
    float entryTime;
    float exitTime;

    static RaycastHit<IdType> fromRay(IdType id, Ray ray, float entryTime, float exitTime)
    {
        return RaycastHit<IdType>
        {
            id,
            ray.start + (ray.end - ray.start) * entryTime,
            ray.start + (ray.end - ray.start) * exitTime,
            entryTime,
            exitTime
        };
    }
    static RaycastHit<IdType> fromRay(IdType id, Ray ray, std::pair<float, float> intersectionTimes)
    {
        return RaycastHit<IdType>
        {
            id,
            ray.start + (ray.end - ray.start) * intersectionTimes.first,
            ray.start + (ray.end - ray.start) * intersectionTimes.second,
            intersectionTimes.first,
            intersectionTimes.second
        };
    }
    static RaycastHit<IdType> fromRay(IdType id, InfiniteRay ray, float entryTime, float exitTime)
    {
        return RaycastHit<IdType>
        {
            id,
            ray.start + ray.direction * entryTime,
            ray.start + ray.direction * exitTime,
            entryTime,
            exitTime
        };
    }
    static RaycastHit<IdType> fromRay(IdType id, InfiniteRay ray, std::pair<float, float> intersectionTimes)
    {
        return RaycastHit<IdType>
        {
            id,
            ray.start + ray.direction * intersectionTimes.first,
            ray.start + ray.direction * intersectionTimes.second,
            intersectionTimes.first,
            intersectionTimes.second
        };
    }

    bool operator==(const RaycastHit<IdType>& other) const
    {
        return id == other.id;
    }
    bool operator!=(const RaycastHit<IdType>& other) const
    {
        return !(*this == other);
    }
    bool operator<(const RaycastHit<IdType>& other) const
    {
        return entryTime < other.entryTime || (entryTime == other.entryTime && exitTime < other.exitTime);
    }
    bool operator>(const RaycastHit<IdType>& other) const
    {
        return entryTime > other.entryTime || (entryTime == other.entryTime && exitTime > other.exitTime);
    }
};

}