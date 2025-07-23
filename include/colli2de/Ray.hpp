#pragma once

#include <sstream>
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
    bool operator<=(const RaycastHit<IdType>& other) const
    {
        return !(*this > other);
    }
    bool operator>=(const RaycastHit<IdType>& other) const
    {
        return !(*this < other);
    }
};

}

// Specialization for std::formatter to allow formatted output of Ray
template<>
struct std::formatter<c2d::Ray> : std::formatter<std::string>
{
    template<typename FormatContext>
    auto format(c2d::Ray ray, FormatContext& ctx) const
    {
        std::stringstream ss;
        ss << "Ray(start: " << ray.start << ", end: " << ray.end << ")";
        return std::formatter<std::string>::format(ss.str(), ctx);
    }
};

// Specialization for std::formatter to allow formatted output of InfiniteRay
template<>
struct std::formatter<c2d::InfiniteRay> : std::formatter<std::string>
{
    template<typename FormatContext>
    auto format(c2d::InfiniteRay ray, FormatContext& ctx) const
    {
        std::stringstream ss;
        ss << "InfiniteRay(start: " << ray.start << ", direction: " << ray.direction << ")";
        return std::formatter<std::string>::format(ss.str(), ctx);
    }
};

// Specialization for std::formatter to allow formatted output of RaycastHit
template<typename IdType>
struct std::formatter<c2d::RaycastHit<IdType>> : std::formatter<std::string>
{
    template<typename FormatContext>
    auto format(c2d::RaycastHit<IdType> hit, FormatContext& ctx) const
    {
        std::stringstream ss;
        ss << "RaycastHit(id: " << hit.id << ", entry: " << hit.entry << ", exit: " << hit.exit
           << ", entryTime: " << hit.entryTime << ", exitTime: " << hit.exitTime << ")";
        return std::formatter<std::string>::format(ss.str(), ctx);
    }
};
