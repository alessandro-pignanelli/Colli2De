#pragma once

#include <variant>
#include <vector>
#include <unordered_map>
#include <functional>

#include "collision/Manifold.hpp"
#include "colli2de/Ray.hpp"
#include "colli2de/Shapes.hpp"
#include "colli2de/Vec2.hpp"
#include "data_structures/BroadPhaseTree.hpp"
#include "geometry/AABB.hpp"
#include "geometry/Transformations.hpp"
#include "geometry/ShapesComputations.hpp"
#include "geometry/RaycastShapes.hpp"
#include "collision/Collision.hpp"

namespace c2d
{

enum class BodyType : uint8_t
{
    Static = 0,
    Dynamic,
};

template<typename EntityId>
class Registry
{
public:
    void createEntity(EntityId id, BodyType type, Vec2 position, Rotation rotation = {});

    void addShape(EntityId id, const Circle& circle, uint64_t categoryBits = 1, uint64_t maskBits = ~0ull);
    void addShape(EntityId id, const Capsule& capsule, uint64_t categoryBits = 1, uint64_t maskBits = ~0ull);
    void addShape(EntityId id, const Segment& segment, uint64_t categoryBits = 1, uint64_t maskBits = ~0ull);
    void addShape(EntityId id, const Polygon& polygon, uint64_t categoryBits = 1, uint64_t maskBits = ~0ull);

    void removeEntity(EntityId id);

    void setPosition(EntityId id, Vec2 position, Rotation rotation);
    void moveEntity(EntityId id, Vec2 delta, Rotation deltaRotation);

    std::vector<std::pair<EntityId, EntityId>> getCollidingPairs();
    std::vector<EntityId> getCollisions(EntityId id);
    bool areColliding(EntityId a, EntityId b);

    std::optional<RaycastHit<EntityId>> rayCast(Ray ray, float maxFraction = 1.0f);
    std::optional<RaycastHit<EntityId>> rayCast(InfiniteRay ray, float maxFraction = 1.0f);
    std::optional<RaycastHit<EntityId>> shapeCast(EntityId movingId, Vec2 translation, Rotation rotation = Rotation{ 0 });

    void update();

private:
    struct ShapeInstance
    {
        using Variant = std::variant<Circle, Capsule, Segment, Polygon>;

        Variant shape;
        AABB aabb;
        Transform localTransform;
        NodeIndex treeNodeIndex;
        BitMaskType categoryBits{1};
        BitMaskType maskBits{~0ull};
    };

    struct EntityInfo
    {
        std::vector<ShapeInstance> shapes;
        Rotation rotation;
        Vec2 position;
        std::function<void(const SweepManifold&)> onSweepHit;
        BodyType type;
        BroadPhaseTreeHandle treeHandle{};
    };

    std::unordered_map<EntityId, EntityInfo> entities;

    BroadPhaseTree<EntityId> treeStatic;
    BroadPhaseTree<EntityId> treeDynamic;

    constexpr BroadPhaseTree<EntityId>& treeFor(BodyType type);
};

template<typename EntityId>
constexpr BroadPhaseTree<EntityId>& Registry<EntityId>::treeFor(BodyType type)
{
    switch (type)
    {
        case BodyType::Static: return treeStatic;
        case BodyType::Dynamic: return treeDynamic;
        default: throw std::invalid_argument("Invalid body type");
    }
}

template<typename EntityId>
void Registry<EntityId>::createEntity(EntityId id, BodyType type, Vec2 position, Rotation rotation)
{
    assert(entities.find(id) == entities.end() && "Entity with this ID already exists");

    auto& tree = treeFor(type);
    const BroadPhaseTreeHandle handle = tree.addProxy(id, AABB{ position, position }, 1, ~0ull);
    entities.emplace(id, EntityInfo{ {}, rotation, position, {}, type, handle });
}

template<typename EntityId>
void Registry<EntityId>::addShape(EntityId id, const Circle& circle, uint64_t categoryBits, uint64_t maskBits)
{
    auto it = entities.find(id);
    assert(it != entities.end());

    EntityInfo& entity = it->second;

    ShapeInstance instance;
    instance.shape = circle;
    instance.localTransform = Transform{};
    instance.categoryBits = categoryBits;
    instance.maskBits = maskBits;
    instance.treeNodeIndex = -1;
    Transform worldTransform{};
    worldTransform.translation = entity.position;
    worldTransform.rotation = entity.rotation;
    instance.aabb = computeAABB(circle, worldTransform);

    entity.shapes.push_back(instance);
}

template<typename EntityId>
void Registry<EntityId>::addShape(EntityId id, const Capsule& capsule, uint64_t categoryBits, uint64_t maskBits)
{
    auto it = entities.find(id);
    assert(it != entities.end());

    EntityInfo& entity = it->second;

    ShapeInstance instance;
    instance.shape = capsule;
    instance.localTransform = Transform{};
    instance.categoryBits = categoryBits;
    instance.maskBits = maskBits;
    instance.treeNodeIndex = -1;
    Transform worldTransform{};
    worldTransform.translation = entity.position;
    worldTransform.rotation = entity.rotation;
    instance.aabb = computeAABB(capsule, worldTransform);

    entity.shapes.push_back(instance);
}

template<typename EntityId>
void Registry<EntityId>::addShape(EntityId id, const Segment& segment, uint64_t categoryBits, uint64_t maskBits)
{
    auto it = entities.find(id);
    assert(it != entities.end());

    EntityInfo& entity = it->second;

    ShapeInstance instance;
    instance.shape = segment;
    instance.localTransform = Transform{};
    instance.categoryBits = categoryBits;
    instance.maskBits = maskBits;
    instance.treeNodeIndex = -1;
    Transform worldTransform{};
    worldTransform.translation = entity.position;
    worldTransform.rotation = entity.rotation;
    instance.aabb = computeAABB(segment, worldTransform);

    entity.shapes.push_back(instance);
}

template<typename EntityId>
void Registry<EntityId>::addShape(EntityId id, const Polygon& polygon, uint64_t categoryBits, uint64_t maskBits)
{
    auto it = entities.find(id);
    assert(it != entities.end());

    EntityInfo& entity = it->second;

    ShapeInstance instance;
    instance.shape = polygon;
    instance.localTransform = Transform{};
    instance.categoryBits = categoryBits;
    instance.maskBits = maskBits;
    instance.treeNodeIndex = -1;
    Transform worldTransform{};
    worldTransform.translation = entity.position;
    worldTransform.rotation = entity.rotation;
    instance.aabb = computeAABB(polygon, worldTransform);

    entity.shapes.push_back(instance);
}

template<typename EntityId>
void Registry<EntityId>::removeEntity(EntityId id)
{
    auto it = entities.find(id);
    if (it == entities.end())
        return;

    entities.erase(it);
}

template<typename EntityId>
void Registry<EntityId>::setPosition(EntityId id, Vec2 position, Rotation rotation)
{
    auto it = entities.find(id);
    assert(it != entities.end());

    EntityInfo& entity = it->second;
    entity.position = position;
    entity.rotation = rotation;

    for (auto& shape : entity.shapes)
    {
        std::visit([&](const auto& s)
        {
            Transform worldTransform{};
            worldTransform.translation = entity.position;
            worldTransform.rotation = entity.rotation;
            shape.aabb = computeAABB(s, worldTransform);
        }, shape.shape);
    }
}

template<typename EntityId>
void Registry<EntityId>::moveEntity(EntityId id, Vec2 delta, Rotation deltaRotation)
{
    auto it = entities.find(id);
    assert(it != entities.end());

    EntityInfo& entity = it->second;
    entity.position += delta;
    const float sinNew = entity.rotation.sin * deltaRotation.cos + entity.rotation.cos * deltaRotation.sin;
    const float cosNew = entity.rotation.cos * deltaRotation.cos - entity.rotation.sin * deltaRotation.sin;
    entity.rotation.sin = sinNew;
    entity.rotation.cos = cosNew;

    for (auto& shape : entity.shapes)
    {
        std::visit([&](const auto& s)
        {
            Transform worldTransform{};
            worldTransform.translation = entity.position;
            worldTransform.rotation = entity.rotation;
            shape.aabb = computeAABB(s, worldTransform);
        }, shape.shape);
    }
}

template<typename EntityId>
std::vector<std::pair<EntityId, EntityId>> Registry<EntityId>::getCollidingPairs()
{
    std::vector<std::pair<EntityId, EntityId>> pairs;
    for (auto itA = entities.begin(); itA != entities.end(); ++itA)
    {
        auto itB = itA;
        ++itB;
        for (; itB != entities.end(); ++itB)
        {
            if (areColliding(itA->first, itB->first))
                pairs.emplace_back(itA->first, itB->first);
        }
    }
    return pairs;
}

template<typename EntityId>
std::vector<EntityId> Registry<EntityId>::getCollisions(EntityId id)
{
    std::vector<EntityId> ids;
    for (const auto& [otherId, _] : entities)
    {
        if (otherId == id)
            continue;
        if (areColliding(id, otherId))
            ids.push_back(otherId);
    }
    return ids;
}

template<typename EntityId>
bool Registry<EntityId>::areColliding(EntityId a, EntityId b)
{
    const auto itA = entities.find(a);
    const auto itB = entities.find(b);
    if (itA == entities.end() || itB == entities.end())
        return false;

    const EntityInfo& entA = itA->second;
    const EntityInfo& entB = itB->second;

    for (const auto& shapeA : entA.shapes)
    {
        for (const auto& shapeB : entB.shapes)
        {
            bool colliding = std::visit([&](const auto& sa, const auto& sb) {
                Transform ta{}; ta.translation = entA.position; ta.rotation = entA.rotation;
                Transform tb{}; tb.translation = entB.position; tb.rotation = entB.rotation;
                return c2d::areColliding(sa, ta, sb, tb);
            }, shapeA.shape, shapeB.shape);
            if (colliding)
                return true;
        }
    }
    return false;
}

template<typename EntityId>
std::optional<RaycastHit<EntityId>> Registry<EntityId>::rayCast(Ray ray, float maxFraction)
{
    Ray usedRay{ ray.p1, ray.p1 + (ray.p2 - ray.p1) * maxFraction };
    std::optional<RaycastHit<EntityId>> best;
    float bestTime = std::numeric_limits<float>::max();

    for (const auto& [id, entity] : entities)
    {
        for (const auto& shape : entity.shapes)
        {
            std::visit([&](const auto& s)
            {
                Transform worldTransform{};
                worldTransform.translation = entity.position;
                worldTransform.rotation = entity.rotation;
                auto hit = raycast(s, worldTransform, usedRay);
                if (hit && hit->first < bestTime)
                {
                    bestTime = hit->first;
                    best = RaycastHit<EntityId>::fromRay(id, usedRay, *hit);
                }
            }, shape.shape);
        }
    }
    return best;
}

template<typename EntityId>
std::optional<RaycastHit<EntityId>> Registry<EntityId>::rayCast(InfiniteRay ray, float maxFraction)
{
    InfiniteRay usedRay{ ray.start, ray.direction * maxFraction };
    std::optional<RaycastHit<EntityId>> best;
    float bestTime = std::numeric_limits<float>::max();

    for (const auto& [id, entity] : entities)
    {
        for (const auto& shape : entity.shapes)
        {
            std::visit([&](const auto& s)
            {
                Transform worldTransform{};
                worldTransform.translation = entity.position;
                worldTransform.rotation = entity.rotation;
                auto hit = raycast(s, worldTransform, usedRay);
                if (hit && hit->first < bestTime)
                {
                    bestTime = hit->first;
                    best = RaycastHit<EntityId>::fromRay(id, usedRay, *hit);
                }
            }, shape.shape);
        }
    }
    return best;
}

template<typename EntityId>
std::optional<RaycastHit<EntityId>> Registry<EntityId>::shapeCast(EntityId movingId, Vec2 translation, Rotation rotation)
{
    const auto itMoving = entities.find(movingId);
    if (itMoving == entities.end())
        return std::nullopt;

    const EntityInfo& moving = itMoving->second;

    float bestFraction = std::numeric_limits<float>::max();
    std::optional<RaycastHit<EntityId>> best;

    for (const auto& [otherId, other] : entities)
    {
        if (otherId == movingId)
            continue;
        for (const auto& movingShape : moving.shapes)
        {
            for (const auto& targetShape : other.shapes)
            {
                std::visit([&](const auto& ms, const auto& ts)
                {
                    Transform tm{}; tm.translation = moving.position; tm.rotation = moving.rotation;
                    Transform tt{}; tt.translation = other.position; tt.rotation = other.rotation;
                    auto result = sweep(ms,
                                         tm,
                                         translation,
                                         rotation,
                                         ts,
                                         tt);
                    if (result && result->fraction < bestFraction)
                    {
                        bestFraction = result->fraction;
                        best = RaycastHit<EntityId>{ otherId,
                                                     result->manifold.points[0].point,
                                                     result->manifold.points[0].point,
                                                     result->fraction,
                                                     result->fraction };
                    }
                }, movingShape.shape, targetShape.shape);
            }
        }
    }

    return best;
}

template<typename EntityId>
void Registry<EntityId>::update()
{
    // No-op for this simplified implementation
}

} // namespace c2d
