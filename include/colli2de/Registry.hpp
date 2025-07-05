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
    };

    struct EntityInfo
    {
        std::vector<ShapeInstance> shapes;
        Rotation rotation;
        Vec2 position;
        std::function<void(const SweepManifold&)> onSweepHit;
        BodyType type;
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

    entities.emplace(id, EntityInfo{ {}, rotation, position, {}, type });
    auto& tree = treeFor(type);
    tree.addProxy(id, AABB{ position, position }, 1, ~0ull);
}

} // namespace c2d
