#pragma once

#include <colli2de/Manifold.hpp>
#include <colli2de/Ray.hpp>
#include <colli2de/Shapes.hpp>
#include <colli2de/Transform.hpp>
#include <colli2de/internal/collision/Collision.hpp>
#include <colli2de/internal/data_structures/BroadPhaseTree.hpp>
#include <colli2de/internal/geometry/AABB.hpp>
#include <colli2de/internal/geometry/RaycastShapes.hpp>
#include <colli2de/internal/geometry/ShapesUtils.hpp>

#include <map>
#include <variant>
#include <vector>

namespace c2d
{

using ShapeId = uint32_t;

enum class BodyType : uint8_t
{
    Static = 0,
    Dynamic,
    Bullet,
};

enum class PartitioningMethod
{
    None = 0,
    Grid,
};

template <typename EntityId, PartitioningMethod Method = PartitioningMethod::None> class Registry
{
  public:
    struct EntityCollision
    {
        EntityId entityA;
        EntityId entityB;
        ShapeId shapeA;
        ShapeId shapeB;
        Manifold manifold;
    };

    Registry() = default;

    Registry(uint32_t cellSize) requires(Method == PartitioningMethod::Grid)
        : treeStatic(cellSize),
          treeDynamic(cellSize),
          treeBullet(cellSize)
    {
    }

    void createEntity(EntityId id, BodyType type, const Transform& transform = Transform{});
    void removeEntity(EntityId id);

    template <IsShape Shape>
    ShapeId addShape(EntityId entityId, const Shape& shape, uint64_t categoryBits = 1, uint64_t maskBits = ~0ull);
    void removeShape(ShapeId shapeId);
    void setShapeActive(ShapeId shapeId, bool isActive = true);

    void teleportEntity(EntityId id, const Transform& transform);
    void teleportEntity(EntityId id, Translation translation);
    void moveEntity(EntityId id, const Transform& delta);
    void moveEntity(EntityId id, Translation deltaTranslation);
    Transform getEntityTransform(EntityId id) const;

    std::vector<EntityCollision> getCollidingPairs();
    std::vector<EntityCollision> getCollisions(EntityId id) const;
    bool areColliding(EntityId a, EntityId b) const;

    std::optional<RaycastHit<std::pair<EntityId, ShapeId>>> firstHitRayCast(Ray ray,
                                                                            BitMaskType maskBits = ~0ull) const;
    std::optional<RaycastHit<std::pair<EntityId, ShapeId>>> firstHitRayCast(InfiniteRay ray,
                                                                            BitMaskType maskBits = ~0ull) const;
    std::set<RaycastHit<std::pair<EntityId, ShapeId>>> rayCast(Ray ray, BitMaskType maskBits = ~0ull) const;
    std::set<RaycastHit<std::pair<EntityId, ShapeId>>> rayCast(InfiniteRay ray, BitMaskType maskBits = ~0ull) const;

    size_t size() const;
    void clear();

    void serialize(std::ostream& out) const;
    static Registry deserialize(std::istream& in);

    // Used for testing equality
    bool operator==(const Registry& other) const;

    bool operator!=(const Registry& other) const
    {
        return !(*this == other);
    }

  private:
    struct ShapeInstance
    {
        ShapeId id;
        ShapeVariant shape;
        AABB aabb;
        BroadPhaseTreeHandle treeHandle = {};
        BitMaskType categoryBits = {1};
        BitMaskType maskBits = {~0ull};
        bool isActive = true;

        void serialize(std::ostream& out) const;
        static ShapeInstance deserialize(std::istream& in);

        bool operator==(const ShapeInstance& other) const;

        bool operator!=(const ShapeInstance& other) const
        {
            return !(*this == other);
        }
    };

    struct EntityInfo
    {
        std::vector<ShapeInstance> shapes;
        Transform transform;
        BodyType type;

        void serialize(std::ostream& out) const;
        static EntityInfo deserialize(std::istream& in);

        bool operator==(const EntityInfo& other) const;

        bool operator!=(const EntityInfo& other) const
        {
            return !(*this == other);
        }
    };

    std::map<EntityId, EntityInfo> entities;
    std::vector<std::pair<EntityId, size_t>> shapeEntity;
    std::vector<ShapeId> freeShapeIds;

    using TreeType =
        std::conditional_t<Method == PartitioningMethod::Grid, BroadPhaseTree<ShapeId>, DynamicBVH<ShapeId>>;
    TreeType treeStatic;
    TreeType treeDynamic;
    TreeType treeBullet;
    std::map<EntityId, Transform> bulletPreviousTransforms;
    uint32_t previousAllCollisionsCount = 1024;

    constexpr TreeType& treeFor(BodyType type);

    void narrowPhaseCollision(ShapeId shapeAId,
                              ShapeId shapeBId,
                              std::vector<EntityCollision>& outCollisionsInfo) const;
    void narrowPhaseCollisions(ShapeId shapeQueried,
                               std::vector<ShapeId>& collisions,
                               std::vector<EntityCollision>& outCollisionsInfo) const;
    void narrowPhaseCollisions(std::vector<std::pair<ShapeId, ShapeId>>& collisions,
                               std::vector<EntityCollision>& outCollisionsInfo) const;
};

template <typename EntityId, PartitioningMethod Method>
constexpr typename Registry<EntityId, Method>::TreeType& Registry<EntityId, Method>::treeFor(BodyType type)
{
    switch (type)
    {
    case BodyType::Static:
        return treeStatic;
    case BodyType::Dynamic:
        return treeDynamic;
    case BodyType::Bullet:
        return treeBullet;
    default:
        throw std::invalid_argument("Invalid body type");
    }
}

template <typename EntityId, PartitioningMethod Method>
void Registry<EntityId, Method>::createEntity(EntityId id, BodyType type, const Transform& transform)
{
    assert(entities.find(id) == entities.end() && "Entity with this ID already exists");
    entities.emplace(id, EntityInfo{{}, transform, type});

    if (type == BodyType::Bullet)
        bulletPreviousTransforms.emplace(id, transform);
}

template <typename EntityId, PartitioningMethod Method>
template <IsShape Shape>
ShapeId Registry<EntityId, Method>::addShape(EntityId entityId,
                                             const Shape& shape,
                                             uint64_t categoryBits,
                                             uint64_t maskBits)
{
    assert(entities.find(entityId) != entities.end());
    EntityInfo& entity = entities.at(entityId);

    ShapeId shapeId;
    if (!freeShapeIds.empty())
    {
        shapeId = freeShapeIds.back();
        freeShapeIds.pop_back();
    }
    else
    {
        shapeId = shapeEntity.size();
        shapeEntity.emplace_back(entityId, entity.shapes.size());
    }

    auto& tree = treeFor(entity.type);

    ShapeInstance instance;
    instance.id = shapeId;
    instance.categoryBits = categoryBits;
    instance.maskBits = maskBits;
    instance.shape = shape;
    instance.aabb = computeAABB(shape, entity.transform);
    instance.treeHandle = tree.addProxy(shapeId, instance.aabb, categoryBits, maskBits);
    entity.shapes.push_back(instance);

    return shapeId;
}

template <typename EntityId, PartitioningMethod Method> void Registry<EntityId, Method>::removeShape(ShapeId shapeId)
{
    assert(shapeEntity.size() > shapeId && "Shape ID out of bounds");
    const EntityId entityId = shapeEntity.at(shapeId).first;

    assert(entities.find(entityId) != entities.end() && "Entity with this ID does not exist");
    EntityInfo& entity = entities.at(entityId);

    auto it = std::find_if(entity.shapes.begin(),
                           entity.shapes.end(),
                           [shapeId](const ShapeInstance& shape) { return shape.id == shapeId; });
    assert(it != entity.shapes.end() && "Shape with this ID does not exist in the entity");

    auto& tree = treeFor(entity.type);
    tree.removeProxy(it->treeHandle);

    freeShapeIds.push_back(shapeId);
    entity.shapes.erase(it);
}

template <typename EntityId, PartitioningMethod Method>
void Registry<EntityId, Method>::setShapeActive(ShapeId shapeId, bool isActive)
{
    assert(shapeEntity.size() > shapeId && "Shape ID out of bounds");
    const auto [entityId, shapeIndex] = shapeEntity.at(shapeId);

    assert(entities.find(entityId) != entities.end() && "Entity with this ID does not exist");
    EntityInfo& entity = entities.at(entityId);

    entity.shapes[shapeIndex].isActive = isActive;
}

template <typename EntityId, PartitioningMethod Method> void Registry<EntityId, Method>::removeEntity(EntityId id)
{
    auto it = entities.find(id);
    assert(it != entities.end());

    EntityInfo& entity = it->second;

    auto& tree = treeFor(entity.type);
    for (const auto& shape : entity.shapes)
    {
        tree.removeProxy(shape.treeHandle);
        freeShapeIds.push_back(shape.id);
    }

    if (entity.type == BodyType::Bullet)
        bulletPreviousTransforms.erase(id);

    entities.erase(it);
}

template <typename EntityId, PartitioningMethod Method>
void Registry<EntityId, Method>::teleportEntity(EntityId id, const Transform& transform)
{
    assert(entities.find(id) != entities.end());
    EntityInfo& entity = entities.at(id);

    auto& tree = treeFor(entity.type);

    if (entity.type == BodyType::Bullet)
    {
        for (auto& shape : entity.shapes)
        {
            std::visit(
                [&](const auto& concreteShape)
                {
                    const auto oldAABB = shape.aabb;
                    shape.aabb = computeAABB(concreteShape, transform);
                    tree.moveProxy(shape.treeHandle, AABB::combine(oldAABB, shape.aabb));
                },
                shape.shape);
        }

        bulletPreviousTransforms.at(id) = entity.transform;
    }
    else
    {
        for (auto& shape : entity.shapes)
        {
            std::visit(
                [&](const auto& concreteShape)
                {
                    shape.aabb = computeAABB(concreteShape, transform);
                    tree.moveProxy(shape.treeHandle, shape.aabb);
                },
                shape.shape);
        }
    }

    entity.transform = transform;
}

template <typename EntityId, PartitioningMethod Method>
void Registry<EntityId, Method>::teleportEntity(EntityId id, Translation translation)
{
    assert(entities.find(id) != entities.end());
    EntityInfo& entity = entities.at(id);

    const auto deltaTranslation = translation - entity.transform.translation;
    moveEntity(id, deltaTranslation);
}

template <typename EntityId, PartitioningMethod Method>
void Registry<EntityId, Method>::moveEntity(EntityId id, const Transform& delta)
{
    assert(entities.find(id) != entities.end());
    EntityInfo& entity = entities.at(id);

    auto& tree = treeFor(entity.type);

    if (entity.type == BodyType::Bullet)
    {
        bulletPreviousTransforms.at(id) = entity.transform;
        entity.transform += delta;

        for (auto& shape : entity.shapes)
        {
            std::visit(
                [&](const auto& concreteShape)
                {
                    const auto oldAABB = shape.aabb;
                    shape.aabb = computeAABB(concreteShape, entity.transform);
                    tree.moveProxy(shape.treeHandle, AABB::combine(oldAABB, shape.aabb));
                },
                shape.shape);
        }
    }
    else
    {
        entity.transform += delta;

        for (auto& shape : entity.shapes)
        {
            std::visit(
                [&](const auto& concreteShape)
                {
                    shape.aabb = computeAABB(concreteShape, entity.transform);
                    tree.moveProxy(shape.treeHandle, shape.aabb);
                },
                shape.shape);
        }
    }
}

template <typename EntityId, PartitioningMethod Method>
void Registry<EntityId, Method>::moveEntity(EntityId id, Translation deltaTranslation)
{
    assert(entities.find(id) != entities.end());
    EntityInfo& entity = entities.at(id);

    auto& tree = treeFor(entity.type);

    if (entity.type == BodyType::Bullet)
    {
        bulletPreviousTransforms.at(id) = entity.transform;

        for (auto& shape : entity.shapes)
        {
            const auto oldAABB = shape.aabb;
            shape.aabb.translate(deltaTranslation);
            tree.moveProxy(shape.treeHandle, AABB::combine(oldAABB, shape.aabb));
        }
    }
    else
    {
        auto& tree = treeFor(entity.type);
        for (auto& shape : entity.shapes)
        {
            shape.aabb.translate(deltaTranslation);
            tree.moveProxy(shape.treeHandle, shape.aabb);
        }
    }

    entity.transform.translation += deltaTranslation;
}

template <typename EntityId, PartitioningMethod Method>
Transform Registry<EntityId, Method>::getEntityTransform(EntityId id) const
{
    assert(entities.find(id) != entities.end());
    const EntityInfo& entity = entities.at(id);
    return entity.transform;
}

template <typename EntityId, PartitioningMethod Method>
std::vector<typename Registry<EntityId, Method>::EntityCollision> Registry<EntityId, Method>::getCollidingPairs()
{
    std::vector<EntityCollision> collisionsInfo;
    collisionsInfo.reserve(previousAllCollisionsCount + 10);

    std::vector<std::pair<ShapeId, ShapeId>> collisions;

    const auto handlePairCollisions = [&](ShapeId shapeAId, ShapeId shapeBId)
    { collisions.emplace_back(shapeAId, shapeBId); };

    // Dynamic vs Static
    collisions.reserve(previousAllCollisionsCount);
    treeDynamic.findAllCollisions(treeStatic, handlePairCollisions);
    if (!collisions.empty())
        narrowPhaseCollisions(collisions, collisionsInfo);

    // Dynamic vs Dynamic
    collisions.clear();
    collisions.reserve(previousAllCollisionsCount);
    treeDynamic.findAllCollisions(handlePairCollisions);
    if (!collisions.empty())
        narrowPhaseCollisions(collisions, collisionsInfo);

    // Bullet vs Static
    collisions.clear();
    collisions.reserve(previousAllCollisionsCount);
    treeBullet.findAllCollisions(treeStatic, handlePairCollisions);
    if (!collisions.empty())
        narrowPhaseCollisions(collisions, collisionsInfo);

    // Bullet vs Dynamic
    collisions.clear();
    collisions.reserve(previousAllCollisionsCount);
    treeBullet.findAllCollisions(treeDynamic, handlePairCollisions);
    if (!collisions.empty())
        narrowPhaseCollisions(collisions, collisionsInfo);

    // Bullet vs Bullet
    collisions.clear();
    collisions.reserve(previousAllCollisionsCount);
    treeBullet.findAllCollisions(handlePairCollisions);
    if (!collisions.empty())
        narrowPhaseCollisions(collisions, collisionsInfo);

    previousAllCollisionsCount = collisionsInfo.size();
    return collisionsInfo;
}

template <typename EntityId, PartitioningMethod Method>
std::vector<typename Registry<EntityId, Method>::EntityCollision> Registry<EntityId, Method>::getCollisions(
    EntityId id) const
{
    assert(entities.find(id) != entities.end());
    const EntityInfo& entity = entities.at(id);

    std::vector<std::pair<AABB, BitMaskType>> queries;
    std::vector<ShapeId> shapesQueried;

    queries.reserve(entity.shapes.size());
    shapesQueried.reserve(entity.shapes.size());

    if (entity.type == BodyType::Bullet)
    {
        for (const auto& shape : entity.shapes)
        {
            if (!shape.isActive)
                continue;

            const auto& previousTransform = bulletPreviousTransforms.at(id);
            const auto oldAABB =
                std::visit([&](const auto& shape) { return computeAABB(shape, previousTransform); }, shape.shape);

            queries.emplace_back(AABB::combine(oldAABB, shape.aabb), shape.maskBits);
            shapesQueried.push_back(shape.id);
        }
    }
    else
    {
        for (const auto& shape : entity.shapes)
        {
            if (!shape.isActive)
                continue;

            queries.emplace_back(shape.aabb, shape.maskBits);
            shapesQueried.push_back(shape.id);
        }
    }

    std::vector<EntityCollision> collisionsInfo;

    if (entity.type != BodyType::Static)
    {
        treeStatic.batchQuery(queries,
                              [&](size_t i, std::vector<ShapeId> collisions)
                              { narrowPhaseCollisions(shapesQueried[i], collisions, collisionsInfo); });
    }
    treeDynamic.batchQuery(queries,
                           [&](size_t i, std::vector<ShapeId> collisions)
                           { narrowPhaseCollisions(shapesQueried[i], collisions, collisionsInfo); });
    treeBullet.batchQuery(queries,
                          [&](size_t i, std::vector<ShapeId> collisions)
                          { narrowPhaseCollisions(shapesQueried[i], collisions, collisionsInfo); });

    return collisionsInfo;
}

template <typename EntityId, PartitioningMethod Method>
void Registry<EntityId, Method>::narrowPhaseCollision(ShapeId shapeAId,
                                                      ShapeId shapeBId,
                                                      std::vector<EntityCollision>& outCollisionsInfo) const
{
    const auto [entityAId, entityAShapeIndex] = shapeEntity[shapeAId];
    const auto [entityBId, entityBShapeIndex] = shapeEntity[shapeBId];
    const auto& entityA = entities.at(entityAId);
    const auto& entityB = entities.at(entityBId);
    const auto& entityAShape = entityA.shapes[entityAShapeIndex];
    const auto& entityBShape = entityB.shapes[entityBShapeIndex];

    // Skip self-collision
    if (entityAId == entityBId)
        return;

    if (!entityAShape.isActive || !entityBShape.isActive)
        return;

    Manifold manifold;

    // Sweep for bullets
    if (entityA.type == BodyType::Bullet && entityB.type == BodyType::Bullet)
    {
        const auto& previousTransform = bulletPreviousTransforms.at(entityAId);
        const auto& currentTransform = entityA.transform;

        if (entityB.type != BodyType::Bullet)
        {
            manifold = std::visit(
                [&](const auto& queryShapeConcrete, const auto& otherShapeConcrete)
                {
                    const auto sweepManifold = c2d::sweep(
                        queryShapeConcrete, previousTransform, currentTransform, otherShapeConcrete, entityB.transform);
                    return sweepManifold ? sweepManifold->manifold : Manifold{};
                },
                entityAShape.shape,
                entityBShape.shape);
        }
        else
        {
            const auto& otherPreviousTransform = bulletPreviousTransforms.at(entityBId);
            const auto& otherCurrentTransform = entityB.transform;

            manifold = std::visit(
                [&](const auto& queryShapeConcrete, const auto& otherShapeConcrete)
                {
                    const auto sweepManifold = c2d::sweep(queryShapeConcrete,
                                                          previousTransform,
                                                          currentTransform,
                                                          otherShapeConcrete,
                                                          otherPreviousTransform,
                                                          otherCurrentTransform);
                    return sweepManifold ? sweepManifold->manifold : Manifold{};
                },
                entityAShape.shape,
                entityBShape.shape);
        }
    }
    else if (entityA.type == BodyType::Bullet || entityB.type == BodyType::Bullet)
    {
        const auto& bulletEntity = (entityA.type == BodyType::Bullet) ? entityA : entityB;
        const auto& bulletEntityId = (entityA.type == BodyType::Bullet) ? entityAId : entityBId;
        const auto& otherEntity = (entityA.type == BodyType::Bullet) ? entityB : entityA;
        const auto& bulletPreviousTransform = bulletPreviousTransforms.at(bulletEntityId);
        const auto& bulletCurrentTransform = bulletEntity.transform;
        const auto& bulletShape = (entityA.type == BodyType::Bullet) ? entityAShape : entityBShape;
        const auto& otherShape = (entityA.type == BodyType::Bullet) ? entityBShape : entityAShape;

        manifold = std::visit(
            [&](const auto& bulletShapeConcrete, const auto& otherShapeConcrete)
            {
                const auto sweepManifold = c2d::sweep(bulletShapeConcrete,
                                                      bulletPreviousTransform,
                                                      bulletCurrentTransform,
                                                      otherShapeConcrete,
                                                      otherEntity.transform);
                return sweepManifold ? sweepManifold->manifold : Manifold{};
            },
            bulletShape.shape,
            otherShape.shape);
    }
    else
    {
        manifold = std::visit(
            [&](const auto& queryShapeConcrete, const auto& otherShapeConcrete)
            { return c2d::collide(queryShapeConcrete, entityA.transform, otherShapeConcrete, entityB.transform); },
            entityAShape.shape,
            entityBShape.shape);
    }

    if (!manifold.isColliding())
        return;

    outCollisionsInfo.emplace_back(EntityCollision{entityAId, entityBId, shapeAId, shapeBId, std::move(manifold)});
}

template <typename EntityId, PartitioningMethod Method>
void Registry<EntityId, Method>::narrowPhaseCollisions(std::vector<std::pair<ShapeId, ShapeId>>& collisions,
                                                       std::vector<EntityCollision>& outCollisionsInfo) const
{
    // Sort to detect duplicates -> duplicates will be adjacent in the sorted vector
    std::sort(collisions.begin(), collisions.end());

    narrowPhaseCollision(collisions[0].first, collisions[0].second, outCollisionsInfo);
    for (size_t i = 1; i < collisions.size(); i++)
    {
        // Skip duplicates
        if ((collisions[i].first == collisions[i - 1].first && collisions[i].second == collisions[i - 1].second) ||
            (collisions[i].first == collisions[i - 1].second && collisions[i].second == collisions[i - 1].first))
            continue;

        narrowPhaseCollision(collisions[i].first, collisions[i].second, outCollisionsInfo);
    }
}

template <typename EntityId, PartitioningMethod Method>
void Registry<EntityId, Method>::narrowPhaseCollisions(ShapeId shapeQueried,
                                                       std::vector<ShapeId>& collisions,
                                                       std::vector<EntityCollision>& outCollisionsInfo) const
{
    // Sort to detect duplicates -> duplicates will be adjacent in the sorted vector
    std::sort(collisions.begin(), collisions.end());

    narrowPhaseCollision(shapeQueried, collisions[0], outCollisionsInfo);
    for (size_t i = 1; i < collisions.size(); i++)
    {
        // Skip duplicates
        if (collisions[i] == collisions[i - 1])
            continue;

        narrowPhaseCollision(shapeQueried, collisions[i], outCollisionsInfo);
    }
}

template <typename EntityId, PartitioningMethod Method>
bool Registry<EntityId, Method>::areColliding(EntityId entityAId, EntityId entityBId) const
{
    assert(entities.find(entityAId) != entities.end() && "Entity A does not exist");
    assert(entities.find(entityBId) != entities.end() && "Entity B does not exist");

    const EntityInfo& entityA = entities.at(entityAId);
    const EntityInfo& entityB = entities.at(entityBId);

    if (entityA.type == BodyType::Static && entityB.type == BodyType::Static)
        return false;

    if (entityA.type == BodyType::Bullet && entityB.type == BodyType::Bullet)
    {
        const auto& entityAPreviousTransform = bulletPreviousTransforms.at(entityAId);
        const auto& entityBPreviousTransform = bulletPreviousTransforms.at(entityBId);
        const auto& entityACurrentTransform = entityA.transform;
        const auto& entityBCurrentTransform = entityB.transform;

        Manifold manifold;
        for (const auto& shapeA : entityA.shapes)
        {
            if (!shapeA.isActive)
                continue;

            for (const auto& shapeB : entityB.shapes)
            {
                if (!shapeB.isActive)
                    continue;

                std::optional<SweepManifold> manifold = std::visit(
                    [&](const auto& shapeAConcrete, const auto& shapeBConcrete)
                    {
                        return c2d::sweep(shapeAConcrete,
                                          entityAPreviousTransform,
                                          entityACurrentTransform,
                                          shapeBConcrete,
                                          entityBPreviousTransform,
                                          entityBCurrentTransform);
                    },
                    shapeA.shape,
                    shapeB.shape);

                if (manifold)
                    return true;
            }
        }

        return false;
    }

    if (entityA.type == BodyType::Bullet || entityB.type == BodyType::Bullet)
    {
        const auto& bulletEntity = (entityA.type == BodyType::Bullet) ? entityA : entityB;
        const auto& bulletEntityId = (entityA.type == BodyType::Bullet) ? entityAId : entityBId;
        const auto& otherEntity = (entityA.type == BodyType::Bullet) ? entityB : entityA;
        const auto& bulletPreviousTransform = bulletPreviousTransforms.at(bulletEntityId);
        const auto& bulletCurrentTransform = bulletEntity.transform;

        for (const auto& shapeBullet : bulletEntity.shapes)
        {
            if (!shapeBullet.isActive)
                continue;

            for (const auto& shapeOther : otherEntity.shapes)
            {
                if (!shapeOther.isActive)
                    continue;

                std::optional<SweepManifold> manifold = std::visit(
                    [&](const auto& shapeBulletConcrete, const auto& shapeOtherConcrete)
                    {
                        return c2d::sweep(shapeBulletConcrete,
                                          bulletPreviousTransform,
                                          bulletCurrentTransform,
                                          shapeOtherConcrete,
                                          otherEntity.transform);
                    },
                    shapeBullet.shape,
                    shapeOther.shape);

                if (manifold)
                    return true;
            }
        }

        return false;
    }

    for (const auto& shapeA : entityA.shapes)
    {
        if (!shapeA.isActive)
            continue;

        for (const auto& shapeB : entityB.shapes)
        {
            if (!shapeB.isActive)
                continue;

            bool colliding =
                std::visit([&](const auto& shapeA, const auto& shapeB)
                           { return c2d::areColliding(shapeA, entityA.transform, shapeB, entityB.transform); },
                           shapeA.shape,
                           shapeB.shape);

            if (colliding)
                return true;
        }
    }

    return false;
}

template <typename EntityId, PartitioningMethod Method>
std::optional<RaycastHit<std::pair<EntityId, ShapeId>>> Registry<EntityId, Method>::firstHitRayCast(
    Ray ray, BitMaskType maskBits) const
{
    std::optional<RaycastHit<std::pair<EntityId, ShapeId>>> bestHit;
    float bestEntryTime = std::numeric_limits<float>::max();

    std::set<typename DynamicBVH<ShapeId>::RaycastInfo> staticHits, dynamicHits, bulletHits;
    treeStatic.piercingRaycast(ray, staticHits, maskBits);
    treeDynamic.piercingRaycast(ray, dynamicHits, maskBits);
    treeBullet.piercingRaycast(ray, bulletHits, maskBits);

    uint32_t toCheck = staticHits.size();
    for (const auto& hit : staticHits)
    {
        const auto& shapeInfo = shapeEntity[hit.id];
        const auto entityId = shapeInfo.first;
        const auto& entity = entities.at(entityId);
        const auto& shape = entity.shapes[shapeInfo.second];

        if (!shape.isActive)
            continue;

        const auto narrowHit = std::visit(
            [&](const auto& shapeConcrete) { return raycast(shapeConcrete, entity.transform, ray); }, shape.shape);

        if (narrowHit && narrowHit->first < bestEntryTime)
        {
            bestEntryTime = narrowHit->first;
            bestHit = RaycastHit<std::pair<EntityId, ShapeId>>::fromRay({entityId, hit.id}, ray, *narrowHit);
            toCheck = 5; // Check 5 other hits after finding the first one
        }

        if (--toCheck == 0)
            break;
    }

    toCheck = dynamicHits.size();
    for (const auto& hit : dynamicHits)
    {
        const auto& shapeInfo = shapeEntity[hit.id];
        const auto entityId = shapeInfo.first;
        const auto& entity = entities.at(entityId);
        const auto& shape = entity.shapes[shapeInfo.second];

        if (!shape.isActive)
            continue;

        const auto narrowHit = std::visit(
            [&](const auto& shapeConcrete) { return raycast(shapeConcrete, entity.transform, ray); }, shape.shape);

        if (narrowHit && narrowHit->first < bestEntryTime)
        {
            bestEntryTime = narrowHit->first;
            bestHit = RaycastHit<std::pair<EntityId, ShapeId>>::fromRay({entityId, hit.id}, ray, *narrowHit);
            toCheck = 5; // Check 5 other hits after finding the first one
        }
        else if (narrowHit->first >= bestEntryTime)
            toCheck = std::min(toCheck, 5u); // If the hit is not better, check 5 more hits

        if (--toCheck == 0)
            break;
    }

    toCheck = bulletHits.size();
    for (const auto& hit : bulletHits)
    {
        const auto& shapeInfo = shapeEntity[hit.id];
        const auto entityId = shapeInfo.first;
        const auto& entity = entities.at(entityId);
        const auto& shape = entity.shapes[shapeInfo.second];

        if (!shape.isActive)
            continue;

        const auto& previousTransform = bulletPreviousTransforms.at(entityId);
        const auto narrowHit = std::visit([&](const auto& shapeConcrete)
                                          { return sweep(shapeConcrete, previousTransform, entity.transform, ray); },
                                          shape.shape);

        if (narrowHit && narrowHit->first < bestEntryTime)
        {
            bestEntryTime = narrowHit->first;
            bestHit = RaycastHit<std::pair<EntityId, ShapeId>>::fromRay({entityId, hit.id}, ray, *narrowHit);
            toCheck = 5; // Check 5 other hits after finding the first one
        }
        else if (narrowHit->first >= bestEntryTime)
            toCheck = std::min(toCheck, 5u); // If the hit is not better, check 5 more hits

        if (--toCheck == 0)
            break;
    }

    return bestHit;
}

template <typename EntityId, PartitioningMethod Method>
std::optional<RaycastHit<std::pair<EntityId, ShapeId>>> Registry<EntityId, Method>::firstHitRayCast(
    InfiniteRay ray, BitMaskType maskBits) const
{
    std::set<typename DynamicBVH<ShapeId>::RaycastInfo> staticHits, dynamicHits, bulletHits;
    treeStatic.piercingRaycast(ray, staticHits, maskBits);
    treeDynamic.piercingRaycast(ray, dynamicHits, maskBits);
    treeBullet.piercingRaycast(ray, bulletHits, maskBits);

    std::optional<RaycastHit<std::pair<EntityId, ShapeId>>> bestHit;
    float bestEntryTime = std::numeric_limits<float>::max();

    uint32_t toCheck = staticHits.size();
    for (const auto& hit : staticHits)
    {
        const auto& shapeInfo = shapeEntity[hit.id];
        const auto entityId = shapeInfo.first;
        const auto& entity = entities.at(entityId);
        const auto& shape = entity.shapes[shapeInfo.second];

        if (!shape.isActive)
            continue;

        const auto narrowHit = std::visit(
            [&](const auto& shapeConcrete) { return raycast(shapeConcrete, entity.transform, ray); }, shape.shape);

        if (narrowHit && narrowHit->first < bestEntryTime)
        {
            bestEntryTime = narrowHit->first;
            bestHit = RaycastHit<std::pair<EntityId, ShapeId>>::fromRay({entityId, hit.id}, ray, *narrowHit);
            toCheck = 5; // Check 5 other hits after finding the first one
        }

        if (--toCheck == 0)
            break;
    }

    toCheck = dynamicHits.size();
    for (const auto& hit : dynamicHits)
    {
        const auto& shapeInfo = shapeEntity[hit.id];
        const auto entityId = shapeInfo.first;
        const auto& entity = entities.at(entityId);
        const auto& shape = entity.shapes[shapeInfo.second];

        if (!shape.isActive)
            continue;

        const auto narrowHit = std::visit(
            [&](const auto& shapeConcrete) { return raycast(shapeConcrete, entity.transform, ray); }, shape.shape);

        if (narrowHit && narrowHit->first < bestEntryTime)
        {
            bestEntryTime = narrowHit->first;
            bestHit = RaycastHit<std::pair<EntityId, ShapeId>>::fromRay({entityId, hit.id}, ray, *narrowHit);
            toCheck = 5; // Check 5 other hits after finding the first one
        }
        else if (narrowHit->first >= bestEntryTime)
            toCheck = std::min(toCheck, 5u); // If the hit is not better, check 5 more hits

        if (--toCheck == 0)
            break;
    }

    toCheck = bulletHits.size();
    for (const auto& hit : bulletHits)
    {
        const auto& shapeInfo = shapeEntity[hit.id];
        const auto entityId = shapeInfo.first;
        const auto& entity = entities.at(entityId);
        const auto& shape = entity.shapes[shapeInfo.second];

        if (!shape.isActive)
            continue;

        const auto& previousTransform = bulletPreviousTransforms.at(entityId);
        const auto narrowHit = std::visit([&](const auto& shapeConcrete)
                                          { return sweep(shapeConcrete, previousTransform, entity.transform, ray); },
                                          shape.shape);

        if (narrowHit && narrowHit->first < bestEntryTime)
        {
            bestEntryTime = narrowHit->first;
            bestHit = RaycastHit<std::pair<EntityId, ShapeId>>::fromRay({entityId, hit.id}, ray, *narrowHit);
            toCheck = 5; // Check 5 other hits after finding the first one
        }
        else if (narrowHit->first >= bestEntryTime)
            toCheck = std::min(toCheck, 5u); // If the hit is not better, check 5 more hits

        if (--toCheck == 0)
            break;
    }

    return bestHit;
}

template <typename EntityId, PartitioningMethod Method>
std::set<RaycastHit<std::pair<EntityId, ShapeId>>> Registry<EntityId, Method>::rayCast(Ray ray,
                                                                                       BitMaskType maskBits) const
{
    std::set<typename DynamicBVH<ShapeId>::RaycastInfo> staticHits, dynamicHits, bulletHits;
    treeStatic.piercingRaycast(ray, staticHits, maskBits);
    treeDynamic.piercingRaycast(ray, dynamicHits, maskBits);
    treeBullet.piercingRaycast(ray, bulletHits, maskBits);

    std::set<RaycastHit<std::pair<EntityId, ShapeId>>> hits;

    for (const auto& hit : staticHits)
    {
        const auto& shapeInfo = shapeEntity[hit.id];
        const auto entityId = shapeInfo.first;
        const auto& entity = entities.at(entityId);
        const auto& shape = entity.shapes[shapeInfo.second];

        if (!shape.isActive)
            continue;

        const auto narrowHit = std::visit(
            [&](const auto& shapeConcrete) { return raycast(shapeConcrete, entity.transform, ray); }, shape.shape);

        if (narrowHit)
            hits.insert(RaycastHit<std::pair<EntityId, ShapeId>>::fromRay({entityId, hit.id}, ray, *narrowHit));
    }

    for (const auto& hit : dynamicHits)
    {
        const auto& shapeInfo = shapeEntity[hit.id];
        const auto entityId = shapeInfo.first;
        const auto& entity = entities.at(entityId);
        const auto& shape = entity.shapes[shapeInfo.second];

        if (!shape.isActive)
            continue;

        const auto narrowHit = std::visit(
            [&](const auto& shapeConcrete) { return raycast(shapeConcrete, entity.transform, ray); }, shape.shape);

        if (narrowHit)
            hits.insert(RaycastHit<std::pair<EntityId, ShapeId>>::fromRay({entityId, hit.id}, ray, *narrowHit));
    }

    for (const auto& hit : bulletHits)
    {
        const auto& shapeInfo = shapeEntity[hit.id];
        const auto entityId = shapeInfo.first;
        const auto& entity = entities.at(entityId);
        const auto& shape = entity.shapes[shapeInfo.second];

        if (!shape.isActive)
            continue;

        const auto& previousTransform = bulletPreviousTransforms.at(entityId);
        const auto narrowHit = std::visit([&](const auto& shapeConcrete)
                                          { return sweep(shapeConcrete, previousTransform, entity.transform, ray); },
                                          shape.shape);

        if (narrowHit)
            hits.insert(RaycastHit<std::pair<EntityId, ShapeId>>::fromRay({entityId, hit.id}, ray, *narrowHit));
    }

    return hits;
}

template <typename EntityId, PartitioningMethod Method>
std::set<RaycastHit<std::pair<EntityId, ShapeId>>> Registry<EntityId, Method>::rayCast(InfiniteRay ray,
                                                                                       BitMaskType maskBits) const
{
    std::set<typename DynamicBVH<ShapeId>::RaycastInfo> staticHits, dynamicHits, bulletHits;
    treeStatic.piercingRaycast(ray, staticHits, maskBits);
    treeDynamic.piercingRaycast(ray, dynamicHits, maskBits);
    treeBullet.piercingRaycast(ray, bulletHits, maskBits);

    std::set<RaycastHit<std::pair<EntityId, ShapeId>>> hits;

    for (const auto& hit : staticHits)
    {
        const auto& shapeInfo = shapeEntity[hit.id];
        const auto entityId = shapeInfo.first;
        const auto& entity = entities.at(entityId);
        const auto& shape = entity.shapes[shapeInfo.second];

        if (!shape.isActive)
            continue;

        const auto narrowHit = std::visit(
            [&](const auto& shapeConcrete) { return raycast(shapeConcrete, entity.transform, ray); }, shape.shape);

        if (narrowHit)
            hits.insert(RaycastHit<std::pair<EntityId, ShapeId>>::fromRay({entityId, hit.id}, ray, *narrowHit));
    }

    for (const auto& hit : dynamicHits)
    {
        const auto& shapeInfo = shapeEntity[hit.id];
        const auto entityId = shapeInfo.first;
        const auto& entity = entities.at(entityId);
        const auto& shape = entity.shapes[shapeInfo.second];

        if (!shape.isActive)
            continue;

        const auto narrowHit = std::visit(
            [&](const auto& shapeConcrete) { return raycast(shapeConcrete, entity.transform, ray); }, shape.shape);

        if (narrowHit)
            hits.insert(RaycastHit<std::pair<EntityId, ShapeId>>::fromRay({entityId, hit.id}, ray, *narrowHit));
    }

    for (const auto& hit : bulletHits)
    {
        const auto& shapeInfo = shapeEntity[hit.id];
        const auto entityId = shapeInfo.first;
        const auto& entity = entities.at(entityId);
        const auto& shape = entity.shapes[shapeInfo.second];

        if (!shape.isActive)
            continue;

        const auto& previousTransform = bulletPreviousTransforms.at(entityId);
        const auto narrowHit = std::visit([&](const auto& shapeConcrete)
                                          { return sweep(shapeConcrete, previousTransform, entity.transform, ray); },
                                          shape.shape);

        if (narrowHit)
            hits.insert(RaycastHit<std::pair<EntityId, ShapeId>>::fromRay({entityId, hit.id}, ray, *narrowHit));
    }

    return hits;
}

template <typename EntityId, PartitioningMethod Method> size_t Registry<EntityId, Method>::size() const
{
    return entities.size();
}

template <typename EntityId, PartitioningMethod Method> void Registry<EntityId, Method>::clear()
{
    entities.clear();
    shapeEntity.clear();
    freeShapeIds.clear();
    treeStatic.clear();
    treeDynamic.clear();
    treeBullet.clear();
    bulletPreviousTransforms.clear();
    previousAllCollisionsCount = 1000;
}

template <typename EntityId, PartitioningMethod Method>
void Registry<EntityId, Method>::serialize(std::ostream& out) const
{
    Writer writer(out);

    // Write entities
    size_t entitiesSize = entities.size();
    writer(entitiesSize);
    for (const auto& [entityId, entityInfo] : entities)
    {
        writer(entityId);
        entityInfo.serialize(out);
    }

    // Write shape to entity map
    size_t shapeEntitySize = shapeEntity.size();
    writer(shapeEntitySize);
    for (const auto& [shapeId, shapesCount] : shapeEntity)
    {
        writer(shapeId);
        writer(shapesCount);
    }

    // Write free shape IDs
    size_t freeShapeIdsSize = freeShapeIds.size();
    writer(freeShapeIdsSize);
    for (const ShapeId& shapeId : freeShapeIds)
        writer(shapeId);

    // Write broad-phase trees
    treeStatic.serialize(out);
    treeDynamic.serialize(out);
    treeBullet.serialize(out);

    // Write bullet previous transforms
    size_t bulletPrevTransformsSize = bulletPreviousTransforms.size();
    writer(bulletPrevTransformsSize);
    for (const auto& [entityId, transform] : bulletPreviousTransforms)
    {
        writer(entityId);

        writer(transform.translation.x);
        writer(transform.translation.y);
        writer(transform.rotation.angleRadians);
        writer(transform.rotation.sin);
        writer(transform.rotation.cos);
    }

    writer(previousAllCollisionsCount);
}

template <typename EntityId, PartitioningMethod Method>
Registry<EntityId, Method> Registry<EntityId, Method>::deserialize(std::istream& in)
{
    Reader reader(in);
    Registry<EntityId, Method> registry;

    // Read entities
    size_t entitiesSize;
    reader(entitiesSize);

    for (size_t i = 0; i < entitiesSize; i++)
    {
        EntityId entityId;
        reader(entityId);

        auto entityInfo = EntityInfo::deserialize(in);

        registry.entities.emplace(entityId, std::move(entityInfo));
    }

    // Read shape to entity map
    size_t shapeEntitySize;
    reader(shapeEntitySize);

    registry.shapeEntity.resize(shapeEntitySize);
    for (size_t i = 0; i < shapeEntitySize; i++)
    {
        reader(registry.shapeEntity[i].first);
        reader(registry.shapeEntity[i].second);
    }

    // Read free shape IDs
    size_t freeShapeIdsSize;
    reader(freeShapeIdsSize);

    registry.freeShapeIds.resize(freeShapeIdsSize);
    for (size_t i = 0; i < freeShapeIdsSize; i++)
        reader(registry.freeShapeIds[i]);

    // Read broad-phase trees
    registry.treeStatic = TreeType::deserialize(in);
    registry.treeDynamic = TreeType::deserialize(in);
    registry.treeBullet = TreeType::deserialize(in);

    // Read bullet previous transforms
    size_t bulletPrevTransformsSize;
    reader(bulletPrevTransformsSize);

    for (size_t i = 0; i < bulletPrevTransformsSize; i++)
    {
        EntityId entityId;
        reader(entityId);

        Transform transform;
        reader(transform.translation.x);
        reader(transform.translation.y);
        reader(transform.rotation.angleRadians);
        reader(transform.rotation.sin);
        reader(transform.rotation.cos);

        registry.bulletPreviousTransforms.emplace(entityId, std::move(transform));
    }

    reader(registry.previousAllCollisionsCount);

    return registry;
}

template <typename EntityId, PartitioningMethod Method>
void Registry<EntityId, Method>::EntityInfo::serialize(std::ostream& out) const
{
    Writer writer(out);

    // Write body type
    writer(static_cast<uint8_t>(type));

    // Write transform
    writer(transform.translation.x);
    writer(transform.translation.y);
    writer(transform.rotation.angleRadians);
    writer(transform.rotation.sin);
    writer(transform.rotation.cos);

    // Write shapes
    size_t shapesSize = shapes.size();
    writer(shapesSize);
    for (const auto& shapeInstance : shapes)
        shapeInstance.serialize(out);
}

template <typename EntityId, PartitioningMethod Method>
Registry<EntityId, Method>::EntityInfo Registry<EntityId, Method>::EntityInfo::deserialize(std::istream& in)
{
    Reader reader(in);
    EntityInfo entityInfo;

    // Read body type
    reader(entityInfo.type);

    // Read transform
    reader(entityInfo.transform.translation.x);
    reader(entityInfo.transform.translation.y);
    reader(entityInfo.transform.rotation.angleRadians);
    reader(entityInfo.transform.rotation.sin);
    reader(entityInfo.transform.rotation.cos);

    // Read shapes
    size_t shapesSize;
    reader(shapesSize);

    entityInfo.shapes.resize(shapesSize);
    for (auto& shapeInstance : entityInfo.shapes)
        shapeInstance = ShapeInstance::deserialize(in);

    return entityInfo;
}

template <typename EntityId, PartitioningMethod Method>
void Registry<EntityId, Method>::ShapeInstance::serialize(std::ostream& out) const
{
    Writer writer(out);

    // Write shape ID
    writer(id);

    // Write shape variant
    uint8_t shapeType = static_cast<uint8_t>(shape.index());
    writer(shapeType);

    // Write AABB
    writer(aabb.min.x);
    writer(aabb.min.y);
    writer(aabb.max.x);
    writer(aabb.max.y);

    // Write tree handle
    writer(treeHandle);

    // Write mask bits
    writer(categoryBits);
    writer(maskBits);

    // Write isActive
    writer(isActive);

    // Write shape data
    if (std::holds_alternative<Circle>(shape))
    {
        const Circle& circle = std::get<Circle>(shape);
        writer(circle.center.x);
        writer(circle.center.y);
        writer(circle.radius);
    }
    else if (std::holds_alternative<Capsule>(shape))
    {
        const Capsule& capsule = std::get<Capsule>(shape);
        writer(capsule.center1.x);
        writer(capsule.center1.y);
        writer(capsule.center2.x);
        writer(capsule.center2.y);
        writer(capsule.radius);
    }
    else if (std::holds_alternative<Segment>(shape))
    {
        const Segment& segment = std::get<Segment>(shape);
        writer(segment.start.x);
        writer(segment.start.y);
        writer(segment.end.x);
        writer(segment.end.y);
    }
    else if (std::holds_alternative<Polygon>(shape))
    {
        const Polygon& polygon = std::get<Polygon>(shape);
        writer(polygon.count);

        for (size_t i = 0; i < polygon.count; i++)
        {
            writer(polygon.vertices[i].x);
            writer(polygon.vertices[i].y);
        }
        for (size_t i = 0; i < polygon.count; i++)
        {
            writer(polygon.normals[i].x);
            writer(polygon.normals[i].y);
        }
    }
}

template <typename EntityId, PartitioningMethod Method>
typename Registry<EntityId, Method>::ShapeInstance Registry<EntityId, Method>::ShapeInstance::deserialize(
    std::istream& in)
{
    Reader reader(in);
    ShapeInstance shapeInstance;

    // Read shape ID
    reader(shapeInstance.id);

    // Read shape variant
    uint8_t shapeType;
    reader(shapeType);

    // Read AABB
    reader(shapeInstance.aabb.min.x);
    reader(shapeInstance.aabb.min.y);
    reader(shapeInstance.aabb.max.x);
    reader(shapeInstance.aabb.max.y);

    // Read tree handle
    reader(shapeInstance.treeHandle);

    // Read mask bits
    reader(shapeInstance.categoryBits);
    reader(shapeInstance.maskBits);

    // Read isActive
    reader(shapeInstance.isActive);

    // Read shape data
    if (shapeType == static_cast<uint8_t>(ShapeType::Circle))
    {
        Circle circle;
        reader(circle.center.x);
        reader(circle.center.y);
        reader(circle.radius);
        shapeInstance.shape = circle;
    }
    else if (shapeType == static_cast<uint8_t>(ShapeType::Capsule))
    {
        Capsule capsule;
        reader(capsule.center1.x);
        reader(capsule.center1.y);
        reader(capsule.center2.x);
        reader(capsule.center2.y);
        reader(capsule.radius);
        shapeInstance.shape = capsule;
    }
    else if (shapeType == static_cast<uint8_t>(ShapeType::Segment))
    {
        Segment segment;
        reader(segment.start.x);
        reader(segment.start.y);
        reader(segment.end.x);
        reader(segment.end.y);
        shapeInstance.shape = segment;
    }
    else if (shapeType == static_cast<uint8_t>(ShapeType::Polygon))
    {
        Polygon polygon;
        reader(polygon.count);

        for (size_t i = 0; i < polygon.count; i++)
        {
            reader(polygon.vertices[i].x);
            reader(polygon.vertices[i].y);
        }
        for (size_t i = 0; i < polygon.count; i++)
        {
            reader(polygon.normals[i].x);
            reader(polygon.normals[i].y);
        }

        shapeInstance.shape = polygon;
    }

    return shapeInstance;
}

template <typename EntityId, PartitioningMethod Method>
bool Registry<EntityId, Method>::operator==(const Registry& other) const
{
    if (entities.size() != other.entities.size())
        return false;

    for (const auto& [id, entity] : entities)
    {
        auto it = other.entities.find(id);
        if (it == other.entities.end() || it->second != entity)
            return false;
    }

    if (shapeEntity != other.shapeEntity)
        return false;

    if (freeShapeIds != other.freeShapeIds)
        return false;

    if (treeStatic != other.treeStatic)
        return false;
    if (treeDynamic != other.treeDynamic)
        return false;
    if (treeBullet != other.treeBullet)
        return false;

    if (bulletPreviousTransforms.size() != other.bulletPreviousTransforms.size())
        return false;

    for (const auto& [id, transform] : bulletPreviousTransforms)
    {
        auto it = other.bulletPreviousTransforms.find(id);
        if (it == other.bulletPreviousTransforms.end() || it->second != transform)
            return false;
    }

    if (previousAllCollisionsCount != other.previousAllCollisionsCount)
        return false;

    return true;
}

template <typename EntityId, PartitioningMethod Method>
bool Registry<EntityId, Method>::EntityInfo::operator==(const EntityInfo& other) const
{
    if (shapes != other.shapes)
        return false;
    if (transform != other.transform)
        return false;
    if (type != other.type)
        return false;

    return true;
}

template <typename EntityId, PartitioningMethod Method>
bool Registry<EntityId, Method>::ShapeInstance::operator==(const ShapeInstance& other) const
{
    if (id != other.id)
        return false;
    if (shape != other.shape)
        return false;
    if (aabb != other.aabb)
        return false;
    if (treeHandle != other.treeHandle)
        return false;
    if (categoryBits != other.categoryBits)
        return false;
    if (maskBits != other.maskBits)
        return false;
    if (isActive != other.isActive)
        return false;

    return true;
}

} // namespace c2d
