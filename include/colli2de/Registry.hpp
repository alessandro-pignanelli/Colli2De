#pragma once

#include <variant>
#include <vector>
#include <map>

#include <colli2de/Ray.hpp>
#include <colli2de/Shapes.hpp>
#include <colli2de/Manifold.hpp>
#include <colli2de/Transform.hpp>
#include <colli2de/internal/data_structures/BroadPhaseTree.hpp>
#include <colli2de/internal/geometry/AABB.hpp>
#include <colli2de/internal/geometry/ShapesUtils.hpp>
#include <colli2de/internal/geometry/RaycastShapes.hpp>
#include <colli2de/internal/collision/Collision.hpp>

namespace c2d
{

using ShapeId = uint32_t;

enum class BodyType : uint8_t
{
    Static = 0,
    Dynamic,
    Bullet,
};

template<typename EntityId>
class Registry
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
    Registry(uint32_t cellSize) : treeStatic(cellSize), treeDynamic(cellSize), treeBullet(cellSize) {}

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

    std::vector<EntityCollision> getCollidingPairs();
    std::vector<EntityCollision> getCollisions(EntityId id) const;
    bool areColliding(EntityId a, EntityId b) const;

    std::optional<RaycastHit<std::pair<EntityId, ShapeId>>> firstHitRayCast(Ray ray, BitMaskType maskBits = ~0ull) const;
    std::optional<RaycastHit<std::pair<EntityId, ShapeId>>> firstHitRayCast(InfiniteRay ray, BitMaskType maskBits = ~0ull) const;
    std::set<RaycastHit<std::pair<EntityId, ShapeId>>> rayCast(Ray ray, BitMaskType maskBits = ~0ull) const;
    std::set<RaycastHit<std::pair<EntityId, ShapeId>>> rayCast(InfiniteRay ray, BitMaskType maskBits = ~0ull) const;

    size_t size() const;
    void clear();

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
    };

    struct EntityInfo
    {
        std::vector<ShapeInstance> shapes;
        Transform transform;
        BodyType type;
    };

    std::map<EntityId, EntityInfo> entities;
    std::vector<std::pair<EntityId, size_t>> shapeEntity;
    std::vector<ShapeId> freeShapeIds;

    BroadPhaseTree<ShapeId> treeStatic;
    BroadPhaseTree<ShapeId> treeDynamic;
    BroadPhaseTree<ShapeId> treeBullet;
    std::map<EntityId, Transform> bulletPreviousTransforms;
    uint32_t previousAllCollisionsCount = 0;

    constexpr BroadPhaseTree<ShapeId>& treeFor(BodyType type);

    void getAllCollisionAABBQueries(std::vector<std::pair<AABB, BitMaskType>>& dynamicQueries,
                                    std::vector<std::pair<AABB, BitMaskType>>& staticQueries,
                                    std::vector<ShapeId>& dynamicShapesQueried,
                                    std::vector<ShapeId>& staticShapesQueried) const;
    void narrowPhaseCollision(ShapeId shapeAId,
                              ShapeId shapeBId,
                              std::vector<EntityCollision>& outCollisionsInfo) const;
    void narrowPhaseCollisions(ShapeId shapeQueried,
                               std::vector<ShapeId>& collisions,
                               std::vector<EntityCollision>& outCollisionsInfo) const;
    void narrowPhaseCollisions(std::vector<std::pair<ShapeId, ShapeId>>& collisions,
                               std::vector<EntityCollision>& outCollisionsInfo) const;
};

template<typename EntityId>
constexpr BroadPhaseTree<ShapeId>& Registry<EntityId>::treeFor(BodyType type)
{
    switch (type)
    {
        case BodyType::Static: return treeStatic;
        case BodyType::Dynamic: return treeDynamic;
        case BodyType::Bullet: return treeBullet;
        default: throw std::invalid_argument("Invalid body type");
    }
}

template<typename EntityId>
void Registry<EntityId>::createEntity(EntityId id, BodyType type, const Transform& transform)
{
    assert(entities.find(id) == entities.end() && "Entity with this ID already exists");
    entities.emplace(id, EntityInfo{ {}, transform, type });

    if (type == BodyType::Bullet)
        bulletPreviousTransforms.emplace(id, transform);
}

template<typename EntityId>
template <IsShape Shape>
ShapeId Registry<EntityId>::addShape(EntityId entityId, const Shape& shape, uint64_t categoryBits, uint64_t maskBits)
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

template<typename EntityId>
void Registry<EntityId>::removeShape(ShapeId shapeId)
{
    assert(shapeEntity.size() > shapeId && "Shape ID out of bounds");
    const EntityId entityId = shapeEntity.at(shapeId).first;
    
    assert(entities.find(entityId) != entities.end() && "Entity with this ID does not exist");
    EntityInfo& entity = entities.at(entityId);

    auto it = std::find_if(entity.shapes.begin(), entity.shapes.end(),
                           [shapeId](const ShapeInstance& shape) { return shape.id == shapeId; });
    assert(it != entity.shapes.end() && "Shape with this ID does not exist in the entity");

    auto& tree = treeFor(entity.type);
    tree.removeProxy(it->treeHandle);

    freeShapeIds.push_back(shapeId);
    entity.shapes.erase(it);
}

template<typename EntityId>
void Registry<EntityId>::setShapeActive(ShapeId shapeId, bool isActive)
{
    assert(shapeEntity.size() > shapeId && "Shape ID out of bounds");
    const auto [entityId, shapeIndex] = shapeEntity.at(shapeId);

    assert(entities.find(entityId) != entities.end() && "Entity with this ID does not exist");
    EntityInfo& entity = entities.at(entityId);

    entity.shapes[shapeIndex].isActive = isActive;
}

template<typename EntityId>
void Registry<EntityId>::removeEntity(EntityId id)
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

template<typename EntityId>
void Registry<EntityId>::teleportEntity(EntityId id, const Transform& transform)
{
    assert(entities.find(id) != entities.end());
    EntityInfo& entity = entities.at(id);
    
    auto& tree = treeFor(entity.type);

    if (entity.type == BodyType::Bullet)
    {
        for (auto& shape : entity.shapes)
        {
            std::visit([&](const auto& concreteShape)
            {
                const auto oldAABB = shape.aabb;
                shape.aabb = computeAABB(concreteShape, transform);
                tree.moveProxy(shape.treeHandle, AABB::combine(oldAABB, shape.aabb));
            }, shape.shape);
        }
        
        bulletPreviousTransforms.at(id) = entity.transform;
    }
    else
    {
        for (auto& shape : entity.shapes)
        {
            std::visit([&](const auto& concreteShape)
            {
                shape.aabb = computeAABB(concreteShape, transform);
                tree.moveProxy(shape.treeHandle, shape.aabb);
            }, shape.shape);
        }
    }
    
    entity.transform = transform;
}

template<typename EntityId>
void Registry<EntityId>::teleportEntity(EntityId id, Translation translation)
{
    assert(entities.find(id) != entities.end());
    EntityInfo& entity = entities.at(id);
    
    const auto deltaTranslation = translation - entity.transform.translation;
    moveEntity(id, deltaTranslation);
}

template<typename EntityId>
void Registry<EntityId>::moveEntity(EntityId id, const Transform& delta)
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
            std::visit([&](const auto& concreteShape)
            {
                const auto oldAABB = shape.aabb;
                shape.aabb = computeAABB(concreteShape, entity.transform);
                tree.moveProxy(shape.treeHandle, AABB::combine(oldAABB, shape.aabb));
            }, shape.shape);
        }
    }
    else
    {
        entity.transform += delta;

        for (auto& shape : entity.shapes)
        {
            std::visit([&](const auto& concreteShape)
            {
                shape.aabb = computeAABB(concreteShape, entity.transform);
                tree.moveProxy(shape.treeHandle, shape.aabb);
            }, shape.shape);
        }
    }
}

template<typename EntityId>
void Registry<EntityId>::moveEntity(EntityId id, Translation deltaTranslation)
{
    assert(entities.find(id) != entities.end());
    EntityInfo& entity = entities.at(id);
    
    auto& tree = treeFor(entity.type);
    
    if (entity.type == BodyType::Bullet)
    {
        bulletPreviousTransforms.at(id) = entity.transform;

        for (auto& shape : entity.shapes)
        {
            std::visit([&](const auto& concreteShape)
            {
                const auto oldAABB = shape.aabb;
                shape.aabb.translate(deltaTranslation);
                tree.moveProxy(shape.treeHandle, AABB::combine(oldAABB, shape.aabb));
            }, shape.shape);
        }
    }
    else
    {
        auto& tree = treeFor(entity.type);
        for (auto& shape : entity.shapes)
        {
            std::visit([&](const auto& concreteShape)
            {
                shape.aabb.translate(deltaTranslation);
                tree.moveProxy(shape.treeHandle, shape.aabb);
            }, shape.shape);
        }
    }
    
    entity.transform.translation += deltaTranslation;
}

template<typename EntityId>
std::vector<typename Registry<EntityId>::EntityCollision> Registry<EntityId>::getCollidingPairs()
{
    // std::vector<std::pair<AABB, BitMaskType>> dynamicQueries;
    // std::vector<ShapeId> dynamicShapesQueried;
    // std::vector<std::pair<AABB, BitMaskType>> staticQueries;
    // std::vector<ShapeId> staticShapesQueried;

    // getAllCollisionAABBQueries(dynamicQueries, staticQueries, dynamicShapesQueried, staticShapesQueried);

    std::vector<EntityCollision> collisionsInfo;
    collisionsInfo.reserve(previousAllCollisionsCount);

    const auto handlePairCollisions = [&](std::vector<std::pair<ShapeId, ShapeId>> collisions) {
        narrowPhaseCollisions(collisions, collisionsInfo);
    };

    treeDynamic.findAllCollisions(treeStatic, handlePairCollisions);
    treeBullet.findAllCollisions(treeStatic, handlePairCollisions);
    treeBullet.findAllCollisions(treeDynamic, handlePairCollisions);
    treeDynamic.findAllCollisions(handlePairCollisions);
    treeBullet.findAllCollisions(handlePairCollisions);

    previousAllCollisionsCount = collisionsInfo.size();
    return collisionsInfo;
}

template<typename EntityId>
std::vector<typename Registry<EntityId>::EntityCollision> Registry<EntityId>::getCollisions(EntityId id) const
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
            const auto oldAABB = std::visit([&](const auto& shape) {
                return computeAABB(shape, previousTransform);
            }, shape.shape);

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
        treeStatic.batchQuery(queries, [&](size_t i, std::vector<ShapeId> collisions) {
            narrowPhaseCollisions(shapesQueried[i], collisions, collisionsInfo);
        });
    }
    treeDynamic.batchQuery(queries, [&](size_t i, std::vector<ShapeId> collisions) {
        narrowPhaseCollisions(shapesQueried[i], collisions, collisionsInfo);
    });
    treeBullet.batchQuery(queries, [&](size_t i, std::vector<ShapeId> collisions) {
        narrowPhaseCollisions(shapesQueried[i], collisions, collisionsInfo);
    });

    return collisionsInfo;
}

template <typename EntityId>
void Registry<EntityId>::getAllCollisionAABBQueries(std::vector<std::pair<AABB, BitMaskType>>& dynamicQueries,
                                                    std::vector<std::pair<AABB, BitMaskType>>& staticQueries,
                                                    std::vector<ShapeId>& dynamicShapesQueried,
                                                    std::vector<ShapeId>& staticShapesQueried) const
{
    dynamicQueries.reserve(treeDynamic.size());
    staticQueries.reserve(treeStatic.size());
    dynamicShapesQueried.reserve(treeDynamic.size());
    staticShapesQueried.reserve(treeStatic.size());

    for (const auto& [entityId, entity] : entities)
    {
        if (entity.type == BodyType::Static)
        {
            for (const auto& shape : entity.shapes)
            {
                if (!shape.isActive)
                    continue;

                staticQueries.emplace_back(shape.aabb, shape.maskBits);
                staticShapesQueried.push_back(shape.id);
            }
        }
        else if (entity.type == BodyType::Dynamic)
        {
            for (const auto& shape : entity.shapes)
            {
                if (!shape.isActive)
                    continue;

                dynamicQueries.emplace_back(shape.aabb, shape.maskBits);
                dynamicShapesQueried.push_back(shape.id);
            }
        }
    }
}

template<typename EntityId>
void Registry<EntityId>::narrowPhaseCollision(ShapeId shapeAId,
                                              ShapeId shapeBId,
                                              std::vector<EntityCollision>& outCollisionsInfo) const
{
    const auto [queryEntityId, queryShapeIndex] = shapeEntity[shapeAId];
    const auto [otherEntityId, otherShapeIndex] = shapeEntity[shapeBId];
    const auto& queryEntity = entities.at(queryEntityId);
    const auto& otherEntity = entities.at(otherEntityId);
    const auto& queryShape = queryEntity.shapes[queryShapeIndex];
    const auto& otherShape = otherEntity.shapes[otherShapeIndex];

    // Skip self-collision
    if (queryEntityId == otherEntityId)
        return;

    if (!queryShape.isActive || !otherShape.isActive)
        return;
    
    Manifold manifold;

    // Sweep for bullets
    if (queryEntity.type == BodyType::Bullet)
    {
        const auto& previousTransform = bulletPreviousTransforms.at(queryEntityId);
        const auto& currentTransform = queryEntity.transform;

        if (otherEntity.type != BodyType::Bullet)
        {
            manifold = std::visit([&](const auto& queryShapeConcrete, const auto& otherShapeConcrete) {
                const auto sweepManifold = c2d::sweep(queryShapeConcrete, previousTransform, currentTransform,
                                                        otherShapeConcrete, otherEntity.transform);
                return sweepManifold ? sweepManifold->manifold : Manifold{};
            }, queryShape.shape, otherShape.shape);
        }
        else
        {
            const auto& otherPreviousTransform = bulletPreviousTransforms.at(otherEntityId);
            const auto& otherCurrentTransform = otherEntity.transform;

            manifold = std::visit([&](const auto& queryShapeConcrete, const auto& otherShapeConcrete) {
                const auto sweepManifold = c2d::sweep(queryShapeConcrete, previousTransform, currentTransform,
                                                        otherShapeConcrete, otherPreviousTransform, otherCurrentTransform);
                return sweepManifold ? sweepManifold->manifold : Manifold{};
            }, queryShape.shape, otherShape.shape);
        }
    }
    else
    {
        manifold = std::visit([&](const auto& queryShapeConcrete, const auto& otherShapeConcrete) {
            return c2d::collide(queryShapeConcrete, queryEntity.transform, otherShapeConcrete, otherEntity.transform);
        }, queryShape.shape, otherShape.shape);
    }

    if (!manifold.isColliding())
        return;

    outCollisionsInfo.emplace_back(EntityCollision{queryEntityId,
                                                   otherEntityId,
                                                   shapeAId,
                                                   shapeBId,
                                                   std::move(manifold)});
}

template<typename EntityId>
void Registry<EntityId>::narrowPhaseCollisions(std::vector<std::pair<ShapeId, ShapeId>>& collisions,
                                               std::vector<EntityCollision>& outCollisionsInfo) const
{
    // Sort to detect duplicates -> duplicates will be adjacent in the sorted vector
    std::sort(collisions.begin(), collisions.end());

    narrowPhaseCollision(collisions[0].first, collisions[0].second, outCollisionsInfo);
    for (size_t i = 1; i < collisions.size(); ++i)
    {
        // Skip duplicates
        if ((collisions[i].first == collisions[i - 1].first && collisions[i].second == collisions[i - 1].second) ||
            (collisions[i].first == collisions[i - 1].second && collisions[i].second == collisions[i - 1].first))
            continue;

        narrowPhaseCollision(collisions[i].first, collisions[i].second, outCollisionsInfo);
    }
}

template<typename EntityId>
void Registry<EntityId>::narrowPhaseCollisions(ShapeId shapeQueried,
                                               std::vector<ShapeId>& collisions,
                                               std::vector<EntityCollision>& outCollisionsInfo) const
{
    // Sort to detect duplicates -> duplicates will be adjacent in the sorted vector
    std::sort(collisions.begin(), collisions.end());
    
    narrowPhaseCollision(shapeQueried, collisions[0], outCollisionsInfo);
    for (size_t i = 1; i < collisions.size(); ++i)
    {
        // Skip duplicates
        if (collisions[i] == collisions[i - 1])
            continue;

        narrowPhaseCollision(shapeQueried, collisions[i], outCollisionsInfo);
    }
}

template<typename EntityId>
bool Registry<EntityId>::areColliding(EntityId a, EntityId b) const
{
    assert(entities.find(a) != entities.end() && "Entity A does not exist");
    assert(entities.find(b) != entities.end() && "Entity B does not exist");

    const EntityInfo& entityA = entities.at(a);
    const EntityInfo& entityB = entities.at(b);

    if (entityA.type == BodyType::Static && entityB.type == BodyType::Static)
        return false;

    if (entityA.type == BodyType::Bullet && entityB.type == BodyType::Bullet)
    {
        const auto& entityAPreviousTransform = bulletPreviousTransforms.at(a);
        const auto& entityBPreviousTransform = bulletPreviousTransforms.at(b);
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
                
                std::optional<SweepManifold> manifold = std::visit([&](const auto& shapeAConcrete, const auto& shapeBConcrete) {
                    return c2d::sweep(shapeAConcrete, entityAPreviousTransform, entityACurrentTransform,
                                      shapeBConcrete, entityBPreviousTransform, entityBCurrentTransform);
                }, shapeA.shape, shapeB.shape);

                if (manifold)
                    return true;
            }
        }

        return false;
    }

    if (entityA.type == BodyType::Bullet || entityB.type == BodyType::Bullet)
    {
        const auto& bulletEntity = (entityA.type == BodyType::Bullet) ? entityA : entityB;
        const auto& bulletEntityId = (bulletEntity.type == BodyType::Bullet) ? a : b;
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

                std::optional<SweepManifold> manifold = std::visit([&](const auto& shapeBulletConcrete, const auto& shapeOtherConcrete) {
                    return c2d::sweep(shapeBulletConcrete, bulletPreviousTransform, bulletCurrentTransform,
                                      shapeOtherConcrete, otherEntity.transform);
                }, shapeBullet.shape, shapeOther.shape);

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

            bool colliding = std::visit([&](const auto& shapeA, const auto& shapeB) {
                return c2d::areColliding(shapeA, entityA.transform, shapeB, entityB.transform);
            }, shapeA.shape, shapeB.shape);

            if (colliding)
                return true;
        }
    }

    return false;
}

template<typename EntityId>
std::optional<RaycastHit<std::pair<EntityId, ShapeId>>> Registry<EntityId>::firstHitRayCast(Ray ray, BitMaskType maskBits) const
{
    std::optional<RaycastHit<std::pair<EntityId, ShapeId>>> bestHit;
    float bestEntryTime = std::numeric_limits<float>::max();

    const auto staticHits = treeStatic.piercingRaycast(ray, maskBits);
    const auto dynamicHits = treeDynamic.piercingRaycast(ray, maskBits);
    const auto bulletHits = treeBullet.piercingRaycast(ray, maskBits);

    uint32_t toCheck = staticHits.size();
    for (const auto& hit : staticHits)
    {
        const auto& shapeInfo = shapeEntity[hit.id];
        const auto entityId = shapeInfo.first;
        const auto& entity = entities.at(entityId);
        const auto& shape = entity.shapes[shapeInfo.second];

        if (!shape.isActive)
            continue;
        
        const auto narrowHit = std::visit([&](const auto& shapeConcrete) {
            return raycast(shapeConcrete, entity.transform, ray);
        }, shape.shape);
        
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
        
        const auto narrowHit = std::visit([&](const auto& shapeConcrete) {
            return raycast(shapeConcrete, entity.transform, ray);
        }, shape.shape);

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
        const auto narrowHit = std::visit([&](const auto& shapeConcrete) {
            return sweep(shapeConcrete, previousTransform, entity.transform, ray);
        }, shape.shape);

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

template<typename EntityId>
std::optional<RaycastHit<std::pair<EntityId, ShapeId>>> Registry<EntityId>::firstHitRayCast(InfiniteRay ray, BitMaskType maskBits) const
{
    const auto staticHits = treeStatic.piercingRaycast(ray, maskBits);
    const auto dynamicHits = treeDynamic.piercingRaycast(ray, maskBits);
    const auto bulletHits = treeBullet.piercingRaycast(ray, maskBits);

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
        
        const auto narrowHit = std::visit([&](const auto& shapeConcrete) {
            return raycast(shapeConcrete, entity.transform, ray);
        }, shape.shape);

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
        
        const auto narrowHit = std::visit([&](const auto& shapeConcrete) {
            return raycast(shapeConcrete, entity.transform, ray);
        }, shape.shape);
        
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
        const auto narrowHit = std::visit([&](const auto& shapeConcrete) {
            return sweep(shapeConcrete, previousTransform, entity.transform, ray);
        }, shape.shape);

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

template<typename EntityId>
std::set<RaycastHit<std::pair<EntityId, ShapeId>>> Registry<EntityId>::rayCast(Ray ray, BitMaskType maskBits) const
{
    const auto staticHits = treeStatic.piercingRaycast(ray, maskBits);
    const auto dynamicHits = treeDynamic.piercingRaycast(ray, maskBits);
    const auto bulletHits = treeBullet.piercingRaycast(ray, maskBits);

    std::set<RaycastHit<std::pair<EntityId, ShapeId>>> hits;

    for (const auto& hit : staticHits)
    {
        const auto& shapeInfo = shapeEntity[hit.id];
        const auto entityId = shapeInfo.first;
        const auto& entity = entities.at(entityId);
        const auto& shape = entity.shapes[shapeInfo.second];

        if (!shape.isActive)
            continue;
        
        const auto narrowHit = std::visit([&](const auto& shapeConcrete) {
            return raycast(shapeConcrete, entity.transform, ray);
        }, shape.shape);

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
        
        const auto narrowHit = std::visit([&](const auto& shapeConcrete) {
            return raycast(shapeConcrete, entity.transform, ray);
        }, shape.shape);

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
        const auto narrowHit = std::visit([&](const auto& shapeConcrete) {
            return sweep(shapeConcrete, previousTransform, entity.transform, ray);
        }, shape.shape);

        if (narrowHit)
            hits.insert(RaycastHit<std::pair<EntityId, ShapeId>>::fromRay({entityId, hit.id}, ray, *narrowHit));
    }

    return hits;
}

template<typename EntityId>
std::set<RaycastHit<std::pair<EntityId, ShapeId>>> Registry<EntityId>::rayCast(InfiniteRay ray, BitMaskType maskBits) const
{
    const auto staticHits = treeStatic.piercingRaycast(ray, maskBits);
    const auto dynamicHits = treeDynamic.piercingRaycast(ray, maskBits);
    const auto bulletHits = treeBullet.piercingRaycast(ray, maskBits);

    std::set<RaycastHit<std::pair<EntityId, ShapeId>>> hits;
    
    for (const auto& hit : staticHits)
    {
        const auto& shapeInfo = shapeEntity[hit.id];
        const auto entityId = shapeInfo.first;
        const auto& entity = entities.at(entityId);
        const auto& shape = entity.shapes[shapeInfo.second];

        if (!shape.isActive)
            continue;
        
        const auto narrowHit = std::visit([&](const auto& shapeConcrete) {
            return raycast(shapeConcrete, entity.transform, ray);
        }, shape.shape);
        
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
        
        const auto narrowHit = std::visit([&](const auto& shapeConcrete) {
            return raycast(shapeConcrete, entity.transform, ray);
        }, shape.shape);
        
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
        const auto narrowHit = std::visit([&](const auto& shapeConcrete) {
            return sweep(shapeConcrete, previousTransform, entity.transform, ray);
        }, shape.shape);

        if (narrowHit)
            hits.insert(RaycastHit<std::pair<EntityId, ShapeId>>::fromRay({entityId, hit.id}, ray, *narrowHit));
    }

    return hits;
}

template<typename EntityId>
size_t Registry<EntityId>::size() const
{
    return entities.size();
}

template<typename EntityId>
void Registry<EntityId>::clear()
{
    entities.clear();
    shapeEntity.clear();
    freeShapeIds.clear();
    treeStatic.clear();
    treeDynamic.clear();
    treeBullet.clear();
    bulletPreviousTransforms.clear();
}

} // namespace c2d
