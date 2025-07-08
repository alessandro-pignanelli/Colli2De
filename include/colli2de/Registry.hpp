#pragma once

#include <variant>
#include <vector>
#include <unordered_map>

#include "collision/Manifold.hpp"
#include "colli2de/Ray.hpp"
#include "colli2de/Shapes.hpp"
#include "data_structures/BroadPhaseTree.hpp"
#include "geometry/AABB.hpp"
#include "geometry/Transformations.hpp"
#include "geometry/ShapesUtils.hpp"
#include "geometry/RaycastShapes.hpp"
#include "collision/Collision.hpp"

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

    void createEntity(EntityId id, BodyType type, const Transform& transform = Transform{});
    void removeEntity(EntityId id);

    template <IsShape Shape>
    ShapeId addShape(EntityId entityId, const Shape& shape, uint64_t categoryBits = 1, uint64_t maskBits = ~0ull);
    void removeShape(ShapeId shapeId);

    void teleportEntity(EntityId id, const Transform& transform);
    void moveEntity(EntityId id, const Transform& delta);

    std::vector<EntityCollision> getCollidingPairs();
    std::vector<EntityCollision> getCollisions(EntityId id);
    bool areColliding(EntityId a, EntityId b);

    std::optional<RaycastHit<std::pair<EntityId, ShapeId>>> firstHitRayCast(Ray ray, BitMaskType maskBits = ~0ull);
    std::optional<RaycastHit<std::pair<EntityId, ShapeId>>> firstHitRayCast(InfiniteRay ray, BitMaskType maskBits = ~0ull);
    std::set<RaycastHit<std::pair<EntityId, ShapeId>>> rayCast(Ray ray, BitMaskType maskBits = ~0ull);
    std::set<RaycastHit<std::pair<EntityId, ShapeId>>> rayCast(InfiniteRay ray, BitMaskType maskBits = ~0ull);

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
    };

    struct EntityInfo
    {
        std::vector<ShapeInstance> shapes;
        Transform transform;
        BodyType type;
    };

    std::unordered_map<EntityId, EntityInfo> entities;
    std::vector<std::pair<EntityId, size_t>> shapeEntity;
    std::vector<ShapeId> freeShapeIds;

    BroadPhaseTree<ShapeId> treeStatic;
    BroadPhaseTree<ShapeId> treeDynamic;
    std::unordered_map<EntityId, Transform> bulletPreviousTransforms;
    uint32_t totalDynamicShapes = 0;

    constexpr BroadPhaseTree<ShapeId>& treeFor(BodyType type);
    void narrowPhaseCollisions(const std::vector<ShapeId>& shapesQueried,
                               const std::vector<std::set<ShapeId>>& collisions,
                               std::vector<EntityCollision>& outCollisionsInfo);
};

template<typename EntityId>
constexpr BroadPhaseTree<ShapeId>& Registry<EntityId>::treeFor(BodyType type)
{
    switch (type)
    {
        case BodyType::Static: return treeStatic;
        case BodyType::Dynamic: return treeDynamic;
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
    
    if (entity.type == BodyType::Dynamic)
        totalDynamicShapes++;

    ShapeInstance instance;
    instance.id = shapeId;
    instance.categoryBits = categoryBits;
    instance.maskBits = maskBits;
    instance.shape = shape;
    instance.aabb = computeAABB(shape, entity.transform);

    if (entity.type != BodyType::Bullet)
    {
        auto& tree = treeFor(entity.type);
        instance.treeHandle = tree.addProxy(shapeId, instance.aabb, categoryBits, maskBits);
    }
    
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

    if (entity.type != BodyType::Bullet)
    {
        auto& tree = treeFor(entity.type);
        tree.removeProxy(it->treeHandle);
    }

    freeShapeIds.push_back(shapeId);
    entity.shapes.erase(it);
}

template<typename EntityId>
void Registry<EntityId>::removeEntity(EntityId id)
{
    auto it = entities.find(id);
    assert(it != entities.end());

    EntityInfo& entity = it->second;

    if (entity.type != BodyType::Bullet)
    {
        auto& tree = treeFor(entity.type);
        for (const auto& shape : entity.shapes)
        {
            tree.removeProxy(shape.treeHandle);
            freeShapeIds.push_back(shape.id);
        }

        if (entity.type == BodyType::Dynamic)
            totalDynamicShapes -= entity.shapes.size();
    }
    else
    {
        bulletPreviousTransforms.erase(id);
    }

    entities.erase(it);
}

template<typename EntityId>
void Registry<EntityId>::teleportEntity(EntityId id, const Transform& transform)
{
    assert(entities.find(id) != entities.end());
    EntityInfo& entity = entities.at(id);
    
    if (entity.type == BodyType::Bullet)
    {
        bulletPreviousTransforms[id] = entity.transform;
        entity.transform = transform;
    }
    else
    {
        entity.transform = transform;
        auto& tree = treeFor(entity.type);

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
void Registry<EntityId>::moveEntity(EntityId id, const Transform& delta)
{
    assert(entities.find(id) != entities.end());
    EntityInfo& entity = entities.at(id);

    if (entity.type == BodyType::Bullet)
    {
        bulletPreviousTransforms[id] = entity.transform;
        entity.transform += delta;
    }
    else
    {
        entity.transform += delta;
        auto& tree = treeFor(entity.type);

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
void Registry<EntityId>::narrowPhaseCollisions(const std::vector<ShapeId>& shapesQueried,
                                               const std::vector<std::set<ShapeId>>& collisions,
                                               std::vector<EntityCollision>& outCollisionsInfo)
{
    for (size_t i = 0; i < collisions.size(); ++i)
        for (const auto& otherId : collisions[i])
        {
            // Skip self-collision
            if (otherId == shapesQueried[i])
                continue;

            const ShapeId queryShapeId = shapesQueried[i];
            const ShapeId otherShapeId = otherId;
            const auto [queryEntityId, queryShapeIndex] = shapeEntity[queryShapeId];
            const auto [otherEntityId, otherShapeIndex] = shapeEntity[otherShapeId];
            const auto& queryEntity = entities.at(queryEntityId);
            const auto& otherEntity = entities.at(otherEntityId);
            const auto& queryShape = queryEntity.shapes[queryShapeIndex].shape;
            const auto& otherShape = otherEntity.shapes[otherShapeIndex].shape;
            
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
                    }, queryShape, otherShape);
                }
                else
                {
                    const auto& otherPreviousTransform = bulletPreviousTransforms.at(otherEntityId);
                    const auto& otherCurrentTransform = otherEntity.transform;

                    manifold = std::visit([&](const auto& queryShapeConcrete, const auto& otherShapeConcrete) {
                        const auto sweepManifold = c2d::sweep(queryShapeConcrete, previousTransform, currentTransform,
                                                              otherShapeConcrete, otherPreviousTransform, otherCurrentTransform);
                        return sweepManifold ? sweepManifold->manifold : Manifold{};
                    }, queryShape, otherShape);
                }
            }
            else
            {
                manifold = std::visit([&](const auto& queryShapeConcrete, const auto& otherShapeConcrete) {
                    return c2d::collide(queryShapeConcrete, queryEntity.transform, otherShapeConcrete, otherEntity.transform);
                }, queryShape, otherShape);
            }

            if (!manifold.isColliding())
                continue;

            outCollisionsInfo.emplace_back(EntityCollision{queryEntityId,
                                                                otherEntityId,
                                                                queryShapeId,
                                                                otherShapeId,
                                                                std::move(manifold)});
        }
}

template<typename EntityId>
std::vector<typename Registry<EntityId>::EntityCollision> Registry<EntityId>::getCollidingPairs()
{
    std::vector<std::pair<AABB, BitMaskType>> queries;
    std::vector<std::tuple<AABB, AABB, BitMaskType>> sweepQueries;
    std::vector<ShapeId> shapesQueried;
    std::vector<ShapeId> shapesSweepQueried;

    queries.reserve(totalDynamicShapes);
    sweepQueries.reserve(bulletPreviousTransforms.size());
    shapesQueried.reserve(totalDynamicShapes);
    shapesSweepQueried.reserve(bulletPreviousTransforms.size());

    for (const auto& [entityId, entity] : entities)
    {
        // Static entities are not checked for collisions
        if (entity.type == BodyType::Static)
            continue;

        if (entity.type == BodyType::Dynamic)
        {
            for (const auto& shape : entity.shapes)
            {
                queries.emplace_back(shape.aabb, shape.maskBits);
                shapesQueried.push_back(shape.id);
            }
        }
        else
        {
            // Bullet entities are checked with a sweep query
            const auto& previousTransform = bulletPreviousTransforms.at(entityId);
            const auto& currentTransform = entity.transform;
            const auto deltaTransform = previousTransform - currentTransform;
            for (const auto& shape : entity.shapes)
            {
                const auto previousAABB = shape.aabb.move(deltaTransform.translation);
                sweepQueries.emplace_back(shape.aabb, previousAABB, shape.maskBits);
                shapesSweepQueried.push_back(shape.id);
            }
        }
    }

    std::vector<EntityCollision> collisionsInfo;

    auto collisionsStatic = treeStatic.batchQuery(queries, std::thread::hardware_concurrency());
    assert(collisionsStatic.size() == queries.size() && "Batch query results size mismatch for static shapes");

    auto collisionsDynamic = treeDynamic.batchQuery(queries, std::thread::hardware_concurrency());
    assert(collisionsDynamic.size() == queries.size() && "Batch query results size mismatch for dynamic shapes");

    auto collisionsBulletsStatic = treeStatic.batchSweepQuery(sweepQueries, std::thread::hardware_concurrency());
    assert(collisionsBulletsStatic.size() == sweepQueries.size() && "Batch sweep query results size mismatch for static bullets");

    auto collisionsBulletsDynamic = treeDynamic.batchSweepQuery(sweepQueries, std::thread::hardware_concurrency());
    assert(collisionsBulletsDynamic.size() == sweepQueries.size() && "Batch sweep query results size mismatch for dynamic bullets");

    collisionsInfo.reserve(collisionsStatic.size() + collisionsDynamic.size() +
                           collisionsBulletsStatic.size() + collisionsBulletsDynamic.size());
    narrowPhaseCollisions(shapesQueried, collisionsStatic, collisionsInfo);
    narrowPhaseCollisions(shapesQueried, collisionsDynamic, collisionsInfo);
    narrowPhaseCollisions(shapesSweepQueried, collisionsBulletsStatic, collisionsInfo);
    narrowPhaseCollisions(shapesSweepQueried, collisionsBulletsDynamic, collisionsInfo);

    return collisionsInfo;
}

template<typename EntityId>
std::vector<typename Registry<EntityId>::EntityCollision> Registry<EntityId>::getCollisions(EntityId id)
{
    assert(entities.find(id) != entities.end());
    const EntityInfo& entity = entities.at(id);

    if (entity.type == BodyType::Bullet)
    {
        std::vector<std::tuple<AABB, AABB, BitMaskType>> sweepQueries;
        std::vector<ShapeId> shapesSweepQueried;

        sweepQueries.reserve(entity.shapes.size());
        shapesSweepQueried.reserve(entity.shapes.size());

        const auto& previousTransform = bulletPreviousTransforms.at(id);
        const auto& currentTransform = entity.transform;
        const auto deltaTransform = previousTransform - currentTransform;

        for (const auto& shape : entity.shapes)
        {
            const auto previousAABB = shape.aabb.move(deltaTransform.translation);
            sweepQueries.emplace_back(shape.aabb, previousAABB, shape.maskBits);
            shapesSweepQueried.push_back(shape.id);
        }

        std::vector<EntityCollision> collisionsInfo;

        auto collisionsBulletsStatic = treeStatic.batchSweepQuery(sweepQueries, std::thread::hardware_concurrency());
        assert(collisionsBulletsStatic.size() == sweepQueries.size() && "Batch sweep query results size mismatch for static bullets");

        auto collisionsBulletsDynamic = treeDynamic.batchSweepQuery(sweepQueries, std::thread::hardware_concurrency());
        assert(collisionsBulletsDynamic.size() == sweepQueries.size() && "Batch sweep query results size mismatch for dynamic bullets");

        narrowPhaseCollisions(shapesSweepQueried, collisionsBulletsStatic, collisionsInfo);
        narrowPhaseCollisions(shapesSweepQueried, collisionsBulletsDynamic, collisionsInfo);

        return collisionsInfo;
    }

    std::vector<std::pair<AABB, BitMaskType>> queries;
    std::vector<ShapeId> shapesQueried;

    queries.reserve(entity.shapes.size());
    shapesQueried.reserve(entity.shapes.size());

    for (const auto& shape : entity.shapes)
    {
        queries.emplace_back(shape.aabb, shape.maskBits);
        shapesQueried.push_back(shape.id);
    }
    
    std::vector<EntityCollision> collisionsInfo;

    if (entity.type != BodyType::Static)
    {
        const auto collisionsStatic = treeStatic.batchQuery(queries, std::thread::hardware_concurrency());
        assert(collisionsStatic.size() == queries.size() && "Batch query results size mismatch for static shapes");
        narrowPhaseCollisions(shapesQueried, collisionsStatic, collisionsInfo);
    }

    const auto collisionsDynamic = treeDynamic.batchQuery(queries, std::thread::hardware_concurrency());
    assert(collisionsDynamic.size() == queries.size() && "Batch query results size mismatch for dynamic shapes");
    narrowPhaseCollisions(shapesQueried, collisionsDynamic, collisionsInfo);

    for (const auto& [bulletEntityId, bulletPreviousTransform] : bulletPreviousTransforms)
    {
        const auto& otherEntity = entities.at(bulletEntityId);
        const auto& bulletCurrentTransform = bulletPreviousTransforms.at(bulletEntityId);

        for (const auto& bulletShape : otherEntity.shapes)
        {
            for (const auto& shape : entity.shapes)
            {
                std::optional<SweepManifold> manifold = std::visit([&](const auto& queryShapeConcrete, const auto& bulletShapeConcrete) {
                    return c2d::sweep(bulletShapeConcrete, bulletPreviousTransform, bulletCurrentTransform,
                                      queryShapeConcrete, entity.transform);
                }, shape.shape, bulletShape.shape);

                if (manifold)
                {
                    manifold->manifold.reverse();
                    collisionsInfo.emplace_back(EntityCollision{id,
                                                                    bulletEntityId,
                                                                    shape.id,
                                                                    bulletShape.id,
                                                                    std::move(manifold->manifold)});
                }
            }
        }
    }

    return collisionsInfo;
}

template<typename EntityId>
bool Registry<EntityId>::areColliding(EntityId a, EntityId b)
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
            for (const auto& shapeB : entityB.shapes)
            {
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
            for (const auto& shapeOther : otherEntity.shapes)
            {
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
        for (const auto& shapeB : entityB.shapes)
        {
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
std::optional<RaycastHit<std::pair<EntityId, ShapeId>>> Registry<EntityId>::firstHitRayCast(Ray ray, BitMaskType maskBits)
{
    const auto staticHits = treeStatic.piercingRaycast(ray, maskBits);
    const auto dynamicHits = treeDynamic.piercingRaycast(ray, maskBits);
    std::optional<RaycastHit<std::pair<EntityId, ShapeId>>> bestHit;
    float bestEntryTime = std::numeric_limits<float>::max();

    for (const auto& hit : staticHits)
    {
        const auto& shapeInfo = shapeEntity[hit.id];
        const auto entityId = shapeInfo.first;
        const auto& entity = entities[entityId];
        const auto& shape = entity.shapes[shapeInfo.second].shape;
        
        const auto narrowHit = std::visit([&](const auto& shapeConcrete) {
            return raycast(shapeConcrete, entity.transform, ray);
        }, shape);
        
        if (narrowHit && narrowHit->first < bestEntryTime)
        {
            bestEntryTime = narrowHit->first;
            bestHit = RaycastHit<std::pair<EntityId, ShapeId>>::fromRay({entityId, hit.id}, ray, *narrowHit);
        }
    }

    for (const auto& hit : dynamicHits)
    {
        const auto& shapeInfo = shapeEntity[hit.id];
        const auto entityId = shapeInfo.first;
        const auto& entity = entities[entityId];
        const auto& shape = entity.shapes[shapeInfo.second].shape;
        
        const auto narrowHit = std::visit([&](const auto& shapeConcrete) {
            return raycast(shapeConcrete, entity.transform, ray);
        }, shape);

        if (narrowHit && narrowHit->first < bestEntryTime)
        {
            bestEntryTime = narrowHit->first;
            bestHit = RaycastHit<std::pair<EntityId, ShapeId>>::fromRay({entityId, hit.id}, ray, *narrowHit);
        }
    }

    return bestHit;
}

template<typename EntityId>
std::optional<RaycastHit<std::pair<EntityId, ShapeId>>> Registry<EntityId>::firstHitRayCast(InfiniteRay ray, BitMaskType maskBits)
{
    const auto staticHits = treeStatic.piercingRaycast(ray, maskBits);
    const auto dynamicHits = treeDynamic.piercingRaycast(ray, maskBits);
    std::optional<RaycastHit<std::pair<EntityId, ShapeId>>> bestHit;
    float bestEntryTime = std::numeric_limits<float>::max();

    uint32_t toCheck = staticHits.size();
    for (const auto& hit : staticHits)
    {
        const auto& shapeInfo = shapeEntity[hit.id];
        const auto entityId = shapeInfo.first;
        const auto& entity = entities[entityId];
        const auto& shape = entity.shapes[shapeInfo.second].shape;
        
        const auto narrowHit = std::visit([&](const auto& shapeConcrete) {
            return raycast(shapeConcrete, entity.transform, ray);
        }, shape);

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
        const auto& entity = entities[entityId];
        const auto& shape = entity.shapes[shapeInfo.second].shape;
        
        const auto narrowHit = std::visit([&](const auto& shapeConcrete) {
            return raycast(shapeConcrete, entity.transform, ray);
        }, shape);
        
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

    // TODO: Bullets

    return bestHit;
}

template<typename EntityId>
std::set<RaycastHit<std::pair<EntityId, ShapeId>>> Registry<EntityId>::rayCast(Ray ray, BitMaskType maskBits)
{
    const auto staticHits = treeStatic.piercingRaycast(ray, maskBits);
    const auto dynamicHits = treeDynamic.piercingRaycast(ray, maskBits);
    std::set<RaycastHit<std::pair<EntityId, ShapeId>>> hits;

    for (const auto& hit : staticHits)
    {
        const auto& shapeInfo = shapeEntity[hit.id];
        const auto entityId = shapeInfo.first;
        const auto& entity = entities[entityId];
        const auto& shape = entity.shapes[shapeInfo.second].shape;
        
        const auto narrowHit = std::visit([&](const auto& shapeConcrete) {
            return raycast(shapeConcrete, entity.transform, ray);
        }, shape);

        if (narrowHit)
            hits.insert(RaycastHit<std::pair<EntityId, ShapeId>>::fromRay({entityId, hit.id}, ray, *narrowHit));
    }

    for (const auto& hit : dynamicHits)
    {
        const auto& shapeInfo = shapeEntity[hit.id];
        const auto entityId = shapeInfo.first;
        const auto& entity = entities[entityId];
        const auto& shape = entity.shapes[shapeInfo.second].shape;
        
        const auto narrowHit = std::visit([&](const auto& shapeConcrete) {
            return raycast(shapeConcrete, entity.transform, ray);
        }, shape);

        if (!narrowHit)
            hits.insert(RaycastHit<std::pair<EntityId, ShapeId>>::fromRay({entityId, hit.id}, ray, *narrowHit));
    }

    // TODO: Bullets

    return hits;
}

template<typename EntityId>
std::set<RaycastHit<std::pair<EntityId, ShapeId>>> Registry<EntityId>::rayCast(InfiniteRay ray, BitMaskType maskBits)
{
    const auto staticHits = treeStatic.piercingRaycast(ray, maskBits);
    const auto dynamicHits = treeDynamic.piercingRaycast(ray, maskBits);
    std::set<RaycastHit<std::pair<EntityId, ShapeId>>> hits;
    
    for (const auto& hit : staticHits)
    {
        const auto& shapeInfo = shapeEntity[hit.id];
        const auto entityId = shapeInfo.first;
        const auto& entity = entities[entityId];
        const auto& shape = entity.shapes[shapeInfo.second].shape;
        
        const auto narrowHit = std::visit([&](const auto& shapeConcrete) {
            return raycast(shapeConcrete, entity.transform, ray);
        }, shape);
        
        if (narrowHit)
            hits.insert(RaycastHit<std::pair<EntityId, ShapeId>>::fromRay({entityId, hit.id}, ray, *narrowHit));
    }

    for (const auto& hit : dynamicHits)
    {
        const auto& shapeInfo = shapeEntity[hit.id];
        const auto entityId = shapeInfo.first;
        const auto& entity = entities[entityId];
        const auto& shape = entity.shapes[shapeInfo.second].shape;
        
        const auto narrowHit = std::visit([&](const auto& shapeConcrete) {
            return raycast(shapeConcrete, entity.transform, ray);
        }, shape);
        
        if (narrowHit)
            hits.insert(RaycastHit<std::pair<EntityId, ShapeId>>::fromRay({entityId, hit.id}, ray, *narrowHit));
    }

    // TODO: Bullets

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
    bulletPreviousTransforms.clear();
    totalDynamicShapes = 0;
}

} // namespace c2d
