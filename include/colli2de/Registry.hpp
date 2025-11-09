#pragma once

#include "colli2de/internal/utils/Methods.hpp"

#include <colli2de/Manifold.hpp>
#include <colli2de/Ray.hpp>
#include <colli2de/Shapes.hpp>
#include <colli2de/Transform.hpp>
#include <colli2de/internal/collision/Collision.hpp>
#include <colli2de/internal/data_structures/BroadPhaseTree.hpp>
#include <colli2de/internal/geometry/AABB.hpp>
#include <colli2de/internal/geometry/RaycastShapes.hpp>
#include <colli2de/internal/geometry/ShapesUtils.hpp>
#include <colli2de/internal/utils/Debug.hpp>

#include <tsl/robin_map.h>
#include <algorithm>
#include <variant>
#include <vector>


#ifdef C2D_USE_CEREAL
#include <cereal/types/utility.hpp>
#include <cereal/types/variant.hpp>
#include <cereal/types/vector.hpp>
#endif

namespace c2d
{

using ShapeId = uint32_t;

constexpr uint8_t bodyTypesCount = 3;
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

template <typename EntityId, PartitioningMethod Method = PartitioningMethod::None>
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

    Registry(uint32_t cellSize) requires(Method == PartitioningMethod::Grid)
        : trees{BroadPhaseTree<ShapeId>(cellSize), BroadPhaseTree<ShapeId>(cellSize), BroadPhaseTree<ShapeId>(cellSize)}
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

#ifdef C2D_USE_CEREAL
    template <class Archive>
    void save(Archive& archive) const;

    template <class Archive>
    void load(Archive& archive);
#endif

    // Used for testing equality
    bool operator==(const Registry& other) const;

    bool operator!=(const Registry& other) const
    {
        return !(*this == other);
    }

  private:
    struct ShapeInstance
    {
        ShapeVariant shape;
        AABB aabb;
        BitMaskType categoryBits = {1};
        BitMaskType maskBits = {~0ull};
        std::pair<uint8_t, uint8_t> moveFncIndices;
        uint8_t translateFncIndex;
        bool isActive = true;

        void serialize(std::ostream& out) const;
        static ShapeInstance deserialize(std::istream& in);

#ifdef C2D_USE_CEREAL
        template <class Archive>
        void serialize(Archive& archive);
#endif

        bool operator==(const ShapeInstance& other) const;

        bool operator!=(const ShapeInstance& other) const
        {
            return !(*this == other);
        }
    };

    struct EntityInfo
    {
        std::vector<ShapeId> shapeIds;
        Transform transform;
        BodyType type;

        void serialize(std::ostream& out) const;
        static EntityInfo deserialize(std::istream& in);

#ifdef C2D_USE_CEREAL
        template <class Archive>
        void serialize(Archive& archive);
#endif

        bool operator==(const EntityInfo& other) const;

        bool operator!=(const EntityInfo& other) const
        {
            return !(*this == other);
        }
    };

    tsl::robin_map<EntityId, EntityInfo> entities;
    std::vector<EntityId> shapeEntity;
    std::vector<ShapeInstance> shapes;
    std::vector<BroadPhaseTreeHandle> shapeTreeHandles;
    std::vector<ShapeId> freeShapeIds;

    using TreeType =
        std::conditional_t<Method == PartitioningMethod::Grid, BroadPhaseTree<ShapeId>, DynamicBVH<ShapeId>>;
    TreeType trees[3]; // 0: Static, 1: Dynamic, 2: Bullet

    tsl::robin_map<EntityId, Transform> bulletPreviousTransforms;
    uint32_t previousAllCollisionsCount = 1024;

    constexpr inline TreeType& treeFor(BodyType type)
    {
        return trees[static_cast<size_t>(type)];
    }

    constexpr inline const TreeType& treeFor(BodyType type) const
    {
        return trees[static_cast<size_t>(type)];
    }

    void narrowPhaseCollision(ShapeId shapeAId,
                              ShapeId shapeBId,
                              std::vector<EntityCollision>& outCollisionsInfo) const;
    template <typename Allocator>
    void narrowPhaseCollisions(ShapeId shapeQueried,
                               std::vector<ShapeId, Allocator>& collisions,
                               std::vector<EntityCollision>& outCollisionsInfo) const;
    template <typename Allocator>
    void narrowPhaseCollisions(std::vector<std::pair<ShapeId, ShapeId>, Allocator>& collisions,
                               std::vector<EntityCollision>& outCollisionsInfo) const;

    template <IsShape Shape>
    static void moveDynamicShape(ShapeInstance& shapeInstance,
                                 Transform newTransform,
                                 TreeType& tree,
                                 BroadPhaseTreeHandle treeHandle)
    {
        const auto& concreteShape = std::get<Shape>(shapeInstance.shape);
        shapeInstance.aabb = computeAABB(concreteShape, newTransform);
        tree.moveProxy(treeHandle, shapeInstance.aabb);
    }

    template <IsShape Shape>
    static void moveBulletShape(ShapeInstance& shapeInstance,
                                Transform newTransform,
                                TreeType& tree,
                                BroadPhaseTreeHandle treeHandle)
    {
        const auto& concreteShape = std::get<Shape>(shapeInstance.shape);
        const auto oldAABB = shapeInstance.aabb;
        shapeInstance.aabb = computeAABB(concreteShape, newTransform);
        tree.moveProxy(treeHandle, AABB::combine(oldAABB, shapeInstance.aabb));
    }

    static void translateDynamicShape(ShapeInstance& shapeInstance,
                                      Translation translation,
                                      TreeType& tree,
                                      BroadPhaseTreeHandle treeHandle)
    {
        shapeInstance.aabb.translate(translation);
        tree.moveProxy(treeHandle, shapeInstance.aabb);
    }

    static void translateBulletShape(ShapeInstance& shapeInstance,
                                     Translation translation,
                                     TreeType& tree,
                                     BroadPhaseTreeHandle treeHandle)
    {
        const auto oldAABB = shapeInstance.aabb;
        shapeInstance.aabb.translate(translation);
        tree.moveProxy(treeHandle, AABB::combine(oldAABB, shapeInstance.aabb));
    }

    using MoveFunction = std::function<void(ShapeInstance&, Transform, TreeType&, BroadPhaseTreeHandle)>;
    using TranslateFunction = std::function<void(ShapeInstance&, Translation, TreeType&, BroadPhaseTreeHandle)>;

    inline static MoveFunction moveFnc[bodyTypesCount][shapeTypesCount] = {
        {
            &Registry<EntityId, Method>::moveDynamicShape<Shape0>,
            &Registry<EntityId, Method>::moveDynamicShape<Shape1>,
            &Registry<EntityId, Method>::moveDynamicShape<Shape2>,
            &Registry<EntityId, Method>::moveDynamicShape<Shape3>,
        },
        {
            &Registry<EntityId, Method>::moveDynamicShape<Shape0>,
            &Registry<EntityId, Method>::moveDynamicShape<Shape1>,
            &Registry<EntityId, Method>::moveDynamicShape<Shape2>,
            &Registry<EntityId, Method>::moveDynamicShape<Shape3>,
        },
        {
            &Registry<EntityId, Method>::moveBulletShape<Shape0>,
            &Registry<EntityId, Method>::moveBulletShape<Shape1>,
            &Registry<EntityId, Method>::moveBulletShape<Shape2>,
            &Registry<EntityId, Method>::moveBulletShape<Shape3>,
        },
    };
    inline static TranslateFunction translateFnc[bodyTypesCount] = {
        &Registry<EntityId, Method>::translateDynamicShape,
        &Registry<EntityId, Method>::translateDynamicShape,
        &Registry<EntityId, Method>::translateBulletShape,
    };
};

template <typename EntityId, PartitioningMethod Method>
void Registry<EntityId, Method>::createEntity(EntityId id, BodyType type, const Transform& transform)
{
    DEBUG_ASSERT(entities.find(id) == entities.end(), "Entity with this ID already exists");
    entities.emplace(id, EntityInfo{.shapeIds = {}, .transform = transform, .type = type});

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
    DEBUG_ASSERT(entities.find(entityId) != entities.end(), "Entity with this ID does not exist");
    EntityInfo& entity = entities.at(entityId);

    const bool reuseShapeId = !freeShapeIds.empty();
    ShapeId shapeId;
    if (reuseShapeId)
    {
        shapeId = freeShapeIds.back();
        freeShapeIds.pop_back();
    }
    else
    {
        shapeId = shapes.size();
        DEBUG_ASSERT(shapeId == shapeEntity.size());
        DEBUG_ASSERT(shapeId == shapeTreeHandles.size());
    }

    auto& tree = treeFor(entity.type);
    const auto shapeAABB = computeAABB(shape, entity.transform);
    const auto treeHandle = tree.addProxy(shapeId, shapeAABB, categoryBits, maskBits);

    if (reuseShapeId)
    {
        shapeEntity[shapeId] = entityId;
        shapes[shapeId] = ShapeInstance{
            .shape = shape,
            .aabb = shapeAABB,
            .categoryBits = categoryBits,
            .maskBits = maskBits,
            .moveFncIndices = {static_cast<uint8_t>(entity.type), static_cast<uint8_t>(shape.getType())},
            .translateFncIndex = static_cast<uint8_t>(entity.type),
        };
        shapeTreeHandles[shapeId] = treeHandle;
        entity.shapeIds.push_back(shapeId);
    }
    else
    {
        shapeEntity.push_back(entityId);
        shapes.emplace_back(ShapeInstance{
            .shape = shape,
            .aabb = shapeAABB,
            .categoryBits = categoryBits,
            .maskBits = maskBits,
            .moveFncIndices = {static_cast<uint8_t>(entity.type), static_cast<uint8_t>(shape.getType())},
            .translateFncIndex = static_cast<uint8_t>(entity.type),
        });
        shapeTreeHandles.push_back(treeHandle);
        entity.shapeIds.push_back(shapeId);
    }

    return shapeId;
}

template <typename EntityId, PartitioningMethod Method>
void Registry<EntityId, Method>::removeShape(ShapeId shapeId)
{
    DEBUG_ASSERT(shapeId < shapes.size(), "Shape ID out of bounds");
    const EntityId entityId = shapeEntity.at(shapeId);

    DEBUG_ASSERT(entities.find(entityId) != entities.end(), "Entity with this ID does not exist");
    EntityInfo& entity = entities.at(entityId);

    const auto treeHandle = shapeTreeHandles.at(shapeId);
    auto& tree = treeFor(entity.type);
    tree.removeProxy(treeHandle);

    freeShapeIds.push_back(shapeId);

    const auto shapeIt = std::find(entity.shapeIds.begin(), entity.shapeIds.end(), shapeId);
    DEBUG_ASSERT(shapeIt != entity.shapeIds.end(), "Shape with this ID does not exist in the entity");
    entity.shapeIds.erase(shapeIt);
}

template <typename EntityId, PartitioningMethod Method>
void Registry<EntityId, Method>::setShapeActive(ShapeId shapeId, bool isActive)
{
    DEBUG_ASSERT(shapeId < shapes.size(), "Shape ID out of bounds");
    shapes[shapeId].isActive = isActive;
}

template <typename EntityId, PartitioningMethod Method>
void Registry<EntityId, Method>::removeEntity(EntityId id)
{
    auto entityIt = entities.find(id);
    DEBUG_ASSERT(entityIt != entities.end(), "Entity with this ID does not exist");
    auto& entity = entityIt->second;

    auto& tree = treeFor(entity.type);
    for (const auto shapeId : entity.shapeIds)
    {
        DEBUG_ASSERT(shapeId < shapes.size(), "Shape ID out of bounds");
        const auto treeHandle = shapeTreeHandles[shapeId];
        tree.removeProxy(treeHandle);
        freeShapeIds.push_back(shapeId);
    }

    if (entity.type == BodyType::Bullet)
        bulletPreviousTransforms.erase(id);

    entities.erase(entityIt);
}

template <typename EntityId, PartitioningMethod Method>
void Registry<EntityId, Method>::teleportEntity(EntityId entityId, const Transform& transform)
{
    DEBUG_ASSERT(entities.find(entityId) != entities.end());
    EntityInfo& entity = entities.at(entityId);
    auto& tree = treeFor(entity.type);

    for (const auto shapeId : entity.shapeIds)
    {
        DEBUG_ASSERT(shapeId < shapes.size(), "Shape ID out of bounds");
        const auto treeHandle = shapeTreeHandles[shapeId];
        auto& shape = shapes[shapeId];
        moveFnc[shape.moveFncIndices.first][shape.moveFncIndices.second](shape, transform, tree, treeHandle);
    }

    if (entity.type == BodyType::Bullet)
        bulletPreviousTransforms.at(entityId) = entity.transform;

    entity.transform = transform;
}

template <typename EntityId, PartitioningMethod Method>
void Registry<EntityId, Method>::teleportEntity(EntityId entityId, Translation translation)
{
    DEBUG_ASSERT(entities.find(entityId) != entities.end());
    const EntityInfo& entity = entities.at(entityId);

    const auto deltaTranslation = translation - entity.transform.translation;
    moveEntity(entityId, deltaTranslation);
}

template <typename EntityId, PartitioningMethod Method>
void Registry<EntityId, Method>::moveEntity(EntityId entityId, const Transform& delta)
{
    DEBUG_ASSERT(entities.find(entityId) != entities.end());
    auto& entity = entities.at(entityId);
    auto& tree = treeFor(entity.type);

    if (entity.type == BodyType::Bullet)
        bulletPreviousTransforms.at(entityId) = entity.transform;

    entity.transform += delta;

    for (const auto shapeId : entity.shapeIds)
    {
        DEBUG_ASSERT(shapeId < shapes.size(), "Shape ID out of bounds");
        const auto treeHandle = shapeTreeHandles[shapeId];
        auto& shape = shapes[shapeId];
        moveFnc[shape.moveFncIndices.first][shape.moveFncIndices.second](shape, entity.transform, tree, treeHandle);
    }
}

template <typename EntityId, PartitioningMethod Method>
void Registry<EntityId, Method>::moveEntity(EntityId id, Translation deltaTranslation)
{
    DEBUG_ASSERT(entities.find(id) != entities.end());
    auto& entity = entities.at(id);
    auto& tree = treeFor(entity.type);

    for (const auto shapeId : entity.shapeIds)
    {
        DEBUG_ASSERT(shapeId < shapes.size(), "Shape ID out of bounds");
        const auto treeHandle = shapeTreeHandles[shapeId];
        auto& shape = shapes[shapeId];
        translateFnc[shape.translateFncIndex](shape, deltaTranslation, tree, treeHandle);
    }

    if (entity.type == BodyType::Bullet)
        bulletPreviousTransforms.at(id) = entity.transform;

    entity.transform.translation += deltaTranslation;
}

template <typename EntityId, PartitioningMethod Method>
Transform Registry<EntityId, Method>::getEntityTransform(EntityId id) const
{
    DEBUG_ASSERT(entities.find(id) != entities.end());
    const EntityInfo& entity = entities.at(id);
    return entity.transform;
}

template <typename EntityId, PartitioningMethod Method>
std::vector<typename Registry<EntityId, Method>::EntityCollision> Registry<EntityId, Method>::getCollidingPairs()
{
    std::vector<EntityCollision> collisionsInfo;
    collisionsInfo.reserve(previousAllCollisionsCount + 10);

    const bool runInParallel = shapes.size() > 9'000;

    std::vector<std::pmr::unsynchronized_pool_resource> poolResource(runInParallel ? 5 : 1);

    auto collisions = runInParallel ? std::array<std::pmr::vector<std::pair<ShapeId, ShapeId>>, 5>{
        std::pmr::vector<std::pair<ShapeId, ShapeId>>{&poolResource[0]},
        std::pmr::vector<std::pair<ShapeId, ShapeId>>{&poolResource[1]},
        std::pmr::vector<std::pair<ShapeId, ShapeId>>{&poolResource[2]},
        std::pmr::vector<std::pair<ShapeId, ShapeId>>{&poolResource[3]},
        std::pmr::vector<std::pair<ShapeId, ShapeId>>{&poolResource[4]},
    } : std::array<std::pmr::vector<std::pair<ShapeId, ShapeId>>, 5>{
        std::pmr::vector<std::pair<ShapeId, ShapeId>>{&poolResource[0]},
        std::pmr::vector<std::pair<ShapeId, ShapeId>>{&poolResource[0]},
        std::pmr::vector<std::pair<ShapeId, ShapeId>>{&poolResource[0]},
        std::pmr::vector<std::pair<ShapeId, ShapeId>>{&poolResource[0]},
        std::pmr::vector<std::pair<ShapeId, ShapeId>>{&poolResource[0]},
    };

    const auto dynamicVsStatic = [&]()
    {
        treeFor(BodyType::Dynamic)
            .findAllCollisions(treeFor(BodyType::Static),
                               [&](ShapeId shapeAId, ShapeId shapeBId)
                               { collisions[0].emplace_back(shapeAId, shapeBId); });

        // Sort to detect duplicates -> duplicates will be adjacent in the sorted vector
        std::sort(collisions[0].begin(), collisions[0].end());
    };
    const auto dynamicVsDynamic = [&]()
    {
        treeFor(BodyType::Dynamic)
            .findAllCollisions([&](ShapeId shapeAId, ShapeId shapeBId)
                               { collisions[1].emplace_back(shapeAId, shapeBId); });

        // Sort to detect duplicates -> duplicates will be adjacent in the sorted vector
        std::sort(collisions[1].begin(), collisions[1].end());
    };
    const auto bulletVsStatic = [&]()
    {
        treeFor(BodyType::Bullet)
            .findAllCollisions(treeFor(BodyType::Static),
                               [&](ShapeId shapeAId, ShapeId shapeBId)
                               { collisions[2].emplace_back(shapeAId, shapeBId); });

        // Sort to detect duplicates -> duplicates will be adjacent in the sorted vector
        std::sort(collisions[2].begin(), collisions[2].end());
    };
    const auto bulletVsDynamic = [&]()
    {
        treeFor(BodyType::Bullet)
            .findAllCollisions(treeFor(BodyType::Dynamic),
                               [&](ShapeId shapeAId, ShapeId shapeBId)
                               { collisions[3].emplace_back(shapeAId, shapeBId); });

        // Sort to detect duplicates -> duplicates will be adjacent in the sorted vector
        std::sort(collisions[3].begin(), collisions[3].end());
    };
    const auto bulletVsBullet = [&]()
    {
        treeFor(BodyType::Bullet)
            .findAllCollisions([&](ShapeId shapeAId, ShapeId shapeBId)
                               { collisions[4].emplace_back(shapeAId, shapeBId); });

        // Sort to detect duplicates -> duplicates will be adjacent in the sorted vector
        std::sort(collisions[4].begin(), collisions[4].end());
    };

    if (runInParallel)
        C2D_RUN_PARALLEL_AND_WAIT(dynamicVsStatic, dynamicVsDynamic, bulletVsStatic, bulletVsDynamic, bulletVsBullet);
    else
        C2D_RUN_AND_WAIT(dynamicVsStatic, dynamicVsDynamic, bulletVsStatic, bulletVsDynamic, bulletVsBullet);

    for (size_t i = 0; i < 5; ++i)
    {
        if (!collisions[i].empty())
            narrowPhaseCollisions(collisions[i], collisionsInfo);
    }

    previousAllCollisionsCount = collisionsInfo.size();
    return collisionsInfo;
}

template <typename EntityId, PartitioningMethod Method>
std::vector<typename Registry<EntityId, Method>::EntityCollision> Registry<EntityId, Method>::getCollisions(
    EntityId id) const
{
    DEBUG_ASSERT(entities.find(id) != entities.end());
    const EntityInfo& entity = entities.at(id);

    std::vector<std::pair<AABB, BitMaskType>> queries;
    std::vector<ShapeId> shapesQueried;

    queries.reserve(entity.shapeIds.size());
    shapesQueried.reserve(entity.shapeIds.size());

    if (entity.type == BodyType::Bullet)
    {
        for (const auto shapeId : entity.shapeIds)
        {
            const auto& shape = shapes[shapeId];
            if (!shape.isActive)
                continue;

            const auto& previousTransform = bulletPreviousTransforms.at(id);
            const auto oldAABB =
                std::visit([&](const auto& shape) { return computeAABB(shape, previousTransform); }, shape.shape);

            queries.emplace_back(AABB::combine(oldAABB, shape.aabb), shape.maskBits);
            shapesQueried.push_back(shapeId);
        }
    }
    else
    {
        for (const auto shapeId : entity.shapeIds)
        {
            const auto& shape = shapes[shapeId];
            if (!shape.isActive)
                continue;

            queries.emplace_back(shape.aabb, shape.maskBits);
            shapesQueried.push_back(shapeId);
        }
    }

    std::vector<EntityCollision> collisionsInfo;
    const auto onCollision = [&](size_t i, std::vector<ShapeId>& collisions)
    { narrowPhaseCollisions(shapesQueried[i], collisions, collisionsInfo); };

    if (entity.type != BodyType::Static)
    {
        treeFor(BodyType::Static).batchQuery(queries, onCollision);
    }
    treeFor(BodyType::Dynamic).batchQuery(queries, onCollision);
    treeFor(BodyType::Bullet).batchQuery(queries, onCollision);

    return collisionsInfo;
}

template <typename EntityId, PartitioningMethod Method>
void Registry<EntityId, Method>::narrowPhaseCollision(ShapeId shapeAId,
                                                      ShapeId shapeBId,
                                                      std::vector<EntityCollision>& outCollisionsInfo) const
{
    const auto entityAId = shapeEntity[shapeAId];
    const auto entityBId = shapeEntity[shapeBId];
    const auto& entityA = entities.at(entityAId);
    const auto& entityB = entities.at(entityBId);
    const auto& entityAShape = shapes[shapeAId];
    const auto& entityBShape = shapes[shapeBId];

    // Skip self-collision
    if (entityAId == entityBId)
        return;

    if (!entityAShape.isActive || !entityBShape.isActive)
        return;

    std::optional<Manifold> manifold;

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
                    const auto sweepCollision = c2d::sweep(
                        queryShapeConcrete, previousTransform, currentTransform, otherShapeConcrete, entityB.transform);
                    return sweepCollision ? std::make_optional(sweepCollision->manifold) : std::nullopt;
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
                    const auto sweepCollision = c2d::sweep(queryShapeConcrete,
                                                           previousTransform,
                                                           currentTransform,
                                                           otherShapeConcrete,
                                                           otherPreviousTransform,
                                                           otherCurrentTransform);
                    return sweepCollision ? std::make_optional(sweepCollision->manifold) : std::nullopt;
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
                const auto sweepCollision = c2d::sweep(bulletShapeConcrete,
                                                       bulletPreviousTransform,
                                                       bulletCurrentTransform,
                                                       otherShapeConcrete,
                                                       otherEntity.transform);
                return sweepCollision ? std::make_optional(sweepCollision->manifold) : std::nullopt;
            },
            bulletShape.shape,
            otherShape.shape);
    }
    else
    {
        manifold = std::visit(
            [&](const auto& queryShapeConcrete, const auto& otherShapeConcrete)
            {
                const Manifold collision =
                    c2d::collide(queryShapeConcrete, entityA.transform, otherShapeConcrete, entityB.transform);
                return collision.isColliding() ? std::make_optional(collision) : std::nullopt;
            },
            entityAShape.shape,
            entityBShape.shape);
    }

    if (!manifold)
        return;

    DEBUG_ASSERT(manifold->isColliding());
    outCollisionsInfo.emplace_back(EntityCollision{entityAId, entityBId, shapeAId, shapeBId, std::move(*manifold)});
}

template <typename EntityId, PartitioningMethod Method>
template <typename Allocator>
void Registry<EntityId, Method>::narrowPhaseCollisions(std::vector<std::pair<ShapeId, ShapeId>, Allocator>& collisions,
                                                       std::vector<EntityCollision>& outCollisionsInfo) const
{
    // NOTE: collisions are already sorted when passed to this function

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
template <typename Allocator>
void Registry<EntityId, Method>::narrowPhaseCollisions(ShapeId shapeQueried,
                                                       std::vector<ShapeId, Allocator>& collisions,
                                                       std::vector<EntityCollision>& outCollisionsInfo) const
{
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
    DEBUG_ASSERT(entities.find(entityAId) != entities.end(), "Entity A does not exist");
    DEBUG_ASSERT(entities.find(entityBId) != entities.end(), "Entity B does not exist");

    const auto& entityA = entities.at(entityAId);
    const auto& entityB = entities.at(entityBId);

    if (entityA.type == BodyType::Static && entityB.type == BodyType::Static)
        return false;

    if (entityA.type == BodyType::Bullet && entityB.type == BodyType::Bullet)
    {
        const auto& entityAPreviousTransform = bulletPreviousTransforms.at(entityAId);
        const auto& entityBPreviousTransform = bulletPreviousTransforms.at(entityBId);
        const auto& entityACurrentTransform = entityA.transform;
        const auto& entityBCurrentTransform = entityB.transform;

        for (const auto shapeAId : entityA.shapeIds)
        {
            const auto& shapeA = shapes[shapeAId];
            if (!shapeA.isActive)
                continue;

            for (const auto shapeBId : entityB.shapeIds)
            {
                const auto& shapeB = shapes[shapeBId];
                if (!shapeB.isActive)
                    continue;

                const bool areColliding = std::visit(
                    [&](const auto& shapeAConcrete, const auto& shapeBConcrete)
                    {
                        return c2d::sweepAreColliding(shapeAConcrete,
                                                      entityAPreviousTransform,
                                                      entityACurrentTransform,
                                                      shapeBConcrete,
                                                      entityBPreviousTransform,
                                                      entityBCurrentTransform);
                    },
                    shapeA.shape,
                    shapeB.shape);

                if (areColliding)
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

        for (const auto shapeBulletId : bulletEntity.shapeIds)
        {
            const auto& shapeBullet = shapes[shapeBulletId];
            if (!shapeBullet.isActive)
                continue;

            for (const auto shapeOtherId : otherEntity.shapeIds)
            {
                const auto& shapeOther = shapes[shapeOtherId];
                if (!shapeOther.isActive)
                    continue;

                const bool areColliding = std::visit(
                    [&](const auto& shapeBulletConcrete, const auto& shapeOtherConcrete)
                    {
                        return c2d::sweepAreColliding(shapeBulletConcrete,
                                                      bulletPreviousTransform,
                                                      bulletCurrentTransform,
                                                      shapeOtherConcrete,
                                                      otherEntity.transform);
                    },
                    shapeBullet.shape,
                    shapeOther.shape);

                if (areColliding)
                    return true;
            }
        }

        return false;
    }

    for (const auto shapeAId : entityA.shapeIds)
    {
        const auto& shapeA = shapes[shapeAId];
        if (!shapeA.isActive)
            continue;

        for (const auto shapeBId : entityB.shapeIds)
        {
            const auto& shapeB = shapes[shapeBId];
            if (!shapeB.isActive)
                continue;

            const bool colliding =
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
    treeFor(BodyType::Static).piercingRaycast(ray, staticHits, maskBits);
    treeFor(BodyType::Dynamic).piercingRaycast(ray, dynamicHits, maskBits);
    treeFor(BodyType::Bullet).piercingRaycast(ray, bulletHits, maskBits);

    uint32_t toCheck = staticHits.size();
    for (const auto& hit : staticHits)
    {
        const auto shapeId = hit.id;
        const auto entityId = shapeEntity[shapeId];
        const auto& shape = shapes[shapeId];
        const auto& entity = entities.at(entityId);

        if (!shape.isActive)
            continue;

        const auto narrowHit = std::visit(
            [&](const auto& shapeConcrete) { return raycast(shapeConcrete, entity.transform, ray); }, shape.shape);

        if (narrowHit && narrowHit->first < bestEntryTime)
        {
            bestEntryTime = narrowHit->first;
            bestHit = RaycastHit<std::pair<EntityId, ShapeId>>::fromRay({entityId, shapeId}, ray, *narrowHit);
            toCheck = 5; // Check 5 other hits after finding the first one
        }

        if (--toCheck == 0)
            break;
    }

    toCheck = dynamicHits.size();
    for (const auto& hit : dynamicHits)
    {
        const auto shapeId = hit.id;
        const auto entityId = shapeEntity[shapeId];
        const auto& shape = shapes[shapeId];
        const auto& entity = entities.at(entityId);

        if (!shape.isActive)
            continue;

        const auto narrowHit = std::visit(
            [&](const auto& shapeConcrete) { return raycast(shapeConcrete, entity.transform, ray); }, shape.shape);

        if (narrowHit && narrowHit->first < bestEntryTime)
        {
            bestEntryTime = narrowHit->first;
            bestHit = RaycastHit<std::pair<EntityId, ShapeId>>::fromRay({entityId, shapeId}, ray, *narrowHit);
            toCheck = 5; // Check 5 other hits after finding the first one
        }

        if (--toCheck == 0)
            break;
    }

    toCheck = bulletHits.size();
    for (const auto& hit : bulletHits)
    {
        const auto shapeId = hit.id;
        const auto entityId = shapeEntity[shapeId];
        const auto& shape = shapes[shapeId];
        const auto& entity = entities.at(entityId);

        if (!shape.isActive)
            continue;

        const auto& previousTransform = bulletPreviousTransforms.at(entityId);
        const auto narrowHit = std::visit([&](const auto& shapeConcrete)
                                          { return sweep(shapeConcrete, previousTransform, entity.transform, ray); },
                                          shape.shape);

        if (narrowHit && narrowHit->first < bestEntryTime)
        {
            bestEntryTime = narrowHit->first;
            bestHit = RaycastHit<std::pair<EntityId, ShapeId>>::fromRay({entityId, shapeId}, ray, *narrowHit);
            toCheck = 5; // Check 5 other hits after finding the first one
        }

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
    treeFor(BodyType::Static).piercingRaycast(ray, staticHits, maskBits);
    treeFor(BodyType::Dynamic).piercingRaycast(ray, dynamicHits, maskBits);
    treeFor(BodyType::Bullet).piercingRaycast(ray, bulletHits, maskBits);

    std::optional<RaycastHit<std::pair<EntityId, ShapeId>>> bestHit;
    float bestEntryTime = std::numeric_limits<float>::max();

    uint32_t toCheck = staticHits.size();
    for (const auto& hit : staticHits)
    {
        const auto shapeId = hit.id;
        const auto entityId = shapeEntity[shapeId];
        const auto& shape = shapes[shapeId];
        const auto& entity = entities.at(entityId);

        if (!shape.isActive)
            continue;

        const auto narrowHit = std::visit(
            [&](const auto& shapeConcrete) { return raycast(shapeConcrete, entity.transform, ray); }, shape.shape);

        if (narrowHit && narrowHit->first < bestEntryTime)
        {
            bestEntryTime = narrowHit->first;
            bestHit = RaycastHit<std::pair<EntityId, ShapeId>>::fromRay({entityId, shapeId}, ray, *narrowHit);
            toCheck = 5; // Check 5 other hits after finding the first one
        }

        if (--toCheck == 0)
            break;
    }

    toCheck = dynamicHits.size();
    for (const auto& hit : dynamicHits)
    {
        const auto shapeId = hit.id;
        const auto entityId = shapeEntity[shapeId];
        const auto& shape = shapes[shapeId];
        const auto& entity = entities.at(entityId);

        if (!shape.isActive)
            continue;

        const auto narrowHit = std::visit(
            [&](const auto& shapeConcrete) { return raycast(shapeConcrete, entity.transform, ray); }, shape.shape);

        if (narrowHit && narrowHit->first < bestEntryTime)
        {
            bestEntryTime = narrowHit->first;
            bestHit = RaycastHit<std::pair<EntityId, ShapeId>>::fromRay({entityId, shapeId}, ray, *narrowHit);
            toCheck = 5; // Check 5 other hits after finding the first one
        }

        if (--toCheck == 0)
            break;
    }

    toCheck = bulletHits.size();
    for (const auto& hit : bulletHits)
    {
        const auto shapeId = hit.id;
        const auto entityId = shapeEntity[shapeId];
        const auto& shape = shapes[shapeId];
        const auto& entity = entities.at(entityId);

        if (!shape.isActive)
            continue;

        const auto& previousTransform = bulletPreviousTransforms.at(entityId);
        const auto narrowHit = std::visit([&](const auto& shapeConcrete)
                                          { return sweep(shapeConcrete, previousTransform, entity.transform, ray); },
                                          shape.shape);

        if (narrowHit && narrowHit->first < bestEntryTime)
        {
            bestEntryTime = narrowHit->first;
            bestHit = RaycastHit<std::pair<EntityId, ShapeId>>::fromRay({entityId, shapeId}, ray, *narrowHit);
            toCheck = 5; // Check 5 other hits after finding the first one
        }

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
    treeFor(BodyType::Static).piercingRaycast(ray, staticHits, maskBits);
    treeFor(BodyType::Dynamic).piercingRaycast(ray, dynamicHits, maskBits);
    treeFor(BodyType::Bullet).piercingRaycast(ray, bulletHits, maskBits);

    std::set<RaycastHit<std::pair<EntityId, ShapeId>>> hits;

    for (const auto& hit : staticHits)
    {
        const auto shapeId = hit.id;
        const auto entityId = shapeEntity[shapeId];
        const auto& shape = shapes[shapeId];
        const auto& entity = entities.at(entityId);

        if (!shape.isActive)
            continue;

        const auto narrowHit = std::visit(
            [&](const auto& shapeConcrete) { return raycast(shapeConcrete, entity.transform, ray); }, shape.shape);

        if (narrowHit)
            hits.insert(RaycastHit<std::pair<EntityId, ShapeId>>::fromRay({entityId, shapeId}, ray, *narrowHit));
    }

    for (const auto& hit : dynamicHits)
    {
        const auto shapeId = hit.id;
        const auto entityId = shapeEntity[shapeId];
        const auto& shape = shapes[shapeId];
        const auto& entity = entities.at(entityId);

        if (!shape.isActive)
            continue;

        const auto narrowHit = std::visit(
            [&](const auto& shapeConcrete) { return raycast(shapeConcrete, entity.transform, ray); }, shape.shape);

        if (narrowHit)
            hits.insert(RaycastHit<std::pair<EntityId, ShapeId>>::fromRay({entityId, shapeId}, ray, *narrowHit));
    }

    for (const auto& hit : bulletHits)
    {
        const auto shapeId = hit.id;
        const auto entityId = shapeEntity[shapeId];
        const auto& shape = shapes[shapeId];
        const auto& entity = entities.at(entityId);

        if (!shape.isActive)
            continue;

        const auto& previousTransform = bulletPreviousTransforms.at(entityId);
        const auto narrowHit = std::visit([&](const auto& shapeConcrete)
                                          { return sweep(shapeConcrete, previousTransform, entity.transform, ray); },
                                          shape.shape);

        if (narrowHit)
            hits.insert(RaycastHit<std::pair<EntityId, ShapeId>>::fromRay({entityId, shapeId}, ray, *narrowHit));
    }

    return hits;
}

template <typename EntityId, PartitioningMethod Method>
std::set<RaycastHit<std::pair<EntityId, ShapeId>>> Registry<EntityId, Method>::rayCast(InfiniteRay ray,
                                                                                       BitMaskType maskBits) const
{
    std::set<typename DynamicBVH<ShapeId>::RaycastInfo> staticHits, dynamicHits, bulletHits;
    treeFor(BodyType::Static).piercingRaycast(ray, staticHits, maskBits);
    treeFor(BodyType::Dynamic).piercingRaycast(ray, dynamicHits, maskBits);
    treeFor(BodyType::Bullet).piercingRaycast(ray, bulletHits, maskBits);

    std::set<RaycastHit<std::pair<EntityId, ShapeId>>> hits;

    for (const auto& hit : staticHits)
    {
        const auto shapeId = hit.id;
        const auto entityId = shapeEntity[shapeId];
        const auto& shape = shapes[shapeId];
        const auto& entity = entities.at(entityId);

        if (!shape.isActive)
            continue;

        const auto narrowHit = std::visit(
            [&](const auto& shapeConcrete) { return raycast(shapeConcrete, entity.transform, ray); }, shape.shape);

        if (narrowHit)
            hits.insert(RaycastHit<std::pair<EntityId, ShapeId>>::fromRay({entityId, shapeId}, ray, *narrowHit));
    }

    for (const auto& hit : dynamicHits)
    {
        const auto shapeId = hit.id;
        const auto entityId = shapeEntity[shapeId];
        const auto& shape = shapes[shapeId];
        const auto& entity = entities.at(entityId);

        if (!shape.isActive)
            continue;

        const auto narrowHit = std::visit(
            [&](const auto& shapeConcrete) { return raycast(shapeConcrete, entity.transform, ray); }, shape.shape);

        if (narrowHit)
            hits.insert(RaycastHit<std::pair<EntityId, ShapeId>>::fromRay({entityId, shapeId}, ray, *narrowHit));
    }

    for (const auto& hit : bulletHits)
    {
        const auto shapeId = hit.id;
        const auto entityId = shapeEntity[shapeId];
        const auto& shape = shapes[shapeId];
        const auto& entity = entities.at(entityId);

        if (!shape.isActive)
            continue;

        const auto& previousTransform = bulletPreviousTransforms.at(entityId);
        const auto narrowHit = std::visit([&](const auto& shapeConcrete)
                                          { return sweep(shapeConcrete, previousTransform, entity.transform, ray); },
                                          shape.shape);

        if (narrowHit)
            hits.insert(RaycastHit<std::pair<EntityId, ShapeId>>::fromRay({entityId, shapeId}, ray, *narrowHit));
    }

    return hits;
}

template <typename EntityId, PartitioningMethod Method>
size_t Registry<EntityId, Method>::size() const
{
    return entities.size();
}

template <typename EntityId, PartitioningMethod Method>
void Registry<EntityId, Method>::clear()
{
    entities.clear();
    shapeEntity.clear();
    shapes.clear();
    shapeTreeHandles.clear();
    freeShapeIds.clear();
    treeFor(BodyType::Static).clear();
    treeFor(BodyType::Dynamic).clear();
    treeFor(BodyType::Bullet).clear();
    bulletPreviousTransforms.clear();
    previousAllCollisionsCount = 1000;
}

template <typename EntityId, PartitioningMethod Method>
void Registry<EntityId, Method>::serialize(std::ostream& out) const
{
    Writer writer(out);

    // Write entities
    const size_t entitiesSize = entities.size();
    writer(entitiesSize);
    for (const auto& [entityId, entityInfo] : entities)
    {
        writer(entityId);
        entityInfo.serialize(out);
    }

    // Write shape to entity map
    const size_t shapeEntitySize = shapeEntity.size();
    writer(shapeEntitySize);
    for (const auto entityId : shapeEntity)
        writer(entityId);

    // Write shapes
    const size_t shapesSize = shapes.size();
    writer(shapesSize);
    for (const auto& shapeInfo : shapes)
        shapeInfo.serialize(out);

    // Write shape tree handles
    const size_t shapeTreeHandlesSize = shapeTreeHandles.size();
    writer(shapeTreeHandlesSize);
    for (const auto handle : shapeTreeHandles)
        writer(handle);

    // Write free shape IDs
    const size_t freeShapeIdsSize = freeShapeIds.size();
    writer(freeShapeIdsSize);
    for (const auto shapeId : freeShapeIds)
        writer(shapeId);

    // Write broad-phase trees
    treeFor(BodyType::Static).serialize(out);
    treeFor(BodyType::Dynamic).serialize(out);
    treeFor(BodyType::Bullet).serialize(out);

    // Write bullet previous transforms
    const size_t bulletPrevTransformsSize = bulletPreviousTransforms.size();
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
    for (auto& shapeEntity : registry.shapeEntity)
        reader(shapeEntity);

    // Read shapes
    size_t shapesSize;
    reader(shapesSize);

    registry.shapes.resize(shapesSize);
    for (auto& shape : registry.shapes)
        shape = ShapeInstance::deserialize(in);

    // Read shape tree handles
    size_t shapeTreeHandlesSize;
    reader(shapeTreeHandlesSize);

    registry.shapeTreeHandles.resize(shapeTreeHandlesSize);
    for (auto& handle : registry.shapeTreeHandles)
        reader(handle);

    // Read free shape IDs
    size_t freeShapeIdsSize;
    reader(freeShapeIdsSize);

    registry.freeShapeIds.resize(freeShapeIdsSize);
    for (auto& freeShapeId : registry.freeShapeIds)
        reader(freeShapeId);

    // Read broad-phase trees
    registry.treeFor(BodyType::Static) = TreeType::deserialize(in);
    registry.treeFor(BodyType::Dynamic) = TreeType::deserialize(in);
    registry.treeFor(BodyType::Bullet) = TreeType::deserialize(in);

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

#ifdef C2D_USE_CEREAL
template <typename EntityId, PartitioningMethod Method>
template <class Archive>
void Registry<EntityId, Method>::save(Archive& archive) const
{
    // Write entities
    const size_t entitiesSize = entities.size();
    archive(entitiesSize);
    for (const auto& [entityId, entityInfo] : entities)
    {
        archive(entityId);
        archive(entityInfo);
    }

    // Write shape to entity map
    archive(shapeEntity);

    // Write shapes
    archive(shapes);

    // Write shape tree handles
    archive(shapeTreeHandles);

    // Write free shape IDs
    archive(freeShapeIds);

    // Write broad-phase trees
    archive(treeFor(BodyType::Static));
    archive(treeFor(BodyType::Dynamic));
    archive(treeFor(BodyType::Bullet));

    // Write bullet previous transforms
    const size_t bulletPrevTransformsSize = bulletPreviousTransforms.size();
    archive(bulletPrevTransformsSize);
    for (const auto& [entityId, transform] : bulletPreviousTransforms)
    {
        archive(entityId);
        archive(transform);
    }

    archive(previousAllCollisionsCount);
}

template <typename EntityId, PartitioningMethod Method>
template <class Archive>
void Registry<EntityId, Method>::load(Archive& archive)
{
    // Read entities
    size_t entitiesSize;
    archive(entitiesSize);

    for (size_t i = 0; i < entitiesSize; i++)
    {
        EntityId entityId;
        archive(entityId);

        EntityInfo entityInfo;
        archive(entityInfo);

        entities.emplace(entityId, std::move(entityInfo));
    }

    // Read shape to entity map
    archive(shapeEntity);

    // Read shapes
    archive(shapes);

    // Read shape tree handles
    archive(shapeTreeHandles);

    // Read free shape IDs
    archive(freeShapeIds);

    // Read broad-phase trees
    archive(treeFor(BodyType::Static));
    archive(treeFor(BodyType::Dynamic));
    archive(treeFor(BodyType::Bullet));

    // Read bullet previous transforms
    size_t bulletPrevTransformsSize;
    archive(bulletPrevTransformsSize);

    for (size_t i = 0; i < bulletPrevTransformsSize; i++)
    {
        EntityId entityId;
        archive(entityId);

        Transform transform;
        archive(transform);

        bulletPreviousTransforms.emplace(entityId, std::move(transform));
    }

    archive(previousAllCollisionsCount);
}
#endif

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
    const size_t shapesSize = shapeIds.size();
    writer(shapesSize);
    for (const auto shapeId : shapeIds)
        writer(shapeId);
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

    entityInfo.shapeIds.resize(shapesSize);
    for (auto& shapeId : entityInfo.shapeIds)
        reader(shapeId);

    return entityInfo;
}

#ifdef C2D_USE_CEREAL
template <typename EntityId, PartitioningMethod Method>
template <class Archive>
void Registry<EntityId, Method>::EntityInfo::serialize(Archive& archive)
{
    archive(type, transform, shapeIds);
}
#endif

template <typename EntityId, PartitioningMethod Method>
void Registry<EntityId, Method>::ShapeInstance::serialize(std::ostream& out) const
{
    Writer writer(out);

    // Write shape variant
    uint8_t shapeType = static_cast<uint8_t>(shape.index());
    writer(shapeType);

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

    // Write AABB
    writer(aabb.min.x);
    writer(aabb.min.y);
    writer(aabb.max.x);
    writer(aabb.max.y);

    // Write mask bits
    writer(categoryBits);
    writer(maskBits);

    // Write function indices
    writer(moveFncIndices.first);
    writer(moveFncIndices.second);
    writer(translateFncIndex);

    // Write isActive
    writer(isActive);
}

template <typename EntityId, PartitioningMethod Method>
typename Registry<EntityId, Method>::ShapeInstance Registry<EntityId, Method>::ShapeInstance::deserialize(
    std::istream& in)
{
    Reader reader(in);
    ShapeInstance shapeInstance;

    // Read shape variant
    uint8_t shapeType;
    reader(shapeType);

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

    // Read AABB
    reader(shapeInstance.aabb.min.x);
    reader(shapeInstance.aabb.min.y);
    reader(shapeInstance.aabb.max.x);
    reader(shapeInstance.aabb.max.y);

    // Read mask bits
    reader(shapeInstance.categoryBits);
    reader(shapeInstance.maskBits);

    // Read function indices
    reader(shapeInstance.moveFncIndices.first);
    reader(shapeInstance.moveFncIndices.second);
    reader(shapeInstance.translateFncIndex);

    // Read isActive
    reader(shapeInstance.isActive);

    return shapeInstance;
}

#ifdef C2D_USE_CEREAL
template <typename EntityId, PartitioningMethod Method>
template <class Archive>
void Registry<EntityId, Method>::ShapeInstance::serialize(Archive& archive)
{
    archive(shape);
    archive(aabb);
    archive(categoryBits, maskBits);
    archive(moveFncIndices);
    archive(translateFncIndex, isActive);
}
#endif

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
    if (shapes != other.shapes)
        return false;
    if (shapeTreeHandles != other.shapeTreeHandles)
        return false;
    if (freeShapeIds != other.freeShapeIds)
        return false;

    if (treeFor(BodyType::Static) != other.treeFor(BodyType::Static))
        return false;
    if (treeFor(BodyType::Dynamic) != other.treeFor(BodyType::Dynamic))
        return false;
    if (treeFor(BodyType::Bullet) != other.treeFor(BodyType::Bullet))
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
    if (shapeIds != other.shapeIds)
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
    if (shape != other.shape)
        return false;
    if (aabb != other.aabb)
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

template <>
struct std::formatter<c2d::BodyType> : std::formatter<std::string>
{
    template <typename FormatContext>
    auto format(c2d::BodyType bodyType, FormatContext& ctx) const
    {
        switch (bodyType)
        {
        case c2d::BodyType::Static:
            return std::formatter<std::string>::format("Static", ctx);
        case c2d::BodyType::Dynamic:
            return std::formatter<std::string>::format("Dynamic", ctx);
        case c2d::BodyType::Bullet:
            return std::formatter<std::string>::format("Bullet", ctx);
        default:
            return std::formatter<std::string>::format("Unknown", ctx);
        }
    }
};

template <>
struct std::formatter<c2d::PartitioningMethod> : std::formatter<std::string>
{
    template <typename FormatContext>
    auto format(c2d::PartitioningMethod method, FormatContext& ctx) const
    {
        switch (method)
        {
        case c2d::PartitioningMethod::None:
            return std::formatter<std::string>::format("None", ctx);
        case c2d::PartitioningMethod::Grid:
            return std::formatter<std::string>::format("Grid", ctx);
        default:
            return std::formatter<std::string>::format("Unknown", ctx);
        }
    }
};
