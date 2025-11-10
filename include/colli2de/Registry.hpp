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
#include <colli2de/internal/utils/Serialization.hpp>

#include <tsl/robin_map.h>
#include <algorithm>
#include <variant>
#include <vector>

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
        : mTrees{
              BroadPhaseTree<ShapeId>(cellSize), BroadPhaseTree<ShapeId>(cellSize), BroadPhaseTree<ShapeId>(cellSize)}
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
    template <IsCerealArchive Archive>
    void save(Archive& archive) const;

    template <IsCerealArchive Archive>
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
        ShapeVariant mShape;
        AABB mBoundingBox;
        BitMaskType mCategoryBits = {1};
        BitMaskType mMaskBits = {~0ull};
        std::pair<uint8_t, uint8_t> mMoveFncIndices;
        uint8_t mTranslateFncIndex;
        bool mIsActive = true;

        void serialize(std::ostream& out) const;
        static ShapeInstance deserialize(std::istream& in);

#ifdef C2D_USE_CEREAL
        template <IsCerealArchive Archive>
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
        std::vector<ShapeId> mShapeIds;
        Transform mTransform;
        BodyType mType;

        void serialize(std::ostream& out) const;
        static EntityInfo deserialize(std::istream& in);

#ifdef C2D_USE_CEREAL
        template <IsCerealArchive Archive>
        void serialize(Archive& archive);
#endif

        bool operator==(const EntityInfo& other) const;

        bool operator!=(const EntityInfo& other) const
        {
            return !(*this == other);
        }
    };

    tsl::robin_map<EntityId, EntityInfo> mEntities;
    std::vector<EntityId> mShapeEntity;
    std::vector<ShapeInstance> mShapes;
    std::vector<BroadPhaseTreeHandle> mShapeTreeHandles;
    std::vector<ShapeId> mFreeShapeIds;

    using TreeType =
        std::conditional_t<Method == PartitioningMethod::Grid, BroadPhaseTree<ShapeId>, DynamicBVH<ShapeId>>;
    TreeType mTrees[3]; // 0: Static, 1: Dynamic, 2: Bullet

    tsl::robin_map<EntityId, Transform> mBulletPreviousTransforms;
    uint32_t mPreviousAllCollisionsCount = 1024;

    constexpr inline TreeType& treeFor(BodyType type)
    {
        return mTrees[static_cast<size_t>(type)];
    }

    constexpr inline const TreeType& treeFor(BodyType type) const
    {
        return mTrees[static_cast<size_t>(type)];
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
        const auto& concreteShape = std::get<Shape>(shapeInstance.mShape);
        shapeInstance.mBoundingBox = computeAABB(concreteShape, newTransform);
        tree.moveProxy(treeHandle, shapeInstance.mBoundingBox);
    }

    template <IsShape Shape>
    static void moveBulletShape(ShapeInstance& shapeInstance,
                                Transform newTransform,
                                TreeType& tree,
                                BroadPhaseTreeHandle treeHandle)
    {
        const auto& concreteShape = std::get<Shape>(shapeInstance.mShape);
        const auto oldAABB = shapeInstance.mBoundingBox;
        shapeInstance.mBoundingBox = computeAABB(concreteShape, newTransform);
        tree.moveProxy(treeHandle, AABB::combine(oldAABB, shapeInstance.mBoundingBox));
    }

    static void translateDynamicShape(ShapeInstance& shapeInstance,
                                      Translation translation,
                                      TreeType& tree,
                                      BroadPhaseTreeHandle treeHandle)
    {
        shapeInstance.mBoundingBox.translate(translation);
        tree.moveProxy(treeHandle, shapeInstance.mBoundingBox);
    }

    static void translateBulletShape(ShapeInstance& shapeInstance,
                                     Translation translation,
                                     TreeType& tree,
                                     BroadPhaseTreeHandle treeHandle)
    {
        const auto oldAABB = shapeInstance.mBoundingBox;
        shapeInstance.mBoundingBox.translate(translation);
        tree.moveProxy(treeHandle, AABB::combine(oldAABB, shapeInstance.mBoundingBox));
    }

    using MoveFunction = std::function<void(ShapeInstance&, Transform, TreeType&, BroadPhaseTreeHandle)>;
    using TranslateFunction = std::function<void(ShapeInstance&, Translation, TreeType&, BroadPhaseTreeHandle)>;

    inline static MoveFunction mMoveFnc[bodyTypesCount][shapeTypesCount] = {
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
    inline static TranslateFunction mTranslateFnc[bodyTypesCount] = {
        &Registry<EntityId, Method>::translateDynamicShape,
        &Registry<EntityId, Method>::translateDynamicShape,
        &Registry<EntityId, Method>::translateBulletShape,
    };
};

template <typename EntityId, PartitioningMethod Method>
void Registry<EntityId, Method>::createEntity(EntityId id, BodyType type, const Transform& transform)
{
    DEBUG_ASSERT(mEntities.find(id) == mEntities.end(), "Entity with this ID already exists");
    mEntities.emplace(id, EntityInfo{.mShapeIds = {}, .mTransform = transform, .mType = type});

    if (type == BodyType::Bullet)
        mBulletPreviousTransforms.emplace(id, transform);
}

template <typename EntityId, PartitioningMethod Method>
template <IsShape Shape>
ShapeId Registry<EntityId, Method>::addShape(EntityId entityId,
                                             const Shape& shape,
                                             uint64_t categoryBits,
                                             uint64_t maskBits)
{
    DEBUG_ASSERT(mEntities.find(entityId) != mEntities.end(), "Entity with this ID does not exist");
    EntityInfo& entity = mEntities.at(entityId);

    const bool reuseShapeId = !mFreeShapeIds.empty();
    ShapeId shapeId;
    if (reuseShapeId)
    {
        shapeId = mFreeShapeIds.back();
        mFreeShapeIds.pop_back();
    }
    else
    {
        shapeId = mShapes.size();
        DEBUG_ASSERT(shapeId == mShapeEntity.size());
        DEBUG_ASSERT(shapeId == mShapeTreeHandles.size());
    }

    auto& tree = treeFor(entity.mType);
    const auto shapeAABB = computeAABB(shape, entity.mTransform);
    const auto treeHandle = tree.addProxy(shapeId, shapeAABB, categoryBits, maskBits);

    if (reuseShapeId)
    {
        mShapeEntity[shapeId] = entityId;
        mShapes[shapeId] = ShapeInstance{
            .mShape = shape,
            .mBoundingBox = shapeAABB,
            .mCategoryBits = categoryBits,
            .mMaskBits = maskBits,
            .mMoveFncIndices = {static_cast<uint8_t>(entity.mType), static_cast<uint8_t>(shape.getType())},
            .mTranslateFncIndex = static_cast<uint8_t>(entity.mType),
        };
        mShapeTreeHandles[shapeId] = treeHandle;
        entity.mShapeIds.push_back(shapeId);
    }
    else
    {
        mShapeEntity.push_back(entityId);
        mShapes.emplace_back(ShapeInstance{
            .mShape = shape,
            .mBoundingBox = shapeAABB,
            .mCategoryBits = categoryBits,
            .mMaskBits = maskBits,
            .mMoveFncIndices = {static_cast<uint8_t>(entity.mType), static_cast<uint8_t>(shape.getType())},
            .mTranslateFncIndex = static_cast<uint8_t>(entity.mType),
        });
        mShapeTreeHandles.push_back(treeHandle);
        entity.mShapeIds.push_back(shapeId);
    }

    return shapeId;
}

template <typename EntityId, PartitioningMethod Method>
void Registry<EntityId, Method>::removeShape(ShapeId shapeId)
{
    DEBUG_ASSERT(shapeId < mShapes.size(), "Shape ID out of bounds");
    const EntityId entityId = mShapeEntity.at(shapeId);

    DEBUG_ASSERT(mEntities.find(entityId) != mEntities.end(), "Entity with this ID does not exist");
    EntityInfo& entity = mEntities.at(entityId);

    const auto treeHandle = mShapeTreeHandles.at(shapeId);
    auto& tree = treeFor(entity.mType);
    tree.removeProxy(treeHandle);

    mFreeShapeIds.push_back(shapeId);

    const auto shapeIt = std::find(entity.mShapeIds.begin(), entity.mShapeIds.end(), shapeId);
    DEBUG_ASSERT(shapeIt != entity.mShapeIds.end(), "Shape with this ID does not exist in the entity");
    entity.mShapeIds.erase(shapeIt);
}

template <typename EntityId, PartitioningMethod Method>
void Registry<EntityId, Method>::setShapeActive(ShapeId shapeId, bool isActive)
{
    DEBUG_ASSERT(shapeId < mShapes.size(), "Shape ID out of bounds");
    mShapes[shapeId].mIsActive = isActive;
}

template <typename EntityId, PartitioningMethod Method>
void Registry<EntityId, Method>::removeEntity(EntityId id)
{
    auto entityIt = mEntities.find(id);
    DEBUG_ASSERT(entityIt != mEntities.end(), "Entity with this ID does not exist");
    auto& entity = entityIt->second;

    auto& tree = treeFor(entity.mType);
    for (const auto shapeId : entity.mShapeIds)
    {
        DEBUG_ASSERT(shapeId < mShapes.size(), "Shape ID out of bounds");
        const auto treeHandle = mShapeTreeHandles[shapeId];
        tree.removeProxy(treeHandle);
        mFreeShapeIds.push_back(shapeId);
    }

    if (entity.mType == BodyType::Bullet)
        mBulletPreviousTransforms.erase(id);

    mEntities.erase(entityIt);
}

template <typename EntityId, PartitioningMethod Method>
void Registry<EntityId, Method>::teleportEntity(EntityId entityId, const Transform& transform)
{
    DEBUG_ASSERT(mEntities.find(entityId) != mEntities.end());
    EntityInfo& entity = mEntities.at(entityId);
    auto& tree = treeFor(entity.mType);

    for (const auto shapeId : entity.mShapeIds)
    {
        DEBUG_ASSERT(shapeId < mShapes.size(), "Shape ID out of bounds");
        const auto treeHandle = mShapeTreeHandles[shapeId];
        auto& shape = mShapes[shapeId];
        mMoveFnc[shape.mMoveFncIndices.first][shape.mMoveFncIndices.second](shape, transform, tree, treeHandle);
    }

    if (entity.mType == BodyType::Bullet)
        mBulletPreviousTransforms.at(entityId) = entity.mTransform;

    entity.mTransform = transform;
}

template <typename EntityId, PartitioningMethod Method>
void Registry<EntityId, Method>::teleportEntity(EntityId entityId, Translation translation)
{
    DEBUG_ASSERT(mEntities.find(entityId) != mEntities.end());
    const EntityInfo& entity = mEntities.at(entityId);

    const auto deltaTranslation = translation - entity.mTransform.translation;
    moveEntity(entityId, deltaTranslation);
}

template <typename EntityId, PartitioningMethod Method>
void Registry<EntityId, Method>::moveEntity(EntityId entityId, const Transform& delta)
{
    DEBUG_ASSERT(mEntities.find(entityId) != mEntities.end());
    auto& entity = mEntities.at(entityId);
    auto& tree = treeFor(entity.mType);

    if (entity.mType == BodyType::Bullet)
        mBulletPreviousTransforms.at(entityId) = entity.mTransform;

    entity.mTransform += delta;

    for (const auto shapeId : entity.mShapeIds)
    {
        DEBUG_ASSERT(shapeId < mShapes.size(), "Shape ID out of bounds");
        const auto treeHandle = mShapeTreeHandles[shapeId];
        auto& shape = mShapes[shapeId];
        mMoveFnc[shape.mMoveFncIndices.first][shape.mMoveFncIndices.second](shape, entity.mTransform, tree, treeHandle);
    }
}

template <typename EntityId, PartitioningMethod Method>
void Registry<EntityId, Method>::moveEntity(EntityId id, Translation deltaTranslation)
{
    DEBUG_ASSERT(mEntities.find(id) != mEntities.end());
    auto& entity = mEntities.at(id);
    auto& tree = treeFor(entity.mType);

    for (const auto shapeId : entity.mShapeIds)
    {
        DEBUG_ASSERT(shapeId < mShapes.size(), "Shape ID out of bounds");
        const auto treeHandle = mShapeTreeHandles[shapeId];
        auto& shape = mShapes[shapeId];
        mTranslateFnc[shape.mTranslateFncIndex](shape, deltaTranslation, tree, treeHandle);
    }

    if (entity.mType == BodyType::Bullet)
        mBulletPreviousTransforms.at(id) = entity.mTransform;

    entity.mTransform.translation += deltaTranslation;
}

template <typename EntityId, PartitioningMethod Method>
Transform Registry<EntityId, Method>::getEntityTransform(EntityId id) const
{
    DEBUG_ASSERT(mEntities.find(id) != mEntities.end());
    const EntityInfo& entity = mEntities.at(id);
    return entity.mTransform;
}

template <typename EntityId, PartitioningMethod Method>
std::vector<typename Registry<EntityId, Method>::EntityCollision> Registry<EntityId, Method>::getCollidingPairs()
{
    std::vector<EntityCollision> collisionsInfo;
    collisionsInfo.reserve(mPreviousAllCollisionsCount + 10);

    const bool runInParallel = mShapes.size() > 9'000;

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

    mPreviousAllCollisionsCount = collisionsInfo.size();
    return collisionsInfo;
}

template <typename EntityId, PartitioningMethod Method>
std::vector<typename Registry<EntityId, Method>::EntityCollision> Registry<EntityId, Method>::getCollisions(
    EntityId id) const
{
    DEBUG_ASSERT(mEntities.find(id) != mEntities.end());
    const EntityInfo& entity = mEntities.at(id);

    std::vector<std::pair<AABB, BitMaskType>> queries;
    std::vector<ShapeId> shapesQueried;

    queries.reserve(entity.mShapeIds.size());
    shapesQueried.reserve(entity.mShapeIds.size());

    if (entity.mType == BodyType::Bullet)
    {
        for (const auto shapeId : entity.mShapeIds)
        {
            const auto& shape = mShapes[shapeId];
            if (!shape.mIsActive)
                continue;

            const auto& previousTransform = mBulletPreviousTransforms.at(id);
            const auto oldAABB =
                std::visit([&](const auto& shape) { return computeAABB(shape, previousTransform); }, shape.mShape);

            queries.emplace_back(AABB::combine(oldAABB, shape.mBoundingBox), shape.mMaskBits);
            shapesQueried.push_back(shapeId);
        }
    }
    else
    {
        for (const auto shapeId : entity.mShapeIds)
        {
            const auto& shape = mShapes[shapeId];
            if (!shape.mIsActive)
                continue;

            queries.emplace_back(shape.mBoundingBox, shape.mMaskBits);
            shapesQueried.push_back(shapeId);
        }
    }

    std::vector<EntityCollision> collisionsInfo;
    const auto onCollision = [&](size_t i, std::vector<ShapeId>& collisions)
    { narrowPhaseCollisions(shapesQueried[i], collisions, collisionsInfo); };

    if (entity.mType != BodyType::Static)
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
    const auto entityAId = mShapeEntity[shapeAId];
    const auto entityBId = mShapeEntity[shapeBId];
    const auto& entityA = mEntities.at(entityAId);
    const auto& entityB = mEntities.at(entityBId);
    const auto& entityAShape = mShapes[shapeAId];
    const auto& entityBShape = mShapes[shapeBId];

    // Skip self-collision
    if (entityAId == entityBId)
        return;

    if (!entityAShape.mIsActive || !entityBShape.mIsActive)
        return;

    std::optional<Manifold> manifold;

    // Sweep for bullets
    if (entityA.mType == BodyType::Bullet && entityB.mType == BodyType::Bullet)
    {
        const auto& previousTransform = mBulletPreviousTransforms.at(entityAId);
        const auto& currentTransform = entityA.mTransform;

        if (entityB.mType != BodyType::Bullet)
        {
            manifold = std::visit(
                [&](const auto& queryShapeConcrete, const auto& otherShapeConcrete)
                {
                    const auto sweepCollision = c2d::sweep(queryShapeConcrete,
                                                           previousTransform,
                                                           currentTransform,
                                                           otherShapeConcrete,
                                                           entityB.mTransform);
                    return sweepCollision ? std::make_optional(sweepCollision->manifold) : std::nullopt;
                },
                entityAShape.mShape,
                entityBShape.mShape);
        }
        else
        {
            const auto& otherPreviousTransform = mBulletPreviousTransforms.at(entityBId);
            const auto& otherCurrentTransform = entityB.mTransform;

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
                entityAShape.mShape,
                entityBShape.mShape);
        }
    }
    else if (entityA.mType == BodyType::Bullet || entityB.mType == BodyType::Bullet)
    {
        const auto& bulletEntity = (entityA.mType == BodyType::Bullet) ? entityA : entityB;
        const auto& bulletEntityId = (entityA.mType == BodyType::Bullet) ? entityAId : entityBId;
        const auto& otherEntity = (entityA.mType == BodyType::Bullet) ? entityB : entityA;
        const auto& bulletPreviousTransform = mBulletPreviousTransforms.at(bulletEntityId);
        const auto& bulletCurrentTransform = bulletEntity.mTransform;
        const auto& bulletShape = (entityA.mType == BodyType::Bullet) ? entityAShape : entityBShape;
        const auto& otherShape = (entityA.mType == BodyType::Bullet) ? entityBShape : entityAShape;

        manifold = std::visit(
            [&](const auto& bulletShapeConcrete, const auto& otherShapeConcrete)
            {
                const auto sweepCollision = c2d::sweep(bulletShapeConcrete,
                                                       bulletPreviousTransform,
                                                       bulletCurrentTransform,
                                                       otherShapeConcrete,
                                                       otherEntity.mTransform);
                return sweepCollision ? std::make_optional(sweepCollision->manifold) : std::nullopt;
            },
            bulletShape.mShape,
            otherShape.mShape);
    }
    else
    {
        manifold = std::visit(
            [&](const auto& queryShapeConcrete, const auto& otherShapeConcrete)
            {
                const Manifold collision =
                    c2d::collide(queryShapeConcrete, entityA.mTransform, otherShapeConcrete, entityB.mTransform);
                return collision.isColliding() ? std::make_optional(collision) : std::nullopt;
            },
            entityAShape.mShape,
            entityBShape.mShape);
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
    DEBUG_ASSERT(mEntities.find(entityAId) != mEntities.end(), "Entity A does not exist");
    DEBUG_ASSERT(mEntities.find(entityBId) != mEntities.end(), "Entity B does not exist");

    const auto& entityA = mEntities.at(entityAId);
    const auto& entityB = mEntities.at(entityBId);

    if (entityA.mType == BodyType::Static && entityB.mType == BodyType::Static)
        return false;

    if (entityA.mType == BodyType::Bullet && entityB.mType == BodyType::Bullet)
    {
        const auto& entityAPreviousTransform = mBulletPreviousTransforms.at(entityAId);
        const auto& entityBPreviousTransform = mBulletPreviousTransforms.at(entityBId);
        const auto& entityACurrentTransform = entityA.mTransform;
        const auto& entityBCurrentTransform = entityB.mTransform;

        for (const auto shapeAId : entityA.mShapeIds)
        {
            const auto& shapeA = mShapes[shapeAId];
            if (!shapeA.mIsActive)
                continue;

            for (const auto shapeBId : entityB.mShapeIds)
            {
                const auto& shapeB = mShapes[shapeBId];
                if (!shapeB.mIsActive)
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
                    shapeA.mShape,
                    shapeB.mShape);

                if (areColliding)
                    return true;
            }
        }

        return false;
    }

    if (entityA.mType == BodyType::Bullet || entityB.mType == BodyType::Bullet)
    {
        const auto& bulletEntity = (entityA.mType == BodyType::Bullet) ? entityA : entityB;
        const auto& bulletEntityId = (entityA.mType == BodyType::Bullet) ? entityAId : entityBId;
        const auto& otherEntity = (entityA.mType == BodyType::Bullet) ? entityB : entityA;
        const auto& bulletPreviousTransform = mBulletPreviousTransforms.at(bulletEntityId);
        const auto& bulletCurrentTransform = bulletEntity.mTransform;

        for (const auto shapeBulletId : bulletEntity.mShapeIds)
        {
            const auto& shapeBullet = mShapes[shapeBulletId];
            if (!shapeBullet.mIsActive)
                continue;

            for (const auto shapeOtherId : otherEntity.mShapeIds)
            {
                const auto& shapeOther = mShapes[shapeOtherId];
                if (!shapeOther.mIsActive)
                    continue;

                const bool areColliding = std::visit(
                    [&](const auto& shapeBulletConcrete, const auto& shapeOtherConcrete)
                    {
                        return c2d::sweepAreColliding(shapeBulletConcrete,
                                                      bulletPreviousTransform,
                                                      bulletCurrentTransform,
                                                      shapeOtherConcrete,
                                                      otherEntity.mTransform);
                    },
                    shapeBullet.mShape,
                    shapeOther.mShape);

                if (areColliding)
                    return true;
            }
        }

        return false;
    }

    for (const auto shapeAId : entityA.mShapeIds)
    {
        const auto& shapeA = mShapes[shapeAId];
        if (!shapeA.mIsActive)
            continue;

        for (const auto shapeBId : entityB.mShapeIds)
        {
            const auto& shapeB = mShapes[shapeBId];
            if (!shapeB.mIsActive)
                continue;

            const bool colliding =
                std::visit([&](const auto& shapeA, const auto& shapeB)
                           { return c2d::areColliding(shapeA, entityA.mTransform, shapeB, entityB.mTransform); },
                           shapeA.mShape,
                           shapeB.mShape);

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
        const auto entityId = mShapeEntity[shapeId];
        const auto& shape = mShapes[shapeId];
        const auto& entity = mEntities.at(entityId);

        if (!shape.mIsActive)
            continue;

        const auto narrowHit = std::visit(
            [&](const auto& shapeConcrete) { return raycast(shapeConcrete, entity.mTransform, ray); }, shape.mShape);

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
        const auto entityId = mShapeEntity[shapeId];
        const auto& shape = mShapes[shapeId];
        const auto& entity = mEntities.at(entityId);

        if (!shape.mIsActive)
            continue;

        const auto narrowHit = std::visit(
            [&](const auto& shapeConcrete) { return raycast(shapeConcrete, entity.mTransform, ray); }, shape.mShape);

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
        const auto entityId = mShapeEntity[shapeId];
        const auto& shape = mShapes[shapeId];
        const auto& entity = mEntities.at(entityId);

        if (!shape.mIsActive)
            continue;

        const auto& previousTransform = mBulletPreviousTransforms.at(entityId);
        const auto narrowHit = std::visit([&](const auto& shapeConcrete)
                                          { return sweep(shapeConcrete, previousTransform, entity.mTransform, ray); },
                                          shape.mShape);

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
        const auto entityId = mShapeEntity[shapeId];
        const auto& shape = mShapes[shapeId];
        const auto& entity = mEntities.at(entityId);

        if (!shape.mIsActive)
            continue;

        const auto narrowHit = std::visit(
            [&](const auto& shapeConcrete) { return raycast(shapeConcrete, entity.mTransform, ray); }, shape.mShape);

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
        const auto entityId = mShapeEntity[shapeId];
        const auto& shape = mShapes[shapeId];
        const auto& entity = mEntities.at(entityId);

        if (!shape.mIsActive)
            continue;

        const auto narrowHit = std::visit(
            [&](const auto& shapeConcrete) { return raycast(shapeConcrete, entity.mTransform, ray); }, shape.mShape);

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
        const auto entityId = mShapeEntity[shapeId];
        const auto& shape = mShapes[shapeId];
        const auto& entity = mEntities.at(entityId);

        if (!shape.mIsActive)
            continue;

        const auto& previousTransform = mBulletPreviousTransforms.at(entityId);
        const auto narrowHit = std::visit([&](const auto& shapeConcrete)
                                          { return sweep(shapeConcrete, previousTransform, entity.mTransform, ray); },
                                          shape.mShape);

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
        const auto entityId = mShapeEntity[shapeId];
        const auto& shape = mShapes[shapeId];
        const auto& entity = mEntities.at(entityId);

        if (!shape.mIsActive)
            continue;

        const auto narrowHit = std::visit(
            [&](const auto& shapeConcrete) { return raycast(shapeConcrete, entity.mTransform, ray); }, shape.mShape);

        if (narrowHit)
            hits.insert(RaycastHit<std::pair<EntityId, ShapeId>>::fromRay({entityId, shapeId}, ray, *narrowHit));
    }

    for (const auto& hit : dynamicHits)
    {
        const auto shapeId = hit.id;
        const auto entityId = mShapeEntity[shapeId];
        const auto& shape = mShapes[shapeId];
        const auto& entity = mEntities.at(entityId);

        if (!shape.mIsActive)
            continue;

        const auto narrowHit = std::visit(
            [&](const auto& shapeConcrete) { return raycast(shapeConcrete, entity.mTransform, ray); }, shape.mShape);

        if (narrowHit)
            hits.insert(RaycastHit<std::pair<EntityId, ShapeId>>::fromRay({entityId, shapeId}, ray, *narrowHit));
    }

    for (const auto& hit : bulletHits)
    {
        const auto shapeId = hit.id;
        const auto entityId = mShapeEntity[shapeId];
        const auto& shape = mShapes[shapeId];
        const auto& entity = mEntities.at(entityId);

        if (!shape.mIsActive)
            continue;

        const auto& previousTransform = mBulletPreviousTransforms.at(entityId);
        const auto narrowHit = std::visit([&](const auto& shapeConcrete)
                                          { return sweep(shapeConcrete, previousTransform, entity.mTransform, ray); },
                                          shape.mShape);

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
        const auto entityId = mShapeEntity[shapeId];
        const auto& shape = mShapes[shapeId];
        const auto& entity = mEntities.at(entityId);

        if (!shape.mIsActive)
            continue;

        const auto narrowHit = std::visit(
            [&](const auto& shapeConcrete) { return raycast(shapeConcrete, entity.mTransform, ray); }, shape.mShape);

        if (narrowHit)
            hits.insert(RaycastHit<std::pair<EntityId, ShapeId>>::fromRay({entityId, shapeId}, ray, *narrowHit));
    }

    for (const auto& hit : dynamicHits)
    {
        const auto shapeId = hit.id;
        const auto entityId = mShapeEntity[shapeId];
        const auto& shape = mShapes[shapeId];
        const auto& entity = mEntities.at(entityId);

        if (!shape.mIsActive)
            continue;

        const auto narrowHit = std::visit(
            [&](const auto& shapeConcrete) { return raycast(shapeConcrete, entity.mTransform, ray); }, shape.mShape);

        if (narrowHit)
            hits.insert(RaycastHit<std::pair<EntityId, ShapeId>>::fromRay({entityId, shapeId}, ray, *narrowHit));
    }

    for (const auto& hit : bulletHits)
    {
        const auto shapeId = hit.id;
        const auto entityId = mShapeEntity[shapeId];
        const auto& shape = mShapes[shapeId];
        const auto& entity = mEntities.at(entityId);

        if (!shape.mIsActive)
            continue;

        const auto& previousTransform = mBulletPreviousTransforms.at(entityId);
        const auto narrowHit = std::visit([&](const auto& shapeConcrete)
                                          { return sweep(shapeConcrete, previousTransform, entity.mTransform, ray); },
                                          shape.mShape);

        if (narrowHit)
            hits.insert(RaycastHit<std::pair<EntityId, ShapeId>>::fromRay({entityId, shapeId}, ray, *narrowHit));
    }

    return hits;
}

template <typename EntityId, PartitioningMethod Method>
size_t Registry<EntityId, Method>::size() const
{
    return mEntities.size();
}

template <typename EntityId, PartitioningMethod Method>
void Registry<EntityId, Method>::clear()
{
    mEntities.clear();
    mShapeEntity.clear();
    mShapes.clear();
    mShapeTreeHandles.clear();
    mFreeShapeIds.clear();
    treeFor(BodyType::Static).clear();
    treeFor(BodyType::Dynamic).clear();
    treeFor(BodyType::Bullet).clear();
    mBulletPreviousTransforms.clear();
    mPreviousAllCollisionsCount = 1000;
}

template <typename EntityId, PartitioningMethod Method>
void Registry<EntityId, Method>::serialize(std::ostream& out) const
{
    Writer writer(out);

    // Write entities
    const size_t entitiesSize = mEntities.size();
    writer(entitiesSize);
    for (const auto& [entityId, entityInfo] : mEntities)
    {
        writer(entityId);
        entityInfo.serialize(out);
    }

    // Write shape to entity map
    const size_t shapeEntitySize = mShapeEntity.size();
    writer(shapeEntitySize);
    for (const auto entityId : mShapeEntity)
        writer(entityId);

    // Write shapes
    const size_t shapesSize = mShapes.size();
    writer(shapesSize);
    for (const auto& shapeInfo : mShapes)
        shapeInfo.serialize(out);

    // Write shape tree handles
    const size_t shapeTreeHandlesSize = mShapeTreeHandles.size();
    writer(shapeTreeHandlesSize);
    for (const auto handle : mShapeTreeHandles)
        writer(handle);

    // Write free shape IDs
    const size_t freeShapeIdsSize = mFreeShapeIds.size();
    writer(freeShapeIdsSize);
    for (const auto shapeId : mFreeShapeIds)
        writer(shapeId);

    // Write broad-phase trees
    treeFor(BodyType::Static).serialize(out);
    treeFor(BodyType::Dynamic).serialize(out);
    treeFor(BodyType::Bullet).serialize(out);

    // Write bullet previous transforms
    const size_t bulletPrevTransformsSize = mBulletPreviousTransforms.size();
    writer(bulletPrevTransformsSize);
    for (const auto& [entityId, transform] : mBulletPreviousTransforms)
    {
        writer(entityId);

        writer(transform.translation.x);
        writer(transform.translation.y);
        writer(transform.rotation.angleRadians);
        writer(transform.rotation.sin);
        writer(transform.rotation.cos);
    }

    writer(mPreviousAllCollisionsCount);
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

        registry.mEntities.emplace(entityId, std::move(entityInfo));
    }

    // Read shape to entity map
    size_t shapeEntitySize;
    reader(shapeEntitySize);

    registry.mShapeEntity.resize(shapeEntitySize);
    for (auto& shapeEntity : registry.mShapeEntity)
        reader(shapeEntity);

    // Read shapes
    size_t shapesSize;
    reader(shapesSize);

    registry.mShapes.resize(shapesSize);
    for (auto& shape : registry.mShapes)
        shape = ShapeInstance::deserialize(in);

    // Read shape tree handles
    size_t shapeTreeHandlesSize;
    reader(shapeTreeHandlesSize);

    registry.mShapeTreeHandles.resize(shapeTreeHandlesSize);
    for (auto& handle : registry.mShapeTreeHandles)
        reader(handle);

    // Read free shape IDs
    size_t freeShapeIdsSize;
    reader(freeShapeIdsSize);

    registry.mFreeShapeIds.resize(freeShapeIdsSize);
    for (auto& freeShapeId : registry.mFreeShapeIds)
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

        registry.mBulletPreviousTransforms.emplace(entityId, std::move(transform));
    }

    reader(registry.mPreviousAllCollisionsCount);

    return registry;
}

#ifdef C2D_USE_CEREAL
template <typename EntityId, PartitioningMethod Method>
template <IsCerealArchive Archive>
void Registry<EntityId, Method>::save(Archive& archive) const
{
    // Write entities
    const size_t entitiesSize = mEntities.size();
    archive(entitiesSize);
    for (const auto& [entityId, entityInfo] : mEntities)
    {
        archive(entityId);
        archive(entityInfo);
    }

    // Write shape to entity map
    archive(mShapeEntity);

    // Write shapes
    archive(mShapes);

    // Write shape tree handles
    archive(mShapeTreeHandles);

    // Write free shape IDs
    archive(mFreeShapeIds);

    // Write broad-phase trees
    archive(treeFor(BodyType::Static));
    archive(treeFor(BodyType::Dynamic));
    archive(treeFor(BodyType::Bullet));

    // Write bullet previous transforms
    const size_t bulletPrevTransformsSize = mBulletPreviousTransforms.size();
    archive(bulletPrevTransformsSize);
    for (const auto& [entityId, transform] : mBulletPreviousTransforms)
    {
        archive(entityId);
        archive(transform);
    }

    archive(mPreviousAllCollisionsCount);
}

template <typename EntityId, PartitioningMethod Method>
template <IsCerealArchive Archive>
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

        mEntities.emplace(entityId, std::move(entityInfo));
    }

    // Read shape to entity map
    archive(mShapeEntity);

    // Read shapes
    archive(mShapes);

    // Read shape tree handles
    archive(mShapeTreeHandles);

    // Read free shape IDs
    archive(mFreeShapeIds);

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

        mBulletPreviousTransforms.emplace(entityId, std::move(transform));
    }

    archive(mPreviousAllCollisionsCount);
}
#endif

template <typename EntityId, PartitioningMethod Method>
void Registry<EntityId, Method>::EntityInfo::serialize(std::ostream& out) const
{
    Writer writer(out);

    // Write body type
    writer(static_cast<uint8_t>(mType));

    // Write transform
    writer(mTransform.translation.x);
    writer(mTransform.translation.y);
    writer(mTransform.rotation.angleRadians);
    writer(mTransform.rotation.sin);
    writer(mTransform.rotation.cos);

    // Write shapes
    const size_t shapesSize = mShapeIds.size();
    writer(shapesSize);
    for (const auto shapeId : mShapeIds)
        writer(shapeId);
}

template <typename EntityId, PartitioningMethod Method>
Registry<EntityId, Method>::EntityInfo Registry<EntityId, Method>::EntityInfo::deserialize(std::istream& in)
{
    Reader reader(in);
    EntityInfo entityInfo;

    // Read body type
    reader(entityInfo.mType);

    // Read transform
    reader(entityInfo.mTransform.translation.x);
    reader(entityInfo.mTransform.translation.y);
    reader(entityInfo.mTransform.rotation.angleRadians);
    reader(entityInfo.mTransform.rotation.sin);
    reader(entityInfo.mTransform.rotation.cos);

    // Read shapes
    size_t shapesSize;
    reader(shapesSize);

    entityInfo.mShapeIds.resize(shapesSize);
    for (auto& shapeId : entityInfo.mShapeIds)
        reader(shapeId);

    return entityInfo;
}

#ifdef C2D_USE_CEREAL
template <typename EntityId, PartitioningMethod Method>
template <IsCerealArchive Archive>
void Registry<EntityId, Method>::EntityInfo::serialize(Archive& archive)
{
    archive(mType, mTransform, mShapeIds);
}
#endif

template <typename EntityId, PartitioningMethod Method>
void Registry<EntityId, Method>::ShapeInstance::serialize(std::ostream& out) const
{
    Writer writer(out);

    // Write shape variant
    uint8_t shapeType = static_cast<uint8_t>(mShape.index());
    writer(shapeType);

    // Write shape data
    if (std::holds_alternative<Circle>(mShape))
    {
        const Circle& circle = std::get<Circle>(mShape);
        writer(circle.center.x);
        writer(circle.center.y);
        writer(circle.radius);
    }
    else if (std::holds_alternative<Capsule>(mShape))
    {
        const Capsule& capsule = std::get<Capsule>(mShape);
        writer(capsule.center1.x);
        writer(capsule.center1.y);
        writer(capsule.center2.x);
        writer(capsule.center2.y);
        writer(capsule.radius);
    }
    else if (std::holds_alternative<Segment>(mShape))
    {
        const Segment& segment = std::get<Segment>(mShape);
        writer(segment.start.x);
        writer(segment.start.y);
        writer(segment.end.x);
        writer(segment.end.y);
    }
    else if (std::holds_alternative<Polygon>(mShape))
    {
        const Polygon& polygon = std::get<Polygon>(mShape);
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
    writer(mBoundingBox.min.x);
    writer(mBoundingBox.min.y);
    writer(mBoundingBox.max.x);
    writer(mBoundingBox.max.y);

    // Write mask bits
    writer(mCategoryBits);
    writer(mMaskBits);

    // Write function indices
    writer(mMoveFncIndices.first);
    writer(mMoveFncIndices.second);
    writer(mTranslateFncIndex);

    // Write isActive
    writer(mIsActive);
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
        shapeInstance.mShape = circle;
    }
    else if (shapeType == static_cast<uint8_t>(ShapeType::Capsule))
    {
        Capsule capsule;
        reader(capsule.center1.x);
        reader(capsule.center1.y);
        reader(capsule.center2.x);
        reader(capsule.center2.y);
        reader(capsule.radius);
        shapeInstance.mShape = capsule;
    }
    else if (shapeType == static_cast<uint8_t>(ShapeType::Segment))
    {
        Segment segment;
        reader(segment.start.x);
        reader(segment.start.y);
        reader(segment.end.x);
        reader(segment.end.y);
        shapeInstance.mShape = segment;
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

        shapeInstance.mShape = polygon;
    }

    // Read AABB
    reader(shapeInstance.mBoundingBox.min.x);
    reader(shapeInstance.mBoundingBox.min.y);
    reader(shapeInstance.mBoundingBox.max.x);
    reader(shapeInstance.mBoundingBox.max.y);

    // Read mask bits
    reader(shapeInstance.mCategoryBits);
    reader(shapeInstance.mMaskBits);

    // Read function indices
    reader(shapeInstance.mMoveFncIndices.first);
    reader(shapeInstance.mMoveFncIndices.second);
    reader(shapeInstance.mTranslateFncIndex);

    // Read isActive
    reader(shapeInstance.mIsActive);

    return shapeInstance;
}

#ifdef C2D_USE_CEREAL
template <typename EntityId, PartitioningMethod Method>
template <IsCerealArchive Archive>
void Registry<EntityId, Method>::ShapeInstance::serialize(Archive& archive)
{
    archive(mShape);
    archive(mBoundingBox);
    archive(mCategoryBits, mMaskBits);
    archive(mMoveFncIndices);
    archive(mTranslateFncIndex, mIsActive);
}
#endif

template <typename EntityId, PartitioningMethod Method>
bool Registry<EntityId, Method>::operator==(const Registry& other) const
{
    if (mEntities.size() != other.mEntities.size())
        return false;

    for (const auto& [id, entity] : mEntities)
    {
        auto it = other.mEntities.find(id);
        if (it == other.mEntities.end() || it->second != entity)
            return false;
    }

    if (mShapeEntity != other.mShapeEntity)
        return false;
    if (mShapes != other.mShapes)
        return false;
    if (mShapeTreeHandles != other.mShapeTreeHandles)
        return false;
    if (mFreeShapeIds != other.mFreeShapeIds)
        return false;

    if (treeFor(BodyType::Static) != other.treeFor(BodyType::Static))
        return false;
    if (treeFor(BodyType::Dynamic) != other.treeFor(BodyType::Dynamic))
        return false;
    if (treeFor(BodyType::Bullet) != other.treeFor(BodyType::Bullet))
        return false;

    if (mBulletPreviousTransforms.size() != other.mBulletPreviousTransforms.size())
        return false;

    for (const auto& [id, transform] : mBulletPreviousTransforms)
    {
        auto it = other.mBulletPreviousTransforms.find(id);
        if (it == other.mBulletPreviousTransforms.end() || it->second != transform)
            return false;
    }

    if (mPreviousAllCollisionsCount != other.mPreviousAllCollisionsCount)
        return false;

    return true;
}

template <typename EntityId, PartitioningMethod Method>
bool Registry<EntityId, Method>::EntityInfo::operator==(const EntityInfo& other) const
{
    if (mShapeIds != other.mShapeIds)
        return false;
    if (mTransform != other.mTransform)
        return false;
    if (mType != other.mType)
        return false;

    return true;
}

template <typename EntityId, PartitioningMethod Method>
bool Registry<EntityId, Method>::ShapeInstance::operator==(const ShapeInstance& other) const
{
    if (mShape != other.mShape)
        return false;
    if (mBoundingBox != other.mBoundingBox)
        return false;
    if (mCategoryBits != other.mCategoryBits)
        return false;
    if (mMaskBits != other.mMaskBits)
        return false;
    if (mIsActive != other.mIsActive)
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
