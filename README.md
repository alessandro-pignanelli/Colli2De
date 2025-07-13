# Colli2De

Colli2De is a modern C++23 library focused solely on 2D collision detection. It is designed to offer a clean and high-level API suitable for games or any application that requires reliable collision queries without the overhead of a full physics engine. The project was inspired by [Box2D](https://github.com/erincatto/box2d) and aims to provide similar accuracy and performance while presenting a simpler interface.


## Why Colli2De?

Most collision detection libraries are either part of larger physics engines or are too low-level.
You might want to create your own physics engine, especially for simple games like platformers, and avoid dealing with the intricacies of a full physics simulation, but you'll still need a way to detect collisions between your entities without having to dive deep into low-level details of how collision detection works.  
Colli2De fills this gap by providing a lightweight, easy-to-use efficient library that focuses on collision detection without the complexities of a full physics simulation.


## Features

- A `Registry` class that manages entities and their shapes, allowing for easy creation, removal and transformation of entities.
- Support for different body types: static, dynamic, and bullet bodies. [See more details](#body-types).
- Collision detection for various shapes: circles, capsules, line segments, and convex polygons.
- Easy-to-use collision queries that return pairs of colliding entities or check if two entities are colliding.
- Easy-to-use ray casting for both finite and infinite rays, with support for ray casting queries that return hits.


## Building

Colli2De uses CMake (version 3.22 or later). The repository provides helper scripts for common tasks.

### Windows

```batch
REM configure and build in Debug mode
build.bat --cmake

REM build in Release mode
build.bat --cmake --release
```

### MacOS / Linux

```bash
# configure and build in Debug mode
./build.sh --cmake

# build in Release mode
./build.sh --cmake --release
```

The library can also be added to an existing CMake project using `add_subdirectory` and linking to the target `colli2de::colli2de`.

### Running the Tests

Tests are written with [Catch2](https://github.com/catchorg/Catch2) and can be built and executed with:

```batch
REM configure and build tests in Debug mode
test.bat --cmake --build --debug

REM build tests in Release mode to execute performance tests
test.bat --cmake --build --release
```

```bash
# configure and build tests in Debug mode
./test.sh --cmake --build --debug

# build tests in Release mode to execute performance tests
./test.sh --cmake --build --release
```

You can pass additional arguments to the test scripts to filter tests or run specific ones. Refer to the Catch2 documentation for more details on available command-line options.

## Quick Start

```cpp
#include <colli2de/Registry.hpp>
#include <print>

using namespace c2d;

int main()
{
    // Create a registry for managing entities with `uint32_t` as the entity ID type
    Registry<uint32_t> registry;

    // Static entity, ID 1, positioned at (0.0f, 0.0f), no rotation
    // Circle shape with center at (0, 0) and radius of 1.0f
    registry.createEntity(1, BodyType::Static, Transform(Vec2{0.0f, 0.0f}));
    const auto shapeId1 = registry.addShape(1, Circle{Vec2{0.0f, 0.0f}, 1.0f});

    // Dynamic entity, ID 2, positioned at (3.0f, 0.0f) with a rotation of PI/4 radians
    // Regular pentagon, centered at (0, 0), with a radius of 2.0f, and rotated by PI/3 radians
    registry.createEntity(2, BodyType::Dynamic, Transform(Vec2{3.0f, 0.0f}, Rotation{PI / 4}));
    const auto polygonShape = makeRegularPolygon(5, Vec2{0.0f, 0.0f}, 2.0f, PI / 3);
    const auto shapeId2 = registry.addShape(2, std::move(polygonShape));

    // Move the dynamic entity closer
    registry.moveEntity(2, Vec2{-2.0f, 0.0f});

    // Check for collisions
    if (registry.areColliding(1, 2))
        std::println("Entities 1 and 2 are colliding");
    else
        std::println("Entities 1 and 2 are not colliding");

    // Get all colliding pairs
    const auto pairs = registry.getCollidingPairs();
    for (const auto& collision : pairs)
    {
        std::println("Colliding (EntityId={}, ShapeId={}) with (EntityId={}, ShapeId={})",
                     collision.entityA, collision.shapeA,
                     collision.entityB, collision.shapeB);
    }

    // Ray cast
    Ray ray{Vec2{-2.0f, 0.0f}, Vec2{2.0f, 0.0f}};
    const auto hits = registry.rayCast(ray);
    for (const auto& hit : hits)
    {
        const auto [entityId, shapeId] = hit.id;
        std::println("Hit entityId={}, shapeId={}", entityId, shapeId);
    }
}
```


## Public API Overview

### Body Types

The library supports three body types, accessible via the `BodyType` enum in the `c2d` namespace:
- **Static**: Used for entities that do not change position or rotation after creation. They can't collide with other static bodies, but can collide with dynamic and bullet bodies.
- **Dynamic**: Used for entities that can move and rotate. They can collide with static, dynamic, and bullet bodies. For this type of body, collisions are computed at the final position of the entity, after all transformations have been applied. This means that collisions might not be accurate for fast moving bodies, and some collisions might be missed.
- **Bullet**: Similar to dynamic bodies, but optimized for fast moving entities. They can collide with static, dynamic, and other bullet bodies. Bullet bodies are checked for collisions at different positions along their movement path, which helps to prevent missing collisions due to high speeds. Collisions' manifolds are relative to the first detected position in which the body collides with another body, leading to more accurate collision responses. This means that handling bullet bodies is more expensive, so they should be used only when necessary, such as for fast moving projectiles.

### Registry Class

The `Registry` class is the core of Colli2De, managing entities and their shapes for fast collision detection. The class has been designed to be used in a entity-component-system (ECS) style, i.e., entities are identified by a unique ID of the given template type.  
An entity is a combination of an ID, a BodyType, a Transform (position and rotation) and a list of shapes. The registry allows you to create, remove, and manipulate entities and their shapes.  
An entity can be composed of multiple shapes, whose position and rotation are relative to the entity's Transform, and which cannot be moved independently.  
Beware that the registry **is not designed as thread-safe**, so you should not modify it from multiple threads at the same time. You can still query the registry from multiple threads, but you should ensure that no modifications are made while querying it.  
The registry's methods are designed to assume that the user won't make any invalid calls, such as trying to create an entity with an ID that already exists or trying to remove a shape that doesn't exist. If you need to handle such cases, you should check for the existence of the entity or shape before calling the methods, as they will result in failed assertions in debug mode and undefined behavior in release mode.

#### Creating the registry

`Registry<EntityIdType>::Registry()`: creates a new registry with the default cell size of 64.
`Registry<EntityIdType>::Registry(cellSize)`: creates a new registry with the specified cell size.

The `Registry` class is a templated class that takes a single template parameter, `EntityIdType`, which is the type of the entity ID. This allows for flexibility in choosing the type of entity ID, such as `int` or more complex types like `std::string` or `entt::entity` (from the [EnTT](https://github.com/skypjack/entt) library).  
Internally, the registry uses a spatial partitioning technique to efficiently manage collisions and ray casting queries. The cell size can be adjusted to optimize performance for your specific use case, with a default value of 64.  
The `Registry` class is not unit-agnostic, meaning you can use any unit of measurement for positions and sizes of entities and shapes. The `cellSize` parameter can then be adjusted to match the scale of your game world or application. The optimal cell size is balanced based on the scale of your entities and the density of collisions in your scene. A larger cell size leads to a faster entity movement and reduced partition overhead due to fewer cells, but slower collision detection in dense scenes due to larger internal trees.

#### Creating and removing entities

`void createEntity(id, BodyType, Transform)`:
- `id`: unique identifier for the entity.
- `BodyType`: describes how the entity responds to collisions.
- `Transform`: specifies the position and rotation of the entity.  

This method creates a new entity in the registry without any shapes. Until you add shapes to the entity, it will not participate in collision detection. Make sure that the specified entity ID does not already exist in the registry before calling this method.

`void removeEntity(id)`: 
- `id`: the unique identifier for the entity you want to remove.  

This method removes the entity from the registry, along with all its associated shapes. Make sure that the specified entity ID exists in the registry before calling this method.

```cpp
using EntityIdType = uint32_t;
c2d::Registry<EntityIdType> registry;

// Create a static entity with ID 1 at position (0, 0)
registry.createEntity(1, c2d::BodyType::Static, c2d::Transform(c2d::Vec2{0.0f, 0.0f}, c2d::Rotation{0.0f}));

// Remove the entity with ID 1
registry.removeEntity(1);
```

#### Handling shapes

`ShapeId addShape(id, shape, categoryBits, maskBits = ~0ull)`:
- `id`: unique identifier of the entity to which you want to add the shape.
- `shape`: the shape to be added, which can be a `Circle`, `Capsule`, `Segment`, or `Polygon`.
           [See more](#shapes) on how to create shapes.
- `categoryBits`: a bitmask that defines the category of the shape, used for collision filtering.
- `maskBits`: a bitmask that defines which categories this shape can collide with.
- -> `ShapeId`: a unique identifier for the shape within the entity.  

This method adds a shape to the specified entity and returns a `ShapeId`, which is a unique identifier for the shape within the entity. Make sure that the specified entity ID exists in the registry before calling this method.
The shape's position and rotation are relative to the entity's Transform. This means that when you move or rotate the entity, all its shapes will move and rotate accordingly.
The returned `ShapeId` will be referenced in the collision manifolds and ray casting results, allowing you to identify which shape was involved in a collision or hit. You can also use it to remove the shape later.

Read more about collision filtering in the [Collision Filtering](#collision-filtering) section.

`void removeShape(shapeId)`:
- `shapeId`: unique identifier of the shape you want to remove.  

This method removes the specified shape from the entity. Make sure that the specified `shapeId` exists in the registry before calling this method.
Entities are not designed to have their shapes added and removed frequently, so this method is not intended to be used in a tight loop. Instead, it is recommended to create entities with all their shapes at once and then manipulate the entities as a whole, and set shapes as inactive if you want to temporarily disable them without removing them from the entity.

`void setShapeActive(shapeId, isActive = true)`:
- `shapeId`: unique identifier of the shape you want to activate or deactivate.
- `isActive`: a boolean value indicating whether the shape should be active or not.  

This method allows you to activate or deactivate a shape within an entity without removing it. When a shape is inactive, it will not participate in collision detection or ray casting queries, but it will still be part of the entity and can be reactivated later. This is useful for temporarily disabling shapes without losing their data. When a shape is deactivated, it will stay inactive until explicitly reactivated.

```cpp
// Add a circle shape to the entity with ID 1
const c2d::ShapeId shapeId = registry.addShape(1, c2d::Circle{c2d::Vec2{0.0f, 0.0f}, 1.0f}, 0x0001, 0xFFFF);

// Set the shape as inactive
registry.setShapeActive(shapeId, false);

// Remove the shape with the given shapeId from the entity
registry.removeShape(shapeId);
```

#### Transforming entities

Since Colli2De is not a physics engine, it does not handle forces or velocities. Instead, it provides methods to directly manipulate the position and rotation of entities, either by teleporting them to a new position or moving them by a given offset.  
For bullet bodies, the registry stores the last position of the entity before the update, so that collisions can be computed at different positions along the movement path. This means that bullets should be moved only once per frame to ensure accurate collision detection, but it's best to apply this also to dynamic entities for performance reasons: compute the final position of the entity after all transformations have been applied, and then call one of the following methods to update the entity's position and rotation.  
Note that it's possible to move static entities as well, but this is not recommended since they are not designed to change position or rotation after creation. Still, it's more efficient to just move the static entity rather than removing it and creating a new one at the new position, this is why Colli2De allows it.

`void teleportEntity(id, transform)`:
- `id`: unique identifier of the entity you want to teleport.
- `transform`: a `Transform` object representing the new position and rotation of the entity.  

This method replaces the entity's current Transform with the new one. Make sure that the specified entity ID exists in the registry before calling this method.

`void teleportEntity(id, translation)`:
- `id`: unique identifier of the entity you want to teleport.
- `translation`: a `Translation` object describing the new position of the entity, without changing its rotation.  

This method updates the entity's position to the specified translation, keeping its current rotation. It's a faster alternative to the previous overload if you only need to change the position of the entity without changing its rotation.

`void moveEntity(id, deltaTransform)`:
- `id`: unique identifier of the entity you want to move.
- `deltaTransform`: a `Transform` object describing the delta Transform to apply to the entity, relative to its current position and rotation.  

This method updates the entity's Transform by applying the specified delta transform. Make sure that the specified entity ID exists in the registry before calling this method.

`void moveEntity(id, deltaTranslation)`:
- `id`: unique identifier of the entity you want to move.
- `deltaTranslation`: a `Translation` object describing the delta translation to apply to the entity, relative to its current position, without changing its rotation.  

This method updates the entity's position by adding the specified translation to its current position, keeping its current rotation. It's a faster alternative to the previous overload if you only need to change the position of the entity without changing its rotation.  
  
The above four methods are completely interchangeable, meaning you can use any of them to achieve the same result. The choice of which one to use depends on your specific use case and performance considerations.

```cpp
// Teleport entity with ID 1 to a new position (2.0f, 3.0f) with no rotation
registry.teleportEntity(1, c2d::Transform(c2d::Translation{2.0f, 3.0f}, 0.0f));
registry.teleportEntity(1, c2d::Translation{2.0f, 3.0f}); // Same as above, but faster

// Move entity with ID 1 by an offset of (1.0f, -1.0f)
registry.moveEntity(1, c2d::Transform(c2d::Translation{1.0f, -1.0f}, 0.0f));
registry.moveEntity(1, c2d::Translation{1.0f, -1.0f}); // Same as above, but faster
```

#### Collision queries

The registry provides several methods to query collisions between entities and shapes. They are designed to return a list of objects describing the colliding pairs of entities and their shapes, and there is no support for collision callbacks. This is because a callback might access the registry and modify it while the registry is iterating over the colliding pairs, leading to undefined behavior. Instead, you should use the returned list of collisions to handle them in your game logic. If you need to, wrapping the collision queries methods to calling listeners and callbacks on the returned pairs should be pretty straightforward.

`std::vector<EntityCollision> getCollidingPairs()`:
- -> `std::vector<EntityCollision>`: list of objects representing all currently colliding entity pairs in the registry.  

This method returns a list of all pairs of entities that are currently colliding, taking into account their positions, shapes, BodyType and shapes' active state. Each pair is represented by an [`EntityCollision`](#entitycollision) object, describing the two colliding entities and their shapes.  
Note that:
- Each pair is returned only once, i.e., if shape A collides with shape B, either (A, B) or (B, A) will be returned, but not both.
- The same pair of entities may appear multiple times in the list if they have multiple shapes that are colliding.
- Two shapes belonging to the same entity cannot collide since they cannot be moved independently from each other.  
- The pairs are returned in this order:
  - Dynamic vs Static entities.
  - Dynamic vs Dynamic entities.
  - Bullet vs Static entities.
  - Bullet vs Dynamic entities.  

This method performs queries in parallel on different threads to efficiently compute the collisions.

`std::vector<EntityCollision> getCollisions(id)`:
- `id`: unique identifier of the entity for which you want to get the collisions.
- -> `std::vector<EntityCollision>`: list of objects representing all currently colliding pairs for the specified entity.  

This method returns a list of all pairs of entities that are currently colliding with the specified entity, taking into account its position, shapes, BodyType and shapes' active state. Each pair is represented by an [`EntityCollision`](#entitycollision) object, describing the two colliding entities and their shapes.  
Note that:
- Each pair is returned only once, i.e., if shape A collides with shape B, either (A, B) or (B, A) will be returned, but not both.
- The same entity may appear multiple times in the list if it has multiple shapes that are colliding with the specified entity.
- Two shapes belonging to the same entity cannot collide since they cannot be moved independently from each other.   
- The pairs are returned in this order:
  - Static entities colliding with the specified entity.
  - Dynamic entities colliding with the specified entity.
  - Bullet entities colliding with the specified entity.  

`bool areColliding(idA, idB)`:
- `idA`: unique identifier of the first entity.
- `idB`: unique identifier of the second entity.
- -> `bool`: true if the entities are colliding, false otherwise.

This method checks if the two specified entities are currently colliding, taking into account their positions, shapes, BodyType and shapes' active state. It returns true if the entities are colliding, false otherwise.

```cpp
// Get all colliding pairs in the registry
const std::vector<c2d::Registry<int>::EntityCollision> pairs = registry.getCollidingPairs();
for (const auto& collision : pairs)
{
    std::println("Colliding (EntityId={}, ShapeId={}) with (EntityId={}, ShapeId={})",
                 collision.entityA, collision.shapeA,
                 collision.entityB, collision.shapeB);
}

// Get all colliding pairs for the entity with ID 1
const std::vector<c2d::Registry<int>::EntityCollision> collisions = registry.getCollisions(1);
for (const auto& collision : collisions)
    std::println("Entity 1 colliding with (EntityId={}, ShapeId={})", collision.entityB, collision.shapeB);

// Check if entities with IDs 1 and 2 are colliding
if (registry.areColliding(1, 2))
    std::println("Entities 1 and 2 are colliding");
else
    std::println("Entities 1 and 2 are not colliding");
```

#### Ray casting

`std::set<RaycastHit> rayCast(ray, maskBits = ~0ull)`:
- `ray`: an object representing the ray to cast.
- `maskBits`: a bitmask used to filter the shapes that can be hit by the ray.
- -> `std::set<RaycastHit>`: set of objects representing all shapes hit by the ray.  

This method takes either a finite [`Ray`](#ray) object or an [`InfiniteRay`](#infiniteray) object and performs a ray casting query, returning a set of all shapes hit by the ray. Each hit is represented by a [`RaycastHit`](#raycasthit) object, describing the hit entities and shapes. The ray is cast against all shapes in the registry that match the specified mask bits. Since the hits are returned in the order they were hit by the ray, `std::set` is used as return type for automatic sorting of the hits.

`RaycastHit firstHitRayCast(ray, maskBits = ~0ull)`:
- `ray`: an object representing the ray to cast.
- `maskBits`: a bitmask used to filter the shapes that can be hit by the ray.
- -> `std::optional<RaycastHit>`: the first hit shape by the ray or an empty optional if no shapes were hit.  

This method takes either a finite [`Ray`](#ray) object or an [`InfiniteRay`](#infiniteray) object and performs a ray casting query, returning the first hit shape by the ray, if any. The hit is represented by a [`RaycastHit`](#raycasthit) object, describing the first hit entity and shape. The ray is cast against all shapes in the registry that match the specified mask bits until the first hit is found. If no shapes were hit, an empty optional is returned.

```cpp
// Cast a ray from (0, 0) to (1, 1) against all shapes that can be hit by the ray
c2d::Ray ray{
    .start = c2d::Vec2{0.0f, 0.0f},
    .end = c2d::Vec2{1.0f, 1.0f}
};

// Perform a ray casting query to get all shapes hit by the ray
const auto hits = registry.rayCast(ray);
for (const auto& hit : hits)
{
    const auto [entityId, shapeId] = hit.id;
    std::println("Hit entityId={}, shapeId={}", entityId, shapeId);
    std::println("HitPoint=({}, {})", hit.entry.x, hit.entry.y);
    std::println("ExitPoint=({}, {})", hit.exit.x, hit.exit.y);
}

// Cast an infinite ray from (0, 0) along the negative Y axis
c2d::InfiniteRay infiniteRay{
    .start = c2d::Vec2{0.0f, 0.0f},
    .direction = c2d::Vec2{0.0f, -1.0f}
};

// Perform a ray casting query to get the first shape hit by the ray
const auto firstHit = registry.firstHitRayCast(infiniteRay);
if (firstHit)
{
    const auto [entityId, shapeId] = firstHit->id;
    std::println("First hit entityId={}, shapeId={}", entityId, shapeId);
    std::println("HitPoint=({}, {})", firstHit->entry.x, firstHit->entry.y);
    std::println("ExitPoint=({}, {})", firstHit->exit.x, firstHit->exit.y);
}
else
{
    std::println("No hit");
}
```

#### Utility methods

`size_t size()`: returns the number of entities in the registry.  

`void clear()`: removes all entities and shapes from the registry.  


### Shapes

Shapes are the fundamental geometric objects used in Colli2De for collision detection. They can be added to entities and are used to compute collisions and ray casting queries. The library provides several shape types, each with its own characteristics.

#### Circle

A circle is defined by its center and radius. It is represented by the `Circle` struct in the `c2d` namespace.

```cpp
// Circle centered at (0, 0) with radius 1.0f
c2d::Circle circle{c2d::Vec2{0.0f, 0.0f}, 1.0f};
```

#### Capsule

A capsule is defined by its two endpoints and radius. It can be thought as a combination of a rectangle and two semicircles. It is represented by the `Capsule` struct in the `c2d` namespace.

```cpp
// Capsule with endpoints at (0, 0) and (1, 0) with radius 0.5f
c2d::Capsule capsule{c2d::Vec2{0.0f, 0.0f}, c2d::Vec2{1.0f, 0.0f}, 0.5f};
```

#### Segment

A segment is defined by its two endpoints. It is represented by the `Segment` struct in the `c2d` namespace.

```cpp
// Segment with endpoints at (0, 0) and (1, 0)
c2d::Segment segment{c2d::Vec2{0.0f, 0.0f}, c2d::Vec2{1.0f, 0.0f}};
```

#### Polygon

A polygon is defined by its vertices. Colli2De supports only convex polygons with a maximum of 8 vertices. It is represented by the `Polygon` struct in the `c2d` namespace.  

The library provides several utility functions to create polygons:
- `makeRegularPolygon(n, center, radius, rotation = 0.0f)`: creates a regular polygon with `n` vertices, centered at the specified position, with the given radius and rotation in radians.
- `makePolygon(vertices)`: creates a polygon from a list of vertices. The vertices must be provided in counter-clockwise order, and the polygon must be convex. The maximum number of vertices is 8.
- `makeTriangle(v0, v1, v2)`: creates a triangle from three vertices.
- `makeRectangle(center, halfWidth, halfHeight, rotation = 0.0f)`: creates a rectangle with the specified center, half-width, half-height and rotation in radians.

```cpp
// Create a regular pentagon centered at (0, 0) with radius 2.0f and rotated by PI/3 radians
c2d::Polygon polygon = c2d::makeRegularPolygon(5, c2d::Vec2{0.0f, 0.0f}, 2.0f, c2d::PI / 3);

// Create a polygon from a list of vertices
c2d::Polygon polygonFromVertices = c2d::makePolygon({
    c2d::Vec2{0.0f, 0.0f},
    c2d::Vec2{1.0f, 0.0f},
    c2d::Vec2{1.0f, 1.0f},
    c2d::Vec2{0.0f, 1.0f}
});

// Create a triangle from three vertices
c2d::Polygon triangle = c2d::makeTriangle(c2d::Vec2{0.0f, 0.0f}, c2d::Vec2{1.0f, 0.0f}, c2d::Vec2{0.5f, 1.0f});

// Create a rectangle centered at (0, 0) with half-width 1.0f, half-height 0.5f and rotated by PI/4 radians
c2d::Polygon rectangle = c2d::makeRectangle(c2d::Vec2{0.0f, 0.0f}, 1.0f, 0.5f, c2d::PI / 4);
```

### EntityCollision

The `EntityCollision` structure contains information about the entities and shapes involved in a collision, as well as the collision manifold. An `EntityCollision` object is returned by the collision queries methods and is defined as follows:

```cpp
struct EntityCollision
{
    EntityId entityA;
    EntityId entityB;
    ShapeId shapeA;
    ShapeId shapeB;
    Manifold manifold;
};
```

The `Manifold` structure describes the collision between the two shapes, including the contact points and the normal vector of the collision. It is defined as follows:

```cpp
struct Manifold
{
    Vec2 normal;              // Direction of collision, from shape A to shape B
    ManifoldPoint points[2];  // Up to 2 contact points
    uint8_t pointCount{0};    // Actual number of contact points (0, 1, or 2)
};
```

The `ManifoldPoint` structure describes a single contact point in the collision, including the position of the contact point, the anchors relative to each shape's origin, and the separation distance. It is defined as follows:

```cpp
struct ManifoldPoint
{
    Vec2 point;       // Contact point in absolute coordinates
    Vec2 anchorA;     // Contact point relative to shape A origin
    Vec2 anchorB;     // Contact point relative to shape B origin
    float separation; // Negative if overlapping, positive if separated
};
```

### RaycastHit

The `RaycastHit` structure contains information about a ray casting hit, including the entity and shape that was hit, the entry and exit points of the ray, and the distance from the ray's start to the entry and exit point. It is defined as follows:

```cpp
template <typename IdType>
struct RaycastHit
{
    IdType id;
    Vec2 entry;
    Vec2 exit;
    float entryTime;
    float exitTime;
};
```

The `Registry` class returns a `RaycastHit` object for each hit shape in the ray casting queries. The `IdType` is defined as `std::pair<EntityId, ShapeId>`, which allows you to identify the entity and shape that was hit by the ray. The `entry` and `exit` points are the points on the perimeter of the shape where the ray enters and exits the shape, respectively. The `entryTime` and `exitTime` values are the normalized times of entry and exit along the ray's path:
- If the ray is finite (`Ray` class), `entryTime` and `exitTime` are in the range [0, 1], where 0 means the ray hit the shape at its start point, and 1 means it hit the shape at its end point.
- If the ray is infinite (`InfiniteRay` class), `entryTime` and `exitTime` are in the range [0, +∞), where 0 means the ray hit the shape at its start point, and values greater than 0 indicate the distance from the ray's start point to the hit point along the ray's direction.

### Basic types

#### Vec2

The `Vec2` class represents a 2D vector and provides methods for common vector operations such as addition, subtraction, dot product, cross product, normalization, and interpolation. It is defined as follows:

```cpp
class Vec2
{
public:
    float x{0};
    float y{0};

    constexpr Vec2() : Vec2(0, 0) {}
    constexpr Vec2(float x, float y) : x(x), y(y) {}

    std::string toString() const;

    constexpr float length() const;
    constexpr float inverseLength() const;
    constexpr float lengthSqr() const;

    constexpr Vec2 interpolate(Vec2 other, float percentage) const;
    constexpr float dot(Vec2 other) const;
    constexpr float cross(Vec2 other) const;

    constexpr Vec2 normalize() const;
    static constexpr Vec2 normalized(float x, float y);

    constexpr float getBigger() const;
    constexpr Vec2 abs() const;
};
```

#### Rotation

The `Rotation` class represents a rotation in radians and provides methods to apply the rotation to a vector, compute the inverse rotation, and rotate a vector by the rotation. It is defined as follows:

```cpp
struct Rotation
{
public:
    float angleRadians = 0.0f;
    float sin = 0.0f;
    float cos = 1.0f;

    constexpr Rotation() = default;
    constexpr Rotation(float angleRadians);
    constexpr Rotation(float sin, float cos);

    // Rotate a vector
    constexpr Vec2 apply(Vec2 vec) const;

    constexpr Rotation inverse() const;
    constexpr Vec2 inverse(Vec2 vec) const;
};
```

#### Transform

The `Transform` class represents a transformation in 2D space, consisting of a translation (position) and a rotation. It provides methods to apply the transformation to a vector, convert a vector to local coordinates, and construct transformations from various parameters. It is defined as follows:

```cpp
using Translation = Vec2; // Alias for translation, which is a 2D vector

struct Transform
{
    Translation translation{};
    Rotation rotation{};

    constexpr Transform() = default;
    constexpr Transform(Translation translation);
    constexpr Transform(Translation translation, float angleRadians);
    constexpr Transform(Translation translation, Rotation rotation);

    // Transform a vector (rotate then translate)
    constexpr Vec2 apply(Vec2 vec) const;

    constexpr Vec2 toLocal(Vec2 vec) const;
};
```

#### Ray

The `Ray` class represents a finite ray in 2D space, defined by its start and end points. It is used for ray casting queries and is defined as follows:

```cpp
struct Ray
{
    Vec2 start;
    Vec2 end;
};
```

#### InfiniteRay

The `InfiniteRay` class represents an infinite ray in 2D space, defined by its start point and direction. It is used for ray casting queries and is defined as follows:

```cpp
struct InfiniteRay
{
    Vec2 start;
    Vec2 direction;
};
```

#### BodyType

The `BodyType` enum defines the types of bodies that can be created in the registry. It is used to specify how an entity responds to collisions and is defined as follows:

```cpp
enum class BodyType
{
    Static = 0,
    Dynamic,
    Bullet
};
```

See the [Body Types](#body-types) section for more details on each BodyType.


## Collision Filtering

Colli2De supports collision filtering through the use of category and mask bits. Each shape can be assigned a category bitmask and a mask bitmask when added to an entity. This allows you to control which shapes can collide with each other.

### Category Bits

Category bits are used to define the type of a shape. For example, you might have categories for player, enemy, and environment shapes. Each shape can belong to one or more categories.  
Colli2De defines `BitMaskType` as `uint64_t`, allowing you to use up to 64 different categories. You can define your own categories using bitwise operations, such as:

```cpp
constexpr c2d::BitMaskType PLAYER_CATEGORY      = 1 << 0; // 00001
constexpr c2d::BitMaskType ENEMY_CATEGORY       = 1 << 1; // 00010
constexpr c2d::BitMaskType ENVIRONMENT_CATEGORY = 1 << 2; // 00100
constexpr c2d::BitMaskType WEAPON_CATEGORY      = 1 << 3; // 01000
constexpr c2d::BitMaskType SHIELD_CATEGORY      = 1 << 4; // 10000
```

A shape can belong to multiple categories by combining the category bits using bitwise OR:

```cpp
constexpr c2d::BitMaskType WEAPON_AND_SHIELD_CATEGORY = WEAPON_CATEGORY | SHIELD_CATEGORY; // 11000
```

By default, the category bits are set to `1ull` if not specified.

### Mask Bits

Mask bits are used to define which categories a shape can collide with. For example, an enemy shape might only collide with player shapes and environment shapes, but not with other enemy shapes.
The mask bits are also defined as `BitMaskType`, and you can use bitwise operations to define them, such as:

```cpp
constexpr c2d::BitMaskType PLAYER_MASK = ~0ull; // Collides with all categories
constexpr c2d::BitMaskType ENEMY_MASK  = PLAYER_CATEGORY | ENVIRONMENT_CATEGORY; // Collides with player and environment categories
```

It's enough for one shape's mask bits to have one bit set in common with the other shape's category bits for a collision to occur, even if according to the other shape's mask bits the collision is not allowed.  
Mask bits are set to `~0ull` (all bits set) by default, meaning that the shape can collide with all categories unless specified otherwise.  
Mask bits are also used in ray casting queries to filter which shapes can be hit by the ray. When performing a ray casting query, you can specify the mask bits to limit the shapes that can be hit by the ray.  

```cpp
c2d::Ray ray{
    .start = Vec2{0.0f, 0.0f},
    .end = Vec2{1.0f, 1.0f}
};

// Cast a ray against all shapes that can be hit by the ray and belong to the ENEMY category
const auto hits = registry.rayCast(ray, ENEMY_CATEGORY);
```


## Behind the Scenes

Colli2De uses a broad-phase collision detection algorithm to quickly eliminate pairs of entities that are not colliding, followed by a narrow-phase collision detection algorithm to compute the actual collisions.  

### Collision Detection Flow

1. **Entity creation**:
    - A new entry is created in a `std::unordered_map`, using the entity ID as the key.
2. **Shape addition**:
    - The shape is stored in a `std::vector` within the entity's entry in the registry.
    - The AABB (Axis-Aligned Bounding Box) of the shape is computed and stored in the entity's entry.
    - If the shape belongs to a static entity, the AABB is added to the static `BroadPhaseTree`.
    - If the shape belongs to a dynamic entity, the AABB is added to the dynamic `BroadPhaseTree`.
    - If the shape belongs to a bullet entity, the AABB is stored in a `std::vector` within the entity's entry.
3. **Entity movement**:
    - The entity's position and rotation are updated in the registry.
    - The AABB of the entity is updated based on the new position and rotation.
    - The AABB is reinserted into the right `BroadPhaseTree` based on the entity's BodyType.
4. **Collision detection**:
    - For each dynamic or bullet entity, the broad-phase algorithm:
        - Checks for potential collisions by querying the static `BroadPhaseTree` for overlapping AABBs.
        - Checks for potential collisions by querying the dynamic `BroadPhaseTree` for overlapping AABBs.
    - For each pair of potential colliding entities, the narrow-phase algorithm:
        - Computes the actual collision by checking if the shapes are overlapping based on their AABBs.
        - If they are overlapping, it computes the collision manifold and stores it in the `EntityCollision` structure.
        - For bullet entities, it checks for collisions at multiple positions along the movement path to ensure accurate collision detection (swept collision detection).

### DynamicBVH

The `DynamicBVH` class is a dynamic bounding volume hierarchy (BVH) that efficiently manages AABBs and allows for very fast collision queries. The BVH is used for broad-phase collision detection, allowing the registry to quickly eliminate pairs of entities that are not colliding based on their AABBs.
This structure is designed to be automatically rebalanced, and to support fattening and displacement of the bounding boxes to allow for fast movement of entities without causing excessive updates to the BVH.  
[Read more](https://box2d.org/files/ErinCatto_DynamicBVH_Full.pdf) about the dynamic BVH structure described by Erin Catto, the creator of Box2D.

### Broad phase tree

The `BroadPhaseTree` class wraps many DynamicBVH instances, partitioning the scene into smaller regions for more efficient collision detection. Like the `DynamicBVH`, it manages AABBs, it allows for fast collision queries and is used for broad-phase collision detection.  
It can be constructed specifying the `cellSize`, which defines the size of each cell in the grid for spatial partitioning. The `BroadPhaseTree` automatically partitions the scene into cells, allowing for fast collision queries within each cell. The default `cellSize` is 64 pixels, which is suitable for most use cases.

### Narrow phase

The narrow-phase collision detection is performed only for pairs of entities that are potentially colliding, i.e., shapes whose AABBs overlap. The collision manifold is computed using the separating axis theorem (SAT) for convex shapes, and the collision response is computed based on the collision manifold.


## License

Colli2De is distributed under the terms of the MIT license. See the [LICENSE](LICENSE) file for details.
