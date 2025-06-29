# Project: Colli2De

This project is a modern and efficient C++23 library for 2D collision detection. It is designed to be high-level, easy to use, and fast, making it suitable for a wide range of applications, including games and simulations.  
It is not a physics engine, but rather a library that only provides collision detection capabilities, and it might be used inside a physics engine or any other application that requires collision detection.  

The main features of the public API in this library include:
- Creation and management of various geometric shapes (e.g., circles, rectangles, polygons).
- Creation of a `Registry` class that stores and manages these shapes. It can be used to add, remove, and query shapes.
- Efficient collision detection algorithms that can handle both static and dynamic shapes.
The user should never use the BVH or the collision detection algorithms directly, but rather use the `Registry` class to manage shapes and perform collision detection.

This library is highly based on the `Box2D` library, which is a C++ library for 2D collision detection but also a physics engine. Relevant files from `Box2D` can be found in the `box2d_reference/` directory. This library aims to provide a more modern and high-level API to the one provided by `Box2D`, while still being efficient and fast, and removing the whole physics handling done by `Box2D`.

# Project Structure
The project is structured as follows:

```
include/
    colli2de/
        <header files and directories>
src/
    <source files and directories>
test/
    <test files and directories>
```

The most important directories to get some context are:
- `box2d_reference/`: Contains the reference files from the `Box2D` library that were used to implement the collision detection algorithms in this library. Always refer to these files when implementing new features or fixing bugs related to collision detection.
- `include/colli2de/`: Contains the public API headers of the library.
- `src/collision/Collision.cpp`: Contains the implementation of the collision detection algorithms.
- `src/collision/Manifold.hpp`: Contains the implementation of the manifold data structure used in collision detection.
- `src/data_structures/DynamicBVH.hpp`: Contains the implementation of the dynamic bounding volume hierarchy (BVH) data structure.
- `src/geometry/Shapes.cpp`: Contains the implementation of the geometric shapes used in the library.

# Coding Conventions for Agents

Make sure to follow these coding conventions when working on the codebase:
- Follow my code conventions when coding.
- Use long names for variables, like `point1` and `manifold`, instead of single letters like `p1` or `m`.
- Pass every small class or struct to functions by value, and pass larger classes or structs by const reference.
    - Consider a class or struct small if it is less than or equal to 128 bytes in size (e.g., 4 floats).
    - So, always pass `Vec2`, `AABB`, `Transform` and `Rotation` by value, but pass `Manifold`, `Shape`, `Registry` and other larger classes by const reference.
- When using curly brackets, always put them on a new line. Avoid unnecessary curly brackets for single-line statements, but use them for multi-line statements or for clarity when in a if-else structure.
- Use `int32_t`, `uint32_t`, `int64_t` and so on instead of `int`, `long` or other integer types.
- Whenever possible, use `const` in front of local variables that never change after initialization.
- If a method has to return multiple values, use a `std::pair` or a `std::tuple` instead of passing pointers or references to the method.
- When creating functions that return a boolean value, use a name that starts with `is`, `has`, `can` or similar, to make it clear that the function returns a boolean value. For example, `isColliding`, `hasShape`, `canMove`, etc.

# Testing

To run tests, run the following command:
```bash
./test.sh --cmake -b -r <optional_filter>
```
<option_filter> is a string that will be used to filter the tests. For example, if you want to run only the tests that contain the word "collision" in the name, and exclude every test with the "[Benchmark]" tag, you can use:
```bash
./test.sh --cmake -b -r "collision" "~[Benchmark]"
```

The tag "[Benchmark]" is used to mark performance tests. They might take a long time to run, so if you didn't edit them, it's better to exclude them from the majority of the test runs. Make sure to run them at least once before committing your changes.

Tests are created using the `Catch2` framework. When creating new tests, make sure to follow the conventions of the existing tests.
