- Raycast hit filtering and early-exit
- Update/fatten margin per-proxy (Allow dynamic adjustment of fatAABBMargin if some objects need more/less)
- Fast rebuild: construct BVH from a full set in one pass (useful for static scenes or resets).
- Static BVH mode (can be built with sweep SAH for optimality)
- Parallel or batched queries/moves
- Batch/BVH-vs-BVH Traversal (to check collision between two BVH trees, optimal only if there are many objects in both trees)
- Narrow phase collision detection (e.g. GJK, EPA, SAT)
    1. Shape and transform types/utilities
    2. Manifold/contact types
    3. Core collision routines (one at a time, starting with Circle–Circle)
    4. Shape dispatch system
    5. Point/raycast utility
    6. Unit/visual test support
- Static BVH class
    - Use SAH to construct a shallower, higher-quality tree
    - Store data in a flat array instead of pointers for fast access