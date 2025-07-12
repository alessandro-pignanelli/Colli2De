#pragma once

namespace c2d
{

template<typename IdType>
BroadPhaseTreeHandle BroadPhaseTree<IdType>::addProxy(IdType entityId, AABB aabb, BitMaskType categoryBits, BitMaskType maskBits)
{
    BroadPhaseTreeHandle treeHandle;
    if (!freeList.empty())
    {
        treeHandle = freeList.back();
        freeList.pop_back();
    }
    else
    {
        treeHandle = static_cast<IdType>(proxies.size());
        proxies.emplace_back();
    }

    auto& proxy = proxies[treeHandle];
    proxy.entityId = entityId;
    proxy.aabb = aabb;
    proxy.categoryBits = categoryBits;
    proxy.maskBits = maskBits;

    forEachCell(aabb, cellSize, [&](GridCell cell)
    {
        auto& region = regions[cell];
        auto& bvh = region.bvh;

        // Region stores every proxy that belongs to it
        region.proxies.insert(treeHandle);

        // Proxy stores the BVH handle for each cell (region) it belongs to
        const auto bvhHandle = bvh.createProxy(aabb, entityId, categoryBits, maskBits);
        proxy.bvhHandles.emplace(cell, bvhHandle);
    });

    return treeHandle;
}

template<typename IdType>
void BroadPhaseTree<IdType>::removeProxy(BroadPhaseTreeHandle handle)
{
    assert(isValidHandle(handle));

    Proxy& proxy = proxies.at(handle);

    // Remove the proxy from all regions it belongs to
    for (const auto& [cell, bvhHandle] : proxy.bvhHandles)
    {
        auto& region = regions.at(cell);
        region.proxies.erase(handle);
        region.bvh.destroyProxy(bvhHandle);

        if (region.proxies.empty())
            regions.erase(cell);
    }
    
    proxy.bvhHandles.clear();
    freeList.push_back(handle);
}

template<typename IdType>
void BroadPhaseTree<IdType>::moveProxy(BroadPhaseTreeHandle handle, AABB aabb)
{
    assert(isValidHandle(handle));
    Proxy& proxy = proxies.at(handle);

    if (proxy.aabb.contains(aabb))
        return;

    const int32_t oldMinX = int32_t(proxy.aabb.min.x) / cellSize;
    const int32_t oldMinY = int32_t(proxy.aabb.min.y) / cellSize;
    const int32_t oldMaxX = int32_t(proxy.aabb.max.x) / cellSize;
    const int32_t oldMaxY = int32_t(proxy.aabb.max.y) / cellSize;

    const int32_t newMinX = int32_t(aabb.min.x) / cellSize;
    const int32_t newMinY = int32_t(aabb.min.y) / cellSize;
    const int32_t newMaxX = int32_t(aabb.max.x) / cellSize;
    const int32_t newMaxY = int32_t(aabb.max.y) / cellSize;

    proxy.aabb = aabb;
    const bool isSameCellRange = oldMinX == newMinX && oldMinY == newMinY && oldMaxX == newMaxX && oldMaxY == newMaxY;

    if (isSameCellRange)
    {
        for (const auto& [cell, bvhHandle] : proxy.bvhHandles)
            regions.at(cell).bvh.moveProxy(bvhHandle, aabb);
        return;
    }

    const auto addToCell = [this, handle, aabb](GridCell cell)
    {
        // Region stores every proxy that belongs to it
        auto& region = regions[cell];
        region.proxies.insert(handle);

        // Proxy stores the BVH handle for each cell (region) it belongs to
        auto& proxy = proxies.at(handle);
        const auto bvhHandle = region.bvh.createProxy(aabb, proxy.entityId, proxy.categoryBits, proxy.maskBits);
        proxy.bvhHandles.emplace(cell, bvhHandle);
    };
    const auto removeFromCell = [this, handle](GridCell cell)
    {
        // Remove the proxy from the region
        auto& region = regions.at(cell);
        region.proxies.erase(handle);

        // Destroy the proxy from the BVH in that region and remove the bvh handle
        auto& proxy = proxies.at(handle);
        const auto bvhHandleInRegion = proxy.bvhHandles.find(cell);
        assert(bvhHandleInRegion != proxy.bvhHandles.end());
        region.bvh.destroyProxy(bvhHandleInRegion->second);
        proxy.bvhHandles.erase(bvhHandleInRegion);

        if (region.proxies.empty())
            regions.erase(cell);
    };
    const auto moveSameCell = [this, handle, aabb](GridCell cell)
    {
        // If the proxy is still in the same cell, just update the BVH
        auto& region = regions.at(cell);
        const auto& proxy = proxies.at(handle);
        region.bvh.moveProxy(proxy.bvhHandles.at(cell), aabb);
    };

    const int32_t minX = std::min(oldMinX, newMinX);
    const int32_t minY = std::min(oldMinY, newMinY);
    const int32_t maxX = std::max(oldMaxX, newMaxX);
    const int32_t maxY = std::max(oldMaxY, newMaxY);

    for (int32_t x = minX; x <= maxX; ++x)
        for (int32_t y = minY; y <= maxY; ++y)
        {
            const GridCell cell{x, y};
            const bool isNotInOld = (x < oldMinX || x > oldMaxX || y < oldMinY || y > oldMaxY);
            const bool isNotInNew = (x < newMinX || x > newMaxX || y < newMinY || y > newMaxY);

            if (isNotInOld)
                addToCell(cell);
            else if (isNotInNew)
                removeFromCell(cell);
            else
                moveSameCell(cell);
        }
}

// AABB queries
template<typename IdType>
std::set<IdType> BroadPhaseTree<IdType>::query(AABB queryAABB, BitMaskType maskBits) const
{
    std::set<IdType> intersections;

    forEachCell(queryAABB, cellSize, [&](GridCell cell)
    {
        const auto regionIt = regions.find(cell);
        if (regionIt == regions.end())
            return; // No region here

        const auto& region = regionIt->second;
        region.bvh.query(queryAABB, intersections, maskBits);
    });

    return intersections;
}

template<typename IdType>
std::vector<std::set<IdType>> BroadPhaseTree<IdType>::batchQuery(const std::vector<std::pair<AABB, BitMaskType>>& queries, size_t numThreads) const
{
    const size_t n = queries.size();
    std::vector<std::set<IdType>> results(n);

    constexpr size_t minQueriesPerThread = 5;
    numThreads = std::min(numThreads, (n - minQueriesPerThread) / minQueriesPerThread + 1);
    if (numThreads == 1)
    {
        for (size_t i = 0; i < n; ++i)
            results[i] = query(queries[i].first, queries[i].second);
        return results;
    }

    const size_t chunk = (n + numThreads - 1) / numThreads;
    std::vector<std::future<void>> futures;

    for (size_t t = 0; t < numThreads; ++t) {
        const size_t begin = t * chunk;
        const size_t end = std::min(n, begin + chunk);

        futures.push_back(std::async(std::launch::async, [this, &queries, &results, begin, end]() {
            for (size_t i = begin; i < end; ++i)
                results[i] = this->query(queries[i].first, queries[i].second);
        }));
    }

    for (auto& f : futures)
        f.get();

    return results;
}


template<typename IdType>
std::set<IdType> BroadPhaseTree<IdType>::sweepQuery(AABB startAABB, AABB endAABB, BitMaskType maskBits) const
{
    std::set<IdType> intersections;

    AABB combinedAABB = AABB::combine(startAABB, endAABB);

    forEachCell(combinedAABB, cellSize, [&](GridCell cell)
    {
        const auto regionIt = regions.find(cell);
        if (regionIt == regions.end())
            return; // No region here

        const auto& region = regionIt->second;
        region.bvh.sweepQuery(startAABB, endAABB, intersections, maskBits);
    });

    return intersections;
}

template<typename IdType>
std::vector<std::set<IdType>> BroadPhaseTree<IdType>::batchSweepQuery(
    const std::vector<std::tuple<AABB, AABB, BitMaskType>>& queries,
    size_t numThreads
) const
{
    const size_t n = queries.size();
    std::vector<std::set<IdType>> results(n);

    constexpr size_t minQueriesPerThread = 2;
    numThreads = std::min(numThreads, (n - minQueriesPerThread) / minQueriesPerThread + 1);
    if (numThreads == 1)
    {
        for (size_t i = 0; i < n; ++i)
            results[i] = sweepQuery(std::get<0>(queries[i]), std::get<1>(queries[i]), std::get<2>(queries[i]));
        return results;
    }

    const size_t chunk = (n + numThreads - 1) / numThreads;
    std::vector<std::future<void>> futures;

    for (size_t t = 0; t < numThreads; ++t) {
        const size_t begin = t * chunk;
        const size_t end = std::min(n, begin + chunk);

        futures.push_back(std::async(std::launch::async, [this, &queries, &results, begin, end]() {
            for (size_t i = begin; i < end; ++i)
                results[i] = this->sweepQuery(std::get<0>(queries[i]), std::get<1>(queries[i]), std::get<2>(queries[i]));
        }));
    }

    for (auto& f : futures)
        f.get();

    return results;
}

template<typename IdType>
std::set<std::pair<IdType, IdType>> BroadPhaseTree<IdType>::findAllCollisions() const
{
    std::set<std::pair<IdType, IdType>> collisions;

    for (const auto& [cell, region] : regions)
        region.bvh.findAllCollisions(collisions);

    return collisions;
}

// Raycast queries
template<typename IdType>
std::optional<RaycastInfo<IdType>> BroadPhaseTree<IdType>::firstHitRaycast(Ray ray, BitMaskType maskBits) const
{
    const auto direction = ray.p2 - ray.p1;
    const int32_t directionX = (direction.x > 0 ? cellSize : -cellSize);
    const int32_t directionY = (direction.y > 0 ? cellSize : -cellSize);

    const bool isParallelX = std::abs(direction.y) < 1e-6f;
    const bool isParallelY = std::abs(direction.x) < 1e-6f;

    const GridCell lastCell = getCellFor(ray.p2, cellSize);
    auto currentCell = getCellFor(ray.p1, cellSize);
    auto currentPoint = ray.p1;

    while (currentCell.x * directionX <= lastCell.x * directionX &&
           currentCell.y * directionY <= lastCell.y * directionY)
    {
        if (const auto regionIt = regions.find(currentCell); regionIt != regions.end())
        {
            const auto& region = regionIt->second;
            const auto firstHit = region.bvh.firstHitRaycast(ray, maskBits);
            if (firstHit)
                return firstHit;
        }

        if (isParallelX)
        {
            currentCell.x += directionX;
            continue;
        }
        if (isParallelY)
        {
            currentCell.y += directionY;
            continue;
        }

        const auto nextCellX = currentCell.x + directionX;
        const auto nextCellY = currentCell.y + directionY;

        // Calculate the next intersection point with the grid lines
        const float timeX = (nextCellX - currentPoint.x) / direction.x;
        const float timeY = (nextCellY - currentPoint.y) / direction.y;

        if (timeX < timeY)
        {
            // Move to the next vertical grid line
            currentPoint.x = nextCellX;
            currentPoint.y += direction.y * timeX;
            currentCell.x += directionX;
        }
        else
        {
            // Move to the next horizontal grid line
            currentPoint.y = nextCellY;
            currentPoint.x += direction.x * timeY;
            currentCell.y += directionY;
        }
    }

    return std::nullopt;
}

template<typename IdType>
std::optional<RaycastInfo<IdType>> BroadPhaseTree<IdType>::firstHitRaycast(InfiniteRay ray, BitMaskType maskBits) const
{
    return firstHitRaycast(Ray{ray.start, ray.start + ray.direction.normalize() * cellSize * 50}, maskBits);
}

template<typename IdType>
std::set<RaycastInfo<IdType>> BroadPhaseTree<IdType>::piercingRaycast(Ray ray, BitMaskType maskBits) const
{
    const auto direction = ray.p2 - ray.p1;
    const int32_t directionX = (direction.x > 0 ? cellSize : -cellSize);
    const int32_t directionY = (direction.y > 0 ? cellSize : -cellSize);

    const bool isParallelX = std::abs(direction.y) < 1e-6f;
    const bool isParallelY = std::abs(direction.x) < 1e-6f;

    const GridCell lastCell = getCellFor(ray.p2, cellSize);
    auto currentCell = getCellFor(ray.p1, cellSize);
    auto currentPoint = ray.p1;

    std::set<RaycastInfo<IdType>> hits;

    while (currentCell.x * directionX <= lastCell.x * directionX &&
           currentCell.y * directionY <= lastCell.y * directionY)
    {
        if (const auto regionIt = regions.find(currentCell); regionIt != regions.end())
        {
            const auto& region = regionIt->second;
            region.bvh.piercingRaycast(ray, hits, maskBits);
        }

        if (isParallelX)
        {
            currentCell.x += directionX;
            continue;
        }
        if (isParallelY)
        {
            currentCell.y += directionY;
            continue;
        }

        const auto nextCellX = currentCell.x + directionX;
        const auto nextCellY = currentCell.y + directionY;

        // Calculate the next intersection point with the grid lines
        const float timeX = (nextCellX - currentPoint.x) / direction.x;
        const float timeY = (nextCellY - currentPoint.y) / direction.y;

        if (timeX < timeY)
        {
            // Move to the next vertical grid line
            currentPoint.x = nextCellX;
            currentPoint.y += direction.y * timeX;
            currentCell.x += directionX;
        }
        else
        {
            // Move to the next horizontal grid line
            currentPoint.y = nextCellY;
            currentPoint.x += direction.x * timeY;
            currentCell.y += directionY;
        }
    }

    return hits;
}

template<typename IdType>
std::set<RaycastInfo<IdType>> BroadPhaseTree<IdType>::piercingRaycast(InfiniteRay ray, BitMaskType maskBits) const
{
    return piercingRaycast(Ray{ray.start,
                               ray.start + ray.direction.normalize() * cellSize * INFINITE_RAY_CELL_SPAN},
                           maskBits);
}

template<typename IdType>
bool BroadPhaseTree<IdType>::isValidHandle(BroadPhaseTreeHandle handle) const
{
    return handle >= 0 && static_cast<size_t>(handle) < proxies.size();
}

template<typename IdType>
void BroadPhaseTree<IdType>::clear()
{
    proxies.clear();
    freeList.clear();
    regions.clear();
    regions.reserve(32);
}

} // namespace c2d
