#pragma once

#include <algorithm>
#include <future>
#include <ranges>
#include <stack>

namespace c2d
{

template<typename IdType>
std::vector<IdType> DynamicBVH<IdType>::query(AABB queryAABB) const
{
    if (rootIndex == INVALID_NODE_INDEX)
        return {};

    std::vector<IdType> intersections;
    std::stack<NodeIndex> stack;
    stack.push(rootIndex);

    while (!stack.empty())
    {
        NodeIndex nodeIndex = stack.top();
        stack.pop();
        const auto& node = nodes[nodeIndex];

        // If the node's AABB does not intersect with the query AABB
        // Skip the whole subtree (since it is contained in the node's AABB)
        if (!node.aabb.intersects(queryAABB))
            continue;

        if (node.isLeaf())
        {
            intersections.push_back(node.id);
        }
        else
        {
            stack.push(node.child1Index);
            stack.push(node.child2Index);
        }
    }

    return intersections;
}

template<typename IdType>
std::vector<std::vector<IdType>> DynamicBVH<IdType>::batchQuery(const std::vector<AABB>& queries,
                                                                size_t numThreads) const
{
    size_t n = queries.size();
    std::vector<std::vector<IdType>> results(n);
    std::vector<std::future<void>> futures;

    size_t chunk = (n + numThreads - 1) / numThreads;

    for (size_t t = 0; t < numThreads; ++t) {
        size_t begin = t * chunk;
        size_t end = std::min(n, begin + chunk);

        futures.push_back(std::async(std::launch::async, [this, &queries, &results, begin, end]() {
            for (size_t i = begin; i < end; ++i)
                results[i] = this->query(queries[i]);
        }));
    }

    for (auto& f : futures)
        f.get();

    return results;
}

template<typename IdType>
std::vector<IdType> DynamicBVH<IdType>::piercingRaycast(Ray ray) const
{
    if (rootIndex == INVALID_NODE_INDEX)
        return {};

    std::vector<IdType> hits;
    std::stack<NodeIndex> stack;
    stack.push(rootIndex);

    while (!stack.empty())
    {
        NodeIndex nodeIndex = stack.top();
        stack.pop();
        const auto& node = nodes[nodeIndex];

        if (!node.aabb.intersects(ray))
            continue;

        if (node.isLeaf())
        {
            hits.push_back(node.id);
        }
        else
        {
            stack.push(node.child1Index);
            stack.push(node.child2Index);
        }
    }

    return hits;
}

template<typename IdType>
std::optional<IdType> DynamicBVH<IdType>::firstHitRaycast(Ray ray) const
{
    if (rootIndex == INVALID_NODE_INDEX)
        return std::nullopt;

    std::optional<IdType> closestId;
    float firstHitTime = std::numeric_limits<float>::max();

    std::stack<NodeIndex> stack;
    stack.push(rootIndex);

    while (!stack.empty())
    {
        NodeIndex nodeIndex = stack.top();
        stack.pop();
        const auto& node = nodes[nodeIndex];

        const auto intersection = node.aabb.intersects(ray);
        if (!intersection)
            continue;

        if (node.isLeaf())
        {
            if (intersection->first < firstHitTime)
            {
                firstHitTime = intersection->first;
                closestId = node.id;
            }
        }
        else
        {
            stack.push(node.child1Index);
            stack.push(node.child2Index);
        }
    }

    return closestId;
}

template<typename IdType>
std::vector<typename DynamicBVH<IdType>::RaycastInfo> DynamicBVH<IdType>::piercingRaycastDetailed(Ray ray) const
{
    if (rootIndex == INVALID_NODE_INDEX)
        return {};

    std::vector<RaycastInfo> hits;
    std::stack<NodeIndex> stack;
    stack.push(rootIndex);

    while (!stack.empty())
    {
        NodeIndex nodeIndex = stack.top();
        stack.pop();
        const auto& node = nodes[nodeIndex];

        const auto intersection = node.aabb.intersects(ray);
        if (!intersection)
            continue;

        if (node.isLeaf())
        {
            hits.push_back(RaycastInfo::fromRay(node.id, ray, *intersection));
        }
        else
        {
            stack.push(node.child1Index);
            stack.push(node.child2Index);
        }
    }

    std::sort(hits.begin(), hits.end(), [&](const RaycastInfo& a, const RaycastInfo& b)
    {
        return (a.entry - ray.p1).lengthSqr() < (b.entry - ray.p1).lengthSqr();
    });

    return hits;
}

template<typename IdType>
std::optional<typename DynamicBVH<IdType>::RaycastInfo> DynamicBVH<IdType>::firstHitRaycastDetailed(Ray ray) const
{
    if (rootIndex == INVALID_NODE_INDEX)
        return std::nullopt;

    std::optional<RaycastInfo> firstHit;
    float firstHitTime = std::numeric_limits<float>::max();

    std::stack<NodeIndex> stack;
    stack.push(rootIndex);

    while (!stack.empty())
    {
        NodeIndex nodeIndex = stack.top();
        stack.pop();
        const auto& node = nodes[nodeIndex];

        const auto intersection = node.aabb.intersects(ray);
        if (!intersection)
            continue;

        if (node.isLeaf())
        {
            if (intersection->first < firstHitTime)
            {
                firstHitTime = intersection->first;
                firstHit = RaycastInfo::fromRay(node.id, ray, *intersection);
            }
        }
        else
        {
            stack.push(node.child1Index);
            stack.push(node.child2Index);
        }
    }

    return firstHit;
}

template<typename IdType>
std::vector<IdType> DynamicBVH<IdType>::piercingRaycast(InfiniteRay ray) const
{
    if (rootIndex == INVALID_NODE_INDEX)
        return {};

    std::vector<IdType> hits;
    std::stack<NodeIndex> stack;
    stack.push(rootIndex);

    while (!stack.empty())
    {
        NodeIndex nodeIndex = stack.top();
        stack.pop();
        const auto& node = nodes[nodeIndex];

        if (!node.aabb.intersects(ray))
            continue;

        if (node.isLeaf())
        {
            hits.push_back(node.id);
        }
        else
        {
            stack.push(node.child1Index);
            stack.push(node.child2Index);
        }
    }

    return hits;
}

template<typename IdType>
std::optional<IdType> DynamicBVH<IdType>::firstHitRaycast(InfiniteRay ray) const
{
    if (rootIndex == INVALID_NODE_INDEX)
        return std::nullopt;

    std::optional<IdType> closestId;
    float firstHitTime = std::numeric_limits<float>::max();

    std::stack<NodeIndex> stack;
    stack.push(rootIndex);

    while (!stack.empty())
    {
        NodeIndex nodeIndex = stack.top();
        stack.pop();
        const auto& node = nodes[nodeIndex];

        const auto intersection = node.aabb.intersects(ray);
        if (!intersection)
            continue;

        if (node.isLeaf())
        {
            if (intersection->first < firstHitTime)
            {
                firstHitTime = intersection->first;
                closestId = node.id;
            }
        }
        else
        {
            stack.push(node.child1Index);
            stack.push(node.child2Index);
        }
    }

    return closestId;
}

template<typename IdType>
std::vector<typename DynamicBVH<IdType>::RaycastInfo> DynamicBVH<IdType>::piercingRaycastDetailed(InfiniteRay ray) const
{
    if (rootIndex == INVALID_NODE_INDEX)
        return {};

    std::vector<RaycastInfo> hits;
    std::stack<NodeIndex> stack;
    stack.push(rootIndex);

    while (!stack.empty())
    {
        NodeIndex nodeIndex = stack.top();
        stack.pop();
        const auto& node = nodes[nodeIndex];

        const auto intersection = node.aabb.intersects(ray);
        if (!intersection)
            continue;

        if (node.isLeaf())
        {
            hits.push_back(RaycastInfo::fromRay(node.id, ray, *intersection));
        }
        else
        {
            stack.push(node.child1Index);
            stack.push(node.child2Index);
        }
    }

    std::sort(hits.begin(), hits.end(), [&](const RaycastInfo& a, const RaycastInfo& b)
    {
        return (a.entry - ray.start).lengthSqr() < (b.entry - ray.start).lengthSqr();
    });

    return hits;
}

template<typename IdType>
std::optional<typename DynamicBVH<IdType>::RaycastInfo> DynamicBVH<IdType>::firstHitRaycastDetailed(InfiniteRay ray) const
{
    if (rootIndex == INVALID_NODE_INDEX)
        return std::nullopt;

    std::optional<RaycastInfo> firstHit;
    float firstHitTime = std::numeric_limits<float>::max();

    std::stack<NodeIndex> stack;
    stack.push(rootIndex);

    while (!stack.empty())
    {
        NodeIndex nodeIndex = stack.top();
        stack.pop();
        const auto& node = nodes[nodeIndex];

        const auto intersection = node.aabb.intersects(ray);
        if (!intersection)
            continue;

        if (node.isLeaf())
        {
            if (intersection->first < firstHitTime)
            {
                firstHitTime = intersection->first;
                firstHit = RaycastInfo::fromRay(node.id, ray, *intersection);
            }
        }
        else
        {
            stack.push(node.child1Index);
            stack.push(node.child2Index);
        }
    }

    return firstHit;
}

template<typename IdType>
std::vector<std::pair<IdType, IdType>> DynamicBVH<IdType>::findBroadPhaseCollisions() const
{
    if (rootIndex == INVALID_NODE_INDEX)
        return {};

    std::vector<std::pair<IdType, IdType>> pairs;
    findBroadPhaseCollisionsRecursive(rootIndex, pairs);
    return pairs;
}

template<typename IdType>
void DynamicBVH<IdType>::collectLeaves(NodeIndex nodeIdx, std::vector<NodeIndex>& out) const
{
    const auto& node = nodes[nodeIdx];
    if (node.isLeaf())
    {
        out.push_back(nodeIdx);
        return;
    }

    collectLeaves(node.child1Index, out);
    collectLeaves(node.child2Index, out);
}

template<typename IdType>
void DynamicBVH<IdType>::findPairsBetween(NodeIndex nodeAIdx,
                                          NodeIndex nodeBIdx,
                                          std::vector<std::pair<IdType, IdType>>& out) const
{
    if (nodeAIdx == INVALID_NODE_INDEX || nodeBIdx == INVALID_NODE_INDEX || nodeAIdx == nodeBIdx)
        return;

    const auto& nodeA = nodes[nodeAIdx];
    const auto& nodeB = nodes[nodeBIdx];

    if (!nodeA.aabb.intersects(nodeB.aabb))
        return;

    if (nodeA.isLeaf() && nodeB.isLeaf())
    {
        // Always store (min, max) to avoid (A, B) and (B, A) being different
        if (nodeA.id < nodeB.id)
            out.emplace_back(nodeA.id, nodeB.id);
        else
            out.emplace_back(nodeB.id, nodeA.id);
        return;
    }

    // Expand the node with greater height (heuristic)
    if (nodeA.isLeaf() || (!nodeB.isLeaf() && nodeB.height > nodeA.height))
    {
        findPairsBetween(nodeAIdx, nodeB.child1Index, out);
        findPairsBetween(nodeAIdx, nodeB.child2Index, out);
    }
    else
    {
        findPairsBetween(nodeA.child1Index, nodeBIdx, out);
        findPairsBetween(nodeA.child2Index, nodeBIdx, out);
    }
}

template<typename IdType>
void DynamicBVH<IdType>::findBroadPhaseCollisionsRecursive(NodeIndex nodeIdx,
                                                           std::vector<std::pair<IdType, IdType>>& out) const
{
    if (nodeIdx == INVALID_NODE_INDEX)
        return;

    const auto& node = nodes[nodeIdx];
    if (node.isLeaf())
        return;

    // Check all pairs between children
    findPairsBetween(node.child1Index, node.child2Index, out);

    // Check pairs within each child
    findBroadPhaseCollisionsRecursive(node.child1Index, out);
    findBroadPhaseCollisionsRecursive(node.child2Index, out);
}

template<typename IdType>
void DynamicBVH<IdType>::serialize(std::ostream& out) const
{
    write_pod(out, fatAABBMargin);
    write_pod(out, rootIndex);
    write_pod(out, nodeCount);
    write_pod(out, nextAvailableIndex);

    // Write nodes vector size
    uint64_t nodesSize = nodes.size();
    write_pod(out, nodesSize);

    // Write nodes
    for (const auto& node : nodes)
    {
        write_pod(out, node.aabb.min.x); write_pod(out, node.aabb.min.y);
        write_pod(out, node.aabb.max.x); write_pod(out, node.aabb.max.y);
        write_pod(out, node.parentIndex);
        write_pod(out, node.child1Index);
        write_pod(out, node.child2Index);
        write_pod(out, node.height);
        write_pod(out, node.id);
    }
}

template<typename IdType>
DynamicBVH<IdType> DynamicBVH<IdType>::deserialize(std::istream& in)
{
    float fatAABBMargin;
    read_pod(in, fatAABBMargin);
    DynamicBVH<IdType> bvh(fatAABBMargin);

    read_pod(in, bvh.rootIndex);
    read_pod(in, bvh.nodeCount);
    read_pod(in, bvh.nextAvailableIndex);

    uint64_t nodesSize;
    read_pod(in, nodesSize);
    bvh.nodes.resize(nodesSize);

    for (auto& node : bvh.nodes)
    {
        read_pod(in, node.aabb.min.x); read_pod(in, node.aabb.min.y);
        read_pod(in, node.aabb.max.x); read_pod(in, node.aabb.max.y);
        read_pod(in, node.parentIndex);
        read_pod(in, node.child1Index);
        read_pod(in, node.child2Index);
        read_pod(in, node.height);
        read_pod(in, node.id);
    }

    return bvh;
}

template<typename IdType>
bool DynamicBVH<IdType>::operator==(const DynamicBVH<IdType>& other) const
{
    if (!float_equals(fatAABBMargin, other.fatAABBMargin))
        return false;
    if (rootIndex != other.rootIndex)
        return false;
    if (nodeCount != other.nodeCount)
        return false;
    if (nextAvailableIndex != other.nextAvailableIndex)
        return false;
    if (nodes.size() != other.nodes.size())
        return false;

    for (size_t i = 0; i < nodes.size(); ++i)
    {
        const auto& a = nodes[i];
        const auto& b = other.nodes[i];

        if (a.aabb != b.aabb)
            return false;
        if (a.parentIndex != b.parentIndex)
            return false;
        if (a.child1Index != b.child1Index)
            return false;
        if (a.child2Index != b.child2Index)
            return false;
        if (a.height != b.height)
            return false;
        if (a.id != b.id)
            return false;
    }

    return true;
}

}