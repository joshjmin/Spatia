/*
 * m3.cpp — Pathfinding algorithms for ECE297 Milestone 3
 *
 * Implements A* search for driving directions and a two-phase
 * walk + multi-source-A* algorithm for uber-pool-style routing.
 *
 * Key design decisions:
 *   - Generation counter for O(1) search reset
 *   - SegmentCache for O(1) segment info lookups
 *   - Admissible heuristic: Euclidean distance / maxSpeedLimit
 */

#include <vector>
#include <queue>
#include <algorithm>
#include <limits>
#include <cmath>

#include "m3.h"
#include "m1.h"
#include "globals.h"
#include "StreetsDatabaseAPI.h"

// M3-private search state
namespace {

    // Per-intersection data used during A* / Dijkstra search
    struct NodeData {
        double bestTime;              // best g-cost found so far
        StreetSegmentIdx reachEdge;   // segment used to arrive here (-1 = source or unvisited)
        int generation;               // which search populated this entry
    };

    // Search state (persists across calls, reset via generation counter)
    std::vector<NodeData> nodeData;
    int currentGeneration = 0;

    // Priority queue element: (f-cost, intersection index)
    using PQElement = std::pair<double, IntersectionIdx>;
    using MinPQ = std::priority_queue<PQElement, std::vector<PQElement>,
                                      std::greater<PQElement>>;

    // Earth constants (same as in globals.cpp)
    constexpr double kEarth = 6372797.560856;
    constexpr double kDegToRad = 0.017453292519943295769236907684886;

    // Ensure nodeData is sized for the current map
    void ensureSearchData() {
        int numIntersections = getNumIntersections();
        if (static_cast<int>(nodeData.size()) != numIntersections) {
            nodeData.assign(numIntersections,
                {std::numeric_limits<double>::infinity(), -1, 0});
            currentGeneration = 0;
        }
    }

    // O(1) search reset — just bump the generation counter
    void resetSearch() {
        ++currentGeneration;
        // Guard against integer overflow 
        if (currentGeneration <= 0) {
            currentGeneration = 1;
            for (auto& nd : nodeData) {
                nd.generation = 0;
            }
        }
    }

    // Check whether a node has been reached in the current search
    inline bool isVisited(IntersectionIdx id) {
        return nodeData[id].generation == currentGeneration;
    }

    // Record the best cost and reaching edge for a node
    inline void setNode(IntersectionIdx id, double cost, StreetSegmentIdx edge) {
        nodeData[id].bestTime = cost;
        nodeData[id].reachEdge = edge;
        nodeData[id].generation = currentGeneration;
    }

    // A* heuristic: straight-line distance / max speed 
    inline double heuristic(IntersectionIdx from, IntersectionIdx to) {
        const LatLon& p1 = intersectionPositions[from];
        const LatLon& p2 = intersectionPositions[to];

        double dy = kEarth * (p1.latitude() - p2.latitude()) * kDegToRad;
        double dx = kEarth * (p1.longitude() - p2.longitude()) * kDegToRad * cosLatAvg;

        return std::sqrt(dx * dx + dy * dy) / maxSpeedLimit;
    }

    // Reconstruct a path by tracing reachEdge pointers from dst back to src.
    // Uses the provided nodeData vector and generation for lookup.
    std::vector<StreetSegmentIdx> tracePath(
            IntersectionIdx src, IntersectionIdx dst,
            const std::vector<NodeData>& nodes, int gen) {

        std::vector<StreetSegmentIdx> path;
        IntersectionIdx trace = dst;

        while (trace != src) {
            if (nodes[trace].generation != gen || nodes[trace].reachEdge == -1) {
                return {};  // shouldn't happen on a valid path
            }
            StreetSegmentIdx edge = nodes[trace].reachEdge;
            path.push_back(edge);
            const SegmentCache& sc = segmentCache[edge];
            trace = (sc.to == trace) ? sc.from : sc.to;
        }

        std::reverse(path.begin(), path.end());
        return path;
    }

}  


// computePathTravelTime — sum of driving times + turn penalties
double computePathTravelTime(const double turn_penalty,
                             const std::vector<StreetSegmentIdx>& path) {
    if (path.empty()) return 0.0;

    double totalTime = segmentTravelTimes[path[0]];

    for (size_t i = 1; i < path.size(); ++i) {
        totalTime += segmentTravelTimes[path[i]];

        // Turn penalty when street changes between consecutive segments
        if (segmentCache[path[i - 1]].streetID != segmentCache[path[i]].streetID) {
            totalTime += turn_penalty;
        }
    }

    return totalTime;
}


// computePathWalkingTime — sum of walking times + turn penalties
double computePathWalkingTime(const std::vector<StreetSegmentIdx>& path,
                              const double walking_speed,
                              const double turn_penalty) {
    if (path.empty()) return 0.0;

    double totalTime = segmentLengths[path[0]] / walking_speed;

    for (size_t i = 1; i < path.size(); ++i) {
        totalTime += segmentLengths[path[i]] / walking_speed;

        if (segmentCache[path[i - 1]].streetID != segmentCache[path[i]].streetID) {
            totalTime += turn_penalty;
        }
    }

    return totalTime;
}

// findPathBetweenIntersections — A* search for shortest driving time
std::vector<StreetSegmentIdx> findPathBetweenIntersections(
        const double turn_penalty,
        const std::pair<IntersectionIdx, IntersectionIdx> intersect_ids) {

    IntersectionIdx src = intersect_ids.first;
    IntersectionIdx dst = intersect_ids.second;

    // check if already at destination
    if (src == dst) return {};

    ensureSearchData();
    resetSearch();

    MinPQ openSet;
    setNode(src, 0.0, -1);
    openSet.push({heuristic(src, dst), src});

    while (!openSet.empty()) {
        auto [fCost, current] = openSet.top();
        openSet.pop();

        // Found the destination — stop searching
        if (current == dst) break;

        double currentBest = nodeData[current].bestTime;

        // Skip stale priority queue entries (node already reached via better path)
        if (fCost > currentBest + heuristic(current, dst) + 1e-9) continue;

        // Street of the edge that reached this node (for turn penalty check)
        StreetIdx reachingStreet = -1;
        if (nodeData[current].reachEdge != -1) {
            reachingStreet = segmentCache[nodeData[current].reachEdge].streetID;
        }

        // Expand all segments connected to the current intersection
        const std::vector<StreetSegmentIdx>& segs = streetSegmentsByIntersection[current];
        for (StreetSegmentIdx seg : segs) {
            const SegmentCache& sc = segmentCache[seg];

            // Determine the neighbor and enforce one-way constraints
            IntersectionIdx neighbor;
            if (sc.from == current) {
                neighbor = sc.to;
            } else if (!sc.oneWay) {
                neighbor = sc.from;
            } else {
                continue;  // can't go backwards on a one-way street
            }

            // Edge cost = travel time + possible turn penalty
            double edgeCost = segmentTravelTimes[seg];
            if (nodeData[current].reachEdge != -1 && sc.streetID != reachingStreet) {
                edgeCost += turn_penalty;
            }

            double newCost = currentBest + edgeCost;

            // Only update if this is a better path to the neighbor
            if (!isVisited(neighbor) || newCost < nodeData[neighbor].bestTime - 1e-9) {
                setNode(neighbor, newCost, seg);
                openSet.push({newCost + heuristic(neighbor, dst), neighbor});
            }
        }
    }

    // Reconstruct path (returns empty if dst was never reached)
    if (!isVisited(dst)) return {};
    return tracePath(src, dst, nodeData, currentGeneration);
}

// findPathWithWalkToPickUp — walk to a pickup, then drive to destination
std::pair<std::vector<StreetSegmentIdx>, std::vector<StreetSegmentIdx>>
findPathWithWalkToPickUp(
        const IntersectionIdx start_intersection,
        const IntersectionIdx end_intersection,
        const double turn_penalty,
        const double walking_speed,
        const double walking_time_limit) {

    // Trivial case
    if (start_intersection == end_intersection) return {{}, {}};

    ensureSearchData();

    // Phase 1: Walking Dijkstra from start (ignoring one-way constraints)
    resetSearch();

    MinPQ walkPQ;
    setNode(start_intersection, 0.0, -1);
    walkPQ.push({0.0, start_intersection});

    while (!walkPQ.empty()) {
        auto [walkTime, current] = walkPQ.top();
        walkPQ.pop();

        // Skip stale entries
        if (walkTime > nodeData[current].bestTime + 1e-9) continue;

        // Don't expand beyond the walk time limit
        if (walkTime > walking_time_limit) continue;

        // Street of the edge that reached this node
        StreetIdx reachingStreet = -1;
        if (nodeData[current].reachEdge != -1) {
            reachingStreet = segmentCache[nodeData[current].reachEdge].streetID;
        }

        // Expand all segments — pedestrians can walk both directions
        const std::vector<StreetSegmentIdx>& segs = streetSegmentsByIntersection[current];
        for (StreetSegmentIdx seg : segs) {
            const SegmentCache& sc = segmentCache[seg];

            // Walking ignores one-way, so always determine the other end
            IntersectionIdx neighbor = (sc.from == current) ? sc.to : sc.from;

            double edgeCost = segmentLengths[seg] / walking_speed;
            if (nodeData[current].reachEdge != -1 && sc.streetID != reachingStreet) {
                edgeCost += turn_penalty;
            }

            double newTime = walkTime + edgeCost;

            // Don't add if it exceeds the walk limit
            if (newTime > walking_time_limit) continue;

            if (!isVisited(neighbor) || newTime < nodeData[neighbor].bestTime - 1e-9) {
                setNode(neighbor, newTime, seg);
                walkPQ.push({newTime, neighbor});
            }
        }
    }

    // Save walk search results (need them for path reconstruction later)
    std::vector<NodeData> walkNodes = nodeData;
    int walkGeneration = currentGeneration;

    // Collect all walkable intersections (those visited in the walk search)
    // These become the candidate pickup points for Phase 2
    std::vector<IntersectionIdx> walkableNodes;
    int numIntersections = getNumIntersections();
    for (IntersectionIdx i = 0; i < numIntersections; ++i) {
        if (walkNodes[i].generation == walkGeneration) {
            walkableNodes.push_back(i);
        }
    }

    // If no walkable nodes found, no path possible
    if (walkableNodes.empty()) return {{}, {}};

    // Phase 2: Multi-source driving A* from all walkable nodes to dest
    resetSearch();

    MinPQ drivePQ;
    for (IntersectionIdx pickup : walkableNodes) {
        setNode(pickup, 0.0, -1);  // driving cost starts at 0 from each pickup
        drivePQ.push({heuristic(pickup, end_intersection), pickup});
    }

    while (!drivePQ.empty()) {
        auto [fCost, current] = drivePQ.top();
        drivePQ.pop();

        // Found the destination
        if (current == end_intersection) break;

        double currentBest = nodeData[current].bestTime;

        // Skip stale entries
        if (fCost > currentBest + heuristic(current, end_intersection) + 1e-9) continue;

        StreetIdx reachingStreet = -1;
        if (nodeData[current].reachEdge != -1) {
            reachingStreet = segmentCache[nodeData[current].reachEdge].streetID;
        }

        const std::vector<StreetSegmentIdx>& segs = streetSegmentsByIntersection[current];
        for (StreetSegmentIdx seg : segs) {
            const SegmentCache& sc = segmentCache[seg];

            // Driving respects one-way constraints
            IntersectionIdx neighbor;
            if (sc.from == current) {
                neighbor = sc.to;
            } else if (!sc.oneWay) {
                neighbor = sc.from;
            } else {
                continue;
            }

            double edgeCost = segmentTravelTimes[seg];
            if (nodeData[current].reachEdge != -1 && sc.streetID != reachingStreet) {
                edgeCost += turn_penalty;
            }

            double newCost = currentBest + edgeCost;

            if (!isVisited(neighbor) || newCost < nodeData[neighbor].bestTime - 1e-9) {
                setNode(neighbor, newCost, seg);
                drivePQ.push({newCost + heuristic(neighbor, end_intersection), neighbor});
            }
        }
    }

    // Path reconstruction

    // If destination not reached by driving, no valid path
    if (!isVisited(end_intersection)) return {{}, {}};

    // Trace driving path back to find the pickup intersection
    std::vector<StreetSegmentIdx> drivePath;
    IntersectionIdx pickupNode = end_intersection;
    {
        IntersectionIdx trace = end_intersection;
        while (nodeData[trace].reachEdge != -1) {
            StreetSegmentIdx edge = nodeData[trace].reachEdge;
            drivePath.push_back(edge);
            const SegmentCache& sc = segmentCache[edge];
            trace = (sc.to == trace) ? sc.from : sc.to;
        }
        pickupNode = trace;
        std::reverse(drivePath.begin(), drivePath.end());
    }

    // Trace walking path from pickup back to start
    std::vector<StreetSegmentIdx> walkPath;
    if (pickupNode != start_intersection) {
        walkPath = tracePath(start_intersection, pickupNode,
                             walkNodes, walkGeneration);
    }

    return {walkPath, drivePath};
}
