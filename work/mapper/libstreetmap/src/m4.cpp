#include <vector>
#include <queue>
#include <algorithm>
#include <limits>
#include <cmath>
#include <cstring>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <random>
#include <numeric>
#include <functional>

#include "m4.h"
#include "m3.h"
#include "m1.h"
#include "globals.h"
#include "StreetsDatabaseAPI.h"


// Constants

static constexpr float INF_COST = std::numeric_limits<float>::infinity();
static constexpr int   NUM_THREADS = 8;

// Time limits are adjusted dynamically in travelingCourier() based on problem size.
double HARD_TIME_LIMIT = 48.0;   // workers stop here
double FINAL_TIME_LIMIT = 49.5;  // final polish stops here

// Shared state

namespace {

// Wall-clock start time set once at entry to travelingCourier.
std::chrono::high_resolution_clock::time_point g_startTime;

inline double elapsed() {
    return std::chrono::duration<double>(
        std::chrono::high_resolution_clock::now() - g_startTime).count();
}

// POI (point of interest) index layout:
//   delivery i pickup  = index 2*i
//   delivery i dropoff = index 2*i + 1
//   depot j            = index 2*numDel + j

int g_numDel    = 0;  // number of deliveries
int g_numDepots = 0;  // number of depots
int g_numPOI    = 0;  // total POIs = 2*numDel + numDepots

// Intersection ID for each POI.
std::vector<IntersectionIdx> g_poiIntersection;

// Flattened travel-time matrix indexed as g_tt[src * g_numPOI + dst].
std::vector<float> g_tt;

// Cached turn penalty used during final path reconstruction.
float g_turnPenalty = 0.0f;

// Helpers to convert delivery/depot indices to POI indices.
inline int pickupPOI(int del)    { return 2 * del; }
inline int dropoffPOI(int del)   { return 2 * del + 1; }
inline int depotPOI(int dep)     { return 2 * g_numDel + dep; }
inline bool isPOIDepot(int poi)  { return poi >= 2 * g_numDel; }
inline bool isPOIPickup(int poi) { return poi < 2 * g_numDel && (poi % 2 == 0); }
inline bool isPOIDropoff(int poi){ return poi < 2 * g_numDel && (poi % 2 == 1); }
inline int poiDelivery(int poi)  { return poi / 2; } // only valid for pickup/dropoff POIs
inline float tt(int src, int dst){ return g_tt[src * g_numPOI + dst]; }

// POI initialization

void initPOIs(const std::vector<DeliveryInf>& deliveries,
              const std::vector<IntersectionIdx>& depots) {
    g_numDel    = static_cast<int>(deliveries.size());
    g_numDepots = static_cast<int>(depots.size());
    g_numPOI    = 2 * g_numDel + g_numDepots;

    g_poiIntersection.resize(g_numPOI);
    for (int i = 0; i < g_numDel; ++i) {
        g_poiIntersection[2 * i]     = deliveries[i].pickUp;
        g_poiIntersection[2 * i + 1] = deliveries[i].dropOff;
    }
    for (int j = 0; j < g_numDepots; ++j) {
        g_poiIntersection[2 * g_numDel + j] = depots[j];
    }

    // Initialize the travel-time matrix to infinity, then set diagonal to 0.
    g_tt.assign(g_numPOI * g_numPOI, INF_COST);
    for (int i = 0; i < g_numPOI; ++i){
        g_tt[i * g_numPOI + i] = 0.0f;
    }
}

// Nearest-neighbor lists precomputed from the travel-time matrix.
// For each POI, stores the K closest other POIs by travel time.
// Used by repair operators to narrow insertion position search.

constexpr int NN_K = 15; // number of neighbors to cache per POI
std::vector<std::vector<int>> g_nn; // g_nn[poi] = sorted nearest POI indices

void buildNearestNeighbors() {
    g_nn.resize(g_numPOI);
    for (int i = 0; i < g_numPOI; ++i) {
        std::vector<std::pair<float, int>> dists;
        dists.reserve(g_numPOI);
        for (int j = 0; j < g_numPOI; ++j) {
            if (j == i){
                continue;
            }
            float c = tt(i, j);
            if (c < INF_COST){
                dists.push_back({c, j});
            }
        }
        // Partial sort is enough since we only need the top K entries.
        int k = std::min(NN_K, (int)dists.size());
        std::partial_sort(dists.begin(), dists.begin() + k, dists.end());
        g_nn[i].resize(k);
        for (int x = 0; x < k; ++x){
            g_nn[i][x] = dists[x].second;
        }
    }
}

// Multi-destination Dijkstra for the travel-time matrix.
// Same idea as M3's A* but without a heuristic and with early termination
// once all POI intersections have been settled.
// Tracks the incoming segment at each node to apply the turn penalty correctly.

struct DijkBuf {
    std::vector<double> bestTime;
    std::vector<int>    reachSeg;   // segment used to arrive at each node (-1 = source)
    std::vector<int>    gen;
    std::vector<int>    targetGen;  // tracks which targets are still unsettled
    int curGen;

    void init(int numIntersections) {
        bestTime.resize(numIntersections);
        reachSeg.resize(numIntersections);
        gen.assign(numIntersections, 0);
        targetGen.assign(numIntersections, 0);
        curGen = 0;
    }

    void reset() {
        // Increment by 2 because targetGen uses both curGen and curGen+1.
        curGen += 2;
        if (curGen <= 0) {
            std::fill(gen.begin(), gen.end(), 0);
            std::fill(targetGen.begin(), targetGen.end(), 0);
            curGen = 2;
        }
    }

    inline double getCost(int node) const {
        return (gen[node] == curGen) ? bestTime[node] : 1e18;
    }

    inline void setNode(int node, double cost, int seg) {
        bestTime[node] = cost;
        reachSeg[node] = seg;
        gen[node] = curGen;
    }

    inline bool isTargetUnsettled(int node) const {
        return targetGen[node] == curGen;
    }

    inline void markTarget(int node) {
        targetGen[node] = curGen;
    }

    inline void settleTarget(int node) {
        targetGen[node] = curGen + 1;
    }
};

struct PQEntry {
    double cost;
    IntersectionIdx node;
    bool operator>(const PQEntry& o) const { return cost > o.cost; }
};

// Fills one row of the travel-time matrix by running Dijkstra from srcIdx.
void dijkstraRow(int srcIdx, float turnPenalty, DijkBuf& buf) {
    buf.reset();

    const IntersectionIdx source = g_poiIntersection[srcIdx];
    buf.setNode(source, 0.0, -1);

    // Mark all distinct target intersections so we can stop early once all are settled.
    int targetsLeft = 0;
    for (int i = 0; i < g_numPOI; ++i) {
        IntersectionIdx inter = g_poiIntersection[i];
        if (i != srcIdx && inter != source && !buf.isTargetUnsettled(inter)) {
            buf.markTarget(inter);
            ++targetsLeft;
        }
    }

    std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<PQEntry>> pq;
    pq.push({0.0, source});

    while (!pq.empty() && targetsLeft > 0) {
        auto [curCost, curNode] = pq.top();
        pq.pop();

        if (curCost > buf.getCost(curNode) + 1e-9){
            continue; // stale entry, skip
        }

        if (buf.isTargetUnsettled(curNode)) {
            buf.settleTarget(curNode);
            --targetsLeft;
        }

        // Determine incoming street so we can apply the turn penalty on street changes.
        StreetIdx inStreet = -1;
        if (buf.reachSeg[curNode] != -1) {
            inStreet = segmentCache[buf.reachSeg[curNode]].streetID;
        }

        for (StreetSegmentIdx seg : streetSegmentsByIntersection[curNode]) {
            const SegmentCache& sc = segmentCache[seg];

            IntersectionIdx neighbor;
            if (sc.from == curNode) {
                neighbor = sc.to;
            } else if (!sc.oneWay) {
                neighbor = sc.from;
            } else {
                continue;
            }

            double edgeCost = segmentTravelTimes[seg];
            if (inStreet != -1 && sc.streetID != inStreet) {
                edgeCost += turnPenalty;
            }

            double newCost = curCost + edgeCost;
            if (newCost < buf.getCost(neighbor) - 1e-9) {
                buf.setNode(neighbor, newCost, seg);
                pq.push({newCost, neighbor});
            }
        }
    }

    // Write shortest times into the matrix row for this source POI.
    for (int dst = 0; dst < g_numPOI; ++dst) {
        if (dst == srcIdx){
            continue;
        }
        IntersectionIdx inter = g_poiIntersection[dst];
        float cost = (inter == source) ? 0.0f
                    :static_cast<float>(buf.getCost(inter));
        if (cost > 1e17f){
            cost = INF_COST;
        }
        g_tt[srcIdx * g_numPOI + dst] = cost;
    }
}

// Groups POIs by intersection so multiple POIs at the same location share one Dijkstra run.
struct DijkstraTask {
    IntersectionIdx intersection;
    std::vector<int> poiIndices; // all POIs located at this intersection
};

// Dispatches Dijkstra runs across threads to fill the full travel-time matrix.
// Deduplication is critical for test cases where many POIs share intersections.
void precomputeMatrix(float turnPenalty) {
    const int numIntersections = getNumIntersections();

    std::unordered_map<IntersectionIdx, int> interToTask;
    std::vector<DijkstraTask> tasks;
    tasks.reserve(g_numPOI);

    for (int i = 0; i < g_numPOI; ++i) {
        IntersectionIdx inter = g_poiIntersection[i];
        auto it = interToTask.find(inter);
        if (it == interToTask.end()) {
            interToTask[inter] = (int)tasks.size();
            tasks.push_back({inter, {i}});
        } else {
            tasks[it->second].poiIndices.push_back(i);
        }
    }

    const int numTasks   = (int)tasks.size();
    const int numThreads = std::min(NUM_THREADS, numTasks);

    // Distribute tasks evenly across threads, giving the first `rem` threads one extra.
    const int base = numTasks / numThreads;
    const int rem  = numTasks % numThreads;

    std::vector<std::thread> threads;
    threads.reserve(numThreads);

    for (int t = 0; t < numThreads; ++t) {
        const int begin = t * base + std::min(t, rem);
        const int end   = begin + base + (t < rem ? 1 : 0);

        threads.emplace_back([&, begin, end, turnPenalty]() {
            DijkBuf buf;
            buf.init(numIntersections);
            for (int taskIdx = begin; taskIdx < end; ++taskIdx) {
                const DijkstraTask& task = tasks[taskIdx];
                // Run Dijkstra once using the first POI as the representative for this intersection.
                int repPOI = task.poiIndices[0];
                dijkstraRow(repPOI, turnPenalty, buf);
                // All POIs at the same intersection have identical outgoing distances, so just copy.
                for (int k = 1; k < (int)task.poiIndices.size(); ++k) {
                    int otherPOI = task.poiIndices[k];
                    std::memcpy(&g_tt[otherPOI * g_numPOI],
                                &g_tt[repPOI * g_numPOI],
                                g_numPOI * sizeof(float));
                }
            }
        });
    }

    for (auto& th : threads) {
        th.join();
    }
}

// A route stores POI indices (pickups and dropoffs only) in visit order.
// Depot POIs are tracked separately as startDepot and endDepot.
struct Route {
    std::vector<int> stops; // POI indices in visit order (no depots in this list)
    int startDepot;         // depot POI index for the start of the route
    int endDepot;           // depot POI index for the end of the route
    float cost;             // cached total travel time

    void calcCost() {
        if (stops.empty()){ cost = INF_COST; return; }
        float c = tt(startDepot, stops[0]);
        for (int i = 0; i + 1 < (int)stops.size(); ++i) {
            c += tt(stops[i], stops[i + 1]);
        }
        c += tt(stops.back(), endDepot);
        cost = (c >= INF_COST) ? INF_COST : c;
    }
};

// Replaces the start and end depots with the nearest available ones.
void optimizeDepots(Route& r) {
    if (r.stops.empty()){
        return;
    }
    int first = r.stops.front();
    int last  = r.stops.back();

    float bestStart = INF_COST;
    float bestEnd   = INF_COST;
    int bestSD = r.startDepot, bestED = r.endDepot;

    for (int j = 0; j < g_numDepots; ++j) {
        int d = depotPOI(j);
        float sc = tt(d, first);
        if (sc < bestStart) {
            bestStart = sc;
            bestSD = d;
        }
        float ec = tt(last, d);
        if (ec < bestEnd) {
            bestEnd = ec;
            bestED = d;
        }
    }
    r.startDepot = bestSD;
    r.endDepot   = bestED;
}

// Greedy nearest-neighbor initial solution.
// At each step picks the nearest legal POI. Dropoffs are only legal after their pickup.
Route greedyNN(int startDepotIdx, std::mt19937& /*rng*/) {
    Route r;
    r.startDepot = depotPOI(startDepotIdx);
    r.stops.reserve(2 * g_numDel);

    std::vector<bool> pickedUp(g_numDel, false);
    std::vector<bool> visited(g_numPOI, false);

    int curPOI = r.startDepot;
    int placed = 0;

    while (placed < 2 * g_numDel) {
        float bestCost = INF_COST;
        int bestPOI = -1;

        for (int i = 0; i < 2 * g_numDel; ++i) {
            if (visited[i]) {
                continue;
            }
            // Check legality: dropoffs require pickup done first
            if (isPOIDropoff(i) && !pickedUp[poiDelivery(i)]) {
                continue;
            }

            float c = tt(curPOI, i);
            if (c < bestCost) {
                bestCost = c;
                bestPOI = i;
            }
        }

        if (bestPOI < 0) {
            break; // no reachable POI left — should not happen on valid input
        }

        r.stops.push_back(bestPOI);
        visited[bestPOI] = true;
        if (isPOIPickup(bestPOI)) {
            pickedUp[poiDelivery(bestPOI)] = true;
        }
        curPOI = bestPOI;
        ++placed;
    }

    optimizeDepots(r);
    r.calcCost();
    return r;
}

// Randomized greedy initial solution for diversity across threads.
// Like greedy NN but picks from the top-k candidates with a bias toward the best.
Route randomizedGreedy(int startDepotIdx, std::mt19937& rng) {
    Route r;
    r.startDepot = depotPOI(startDepotIdx);
    r.stops.reserve(2 * g_numDel);

    std::vector<bool> pickedUp(g_numDel, false);
    std::vector<bool> visited(g_numPOI, false);

    int curPOI = r.startDepot;
    int placed = 0;

    // Candidates buffer (reused)
    struct Cand { int poi; float cost; };
    std::vector<Cand> cands;
    cands.reserve(2 * g_numDel);

    std::uniform_real_distribution<float> udist(0.0f, 1.0f);

    while (placed < 2 * g_numDel) {
        cands.clear();

        for (int i = 0; i < 2 * g_numDel; ++i) {
            if (visited[i]) {
                continue;
            }
            if (isPOIDropoff(i) && !pickedUp[poiDelivery(i)]) {
                continue;
            }
            cands.push_back({i, tt(curPOI, i)});
        }

        if (cands.empty()) {
            break;
        }

        std::sort(cands.begin(), cands.end(),
            [](const Cand& a, const Cand& b) { return a.cost < b.cost; });

        int k = std::min((int)cands.size(), std::max(1, g_numDel / 5));
        // Power-of-2 sampling biases selection toward the front of the sorted list.
        float r_val = udist(rng);
        int idx = static_cast<int>(r_val * r_val * k);
        if (idx >= k) {
            idx = k - 1;
        }

        int bestPOI = cands[idx].poi;
        r.stops.push_back(bestPOI);
        visited[bestPOI] = true;
        if (isPOIPickup(bestPOI)) {
            pickedUp[poiDelivery(bestPOI)] = true;
        }
        curPOI = bestPOI;
        ++placed;
    }

    optimizeDepots(r);
    r.calcCost();
    return r;
}

// Regret-2 insertion initial solution.
// Finds the best and 2nd-best insertion cost for each unrouted delivery and inserts
// the one with the highest regret (2nd_best - best) first. This prioritizes
// deliveries that have few good positions before the window closes.
Route regret2Init(int startDepotIdx, std::mt19937& rng, float noise = 0.0f) {
    Route r;
    r.startDepot = depotPOI(startDepotIdx);
    r.stops.reserve(2 * g_numDel);

    std::vector<bool> routed(g_numDel, false);
    std::uniform_real_distribution<float> noiseDist(-1.0f, 1.0f);

    int remaining = g_numDel;
    while (remaining > 0) {
        float bestRegret = -INF_COST;
        int   bestDel = -1;
        int   bestP = -1, bestD = -1;

        const int n = (int)r.stops.size();

        // Returns the POI at index idx, treating negative as start depot and out-of-bounds as end depot.
        auto node = [&](int idx) -> int {
            if (idx < 0) { return r.startDepot; }
            if (idx >= n) { return r.endDepot; }
            return r.stops[idx];
        };

        // Temporarily pick the nearest end depot so cost estimates are accurate.
        {
            float bestEC = INF_COST;
            r.endDepot = depotPOI(0);
            if (n > 0) {
                for (int j = 0; j < g_numDepots; ++j) {
                    float ec = tt(r.stops.back(), depotPOI(j));
                    if (ec < bestEC) {
                        bestEC = ec;
                        r.endDepot = depotPOI(j);
                    }
                }
            } else {
                r.endDepot = depotPOI(startDepotIdx);
            }
        }

        for (int del = 0; del < g_numDel; ++del) {
            if (routed[del]) {
                continue;
            }

            int pk = pickupPOI(del);
            int doff = dropoffPOI(del);

            float best1 = INF_COST, best2 = INF_COST;
            int b1p = -1, b1d = -1;

            // Evaluate all (pickup position p, dropoff position d) pairs.
            for (int p = 0; p <= n; ++p) {
                float cpk = tt(node(p - 1), pk) + tt(pk, node(p)) - tt(node(p - 1), node(p));
                if (cpk >= INF_COST) {
                    continue;
                }

                // Conceptual node accessor after inserting pickup at position p.
                auto modNode = [&](int idx) -> int {
                    if (idx < 0)  { return r.startDepot; }
                    if (idx > n)  { return r.endDepot; }
                    if (idx == p) { return pk; }
                    if (idx < p)  { return r.stops[idx]; }
                    return r.stops[idx - 1];
                };

                for (int d = p + 1; d <= n + 1; ++d) {
                    float cdo = tt(modNode(d - 1), doff) + tt(doff, modNode(d))
                              - tt(modNode(d - 1), modNode(d));
                    float total = cpk + cdo + noise * noiseDist(rng);

                    if (total < best1) {
                        best2 = best1;
                        best1 = total;
                        b1p = p; b1d = d;
                    } else if (total < best2) {
                        best2 = total;
                    }
                }
            }

            if (best1 >= INF_COST) {
                continue;
            }

            // Regret is the cost of delaying this delivery. A large regret means insert now.
            float regret;
            if (best2 >= INF_COST) {
                regret = 1e30f; // only one valid position exists, so insert immediately
            } else {
                regret = best2 - best1;
            }

            if (regret > bestRegret) {
                bestRegret = regret;
                bestDel = del;
                bestP = b1p;
                bestD = b1d;
            }
        }

        if (bestDel < 0) {
            break; // no feasible insertion found
        }

        r.stops.insert(r.stops.begin() + bestP, pickupPOI(bestDel));
        r.stops.insert(r.stops.begin() + bestD, dropoffPOI(bestDel));
        routed[bestDel] = true;
        --remaining;
    }

    optimizeDepots(r);
    r.calcCost();
    return r;
}

// ALNS destroy operators

// Removes q randomly chosen deliveries from the route.
void destroyRandom(Route& r, int q, std::mt19937& rng,
                   std::vector<int>& removed) {
    thread_local std::vector<int> routed;
    routed.clear();
    for (int poi : r.stops) {
        if (isPOIPickup(poi)) {
            routed.push_back(poiDelivery(poi));
        }
    }

    q = std::min(q, (int)routed.size());
    for (int i = 0; i < q; ++i) {
        std::uniform_int_distribution<int> d(i, (int)routed.size() - 1);
        std::swap(routed[i], routed[d(rng)]);
        removed.push_back(routed[i]);
    }
}

// Removes the q most expensive deliveries by marginal travel cost.
void destroyWorst(Route& r, int q, std::mt19937& rng,
                  std::vector<int>& removed) {
    const int n = (int)r.stops.size();
    auto node = [&](int i) -> int {
        if (i < 0) { return r.startDepot; }
        if (i >= n) { return r.endDepot; }
        return r.stops[i];
    };

    thread_local std::vector<int> pp, dp;
    pp.assign(g_numDel, -1);
    dp.assign(g_numDel, -1);
    for (int i = 0; i < n; ++i) {
        int poi = r.stops[i];
        if (isPOIPickup(poi)) {
            pp[poiDelivery(poi)] = i;
        } else if (isPOIDropoff(poi)) {
            dp[poiDelivery(poi)] = i;
        }
    }

    // Compute how much travel time is saved by removing each delivery.
    struct DelCost { int del; float cost; };
    std::vector<DelCost> costs;
    for (int del = 0; del < g_numDel; ++del) {
        int ppos = pp[del];
        int dpos = dp[del];
        if (ppos < 0) {
            continue;
        }

        float savings;
        if (dpos == ppos + 1) {
            savings = tt(node(ppos - 1), r.stops[ppos]) + tt(r.stops[ppos], r.stops[dpos])
                    + tt(r.stops[dpos], node(dpos + 1)) - tt(node(ppos - 1), node(dpos + 1));
        } else {
            savings = tt(node(ppos - 1), r.stops[ppos]) + tt(r.stops[ppos], node(ppos + 1))
                    - tt(node(ppos - 1), node(ppos + 1))
                    + tt(node(dpos - 1), r.stops[dpos]) + tt(r.stops[dpos], node(dpos + 1))
                    - tt(node(dpos - 1), node(dpos + 1));
        }
        costs.push_back({del, savings});
    }

    // Sort highest savings first, then select using power-of-3 biased sampling.
    std::sort(costs.begin(), costs.end(),
        [](const DelCost& a, const DelCost& b) { return a.cost > b.cost; });

    q = std::min(q, (int)costs.size());
    std::vector<bool> selected(costs.size(), false);
    std::uniform_real_distribution<float> ud(0.0f, 1.0f);

    for (int count = 0; count < q; ++count) {
        int remaining = 0;
        for (int i = 0; i < (int)costs.size(); ++i) {
            if (!selected[i]) {
                ++remaining;
            }
        }
        if (remaining == 0) {
            break;
        }

        float rv = ud(rng);
        int target = static_cast<int>(rv * rv * rv * remaining);
        if (target >= remaining) {
            target = remaining - 1;
        }

        int seen = 0;
        for (int i = 0; i < (int)costs.size(); ++i) {
            if (!selected[i]) {
                if (seen == target) {
                    selected[i] = true;
                    removed.push_back(costs[i].del);
                    break;
                }
                ++seen;
            }
        }
    }
}

// Removes q deliveries that are geographically related to a randomly chosen seed (Shaw removal).
void destroyShaw(Route& r, int q, std::mt19937& rng,
                 std::vector<int>& removed) {
    thread_local std::vector<int> routed;
    routed.clear();
    for (int poi : r.stops) {
        if (isPOIPickup(poi)) {
            routed.push_back(poiDelivery(poi));
        }
    }

    if (routed.empty()) {
        return;
    }
    q = std::min(q, (int)routed.size());

    std::uniform_int_distribution<int> sd(0, (int)routed.size() - 1);
    int seed   = routed[sd(rng)];
    int seedPk = pickupPOI(seed);

    // Sort by relatedness using combined pickup + dropoff distance (Ropke & Pisinger).
    int seedDo = dropoffPOI(seed);
    struct Rel { int del; float dist; };
    std::vector<Rel> rels;
    for (int del : routed) {
        float pkDist = std::min(tt(seedPk, pickupPOI(del)), tt(pickupPOI(del), seedPk));
        float doDist = std::min(tt(seedDo, dropoffPOI(del)), tt(dropoffPOI(del), seedDo));
        rels.push_back({del, pkDist + doDist});
    }
    std::sort(rels.begin(), rels.end(),
        [](const Rel& a, const Rel& b) { return a.dist < b.dist; });

    for (int i = 0; i < q; ++i) {
        removed.push_back(rels[i].del);
    }
}

// Removes q deliveries whose pickup or dropoff falls in a contiguous route window.
// This is fast since it requires no sorting or distance computation.
void destroySegment(Route& r, int q, std::mt19937& rng,
                    std::vector<int>& removed) {
    const int n = (int)r.stops.size();
    if (n < 2) {
        return;
    }

    // Pick random start position, collect deliveries in a window
    std::uniform_int_distribution<int> startDist(0, n - 1);
    int start = startDist(rng);

    thread_local std::vector<bool> seen;
    seen.assign(g_numDel, false);

    int count = 0;
    for (int offset = 0; offset < n && count < q; ++offset) {
        int idx = (start + offset) % n;
        int poi = r.stops[idx];
        if (poi >= 2 * g_numDel) {
            continue; // depot POIs should never appear in stops
        }
        int del = poiDelivery(poi);
        if (!seen[del]) {
            seen[del] = true;
            removed.push_back(del);
            ++count;
        }
    }
}

// Removes q deliveries closest to a random center point using Euclidean distance.
// Cheaper than travel-time lookups and creates clean geographic holes in the route.
void destroyRadial(Route& r, int q, std::mt19937& rng,
                   std::vector<int>& removed) {
    thread_local std::vector<int> routed;
    routed.clear();
    for (int poi : r.stops) {
        if (isPOIPickup(poi)) {
            routed.push_back(poiDelivery(poi));
        }
    }

    if (routed.empty()) {
        return;
    }
    q = std::min(q, (int)routed.size());

    std::uniform_int_distribution<int> sd(0, (int)routed.size() - 1);
    int centerDel = routed[sd(rng)]; // pick a random delivery as the radial center
    const LatLon& centerPos = intersectionPositions[g_poiIntersection[pickupPOI(centerDel)]];

    struct Dist { int del; double dist; };
    std::vector<Dist> dists;
    dists.reserve(routed.size());
    for (int del : routed) {
        const LatLon& pos = intersectionPositions[g_poiIntersection[pickupPOI(del)]];
        double dy = pos.latitude() - centerPos.latitude();
        double dx = (pos.longitude() - centerPos.longitude()) * cosLatAvg;
        dists.push_back({del, dx*dx + dy*dy});
    }
    std::partial_sort(dists.begin(), dists.begin() + q, dists.end(),
        [](const Dist& a, const Dist& b) { return a.dist < b.dist; });

    for (int i = 0; i < q; ++i) {
        removed.push_back(dists[i].del);
    }
}

// ALNS repair operators

// Greedy repair: inserts each delivery at its cheapest feasible position.
// Uses an O(N) suffix-minimum DP so each delivery's best position is found in linear time.
// For non-adjacent dropoff positions (d >= p+2), the cost is independent of p, so we
// precompute all dropoff insertion costs, build suffix minimums, then scan pickup positions.
void repairGreedy(Route& r, std::vector<int>& removed, std::mt19937& /*rng*/) {

    // Insert the most geographically isolated deliveries first so they get the most options.
    if (removed.size() > 1 && !r.stops.empty()) {
        thread_local std::vector<std::pair<float,int>> sortBuf;
        sortBuf.clear();
        for (int del : removed) {
            float minDist = INF_COST;
            int pk = pickupPOI(del);
            for (int poi : r.stops) {
                float d = std::min(tt(pk, poi), tt(poi, pk));
                if (d < minDist) {
                minDist = d;
            }
            }
            sortBuf.push_back({minDist, del});
        }
        std::sort(sortBuf.begin(), sortBuf.end(),
            [](const auto& a, const auto& b) { return a.first > b.first; });
        for (int i = 0; i < (int)removed.size(); ++i) {
            removed[i] = sortBuf[i].second;
        }
    }

    thread_local std::vector<float> costDO, suffMin;
    thread_local std::vector<int> suffBestD;

    for (int del : removed) {
        int pk   = pickupPOI(del);
        int doff = dropoffPOI(del);
        const int n = (int)r.stops.size();

        auto node = [&](int idx) -> int {
            if (idx < 0) { return r.startDepot; }
            if (idx >= n) { return r.endDepot; }
            return r.stops[idx];
        };

        float bestCost = INF_COST;
        int bestP = 0, bestD = 1;

        // Step 1: Precompute dropoff insertion cost for each non-adjacent position d >= 2.
        // When d >= p+2 the cost is independent of p, so we can reuse these values.
        costDO.assign(n + 2, INF_COST);
        for (int d = 2; d <= n + 1; ++d) {
            int prevNode = node(d - 2);
            int nextNode = node(d - 1);
            costDO[d] = tt(prevNode, doff) + tt(doff, nextNode) - tt(prevNode, nextNode);
        }

        // Step 2: Build suffix minimums so any best-d query for a given p is O(1).
        suffMin.assign(n + 2, INF_COST);
        suffBestD.assign(n + 2, -1);
        float runMin  = INF_COST;
        int runBestD = -1;
        for (int d = n + 1; d >= 2; --d) {
            if (costDO[d] < runMin) {
                runMin   = costDO[d];
                runBestD = d;
            }
            if (d - 2 >= 0) {
                suffMin[d - 2]   = runMin;
                suffBestD[d - 2] = runBestD;
            }
        }

        // Step 3: Scan all pickup positions in O(N) using the precomputed suffix data.
        for (int p = 0; p <= n; ++p) {
            float cpk = tt(node(p - 1), pk) + tt(pk, node(p)) - tt(node(p - 1), node(p));
            if (cpk >= INF_COST || cpk >= bestCost) {
                continue;
            }

            // Case A: dropoff placed immediately after pickup (adjacent).
            float adjCost  = tt(pk, doff) + tt(doff, node(p)) - tt(pk, node(p));
            float totalAdj = cpk + adjCost;
            if (totalAdj < bestCost) {
                bestCost = totalAdj;
                bestP = p;
                bestD = p + 1;
            }

            // Case B: dropoff placed further down the route (non-adjacent).
            if (p <= n - 1 && suffMin[p] < INF_COST) {
                float totalSuff = cpk + suffMin[p];
                if (totalSuff < bestCost) {
                    bestCost = totalSuff;
                    bestP = p;
                    bestD = suffBestD[p];
                }
            }
        }

        r.stops.insert(r.stops.begin() + bestP, pk);
        r.stops.insert(r.stops.begin() + bestD, doff);
    }
    r.calcCost();
}

// Regret-2 repair: inserts the delivery with the highest (2nd_best - best) cost first.
// Uses the same O(N) suffix-minimum DP as repairGreedy but tracks the top 2 costs.
void repairRegret2(Route& r, const std::vector<int>& toInsert, std::mt19937& /*rng*/) {
    std::vector<bool> inserted(g_numDel, false);
    for (int poi : r.stops) {
        if (isPOIPickup(poi)) {
            inserted[poiDelivery(poi)] = true;
        }
    }

    thread_local std::vector<float> costDO, suffMin1, suffMin2;
    thread_local std::vector<int> suffBestD1, suffBestD2;

    int remaining = (int)toInsert.size();
    while (remaining > 0) {
        const int n = (int)r.stops.size();
        auto node = [&](int idx) -> int {
            if (idx < 0) { return r.startDepot; }
            if (idx >= n) { return r.endDepot; }
            return r.stops[idx];
        };

        float bestRegret = -INF_COST;
        int bestDel = -1, bestP = -1, bestD = -1;

        for (int del : toInsert) {
            if (inserted[del]) {
                continue;
            }
            int pk = pickupPOI(del);
            int doff = dropoffPOI(del);

            // Precompute non-adjacent dropoff costs.
            costDO.assign(n + 2, INF_COST);
            for (int d = 2; d <= n + 1; ++d) {
                costDO[d] = tt(node(d - 2), doff) + tt(doff, node(d - 1))
                          - tt(node(d - 2), node(d - 1));
            }

            // Build suffix top-2 minimums to enable O(1) regret queries.
            suffMin1.assign(n + 2, INF_COST);
            suffMin2.assign(n + 2, INF_COST);
            suffBestD1.assign(n + 2, -1);
            suffBestD2.assign(n + 2, -1);
            float rm1 = INF_COST, rm2 = INF_COST;
            int rd1 = -1, rd2 = -1;
            for (int d = n + 1; d >= 2; --d) {
                float c = costDO[d];
                if (c < rm1) {
                    rm2 = rm1; rd2 = rd1;
                    rm1 = c; rd1 = d;
                } else if (c < rm2) {
                    rm2 = c; rd2 = d;
                }
                if (d - 2 >= 0) {
                    suffMin1[d - 2] = rm1; suffBestD1[d - 2] = rd1;
                    suffMin2[d - 2] = rm2; suffBestD2[d - 2] = rd2;
                }
            }

            // Scan pickup positions in O(N), tracking the best and second-best total costs.
            float best1 = INF_COST, best2 = INF_COST;
            int b1p = -1, b1d = -1;

            for (int p = 0; p <= n; ++p) {
                float cpk = tt(node(p - 1), pk) + tt(pk, node(p))
                          - tt(node(p - 1), node(p));
                if (cpk >= INF_COST) {
                    continue;
                }

                // Case A: adjacent dropoff
                float adjCost  = tt(pk, doff) + tt(doff, node(p)) - tt(pk, node(p));
                float totalAdj = cpk + adjCost;
                if (totalAdj < best1) {
                    best2 = best1; best1 = totalAdj; b1p = p; b1d = p + 1;
                } else if (totalAdj < best2) {
                    best2 = totalAdj;
                }

                // Case B: non-adjacent dropoff using suffix top-2
                if (p <= n - 1) {
                    if (suffMin1[p] < INF_COST) {
                        float t1 = cpk + suffMin1[p];
                        if (t1 < best1) {
                            best2 = best1; best1 = t1; b1p = p; b1d = suffBestD1[p];
                        } else if (t1 < best2) {
                            best2 = t1;
                        }
                    }
                    if (suffMin2[p] < INF_COST) {
                        float t2 = cpk + suffMin2[p];
                        if (t2 < best1) {
                            best2 = best1; best1 = t2; b1p = p; b1d = suffBestD2[p];
                        } else if (t2 < best2) {
                            best2 = t2;
                        }
                    }
                }
            }

            if (best1 >= INF_COST) {
                continue;
            }
            float regret = (best2 < INF_COST) ? (best2 - best1) : 1e30f;
            if (regret > bestRegret) {
                bestRegret = regret;
                bestDel = del; bestP = b1p; bestD = b1d;
            }
        }

        if (bestDel < 0) { break; }
        r.stops.insert(r.stops.begin() + bestP, pickupPOI(bestDel));
        r.stops.insert(r.stops.begin() + bestD, dropoffPOI(bestDel));
        inserted[bestDel] = true;
        --remaining;
    }
    r.calcCost();
}

// Regret-3 repair: like regret-2 but uses the top 3 insertion costs to compute regret.
void repairRegret3(Route& r, const std::vector<int>& toInsert, std::mt19937& /*rng*/) {
    std::vector<bool> inserted(g_numDel, false);
    for (int poi : r.stops) {
        if (isPOIPickup(poi)) {
            inserted[poiDelivery(poi)] = true;
        }
    }

    thread_local std::vector<float> costDO, sm1, sm2, sm3;
    thread_local std::vector<int> sd1, sd2, sd3;

    int remaining = (int)toInsert.size();
    while (remaining > 0) {
        const int n = (int)r.stops.size();
        auto node = [&](int idx) -> int {
            if (idx < 0) { return r.startDepot; }
            if (idx >= n) { return r.endDepot; }
            return r.stops[idx];
        };

        float bestRegret = -INF_COST;
        int bestDel = -1, bestP = -1, bestD = -1;

        for (int del : toInsert) {
            if (inserted[del]) {
                continue;
            }
            int pk = pickupPOI(del);
            int doff = dropoffPOI(del);

            // Precompute non-adjacent dropoff costs.
            costDO.assign(n + 2, INF_COST);
            for (int d = 2; d <= n + 1; ++d) {
                costDO[d] = tt(node(d - 2), doff) + tt(doff, node(d - 1))
                          - tt(node(d - 2), node(d - 1));
            }

            // Build suffix top-3 minimums.
            sm1.assign(n + 2, INF_COST);
            sm2.assign(n + 2, INF_COST);
            sm3.assign(n + 2, INF_COST);
            sd1.assign(n + 2, -1);
            sd2.assign(n + 2, -1);
            sd3.assign(n + 2, -1);
            float r1 = INF_COST, r2 = INF_COST, r3 = INF_COST;
            int d1 = -1, d2 = -1, d3 = -1;
            for (int d = n + 1; d >= 2; --d) {
                float c = costDO[d];
                if      (c < r1) { r3 = r2; d3 = d2; r2 = r1; d2 = d1; r1 = c; d1 = d; }
                else if (c < r2) { r3 = r2; d3 = d2; r2 = c; d2 = d; }
                else if (c < r3) { r3 = c; d3 = d; }
                if (d - 2 >= 0) {
                    sm1[d - 2] = r1; sd1[d - 2] = d1;
                    sm2[d - 2] = r2; sd2[d - 2] = d2;
                    sm3[d - 2] = r3; sd3[d - 2] = d3;
                }
            }

            // Scan pickup positions in O(N), maintaining the top-3 insertion costs.
            float best1 = INF_COST, best2 = INF_COST, best3 = INF_COST;
            int b1p = -1, b1d = -1;

            auto updateTop3 = [&](float total, int pp, int dd) {
                if (total < best1) {
                    best3 = best2; best2 = best1; best1 = total; b1p = pp; b1d = dd;
                } else if (total < best2) {
                    best3 = best2; best2 = total;
                } else if (total < best3) {
                    best3 = total;
                }
            };

            for (int p = 0; p <= n; ++p) {
                float cpk = tt(node(p - 1), pk) + tt(pk, node(p))
                          - tt(node(p - 1), node(p));
                if (cpk >= INF_COST) {
                    continue;
                }

                // Adjacent dropoff case.
                float adj = tt(pk, doff) + tt(doff, node(p)) - tt(pk, node(p));
                updateTop3(cpk + adj, p, p + 1);

                // Non-adjacent dropoff using suffix top-3.
                if (p <= n - 1) {
                    if (sm1[p] < INF_COST) {
                        updateTop3(cpk + sm1[p], p, sd1[p]);
                    }
                    if (sm2[p] < INF_COST) {
                        updateTop3(cpk + sm2[p], p, sd2[p]);
                    }
                    if (sm3[p] < INF_COST) {
                        updateTop3(cpk + sm3[p], p, sd3[p]);
                    }
                }
            }

            if (best1 >= INF_COST) {
                continue;
            }
            float regret;
            if (best2 >= INF_COST) {
                regret = 1e30f;
            } else if (best3 >= INF_COST) {
                regret = (best2 - best1);
            } else {
                regret = (best2 - best1) + (best3 - best1);
            }
            if (regret > bestRegret) {
                bestRegret = regret;
                bestDel = del;
                bestP   = b1p;
                bestD   = b1d;
            }
        }

        if (bestDel < 0) { break; }
        r.stops.insert(r.stops.begin() + bestP, pickupPOI(bestDel));
        r.stops.insert(r.stops.begin() + bestD, dropoffPOI(bestDel));
        inserted[bestDel] = true;
        --remaining;
    }
    r.calcCost();
}

// Adaptive operator weights using roulette wheel selection.
// Scores accumulate during a window, then weights are updated using exponential smoothing.
struct AdaptiveWeights {
    static constexpr int ND = 5; // number of destroy operators
    static constexpr int NR = 3; // number of repair operators

    float dw[ND], rw[NR]; // current selection weights
    float ds[ND], rs[NR]; // accumulated scores since last update
    int   du[ND], ru[NR]; // usage counts since last update

    void init() {
        std::fill(dw, dw + ND, 1.0f);
        std::fill(rw, rw + NR, 1.0f);
        resetScores();
    }

    void resetScores() {
        std::fill(ds, ds + ND, 0.0f);
        std::fill(rs, rs + NR, 0.0f);
        std::fill(du, du + ND, 0);
        std::fill(ru, ru + NR, 0);
    }

    int selectDestroy(std::mt19937& rng) { return roulette(dw, ND, rng); }
    int selectRepair(std::mt19937& rng)  { return roulette(rw, NR, rng); }

    void record(int d, int r_op, float score) {
        ds[d] += score; du[d]++;
        rs[r_op] += score; ru[r_op]++;
    }

    void updateWeights(float rho = 0.1f) {
        for (int i = 0; i < ND; ++i) {
            if (du[i] > 0) {
                dw[i] = (1 - rho) * dw[i] + rho * (ds[i] / du[i]);
                dw[i] = std::max(dw[i], 0.1f);
            }
        }
        for (int i = 0; i < NR; ++i) {
            if (ru[i] > 0) {
                rw[i] = (1 - rho) * rw[i] + rho * (rs[i] / ru[i]);
                rw[i] = std::max(rw[i], 0.1f);
            }
        }
        resetScores();
    }

private:
    int roulette(const float* w, int n, std::mt19937& rng) {
        float sum = 0;
        for (int i = 0; i < n; ++i) {
            sum += w[i];
        }
        std::uniform_real_distribution<float> d(0.0f, sum);
        float r = d(rng), acc = 0;
        for (int i = 0; i < n - 1; ++i) {
            acc += w[i];
            if (r < acc) {
                return i;
            }
        }
        return n - 1;
    }
};

// Combined ALNS destroy + repair. Returns true if a new candidate was generated.

// Per-thread scratch buffers to avoid allocations in the hot path.
struct ALNSScratch {
    std::vector<int>  removed;
    std::vector<bool> removePOI;
    bool initialized = false;

    void init() {
        if (initialized) { return; }
        removed.reserve(g_numDel);
        removePOI.resize(g_numPOI, false);
        initialized = true;
    }

    void clear() {
        removed.clear();
        // removePOI flags are reset individually after use, keeping reset cost at O(q).
    }
};

thread_local ALNSScratch g_alnsScratch;

bool alnsIteration(Route& r, std::mt19937& rng, AdaptiveWeights& aw,
                    int& outDestroy, int& outRepair) {
    if (g_numDel < 2) { return false; }

    g_alnsScratch.init();
    g_alnsScratch.clear();
    auto& removed = g_alnsScratch.removed;
    auto& removePOI = g_alnsScratch.removePOI;

    // Remove between 10% and 30% of deliveries in each iteration.
    int qMin = std::max(1, g_numDel / 10);
    int qMax = std::max(2, g_numDel * 3 / 10);
    std::uniform_int_distribution<int> qDist(qMin, qMax);
    int q = qDist(rng);

    int destroyMode = aw.selectDestroy(rng); // adaptive roulette picks the destroy operator
    outDestroy = destroyMode;

    switch (destroyMode) {
        case 0: destroyRandom(r, q, rng, removed); break;
        case 1: destroyWorst(r, q, rng, removed); break;
        case 2: destroyShaw(r, q, rng, removed); break;
        case 3: destroySegment(r, q, rng, removed); break;
        default: destroyRadial(r, q, rng, removed); break;
    }

    // Remove the selected deliveries from the route.
    for (int del : removed) {
        removePOI[pickupPOI(del)] = true;
        removePOI[dropoffPOI(del)] = true;
    }
    auto newEnd = std::remove_if(r.stops.begin(), r.stops.end(),
        [&](int poi) { return removePOI[poi]; });
    r.stops.erase(newEnd, r.stops.end());

    for (int del : removed) { // reset flags in O(q) instead of O(numPOI)
        removePOI[pickupPOI(del)] = false;
        removePOI[dropoffPOI(del)] = false;
    }

    int repairMode = aw.selectRepair(rng); // 0 = greedy, 1 = regret-2, 2 = regret-3
    outRepair = repairMode;
    if (repairMode == 0) {
        repairGreedy(r, removed, rng);
    } else if (repairMode == 1) {
        repairRegret2(r, removed, rng);
    } else {
        repairRegret3(r, removed, rng);
    }

    optimizeDepots(r);
    return true;
}

// Tracks the route position of each pickup and dropoff for O(1) lookups.
struct PosTracker {
    std::vector<int> pickup_pos;  // pickup_pos[del] = route index of delivery del's pickup
    std::vector<int> dropoff_pos; // dropoff_pos[del] = route index of delivery del's dropoff
    std::vector<int> poi_to_pos;  // poi_to_pos[poi] = route index, or -1 if not present

    void rebuild(const std::vector<int>& stops) {
        pickup_pos.assign(g_numDel, -1);
        dropoff_pos.assign(g_numDel, -1);
        poi_to_pos.assign(g_numPOI, -1);
        for (int i = 0; i < (int)stops.size(); ++i) {
            int poi = stops[i];
            poi_to_pos[poi] = i;
            if (isPOIPickup(poi)) {
                pickup_pos[poiDelivery(poi)] = i;
            } else if (isPOIDropoff(poi)) {
                dropoff_pos[poiDelivery(poi)] = i;
            }
        }
    }
};

// 2-Opt local search: tries reversing each segment [a..b] and keeps improving swaps.
// Uses steepest descent and skips segments that are too long or too expensive to be worth checking.
void twoOpt(Route& r, PosTracker& pos, double timeLimit) {
    const int n = (int)r.stops.size();
    if (n < 4) { return; }

    constexpr int MAX_SEG = 10; // maximum segment length to consider reversing

    // Skip moves between POIs that are already far apart (granular neighborhood).
    float avgCost = r.cost / (n + 1);
    (void)avgCost;

    auto node = [&](int i) -> int {
        if (i < 0) { return r.startDepot; }
        if (i >= n) { return r.endDepot; }
        return r.stops[i];
    };

    bool improved = true;
    int passCount = 0;
    while (improved) {
        if (++passCount % 3 == 0 && elapsed() > timeLimit) { break; }

        improved = false;
        float bestDelta = -1e-6f;
        int bestA = -1, bestB = -1;

        for (int a = 0; a < n - 1; ++a) {
            int bMax = std::min(n - 1, a + MAX_SEG);
            for (int b = a + 1; b <= bMax; ++b) {
                // Reject any reversal that would move a dropoff before its pickup.
                int poiB = r.stops[b];
                if (isPOIDropoff(poiB)) {
                    if (pos.pickup_pos[poiDelivery(poiB)] >= a) { break; }
                } else if (isPOIPickup(poiB)) {
                    int dp = pos.dropoff_pos[poiDelivery(poiB)];
                    if (dp >= a && dp < b) { break; }
                }

                int prev = node(a - 1), next = node(b + 1);
                float oldCost = tt(prev, r.stops[a]) + tt(r.stops[b], next);
                float newCost = tt(prev, r.stops[b]) + tt(r.stops[a], next);

                for (int i = a; i < b; ++i) {
                    oldCost += tt(r.stops[i], r.stops[i + 1]);
                }
                for (int i = b; i > a; --i) {
                    newCost += tt(r.stops[i], r.stops[i - 1]);
                }

                float delta = newCost - oldCost;
                if (delta < bestDelta) {
                    bestDelta = delta;
                    bestA = a;
                    bestB = b;
                }
            }
        }

        if (bestA >= 0) {
            std::reverse(r.stops.begin() + bestA, r.stops.begin() + bestB + 1);
            pos.rebuild(r.stops);
            r.cost += bestDelta;
            improved = true;
        }
    }
}

// Delivery relocate: tries removing each delivery and reinserting it at its best position.
void deliveryRelocate(Route& r, PosTracker& pos, double timeLimit) {
    const int n = (int)r.stops.size();
    if (n < 4) { return; }

    auto node = [&](int i) -> int {
        if (i < 0) { return r.startDepot; }
        if (i >= n) { return r.endDepot; }
        return r.stops[i];
    };

    bool improved = true;
    int passCount = 0;
    while (improved) {
        if (++passCount % 2 == 0 && elapsed() > timeLimit) { break; }

        improved = false;
        float bestDelta = -1e-6f;
        int bestDel = -1, bestP = -1, bestD = -1;

        for (int del = 0; del < g_numDel; ++del) {
            int pp = pos.pickup_pos[del];
            int dp = pos.dropoff_pos[del];
            if (pp < 0 || dp < 0) {
                continue;
            }

            int pkPOI = pickupPOI(del);
            int doPOI = dropoffPOI(del);

            // Compute how much travel time is saved by removing this delivery.
            float removal;
            if (dp == pp + 1) {
                // Pickup and dropoff are adjacent in the route.
                removal = tt(node(pp - 1), r.stops[pp])
                        + tt(r.stops[pp], r.stops[dp])
                        + tt(r.stops[dp], node(dp + 1))
                        - tt(node(pp - 1), node(dp + 1));
            } else {
                removal = tt(node(pp - 1), r.stops[pp])
                        + tt(r.stops[pp], node(pp + 1))
                        - tt(node(pp - 1), node(pp + 1))
                        + tt(node(dp - 1), r.stops[dp])
                        + tt(r.stops[dp], node(dp + 1))
                        - tt(node(dp - 1), node(dp + 1));
            }

            if (removal <= 0.0f) {
                continue;
            }

            // Build a temporary view of the route without this delivery.
            thread_local std::vector<int> reduced;
            reduced.clear();
            reduced.reserve(n);
            for (int i = 0; i < n; ++i) {
                if (i != pp && i != dp) { reduced.push_back(r.stops[i]); }
            }
            int rn = (int)reduced.size();

            auto redNode = [&](int i) -> int {
                if (i < 0) { return r.startDepot; }
                if (i >= rn) { return r.endDepot; }
                return reduced[i];
            };

            // Find the best reinsertion position using the O(N) suffix-minimum DP.
            float bestIns = INF_COST;
            int bip = -1, bid = -1;

            // Precompute non-adjacent dropoff costs.
            thread_local std::vector<float> relCostDO, relSuffMin;
            thread_local std::vector<int> relSuffBestD;
            relCostDO.assign(rn + 2, INF_COST);
            for (int d = 2; d <= rn + 1; ++d) {
                relCostDO[d] = tt(redNode(d - 2), doPOI) + tt(doPOI, redNode(d - 1))
                             - tt(redNode(d - 2), redNode(d - 1));
            }

            // Build suffix minimums.
            relSuffMin.assign(rn + 2, INF_COST);
            relSuffBestD.assign(rn + 2, -1);
            float rMin = INF_COST;
            int rBestD = -1;
            for (int d = rn + 1; d >= 2; --d) {
                if (relCostDO[d] < rMin) {
                    rMin   = relCostDO[d];
                    rBestD = d;
                }
                if (d - 2 >= 0) {
                    relSuffMin[d - 2]   = rMin;
                    relSuffBestD[d - 2] = rBestD;
                }
            }

            // Scan pickup positions in O(N).
            for (int p = 0; p <= rn; ++p) {
                float dp_cost = tt(redNode(p - 1), pkPOI) + tt(pkPOI, redNode(p))
                              - tt(redNode(p - 1), redNode(p));
                if (dp_cost >= INF_COST || dp_cost >= bestIns) {
                    continue;
                }

                // Case A: adjacent dropoff
                float adjCost  = tt(pkPOI, doPOI) + tt(doPOI, redNode(p)) - tt(pkPOI, redNode(p));
                float totalAdj = dp_cost + adjCost;
                if (totalAdj < bestIns) {
                    bestIns = totalAdj;
                    bip = p;
                    bid = p + 1;
                }

                // Case B: non-adjacent dropoff using suffix minimum
                if (p <= rn - 1 && relSuffMin[p] < INF_COST) {
                    float totalSuff = dp_cost + relSuffMin[p];
                    if (totalSuff < bestIns) {
                        bestIns = totalSuff;
                        bip = p;
                        bid = relSuffBestD[p];
                    }
                }
            }

            if (bestIns >= INF_COST) {
                continue;
            }
            float netDelta = bestIns - removal;
            if (netDelta < bestDelta) {
                bestDelta = netDelta;
                bestDel = del; bestP = bip; bestD = bid;

                // First improvement: apply immediately if meaningfully better
                if (bestDelta < -1e-4f) { break; } // first-improvement early exit
            }
        }

        if (bestDel >= 0) {
            int pp = pos.pickup_pos[bestDel];
            int dp = pos.dropoff_pos[bestDel];

            // Remove higher index first to avoid shifting the lower one.
            int hi = std::max(pp, dp), lo = std::min(pp, dp);
            r.stops.erase(r.stops.begin() + hi);
            r.stops.erase(r.stops.begin() + lo);

            r.stops.insert(r.stops.begin() + bestP, pickupPOI(bestDel));
            r.stops.insert(r.stops.begin() + bestD, dropoffPOI(bestDel));

            pos.rebuild(r.stops);
            r.cost += bestDelta;
            improved = true;
        }
    }
}

// Pair exchange: tries swapping the pickup or dropoff of delivery A with the corresponding
// stop of delivery B. This explores moves that single-delivery relocate cannot reach.
void pairExchange(Route& r, PosTracker& pos, double timeLimit) {
    if (g_numDel < 2) { return; }
    const int n = (int)r.stops.size();

    auto node = [&](int i) -> int {
        if (i < 0) { return r.startDepot; }
        if (i >= n) { return r.endDepot; }
        return r.stops[i];
    };

    float peAvgCost   = r.cost / (n + 1);
    float peThreshold = peAvgCost * 2.0f; // skip pairs that are too far apart

    bool improved = true;
    int passCount = 0;
    while (improved) {
        if (++passCount % 2 == 0 && elapsed() > timeLimit) { break; }

        improved = false;
        float bestDelta = -1e-6f;
        int bestA = -1, bestB = -1;
        int bestMode = 0; // 0 = swap pickups, 1 = swap dropoffs

        // Returns the travel-time delta from swapping the stops at posX and posY.
        auto swapDelta = [&](int posX, int posY) -> float {
            if (posX == posY) { return 0.0f; }
            int poiX = r.stops[posX], poiY = r.stops[posY];
            if (std::abs(posX - posY) == 1) {
                int lo = std::min(posX, posY), hi = std::max(posX, posY);
                float old_c = tt(node(lo-1), r.stops[lo]) + tt(r.stops[lo], r.stops[hi])
                            + tt(r.stops[hi], node(hi+1));
                int newLo = (lo == posX) ? poiY : poiX;
                int newHi = (hi == posX) ? poiY : poiX;
                float new_c = tt(node(lo-1), newLo) + tt(newLo, newHi)
                            + tt(newHi, node(hi+1));
                return new_c - old_c;
            }
            float old_c = tt(node(posX-1), poiX) + tt(poiX, node(posX+1))
                        + tt(node(posY-1), poiY) + tt(poiY, node(posY+1));
            float new_c = tt(node(posX-1), poiY) + tt(poiY, node(posX+1))
                        + tt(node(posY-1), poiX) + tt(poiX, node(posY+1));
            return new_c - old_c;
        };

        for (int a = 0; a < g_numDel; ++a) {
            int ppA = pos.pickup_pos[a], dpA = pos.dropoff_pos[a];
            if (ppA < 0) {
                continue;
            }

            for (int b = a + 1; b < g_numDel; ++b) {
                int ppB = pos.pickup_pos[b], dpB = pos.dropoff_pos[b];
                if (ppB < 0) {
                    continue;
                }

                if (tt(pickupPOI(a), pickupPOI(b)) > peThreshold) {
                    continue; // skip distant pairs
                }

                // Swap pickups: valid when each pickup still comes before its own dropoff.
                if (ppB < dpA && ppA < dpB) {
                    float delta = swapDelta(ppA, ppB);
                    if (delta < bestDelta) {
                        bestDelta = delta;
                        bestA = a; bestB = b; bestMode = 0;
                    }
                }

                // Swap dropoffs: valid when each pickup still comes before its own dropoff.
                if (ppA < dpB && ppB < dpA) {
                    float delta = swapDelta(dpA, dpB);
                    if (delta < bestDelta) {
                        bestDelta = delta;
                        bestA = a; bestB = b; bestMode = 1;
                    }
                }
            }
        }

        if (bestA >= 0) {
            if (bestMode == 0) {
                std::swap(r.stops[pos.pickup_pos[bestA]], r.stops[pos.pickup_pos[bestB]]);
                std::swap(pos.pickup_pos[bestA], pos.pickup_pos[bestB]);
            } else {
                std::swap(r.stops[pos.dropoff_pos[bestA]], r.stops[pos.dropoff_pos[bestB]]);
                std::swap(pos.dropoff_pos[bestA], pos.dropoff_pos[bestB]);
            }
            r.cost += bestDelta;
            improved = true;
        }
    }
}

// Or-Opt: moves chains of 1-3 consecutive POIs to a better position in the route.
void orOpt(Route& r, PosTracker& pos, double timeLimit) {
    const int n = (int)r.stops.size();
    if (n < 3) { return; }

    auto node = [&](int i) -> int {
        if (i < 0) { return r.startDepot; }
        if (i >= n) { return r.endDepot; }
        return r.stops[i];
    };

    bool improved = true;
    int passCount = 0;
    while (improved) {
        if (++passCount % 2 == 0 && elapsed() > timeLimit) { break; }

        improved = false;
        float bestDelta = -1e-6f;
        int bestI = -1, bestJ = -1, bestK = -1;

        for (int k = 1; k <= 3; ++k) {
            for (int i = 0; i + k - 1 < n; ++i) {
                int chainEnd = i + k - 1;
                int firstC = r.stops[i];
                int lastC  = r.stops[chainEnd];
                int prevI  = node(i - 1);
                int nextCE = node(chainEnd + 1);

                float skipEdge = tt(prevI, nextCE); // travel time if the chain is removed
                if (skipEdge >= INF_COST) { continue; }
                float remDelta = skipEdge - tt(prevI, firstC) - tt(lastC, nextCE); // savings

                for (int j = 0; j <= n - k; ++j) {
                    if (j >= i && j <= chainEnd) {
                        continue;
                    }

                    int prevJ, nextJ;
                    if (j < i) {
                        prevJ = node(j - 1);
                        nextJ = node(j);
                    } else {
                        prevJ = node(j + k - 1);
                        nextJ = node(j + k);
                    }

                    float cf = tt(prevJ, firstC);
                    float cl = tt(lastC, nextJ);
                    if (cf >= INF_COST || cl >= INF_COST) {
                        continue;
                    }
                    float insDelta   = cf + cl - tt(prevJ, nextJ); // cost to insert chain at j
                    float totalDelta = remDelta + insDelta;

                    if (totalDelta >= bestDelta) {
                        continue;
                    }

                    // Simulate where the chain ends up and verify precedence still holds.
                    int insPos = (j < i) ? j : j - k;
                    bool legal = true;
                    for (int c = i; c <= chainEnd && legal; ++c) {
                        int poi = r.stops[c];
                        int del = poiDelivery(poi);
                        int newCPos = insPos + (c - i);

                        if (isPOIPickup(poi)) {
                            int dpos = pos.dropoff_pos[del];
                            if (dpos >= i && dpos <= chainEnd) {
                                continue;
                            }
                            int newDP = (dpos < i) ? dpos : dpos - k;
                            if (newDP >= insPos) {
                                newDP += k;
                            }
                            if (newCPos >= newDP) {
                                legal = false;
                            }
                        } else if (isPOIDropoff(poi)) {
                            int ppos = pos.pickup_pos[del];
                            if (ppos >= i && ppos <= chainEnd) {
                                continue;
                            }
                            int newPP = (ppos < i) ? ppos : ppos - k;
                            if (newPP >= insPos) {
                                newPP += k;
                            }
                            if (newPP >= newCPos) {
                                legal = false;
                            }
                        }
                    }
                    if (!legal) {
                        continue;
                    }

                    bestDelta = totalDelta;
                    bestI = i; bestJ = j; bestK = k;
                }
            }
        }

        if (bestI >= 0) {
            std::vector<int> chain(r.stops.begin() + bestI,
                                   r.stops.begin() + bestI + bestK);
            r.stops.erase(r.stops.begin() + bestI,
                          r.stops.begin() + bestI + bestK);
            int insPos = (bestJ < bestI) ? bestJ : bestJ - bestK;
            r.stops.insert(r.stops.begin() + insPos,
                           chain.begin(), chain.end());
            pos.rebuild(r.stops);
            r.cost += bestDelta;
            improved = true;
        }
    }
}

// Runs all local search operators in sequence until no operator finds improvement.
void localSearch(Route& r, double timeLimit) {
    PosTracker pos;
    pos.rebuild(r.stops);

    bool any = true;
    while (any && elapsed() < timeLimit) {
        any = false;
        float before;

        before = r.cost;
        twoOpt(r, pos, timeLimit);
        if (r.cost < before - 1e-6f) {
            any = true;
        }

        if (elapsed() > timeLimit) {
            break;
        }

        before = r.cost;
        deliveryRelocate(r, pos, timeLimit);
        if (r.cost < before - 1e-6f) {
            any = true;
        }

        if (elapsed() > timeLimit) {
            break;
        }

        before = r.cost;
        orOpt(r, pos, timeLimit);
        if (r.cost < before - 1e-6f) {
            any = true;
        }

        if (elapsed() > timeLimit) {
            break;
        }

        before = r.cost;
        pairExchange(r, pos, timeLimit);
        if (r.cost < before - 1e-6f) {
            any = true;
        }

        if (elapsed() > timeLimit) {
            break;
        }
        
        optimizeDepots(r);
        r.calcCost();
    }
}

// Perturbation operators for simulated annealing

// Double-bridge (4-opt) perturbation: splits the route into segments A-B-C-D and
// reconnects as A-C-B-D. This move cannot be reached by 2-opt or 3-opt alone,
// making it the standard way to escape deep local optima.
bool perturbDoubleBridge(Route& r, std::mt19937& rng) {
    const int n = (int)r.stops.size();
    if (n < 8) { return false; }

    // Pick 3 sorted cut points to define the 4 segments.
    std::uniform_int_distribution<int> dist(1, n - 2);
    int cuts[3] = {dist(rng), dist(rng), dist(rng)};
    std::sort(cuts, cuts + 3);
    if (cuts[0] == cuts[1] || cuts[1] == cuts[2]) {
        return false;
    }

    // Build the reconnected sequence A + C + B + D.
    std::vector<int> newStops;
    newStops.reserve(n);
    for (int i = 0; i < cuts[0]; ++i) {
        newStops.push_back(r.stops[i]); // A
    }
    for (int i = cuts[1]; i < cuts[2]; ++i) {
        newStops.push_back(r.stops[i]); // C
    }
    for (int i = cuts[0]; i < cuts[1]; ++i) {
        newStops.push_back(r.stops[i]); // B
    }
    for (int i = cuts[2]; i < n; ++i) {
        newStops.push_back(r.stops[i]); // D
    }

    // Verify precedence: reject the perturbation if any dropoff now precedes its pickup.
    thread_local std::vector<int> pkPos;
    pkPos.assign(g_numDel, -1);
    for (int i = 0; i < n; ++i) {
        int poi = newStops[i];
        if (isPOIPickup(poi)) {
            pkPos[poiDelivery(poi)] = i;
        } else if (isPOIDropoff(poi)) {
            if (pkPos[poiDelivery(poi)] < 0 || pkPos[poiDelivery(poi)] > i) {
                return false;
            }
        }
    }

    r.stops = std::move(newStops);
    r.calcCost();
    return true;
}

// Swaps two random POIs. Only checks legality for the two affected deliveries.
bool perturbSwap(Route& r, std::mt19937& rng) {
    const int n = (int)r.stops.size();
    if (n < 2) { return false; }

    std::uniform_int_distribution<int> dist(0, n - 1);
    int a = dist(rng), b = dist(rng);
    if (a == b) {
        return false;
    }
    if (a > b) {
        std::swap(a, b);
    }

    std::swap(r.stops[a], r.stops[b]);

    // Only verify the two affected deliveries rather than scanning the full route.
    for (int idx : {a, b}) {
        int poi = r.stops[idx];
        if (poi >= 2 * g_numDel) { continue; } // skip if this is a depot POI
        int del  = poiDelivery(poi);
        int pk   = pickupPOI(del);
        int doff = dropoffPOI(del);
        int pkPos = -1, doPos = -1;
        for (int i = 0; i < n; ++i) {
            if (r.stops[i] == pk) {
                pkPos = i;
            }
            if (r.stops[i] == doff) {
                doPos = i;
            }
        }
        if (pkPos > doPos) {
            std::swap(r.stops[a], r.stops[b]); // undo the swap
            return false;
        }
    }

    r.calcCost();
    return true;
}

// Relocates a randomly chosen delivery to a random valid position.
bool perturbRelocate(Route& r, std::mt19937& rng) {
    if (g_numDel < 1) { return false; }

    std::uniform_int_distribution<int> delDist(0, g_numDel - 1);
    int del = delDist(rng);
    int pkPOI = pickupPOI(del);
    int doPOI = dropoffPOI(del);

    int pp = -1, dp = -1; // current route positions of pickup and dropoff
    for (int i = 0; i < (int)r.stops.size(); ++i) {
        if (r.stops[i] == pkPOI) {
            pp = i;
        }
        if (r.stops[i] == doPOI) {
            dp = i;
        }
    }
    if (pp < 0 || dp < 0) {
        return false;
    }

    // Erase higher index first to avoid shifting the lower one.
    int hi = std::max(pp, dp), lo = std::min(pp, dp);
    r.stops.erase(r.stops.begin() + hi);
    r.stops.erase(r.stops.begin() + lo);

    int rn = (int)r.stops.size();
    std::uniform_int_distribution<int> posDist(0, rn);
    int newP = posDist(rng);
    r.stops.insert(r.stops.begin() + newP, pkPOI);

    std::uniform_int_distribution<int> dposDist(newP + 1, rn + 1);
    int newD = dposDist(rng);
    r.stops.insert(r.stops.begin() + newD, doPOI);

    r.calcCost();
    return true;
}

// Global best solution shared across all island threads.
std::atomic<float> g_globalBestCost{INF_COST};
std::mutex         g_globalMutex;
Route              g_globalBestRoute;

// Updates the global best if this route is cheaper, using double-checked locking.
void publishIfBetter(const Route& r) {
    if (r.cost < g_globalBestCost.load(std::memory_order_relaxed)) {
        std::lock_guard<std::mutex> lock(g_globalMutex);
        if (r.cost < g_globalBestCost.load(std::memory_order_relaxed)) {
            g_globalBestCost.store(r.cost, std::memory_order_relaxed);
            g_globalBestRoute = r;
        }
    }
}

// Main optimization loop for one island thread.
// Each thread builds its own initial solution, runs SA + ALNS, and shares improvements globally.
void optimizationWorker(int threadId) {
    std::mt19937 rng(static_cast<unsigned>(threadId * 7919 + 42));

    // Thread 0: best greedy NN across all depots.
    // Threads 1-2: regret-2 insertion for higher quality starts (thread 2 adds noise for diversity).
    // Threads 3-7: randomized greedy for maximum diversity.
    Route current;
    std::uniform_int_distribution<int> dd(0, g_numDepots - 1);

    if (threadId == 0) {
        Route best;
        best.cost = INF_COST;
        for (int d = 0; d < g_numDepots; ++d) {
            Route r = greedyNN(d, rng);
            if (r.cost < best.cost) {
                best = r;
            }
        }
        current = best;
    } else if (threadId <= 2) {
        float noise = (threadId == 1) ? 0.0f : 0.05f * 50000.0f; // light noise on thread 2
        current = regret2Init(dd(rng), rng, noise);
    } else {
        current = randomizedGreedy(dd(rng), rng);
    }

    if (current.cost >= INF_COST || current.stops.empty()) {
        return;
    }

    // Initial local search before entering SA.
    localSearch(current, std::min(elapsed() + 3.0, HARD_TIME_LIMIT));
    optimizeDepots(current);
    current.calcCost();

    Route localBest = current;
    publishIfBetter(localBest);

    // SA initial temperature varies by island for diversity.
    float tempPct = (threadId < 4) ? 0.03f : 0.05f;
    float T0 = tempPct * current.cost / std::log(2.0f);
    float T  = T0;
    int itersSinceImprove = 0;

    AdaptiveWeights aw;
    aw.init();

    Route candidate;
    int iteration  = 0;
    int lastLSIter = -100; // allow local search from the very first improvement

    while (elapsed() < HARD_TIME_LIMIT) {
        ++iteration;

        double frac = std::min(elapsed() / HARD_TIME_LIMIT, 1.0); // normalized time [0,1]

        // Cool the temperature and update operator weights every 100 iterations.
        if (iteration % 100 == 0) {
            T = T0 * static_cast<float>(std::pow(0.001, frac));
            if (iteration > 0) {
                aw.updateWeights();
            }
        }

        candidate = current;
        bool valid = false;
        int dOp = -1, rOp = -1;

        std::uniform_int_distribution<int> modeDist(0, 9);
        int mode = modeDist(rng);

        // 70% ALNS, 10% double-bridge, 10% swap, 10% relocate.
        if (mode <= 6) {
            valid = alnsIteration(candidate, rng, aw, dOp, rOp);
        } else if (mode == 7) {
            valid = perturbDoubleBridge(candidate, rng);
        } else if (mode == 8) {
            valid = perturbSwap(candidate, rng);
        } else {
            valid = perturbRelocate(candidate, rng);
        }

        if (!valid) {
            continue;
        }

        // Metropolis acceptance: always accept improvements, sometimes accept worse.
        float prevGlobal = g_globalBestCost.load(std::memory_order_relaxed);
        float delta = candidate.cost - current.cost;
        bool accept = false;
        if (delta <= 0) {
            accept = true;
        } else if (T > 0) {
            std::uniform_real_distribution<float> ud(0.0f, 1.0f);
            accept = ud(rng) < std::exp(-delta / T);
        }

        if (accept) {
            current = candidate;
            bool isLocalBest = false;
            if (current.cost < localBest.cost) {
                // Polish the new local best with a short local search (at most once per 100 iters).
                if (iteration - lastLSIter >= 100) {
                    localSearch(current, std::min(elapsed() + 0.15, HARD_TIME_LIMIT));
                    optimizeDepots(current);
                    current.calcCost();
                    lastLSIter = iteration;
                }

                localBest = current;
                publishIfBetter(localBest);
                itersSinceImprove = 0;
                isLocalBest = true;
            } else {
                ++itersSinceImprove;
            }

            // Reward the ALNS operator pair based on how good the outcome was.
            if (dOp >= 0) {
                if (current.cost < prevGlobal) {
                    aw.record(dOp, rOp, 33.0f); // beat the global best
                } else if (isLocalBest) {
                    aw.record(dOp, rOp, 13.0f); // new local best
                } else if (delta > 0) {
                    aw.record(dOp, rOp, 9.0f);  // accepted worse solution (exploration)
                }
            }
        } else {
            ++itersSinceImprove;
        }

        // Reheat when stuck. Threshold scales with iteration count so it adapts to problem size.
        int reheatThreshold = std::max(100, std::min(2000, iteration / 5));
        if (itersSinceImprove >= reheatThreshold) {
            T = T0 * 0.5f;
            itersSinceImprove = 0;
        }

        // Cross-island migration every 500 iterations. If this island is 5% worse than the global
        // best, import the global best and apply a heavy perturbation to escape the basin.
        if (iteration % 500 == 0) {
            float gb = g_globalBestCost.load(std::memory_order_relaxed);
            if (localBest.cost > gb * 1.05f) {
                {
                    std::lock_guard<std::mutex> lock(g_globalMutex);
                    current = g_globalBestRoute;
                }
                // Destroy ~35% of the imported solution to diversify.
                int heavyQ = std::max(2, g_numDel * 35 / 100);
                g_alnsScratch.init();
                g_alnsScratch.clear();
                destroyRadial(current, heavyQ, rng, g_alnsScratch.removed);
                for (int del : g_alnsScratch.removed) {
                    g_alnsScratch.removePOI[pickupPOI(del)] = true;
                    g_alnsScratch.removePOI[dropoffPOI(del)] = true;
                }
                auto ne = std::remove_if(current.stops.begin(), current.stops.end(),
                    [](int poi) { return g_alnsScratch.removePOI[poi]; });
                current.stops.erase(ne, current.stops.end());
                for (int del : g_alnsScratch.removed) {
                    g_alnsScratch.removePOI[pickupPOI(del)] = false;
                    g_alnsScratch.removePOI[dropoffPOI(del)] = false;
                }
                repairRegret2(current, g_alnsScratch.removed, rng);
                optimizeDepots(current);

                localBest = current;
            }
        }
    }

    publishIfBetter(localBest);
    if (threadId == 0) {
        std::cerr << "[M4] thread0 iters=" << iteration << std::endl;
    }
}

// Reconstructs the final sequence of CourierSubPaths from the optimized route
// by calling M3's A* for each leg.
std::vector<CourierSubPath> buildResult(const Route& r) {
    std::vector<CourierSubPath> result;
    if (r.stops.empty()) {
        return result;
    }

    const int n = (int)r.stops.size();
    result.reserve(n + 1);

    auto makeLeg = [&](int srcPOI, int dstPOI) {
        CourierSubPath leg;
        IntersectionIdx src = g_poiIntersection[srcPOI];
        IntersectionIdx dst = g_poiIntersection[dstPOI];
        leg.intersections = {src, dst};
        if (src != dst) {
            leg.subpath = findPathBetweenIntersections(
                static_cast<double>(g_turnPenalty), {src, dst});
        }
        return leg;
    };

    result.push_back(makeLeg(r.startDepot, r.stops[0])); // depot to first stop

    for (int i = 0; i + 1 < n; ++i) {
        result.push_back(makeLeg(r.stops[i], r.stops[i + 1])); // stop to stop
    }

    result.push_back(makeLeg(r.stops[n - 1], r.endDepot)); // last stop to depot

    return result;
}

} // anonymous namespace


// Public API

std::vector<CourierSubPath> travelingCourier(
        const float turn_penalty,
        const std::vector<DeliveryInf>& deliveries,
        const std::vector<IntersectionIdx>& depots) {

    if (deliveries.empty() || depots.empty()) return {};

    g_startTime    = std::chrono::high_resolution_clock::now();
    g_turnPenalty  = turn_penalty;

    // Scale the time budget down for small problems so multi-call test cases stay within timeout.
    const int numDel = static_cast<int>(deliveries.size());
    if (numDel <= 5) {
        HARD_TIME_LIMIT  = 5.0;
        FINAL_TIME_LIMIT = 6.0;
    } else if (numDel <= 25) {
        HARD_TIME_LIMIT  = 20.0;
        FINAL_TIME_LIMIT = 22.0;
    } else if (numDel <= 50) {
        HARD_TIME_LIMIT  = 35.0;
        FINAL_TIME_LIMIT = 38.0;
    } else {
        HARD_TIME_LIMIT  = 46.5;
        FINAL_TIME_LIMIT = 48.5;
    }

    // Phase 1: build POI index and precompute the full travel-time matrix.
    initPOIs(deliveries, depots);
    precomputeMatrix(turn_penalty);
    buildNearestNeighbors();

    double t_precomp = elapsed();
    std::cerr << "[M4] deliveries=" << numDel << " POIs=" << g_numPOI
              << " precomp=" << t_precomp << "s" << std::endl;

    // Phase 2: island model optimization — each thread runs SA + ALNS independently.
    g_globalBestCost.store(INF_COST, std::memory_order_relaxed);
    g_globalBestRoute = Route();

    const int numIslands = std::min(NUM_THREADS, std::max(1, g_numDel));
    std::vector<std::thread> islands;
    islands.reserve(numIslands);

    for (int t = 0; t < numIslands; ++t) {
        islands.emplace_back(optimizationWorker, t);
    }

    for (auto& th : islands) { th.join(); }

    double t_optim = elapsed();
    std::cerr << "[M4] optimization=" << (t_optim - t_precomp) << "s"
              << " total=" << t_optim << "s"
              << " cost=" << g_globalBestCost.load() << std::endl;

    // Phase 2.5: run a final local search pass on the global best using any time left.
    if (g_globalBestCost.load() < INF_COST &&
        !g_globalBestRoute.stops.empty()) {

        Route finalRoute = g_globalBestRoute;
        localSearch(finalRoute, FINAL_TIME_LIMIT);
        optimizeDepots(finalRoute);
        finalRoute.calcCost();

        if (finalRoute.cost < g_globalBestCost.load()) {
            g_globalBestRoute = finalRoute;
        }
    }

    // Phase 3: reconstruct the actual street-level paths.
    if (g_globalBestCost.load() >= INF_COST ||
        g_globalBestRoute.stops.empty()) {
        return {};
    }

    return buildResult(g_globalBestRoute);
}