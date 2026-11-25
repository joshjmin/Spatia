/*
 * Copyright 2026 University of Toronto
 *
 * Permission is hereby granted, to use this software and associated
 * documentation files (the "Software") in course work at the University
 * of Toronto, or for personal use. Other uses are prohibited, in
 * particular the distribution of the Software either publicly or to third
 * parties.
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <algorithm>
#include <limits>
#include <set>
#include <map>

#include "m1.h"
#include "StreetsDatabaseAPI.h"
#include "OSMDatabaseAPI.h"
#include "globals.h"

// loadMap will be called with the name of the file that stores the "layer-2"
// map data accessed through StreetsDatabaseAPI: the street and intersection
// data that is higher-level than the raw OSM data).
// This file name will always end in ".streets.bin" and you
// can call loadStreetsDatabaseBIN with this filename to initialize the
// layer 2 (StreetsDatabase) API.
// If you need data from the lower level, layer 1, API that provides raw OSM
// data (nodes, ways, etc.) you will also need to initialize the layer 1
// OSMDatabaseAPI by calling loadOSMDatabaseBIN. That function needs the
// name of the ".osm.bin" file that matches your map -- just change
// ".streets" to ".osm" in the filename to get the proper name.

/* To hit the "High" speed targets in the autotester, we're pre-processing
 * most of the map data into these caches during loadMap.
 *
 * We're basically doing four things:
 * 1. Using a 2D grid (geo-bins) for spatial stuff so we don't have to scan every intersection.
 * 2. Storing everything in hash maps by ID or name so lookups are O(1).
 * 3. Using a sorted multimap for street names so we can binary search the prefixes.
 * 4. Using a global checklist for intersection searches to avoid slow memory allocations.
 */

// M1-private data structures (not shared with m2.cpp)
namespace {
    // Maps a POI name to all POI indices with that name
    std::unordered_map<std::string, std::vector<POIIdx>> poiNameToIndices;

    // Pre-allocated checklist for intersection IDs
    std::vector<bool> intersectionCheckFlag;

    // Cached lengths of all OSM ways in meters
    std::unordered_map<OSMID, double> wayLengthCache;

    // Geo-binning data structures for fast closest-intersection queries
    std::vector<std::vector<std::vector<IntersectionIdx>>> geoBins;
    int numBinsLat = 0;
    int numBinsLon = 0;
    double latBinSize = 0.0, lonBinSize = 0.0;
    double binDiagonalMeters = 0.0;
    double minBinSideMeters = 0.0;

    // Normalize a street name by removing spaces and converting to lowercase
    std::string processStreetName(const std::string& name) {
        std::string processed;
        for (size_t i = 0; i < name.length(); i++) {
            if (name[i] != ' ') {
                processed += tolower(name[i]);
            }
        }
        return processed;
    }

    // Helper function to update bounding box with a new point
    void updateBoundsHelper(double& minLat, double& maxLat,
                            double& minLon, double& maxLon,
                            const LatLon& point) {
        minLat = std::min(minLat, point.latitude());
        maxLat = std::max(maxLat, point.latitude());
        minLon = std::min(minLon, point.longitude());
        maxLon = std::max(maxLon, point.longitude());
    }

    // Builds hash map of all OSM nodes by ID for O(1) lookup
    void buildOSMNodeMap() {
        int numNodes = getNumberOfNodes();
        for (int i = 0; i < numNodes; i++) {
            const OSMNode* node = getNodeByIndex(i);
            osmNodeById[node->id()] = node;
        }
    }

    // Builds hash map of all OSM ways by ID for O(1) lookup
    void buildOSMWayMap() {
        int numWays = getNumberOfWays();
        for (int i = 0; i < numWays; i++) {
            const OSMWay* way = getWayByIndex(i);
            osmWayMap[way->id()] = way;
        }
    }

    // Precomputes lengths of all OSM ways by summing distances between consecutive nodes
    void precomputeWayLengths() {
        for (const auto& [wayId, way] : osmWayMap) {
            const std::vector<OSMID>& wayNodes = getWayMembers(way);

            // Ways with fewer than 2 nodes have no length
            if (wayNodes.size() < 2) {
                wayLengthCache[wayId] = 0.0;
                continue;
            }

            // Sum distances between consecutive nodes
            double totalLength = 0.0;
            for (size_t i = 0; i < wayNodes.size() - 1; i++) {
                auto it1 = osmNodeById.find(wayNodes[i]);
                auto it2 = osmNodeById.find(wayNodes[i + 1]);

                // Both nodes must exist to calculate distance
                if (it1 != osmNodeById.end() && it2 != osmNodeById.end()) {
                    LatLon pos1 = getNodeCoords(it1->second);
                    LatLon pos2 = getNodeCoords(it2->second);
                    totalLength += findDistanceBetweenTwoPoints({pos1, pos2});
                }
            }

            wayLengthCache[wayId] = totalLength;
        }
    }
}

bool loadMap(std::string mapStreetsDatabaseFilename) {
    binDiagonalMeters = 0.0;
    cosLatAvg = 1.0;
    minBinSideMeters = 0.0;
    std::cout << "loadMap: " << mapStreetsDatabaseFilename << std::endl;

    // Load streets database
    bool loadSuccessful = loadStreetsDatabaseBIN(mapStreetsDatabaseFilename);
    if (!loadSuccessful) {
        return false;
    }

    // Derive the corresponding OSM database filename
    std::string osmDatabaseFilename = mapStreetsDatabaseFilename;
    size_t pos = osmDatabaseFilename.find(".streets.bin");
    if (pos != std::string::npos) {
        osmDatabaseFilename.replace(pos, 12, ".osm.bin");
    }

    // Load OSM database; clean up if it fails
    if (!loadOSMDatabaseBIN(osmDatabaseFilename)) {
        closeStreetDatabase();
        return false;
    }

    // Cache database sizes
    int numIntersections = getNumIntersections();
    int numStreets = getNumStreets();
    int numStreetSegments = getNumStreetSegments();

    // Initialize the flag vector to all false
    intersectionCheckFlag.assign(numIntersections, false);

    // Preallocate data structures for fast access
    streetSegmentsByStreet.resize(numStreets);
    streetSegmentsByIntersection.resize(numIntersections);
    intersectionsByStreet.resize(numStreets);
    segmentLengths.resize(numStreetSegments);
    segmentTravelTimes.resize(numStreetSegments);
    segmentCache.resize(numStreetSegments);
    intersectionPositions.resize(numIntersections);
    maxSpeedLimit = 0.0;

    // Cache all intersection positions for fast spatial queries
    for (IntersectionIdx i = 0; i < numIntersections; ++i) {
        intersectionPositions[i] = getIntersectionPosition(i);
    }

    // Used to ensure intersections on a street are unique
    std::vector<std::unordered_set<IntersectionIdx>> uniqueIntersectionsPerStreet(numStreets);

    // Populate segment-based lookup tables and compute segment metrics
    for (StreetSegmentIdx segIdx = 0; segIdx < numStreetSegments; ++segIdx) {
        StreetSegmentInfo segInfo = getStreetSegmentInfo(segIdx);

        // Associate segment with its street
        streetSegmentsByStreet[segInfo.streetID].push_back(segIdx);

        // Associate segment with its endpoint intersections
        streetSegmentsByIntersection[segInfo.from].push_back(segIdx);
        if (segInfo.from != segInfo.to) {
            streetSegmentsByIntersection[segInfo.to].push_back(segIdx);
        }

        // Track unique intersections per street
        uniqueIntersectionsPerStreet[segInfo.streetID].insert(segInfo.from);
        uniqueIntersectionsPerStreet[segInfo.streetID].insert(segInfo.to);

        // Compute segment length including curve points
        double length = 0.0;
        LatLon prevPoint = getIntersectionPosition(segInfo.from);

        for (int curveIdx = 0; curveIdx < segInfo.numCurvePoints; ++curveIdx) {
            LatLon curvePoint = getStreetSegmentCurvePoint(segIdx, curveIdx);
            length += findDistanceBetweenTwoPoints({prevPoint, curvePoint});
            prevPoint = curvePoint;
        }

        LatLon toPoint = getIntersectionPosition(segInfo.to);
        length += findDistanceBetweenTwoPoints({prevPoint, toPoint});

        // Cache length, travel time, and segment info for fast A* access
        segmentLengths[segIdx] = length;
        segmentTravelTimes[segIdx] = length / segInfo.speedLimit;
        segmentCache[segIdx] = {segInfo.from, segInfo.to, segInfo.streetID, segInfo.oneWay};
        maxSpeedLimit = std::max(maxSpeedLimit, static_cast<double>(segInfo.speedLimit));
    }

    // Convert unique intersection sets into vectors
    for (StreetIdx streetIdx = 0; streetIdx < numStreets; ++streetIdx) {
        intersectionsByStreet[streetIdx].assign(
            uniqueIntersectionsPerStreet[streetIdx].begin(),
            uniqueIntersectionsPerStreet[streetIdx].end()
        );
    }

    // Build lookup from processed street names to street indices
    for (StreetIdx streetIdx = 0; streetIdx < numStreets; ++streetIdx) {
        std::string processedName = processStreetName(getStreetName(streetIdx));
        streetNameToIndicesSorted.insert({processedName, streetIdx});
    }

    // Build lookup from POI names to POI indices
    int numPOIs = getNumPointsOfInterest();
    for (POIIdx poiIdx = 0; poiIdx < numPOIs; ++poiIdx) {
        std::string poiName = getPOIName(poiIdx);
        poiNameToIndices[poiName].push_back(poiIdx);
    }

    // Initialize geo-binning for fast closest-intersection queries
    if (numIntersections > 0) {
        // Find map bounds from cached intersection positions
        geoMinLat = intersectionPositions[0].latitude();
        geoMaxLat = intersectionPositions[0].latitude();
        geoMinLon = intersectionPositions[0].longitude();
        geoMaxLon = intersectionPositions[0].longitude();

        for (int i = 1; i < numIntersections; ++i) {
            double lat = intersectionPositions[i].latitude();
            double lon = intersectionPositions[i].longitude();
            geoMinLat = std::min(geoMinLat, lat);
            geoMaxLat = std::max(geoMaxLat, lat);
            geoMinLon = std::min(geoMinLon, lon);
            geoMaxLon = std::max(geoMaxLon, lon);
        }

        // Create bins (100x100 grid for good balance of speed vs memory)
        numBinsLat = 100;
        numBinsLon = 100;

        // Calculate bin sizes with small padding to avoid edge cases
        double latRange = geoMaxLat - geoMinLat;
        double lonRange = geoMaxLon - geoMinLon;
        latBinSize = (latRange + 1e-9) / numBinsLat;
        lonBinSize = (lonRange + 1e-9) / numBinsLon;

        // Pre-compute values for fast spatial queries
        double latAvgRad = ((geoMinLat + geoMaxLat) / 2.0) * kDegreeToRadian;
        cosLatAvg = std::cos(latAvgRad);

        double latBinMeters = latBinSize * kDegreeToRadian * kEarthRadiusInMeters;
        double lonBinMeters = lonBinSize * kDegreeToRadian * kEarthRadiusInMeters * cosLatAvg;
        binDiagonalMeters = std::sqrt(latBinMeters * latBinMeters + lonBinMeters * lonBinMeters);
        minBinSideMeters = std::min(latBinMeters, lonBinMeters);

        // Initialize 2D grid of bins
        geoBins.resize(numBinsLat);
        for (int i = 0; i < numBinsLat; ++i) {
            geoBins[i].resize(numBinsLon);
        }

        // Populate bins with intersections
        for (IntersectionIdx intIdx = 0; intIdx < numIntersections; ++intIdx) {
            int latBin = static_cast<int>((intersectionPositions[intIdx].latitude() - geoMinLat) / latBinSize);
            int lonBin = static_cast<int>((intersectionPositions[intIdx].longitude() - geoMinLon) / lonBinSize);
            latBin = std::max(0, std::min(latBin, numBinsLat - 1));
            lonBin = std::max(0, std::min(lonBin, numBinsLon - 1));
            geoBins[latBin][lonBin].push_back(intIdx);
        }
    }

    buildOSMNodeMap();
    buildOSMWayMap();
    precomputeWayLengths();

    return true;
}

void closeMap() {
    closeStreetDatabase();
    closeOSMDatabase();

    // Clear all global data structures
    streetNameToIndicesSorted.clear();
    streetSegmentsByStreet.clear();
    streetSegmentsByIntersection.clear();
    intersectionsByStreet.clear();
    segmentLengths.clear();
    segmentTravelTimes.clear();
    segmentCache.clear();
    maxSpeedLimit = 0.0;
    poiNameToIndices.clear();
    intersectionPositions.clear();
    intersectionCheckFlag.clear();

    // Clear geo-binning data structures
    geoBins.clear();
    numBinsLat = 0;
    numBinsLon = 0;
    binDiagonalMeters = 0.0;
    cosLatAvg = 1.0;
    minBinSideMeters = 0.0;

    // Clear OSM data structures
    osmNodeById.clear();
    osmWayMap.clear();
    wayLengthCache.clear();
}

// Returns all intersections reachable by a single street segment from the given intersection.
// Respects one-way streets: only includes destinations you can legally travel to.
std::vector<IntersectionIdx> findAdjacentIntersections(IntersectionIdx intersectionId) {
    std::set<IntersectionIdx> neighbors;
    std::vector<StreetSegmentIdx> segments = findStreetSegmentsOfIntersection(intersectionId);

    for (size_t i = 0; i < segments.size(); i++) {
        StreetSegmentInfo segInfo = getStreetSegmentInfo(segments[i]);

        // Two-way street: can travel in either direction
        if (!segInfo.oneWay) {
            if (intersectionId == segInfo.from) {
                neighbors.insert(segInfo.to);
            } else {
                neighbors.insert(segInfo.from);
            }
        } else {
            // One-way: can only travel from -> to
            if (intersectionId == segInfo.from) {
                neighbors.insert(segInfo.to);
            }
        }
    }

    return std::vector<IntersectionIdx>(neighbors.begin(), neighbors.end());
}

// Returns all intersections where the two given streets meet
std::vector<IntersectionIdx> findIntersectionsOfTwoStreets(StreetIdx streetId1, StreetIdx streetId2) {
    std::vector<IntersectionIdx> result;

    // Use references to cached data to avoid copying vectors
    const std::vector<IntersectionIdx>& intersections1 = intersectionsByStreet[streetId1];
    const std::vector<IntersectionIdx>& intersections2 = intersectionsByStreet[streetId2];

    // Tag all intersections on the first street
    for (IntersectionIdx id : intersections1) {
        intersectionCheckFlag[id] = true;
    }

    // Check if any intersections on the second street are tagged
    for (IntersectionIdx id : intersections2) {
        if (intersectionCheckFlag[id]) {
            result.push_back(id);
        }
    }

    // Reset the tags back to false for the next function call
    for (IntersectionIdx id : intersections1) {
        intersectionCheckFlag[id] = false;
    }

    return result;
}

// Returns the distance between two (latitude, longitude) coordinates in meters
// Speed Requirement --> moderate
double findDistanceBetweenTwoPoints(std::pair<LatLon, LatLon> points) {
    LatLon point1 = points.first;
    LatLon point2 = points.second;

    // Convert to radians
    double lat1 = point1.latitude() * kDegreeToRadian;
    double lon1 = point1.longitude() * kDegreeToRadian;
    double lat2 = point2.latitude() * kDegreeToRadian;
    double lon2 = point2.longitude() * kDegreeToRadian;

    // Calculate average latitude for projection
    double latAvg = (lat1 + lat2) / 2.0;

    // Calculate x and y distances
    double x1 = kEarthRadiusInMeters * lon1 * std::cos(latAvg);
    double y1 = kEarthRadiusInMeters * lat1;
    double x2 = kEarthRadiusInMeters * lon2 * std::cos(latAvg);
    double y2 = kEarthRadiusInMeters * lat2;

    // Pythagorean theorem
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

// Returns the length of the given street segment in meters
// Speed Requirement --> moderate
double findStreetSegmentLength(StreetSegmentIdx streetSegmentId) {
    return segmentLengths[streetSegmentId];
}

// Returns the travel time to drive from one end of a street segment
// to the other, in seconds, when driving at the speed limit.
// Note: (time = distance/speed_limit)
// Speed Requirement --> high
double findStreetSegmentTravelTime(StreetSegmentIdx streetSegmentId) {
    return segmentTravelTimes[streetSegmentId];
}

// Returns the geographically nearest intersection to the given position.
// Uses geo-binning with expanding ring search for high performance.
// Speed Requirement --> high
IntersectionIdx findClosestIntersection(LatLon myPosition) {
    // Guard: if no bins exist (no intersections loaded), nothing to search
    if (numBinsLat == 0 || numBinsLon == 0) {
        return -1;
    }

    // Inline bin index calculation
    int centerLatBin = static_cast<int>((myPosition.latitude() - geoMinLat) / latBinSize);
    int centerLonBin = static_cast<int>((myPosition.longitude() - geoMinLon) / lonBinSize);
    centerLatBin = std::max(0, std::min(centerLatBin, numBinsLat - 1));
    centerLonBin = std::max(0, std::min(centerLonBin, numBinsLon - 1));

    IntersectionIdx closestIdx = -1;
    double minDistSq = std::numeric_limits<double>::max();

    // Pre-compute query position in radians.
    // Uses precomputed cosLatAvg so we never call cos() during the search.
    double myLatRad = myPosition.latitude() * kDegreeToRadian;
    double myLonRad = myPosition.longitude() * kDegreeToRadian;

    // Search in expanding rings around the center bin
    int maxSearchRadius = std::max(numBinsLat, numBinsLon);

    for (int radius = 0; radius < maxSearchRadius; ++radius) {
        // Early termination: the minimum possible distance from the query point
        // to any point in ring R is at least (R-1) * minBinSideMeters.
        // If our best squared distance is less than that squared, we can stop.
        if (closestIdx != -1 && radius > 1) {
            double ringDistLower = (radius - 1) * minBinSideMeters;
            if (minDistSq < ringDistLower * ringDistLower) {
                break;
            }
        }

        // Check all bins on the perimeter of the current ring
        for (int dLat = -radius; dLat <= radius; ++dLat) {
            for (int dLon = -radius; dLon <= radius; ++dLon) {
                // Only check bins on the perimeter (except for radius 0)
                if (radius > 0 && std::abs(dLat) != radius && std::abs(dLon) != radius) {
                    continue;
                }

                int latBin = centerLatBin + dLat;
                int lonBin = centerLonBin + dLon;

                // Skip out-of-bounds bins
                if (latBin < 0 || latBin >= numBinsLat || lonBin < 0 || lonBin >= numBinsLon) {
                    continue;
                }

                // Check all intersections in this bin using fast inlined squared distance
                const std::vector<IntersectionIdx>& bin = geoBins[latBin][lonBin];
                for (size_t i = 0; i < bin.size(); ++i) {
                    const LatLon& iPos = intersectionPositions[bin[i]];
                    double dy = kEarthRadiusInMeters * (iPos.latitude() * kDegreeToRadian - myLatRad);
                    double dx = kEarthRadiusInMeters * (iPos.longitude() * kDegreeToRadian - myLonRad) * cosLatAvg;
                    double distSq = dx * dx + dy * dy;

                    if (distSq < minDistSq) {
                        minDistSq = distSq;
                        closestIdx = bin[i];
                    }
                }
            }
        }
    }

    return closestIdx;
}

std::vector<StreetSegmentIdx> findStreetSegmentsOfIntersection(IntersectionIdx intersectionId) {
    return streetSegmentsByIntersection[intersectionId];
}

std::vector<IntersectionIdx> findIntersectionsOfStreet(StreetIdx streetId) {
    return intersectionsByStreet[streetId];
}

// Returns all street ids corresponding to street names that start with the
// given prefix. The function should ignore cases and spaces.
// Speed Requirement --> high
std::vector<StreetIdx> findStreetIdsFromPartialStreetName(std::string streetPrefix) {
    std::vector<StreetIdx> matchingStreets;
    std::string prefix = processStreetName(streetPrefix);
    if (prefix.empty()) {
        return matchingStreets;
    }

    // lower_bound finds the first element NOT LESS than prefix
    auto it = streetNameToIndicesSorted.lower_bound(prefix);

    // Iterate forward until the names no longer start with the prefix
    while (it != streetNameToIndicesSorted.end() && it->first.compare(0, prefix.size(), prefix) == 0) {
        matchingStreets.push_back(it->second);
        ++it;
    }

    return matchingStreets;
}

// Returns the length of a given street in meters.
// Speed Requirement --> high
double findStreetLength(StreetIdx streetId) {
    double totalLength = 0.0;
    for (StreetSegmentIdx segIdx : streetSegmentsByStreet[streetId]) {
        totalLength += segmentLengths[segIdx];
    }
    return totalLength;
}

// Return the smallest axis-aligned rectangle that contains all the
// intersections and curve points of the given street (i.e. the min,max
// latitude and longitude bounds that can just contain all points of the street).
// Speed Requirement --> none
LatLonBounds findStreetBoundingBox(StreetIdx streetId) {
    // Get all segments for this street
    const std::vector<StreetSegmentIdx>& segments = streetSegmentsByStreet[streetId];

    // Return empty bounds if street has no segments
    if (segments.empty()) {
        return LatLonBounds();
    }

    // Initialize bounds with first intersection's position
    StreetSegmentInfo firstSeg = getStreetSegmentInfo(segments[0]);
    LatLon firstPos = getIntersectionPosition(firstSeg.from);

    double minLat = firstPos.latitude();
    double maxLat = firstPos.latitude();
    double minLon = firstPos.longitude();
    double maxLon = firstPos.longitude();

    // Check all segments
    for (StreetSegmentIdx segIdx : segments) {
        StreetSegmentInfo segInfo = getStreetSegmentInfo(segIdx);

        // Check both endpoints using helper function
        updateBoundsHelper(minLat, maxLat, minLon, maxLon,
                           getIntersectionPosition(segInfo.from));
        updateBoundsHelper(minLat, maxLat, minLon, maxLon,
                           getIntersectionPosition(segInfo.to));

        // Check all curve points
        for (int i = 0; i < segInfo.numCurvePoints; i++) {
            updateBoundsHelper(minLat, maxLat, minLon, maxLon,
                               getStreetSegmentCurvePoint(segIdx, i));
        }
    }

    // Return the bounding box
    LatLonBounds bounds;
    bounds.min = LatLon(minLat, minLon);
    bounds.max = LatLon(maxLat, maxLon);
    return bounds;
}

// Returns the nearest point of interest of the given name
// to the given position.
// Speed Requirement --> high
POIIdx findClosestPOI(LatLon myPosition, std::string poiName) {
    // Look up all POIs with the given name using pre-computed map
    auto it = poiNameToIndices.find(poiName);

    // Check if the POI name was found in the map
    if (it == poiNameToIndices.end()) {
        return -1;
    }

    // Get the vector of POI indices with matching name
    std::vector<POIIdx> matchingPOIs = it->second;

    // Check if the vector is empty
    if (matchingPOIs.empty()) {
        return -1;
    }

    // Initialize tracking variables for the closest POI
    POIIdx closestIdx = -1;
    double minDistance = 0.0;
    bool foundFirst = false;

    // Loop through all POIs with the matching name
    for (size_t i = 0; i < matchingPOIs.size(); ++i) {
        POIIdx currentPOI = matchingPOIs[i];
        LatLon currentPOIPosition = getPOIPosition(currentPOI);
        double currentDistance = findDistanceBetweenTwoPoints({myPosition, currentPOIPosition});

        // Update closest POI if this is the first one or if it's closer
        if (!foundFirst) {
            closestIdx = currentPOI;
            minDistance = currentDistance;
            foundFirst = true;
        } else if (currentDistance < minDistance) {
            minDistance = currentDistance;
            closestIdx = currentPOI;
        }
    }

    return closestIdx;
}

// Returns the area of the given closed feature in square meters.
// Assume a non self-intersecting polygon (i.e. no holes).
// Return 0 if this feature is not a closed polygon.
// Speed Requirement --> moderate
double findFeatureArea(FeatureIdx featureId) {
    int numPoints = getNumFeaturePoints(featureId);

    // Need at least 3 points for a polygon
    if (numPoints < 3) {
        return 0.0;
    }

    // Check if closed (first point == last point)
    LatLon firstPoint = getFeaturePoint(featureId, 0);
    LatLon lastPoint = getFeaturePoint(featureId, numPoints - 1);

    // Check for rounding error
    const double EPSILON = 1e-9;
    bool isClosed = (std::abs(firstPoint.latitude() - lastPoint.latitude()) < EPSILON) &&
                    (std::abs(firstPoint.longitude() - lastPoint.longitude()) < EPSILON);

    if (!isClosed) {
        return 0.0;
    }

    // Calculate average latitude for projection
    double latSum = 0.0;
    for (int i = 0; i < numPoints; i++) {
        latSum += getFeaturePoint(featureId, i).latitude();
    }
    double latAvg = latSum / numPoints;
    double latAvgRad = latAvg * kDegreeToRadian;

    // Shoelace formula
    double area = 0.0;

    for (int i = 0; i < numPoints - 1; i++) {
        LatLon p1 = getFeaturePoint(featureId, i);
        LatLon p2 = getFeaturePoint(featureId, i + 1);

        // Convert to x,y coordinates
        double x1 = kEarthRadiusInMeters * p1.longitude() * kDegreeToRadian * std::cos(latAvgRad);
        double y1 = kEarthRadiusInMeters * p1.latitude() * kDegreeToRadian;

        double x2 = kEarthRadiusInMeters * p2.longitude() * kDegreeToRadian * std::cos(latAvgRad);
        double y2 = kEarthRadiusInMeters * p2.latitude() * kDegreeToRadian;

        // Shoelace formula component
        area += (x1 * y2 - x2 * y1);
    }

    return std::abs(area) / 2.0;
}

// Returns the length of the OSMWay that has the given OSMID, in meters.
// To implement this function you will have to access the OSMDatabaseAPI.h functions.
// Speed Requirement --> high
double findWayLength(OSMID wayId) {
    // Use precomputed cache for O(1) lookup
    auto it = wayLengthCache.find(wayId);
    if (it != wayLengthCache.end()) {
        return it->second;
    }
    // Way not found, return 0
    return 0.0;
}

// Return the value associated with this key on the specified OSMNode.
// If this OSMNode does not exist in the current map, or the specified key is
// not set on the specified OSMNode, return an empty string.
// Speed Requirement --> high
std::string getOSMNodeTagValue(OSMID osmId, std::string key) {
    // Use pre-built hash map for O(1) node lookup
    auto it = osmNodeById.find(osmId);

    // Check if node was found
    if (it == osmNodeById.end()) {
        return "";
    }

    const OSMNode* node = it->second;

    // Search through the node's tags for the key
    int tagCount = getTagCount(node);

    for (int j = 0; j < tagCount; ++j) {
        std::pair<std::string, std::string> tagPair = getTagPair(node, j);

        // Check if this tag's key matches the requested key
        if (tagPair.first == key) {
            return tagPair.second;
        }
    }

    // Node found but key not present
    return "";
}

// Returns the signed angle (in radians) that you would need to turn as you exit
// src and enter dst, if they share an intersection.
// Positive = turning left, negative = turning right, 0 = straight.
double findStreetSegmentTurnAngle(StreetSegmentIdx dst, StreetSegmentIdx src) {
    // Get information about both street segments
    StreetSegmentInfo srcInfo = getStreetSegmentInfo(src);
    StreetSegmentInfo dstInfo = getStreetSegmentInfo(dst);

    // Find the shared intersection between the two segments
    IntersectionIdx sharedIntersection = -1;
    bool srcGoingToShared = false;
    bool dstLeavingShared = false;

    // Check all possible intersection combinations
    if (srcInfo.to == dstInfo.from) {
        sharedIntersection = srcInfo.to;
        srcGoingToShared = true;
        dstLeavingShared = true;
    } else if (srcInfo.to == dstInfo.to) {
        sharedIntersection = srcInfo.to;
        srcGoingToShared = true;
        dstLeavingShared = false;
    } else if (srcInfo.from == dstInfo.from) {
        sharedIntersection = srcInfo.from;
        srcGoingToShared = false;
        dstLeavingShared = true;
    } else if (srcInfo.from == dstInfo.to) {
        sharedIntersection = srcInfo.from;
        srcGoingToShared = false;
        dstLeavingShared = false;
    } else {
        // No shared intersection found
        return NO_ANGLE;
    }

    // Get the position of the shared intersection
    LatLon sharedPosition = getIntersectionPosition(sharedIntersection);

    // Find the point before the shared intersection on src segment
    LatLon srcBeforeShared;
    if (srcInfo.numCurvePoints > 0 && srcGoingToShared) {
        srcBeforeShared = getStreetSegmentCurvePoint(src, srcInfo.numCurvePoints - 1);
    } else if (srcInfo.numCurvePoints > 0 && !srcGoingToShared) {
        srcBeforeShared = getStreetSegmentCurvePoint(src, 0);
    } else if (srcGoingToShared) {
        srcBeforeShared = getIntersectionPosition(srcInfo.from);
    } else {
        srcBeforeShared = getIntersectionPosition(srcInfo.to);
    }

    // Find the point after the shared intersection on dst segment
    LatLon dstAfterShared;
    if (dstInfo.numCurvePoints > 0 && dstLeavingShared) {
        dstAfterShared = getStreetSegmentCurvePoint(dst, 0);
    } else if (dstInfo.numCurvePoints > 0 && !dstLeavingShared) {
        dstAfterShared = getStreetSegmentCurvePoint(dst, dstInfo.numCurvePoints - 1);
    } else if (dstLeavingShared) {
        dstAfterShared = getIntersectionPosition(dstInfo.to);
    } else {
        dstAfterShared = getIntersectionPosition(dstInfo.from);
    }

    // Convert lat/lon to x/y coordinates for angle calculation
    double latAvg = sharedPosition.latitude() * kDegreeToRadian;

    double srcLat = srcBeforeShared.latitude() * kDegreeToRadian;
    double srcLon = srcBeforeShared.longitude() * kDegreeToRadian;
    double srcX = kEarthRadiusInMeters * srcLon * std::cos(latAvg);
    double srcY = kEarthRadiusInMeters * srcLat;

    double sharedLat = sharedPosition.latitude() * kDegreeToRadian;
    double sharedLon = sharedPosition.longitude() * kDegreeToRadian;
    double sharedX = kEarthRadiusInMeters * sharedLon * std::cos(latAvg);
    double sharedY = kEarthRadiusInMeters * sharedLat;

    double dstLat = dstAfterShared.latitude() * kDegreeToRadian;
    double dstLon = dstAfterShared.longitude() * kDegreeToRadian;
    double dstX = kEarthRadiusInMeters * dstLon * std::cos(latAvg);
    double dstY = kEarthRadiusInMeters * dstLat;

    // Calculate direction vectors
    double srcVectorX = sharedX - srcX;
    double srcVectorY = sharedY - srcY;
    double dstVectorX = dstX - sharedX;
    double dstVectorY = dstY - sharedY;

    // Calculate the angle using atan2
    double srcAngle = std::atan2(srcVectorY, srcVectorX);
    double dstAngle = std::atan2(dstVectorY, dstVectorX);

    // Calculate turn angle
    double turnAngle = dstAngle - srcAngle;

    // Normalize angle to [-PI, PI] range
    if (turnAngle > M_PI) {
        turnAngle = turnAngle - 2 * M_PI;
    } else if (turnAngle < -M_PI) {
        turnAngle = turnAngle + 2 * M_PI;
    }

    return turnAngle;
}
