/*
 * globals.h
 *
 * Shared data structures and coordinate conversion utilities used by both
 * the M1 backend (m1.cpp) and the M2 graphics layer (m2.cpp).
 *
 * Variables declared here are defined in globals.cpp and populated
 * during loadMap() in m1.cpp.
 */
#pragma once

#include <vector>
#include <unordered_map>
#include <string>
#include <map>
#include "StreetsDatabaseAPI.h"
#include "OSMDatabaseAPI.h"

// Coordinate projection constants populated by loadMap

// Cosine of the average latitude of the current map (radians).
// Used for equirectangular projection: x = R * lon * cos(latAvg)
extern double cosLatAvg;

// Geographic bounding box of the current map (in degrees)
extern double geoMinLat;
extern double geoMaxLat;
extern double geoMinLon;
extern double geoMaxLon;

// Cached data structures from M1 (populated by loadMap)

// For each street, all street segment indices belonging to that street
extern std::vector<std::vector<StreetSegmentIdx>> streetSegmentsByStreet;

// For each intersection, all connected street segment indices
extern std::vector<std::vector<StreetSegmentIdx>> streetSegmentsByIntersection;

// For each street, all intersections that lie on that street
extern std::vector<std::vector<IntersectionIdx>> intersectionsByStreet;

// Pre-cached positions of all intersections for fast spatial queries
extern std::vector<LatLon> intersectionPositions;

// Pre-computed segment metrics
extern std::vector<double> segmentTravelTimes;  // length / speedLimit for each segment
extern std::vector<double> segmentLengths;       // length in meters for each segment
extern double maxSpeedLimit;                     // max speed limit across all segments (m/s)

// Cached StreetSegmentInfo fields for fast access in A* inner loop
struct SegmentCache {
    IntersectionIdx from, to;
    StreetIdx streetID;
    bool oneWay;
};
extern std::vector<SegmentCache> segmentCache;

// Fast lookup for OSM Nodes and Ways by ID
extern std::unordered_map<OSMID, const OSMNode*> osmNodeById;
extern std::unordered_map<OSMID, const OSMWay*> osmWayMap;

// Sorted multimap for efficient prefix-based street name searches
extern std::multimap<std::string, StreetIdx> streetNameToIndicesSorted;

// Coordinate conversion utilities:

// Convert longitude (degrees) to x world coordinate (meters)
double lonToX(double lon);

// Convert latitude (degrees) to y world coordinate (meters)
double latToY(double lat);

// Convert x world coordinate (meters) back to longitude (degrees)
double xToLon(double x);

// Convert y world coordinate (meters) back to latitude (degrees)
double yToLat(double y);
