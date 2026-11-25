/*
 * globals.cpp
 *
 * Definitions of shared global variables declared in globals.h.
 * These are populated by loadMap() in m1.cpp and consumed by m2.cpp.
 */
#include "globals.h"
#include <cmath>

// Earth constants (same values as in StreetsDatabaseAPI)
constexpr double kEarthRadiusInMeters = 6372797.560856;
constexpr double kDegreeToRadian = 0.017453292519943295769236907684886;

// Coordinate projection constants
double cosLatAvg = 1.0;
double geoMinLat = 0.0;
double geoMaxLat = 0.0;
double geoMinLon = 0.0;
double geoMaxLon = 0.0;

// Cached data structures
std::vector<std::vector<StreetSegmentIdx>> streetSegmentsByStreet;
std::vector<std::vector<StreetSegmentIdx>> streetSegmentsByIntersection;
std::vector<std::vector<IntersectionIdx>> intersectionsByStreet;
std::vector<LatLon> intersectionPositions;
std::unordered_map<OSMID, const OSMNode*> osmNodeById;
std::unordered_map<OSMID, const OSMWay*> osmWayMap;
std::multimap<std::string, StreetIdx> streetNameToIndicesSorted;

// Segment metrics 
std::vector<double> segmentTravelTimes;
std::vector<double> segmentLengths;
double maxSpeedLimit = 0.0;
std::vector<SegmentCache> segmentCache;

// Coordinate conversion functions

double lonToX(double lon) {
    return kEarthRadiusInMeters * lon * kDegreeToRadian * cosLatAvg;
}

double latToY(double lat) {
    return kEarthRadiusInMeters * lat * kDegreeToRadian;
}

double xToLon(double x) {
    return x / (kEarthRadiusInMeters * kDegreeToRadian * cosLatAvg);
}

double yToLat(double y) {
    return y / (kEarthRadiusInMeters * kDegreeToRadian);
}
