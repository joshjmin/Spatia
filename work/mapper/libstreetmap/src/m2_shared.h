/*
 * m2_shared.h
 *
 * Internal header shared by every M2 source file.  This is the "glue"
 * that ties the graphics layer together — every m2_*.cpp includes this
 * so they all see the same types, constants, and function declarations.
 *
 * Contents:
 *   - #includes for all libraries used across M2 (STL, GTK, curl, EZGL)
 *   - Zoom-level enum and tier constants for street classification
 *   - ColorPalette struct (day/night themes)
 *   - FeatureDrawInfo, SubwayLine, SubwayStation — precomputed draw data
 *   - AppState singleton — mutable runtime state (highlights, toggles,
 *     weather, chat history, subway data, placed labels, etc.)
 *   - Function declarations for every M2 module:
 *       m2.cpp           — drawMap(), palettes, coordinate helpers
 *       m2_precompute.cpp — precomputeSegmentTiers(), features, subway, etc.
 *       m2_draw.cpp       — drawMainCanvas() and all rendering helpers
 *       m2_callbacks.cpp  — EZGL callbacks, button handlers, chat system
 */

#pragma once

#include <cmath>
#include <string>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <set>
#include <algorithm>
#include <sstream>
#include <thread>
#include <mutex>
#include <iomanip>
#include <cctype>
#include <cstdlib>
#include <curl/curl.h>

#include "m1.h"
#include "m3.h"
#include "globals.h"
#include "StreetsDatabaseAPI.h"
#include "OSMDatabaseAPI.h"
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"

// Road importance tiers
// Controls draw order, line width, and
// at which zoom level the road becomes visible.
constexpr int TIER_HIGHWAY = 1; // motorway, trunk
constexpr int TIER_MAJOR   = 2; // primary, secondary
constexpr int TIER_MEDIUM  = 3; // tertiary
constexpr int TIER_MINOR   = 4; // residential, service, everything else

// Zoom thresholds -> the visible world width (in meters) that separates
// each zoom level.
constexpr double ZOOM_FAR_THRESHOLD          = 50000.0;
constexpr double ZOOM_CITY_THRESHOLD         = 20000.0;
constexpr double ZOOM_DISTRICT_THRESHOLD     = 5000.0;
constexpr double ZOOM_NEIGHBORHOOD_THRESHOLD = 2000.0;
constexpr double ZOOM_BLOCK_THRESHOLD        = 500.0;

// Hard zoom limits so the user can't scroll into oblivion or zoom
// down to sub meter scale where nothing makes sense
constexpr double MAX_ZOOM_OUT_WIDTH = 200000.0;  // 200 km
constexpr double MIN_ZOOM_IN_WIDTH  = 30.0;      // 30 m

constexpr double CLICK_THRESHOLD_METERS = 50.0; // how close a click has to be to an intersection
constexpr double CHAR_WIDTH_RATIO = 0.55; // rough width-to-height ratio for label collision
constexpr double CHAR_HEIGHT_RATIO = 1.2; // extra vertical padding for text bounding boxes
constexpr double SMALL_FEATURE_AREA = 5000.0; // skip features smaller than this when zoomed out

enum ZoomLevel {
    ZOOM_FAR = 0, 
    ZOOM_CITY,
    ZOOM_DISTRICT, 
    ZOOM_NEIGHBORHOOD,
    ZOOM_BLOCK, 
    ZOOM_CLOSE 
};

// All the colors for one theme -> we have a day and night version
struct ColorPalette {
    ezgl::color background;
    ezgl::color water;
    ezgl::color park;
    ezgl::color greenspace;
    ezgl::color golfCourse;
    ezgl::color beach;
    ezgl::color island;
    ezgl::color building;
    ezgl::color highway;
    ezgl::color highwayOutline;
    ezgl::color majorRoad;
    ezgl::color majorRoadOutline;
    ezgl::color mediumRoad;
    ezgl::color minorRoad;
    ezgl::color minorRoadOutline;
    ezgl::color text;
    ezgl::color streetText;
    ezgl::color highlight;
    ezgl::color poiDot;
    ezgl::color poiText;
    ezgl::color oneWayArrow;
    ezgl::color unknown;
};

extern const ColorPalette DAY_PALETTE;
extern const ColorPalette NIGHT_PALETTE;

// Axis aligned bounding box for placed text labels.
// We track these per-frame to prevent street names from overlapping.
struct TextBBox {
    double xMin, yMin, xMax, yMax;

    // Simple AABB overlap test — if any edge is fully past the
    // other box's opposite edge, they can't be overlapping
    bool overlaps(const TextBBox& other) const {
        return !(xMax < other.xMin || other.xMax < xMin ||
                 yMax < other.yMin || other.yMax < yMin);
    }
};

// Parsed from OSM relation data during precomputation
struct SubwayStation {
    ezgl::point2d position;
    std::string name;
};

// Each subway line can have multiple discontinuous path segments
struct SubwayLine {
    std::string name;
    ezgl::color lineColor;
    std::vector<std::vector<ezgl::point2d>> paths;  // each inner vec is one continuous path
    std::vector<SubwayStation> stations;
};

// Pre converted feature data —> we do the lat/lon -> world coordinate
// conversion once at load time instead of every frame
struct FeatureDrawInfo {
    FeatureIdx id;
    FeatureType type;
    double area;        // used for draw ordering 
    bool isClosed;      // true if first point == last point (polygon)
    std::vector<ezgl::point2d> points;
};

// Central state struct —> everything the map needs to remember between frames.
// Single global instance (appState) shared across all m2.cpp files.
struct AppState {
    // UI toggle states
    bool nightMode = false;
    bool showSubway = true;
    bool showPOIs = true;

    // Which intersections are currently highlighted 
    std::unordered_set<IntersectionIdx> highlightedIntersections;
    IntersectionIdx lastClickedIntersection = -1;  // for chat context
    bool autocompleteJustSelected = false;          // suppress click-through from dropdown

    // Precomputed at map load —> avoids re-parsing OSM tags every frame
    std::vector<int> segmentTier; // highway tier per segment 
    std::vector<FeatureDrawInfo> featureDrawOrder;
    std::vector<SubwayLine> subwayLines;
    std::vector<TextBBox> placedLabels; // rebuilt every frame for label collision

    std::string currentMapPath; // needed for chat context & reload
    ezgl::application* ezglApp = nullptr; // stashed so background threads can trigger redraws
    bool mapLoading = false;

    std::unordered_map<StreetSegmentIdx, int> speedLimitKmh; // parsed from OSM maxspeed tags

    // Weather data —> fetched async in a background thread, so mutex-protected
    std::string weatherInfo;
    std::mutex weatherMutex;

    // Chat (Gemini AI assistant) —> also async, also mutex-protected
    bool showChat = false;
    std::string chatHistory;   // Full conversation log shown in the text view
    std::string chatResponse;  // latest bot reply (written by bg thread, read by main thread)
    bool chatWaiting = false;  // prevents double sends while waiting for API
    std::mutex chatMutex;

    // GTK widget pointers for the chat panel —> we look these up once
    // on first toggle instead of calling get_object every time
    bool chatWidgetsResolved = false;
    bool chatAvailable = false;
    GtkWidget* chatPanel = nullptr;
    GtkWidget* chatSendBtn = nullptr;
    GtkWidget* chatEntryW = nullptr;
    GtkWidget* chatHistoryTv = nullptr;

    // M3 Pathfinding state
    bool navigationMode = false;
    bool walkDriveMode  = false;        // true = walk+drive, false = drive only
    double turnPenalty  = 15.0;         // seconds
    double walkSpeed    = 1.4;          // m/s (~5 km/h)
    double walkTimeLimit = 300.0;       // seconds (5 minutes)

    IntersectionIdx navStart = -1;      // first click
    IntersectionIdx navEnd   = -1;      // second click

    // Text-based navigation input
    enum NavInputPhase { NAV_INPUT_NONE, NAV_INPUT_START, NAV_INPUT_END };
    NavInputPhase navInputPhase = NAV_INPUT_NONE;
    IntersectionIdx textNavStart = -1;
    IntersectionIdx textNavEnd = -1;

    std::vector<StreetSegmentIdx> currentDrivePath;
    std::vector<StreetSegmentIdx> currentWalkPath;
    std::string directionsText;         // turn-by-turn directions

    // Directions panel widget pointers (resolved lazily)
    GtkWidget* directionsPanel  = nullptr;
    GtkWidget* directionsTv     = nullptr;
    GtkWidget* walkSpeedEntry   = nullptr;
    GtkWidget* walkLimitEntry   = nullptr;
    GtkWidget* walkParamsBox    = nullptr;

    // Navigation button pointers (for toggling label/style)
    GtkWidget* navigateBtn      = nullptr;
    GtkWidget* walkDriveBtn     = nullptr;
};

extern AppState appState;

// Utility functions (m2.cpp)
ezgl::point2d latLonToPoint(LatLon pos);  // convenience wrapper around lonToX/latToY
ZoomLevel getZoomLevel(double visibleWidthMeters);
const ColorPalette& getPalette();
bool tryPlaceLabel(ezgl::point2d center, const std::string& text,
                   double fontSizeWorld, double rotationDeg);
bool featureBBoxIntersectsViewport(const FeatureDrawInfo& f,
                                   const ezgl::rectangle& visible);
ezgl::color parseHexColor(const std::string& hex);

// Precomputation (m2_precompute.cpp)
void precomputeSegmentTiers();
void precomputeFeatureDrawOrder();
void precomputeSubwayData();
void precomputeSpeedLimits();
void setupAutocomplete(ezgl::application* app);

// Drawing (m2_draw.cpp)
void drawMainCanvas(ezgl::renderer* g);
void drawFeatures(ezgl::renderer* g, ZoomLevel zoom,
                  const ezgl::rectangle& visible, const ColorPalette& palette);
bool segmentIsVisible(const StreetSegmentInfo& info, StreetSegmentIdx seg,
                      const ezgl::rectangle& visible);
void drawStreetSegment(ezgl::renderer* g, StreetSegmentIdx seg,
                       const StreetSegmentInfo& info);
void drawSpeedLimitBadges(ezgl::renderer* g, const ezgl::rectangle& visible,
                          const ColorPalette& palette);
void drawStreets(ezgl::renderer* g, ZoomLevel zoom,
                 const ezgl::rectangle& visible, const ColorPalette& palette);
void drawStreetNames(ezgl::renderer* g, ZoomLevel zoom,
                     const ezgl::rectangle& visible, const ColorPalette& palette);
void drawOneWayArrows(ezgl::renderer* g, ZoomLevel zoom,
                      const ezgl::rectangle& visible, const ColorPalette& palette);
void drawPOIIcon(ezgl::renderer* g, const std::string& poiType,
                 ezgl::point2d pt, double r, const ColorPalette& palette);
void drawPOIs(ezgl::renderer* g, ZoomLevel zoom,
              const ezgl::rectangle& visible, const ColorPalette& palette);
void drawSubwayLines(ezgl::renderer* g, ZoomLevel zoom,
                     const ColorPalette& palette);
void drawSubwayStations(ezgl::renderer* g, ZoomLevel zoom,
                        const ColorPalette& palette);
void fetchWeatherAsync(double lat, double lon);
void drawWeatherOverlay(ezgl::renderer* g, const ColorPalette& palette);
void drawDistanceScale(ezgl::renderer* g, const ColorPalette& palette,
                       const ezgl::rectangle& visible);
void drawHighlights(ezgl::renderer* g, const ColorPalette& palette);

// Path drawing and directions (m2_draw.cpp)
void drawPath(ezgl::renderer* g, const ColorPalette& palette);
std::string generateDirections(const std::vector<StreetSegmentIdx>& path,
                                bool isWalking, double walkSpeed = 1.4,
                                double turnPenalty = 15.0);
std::string formatTime(double seconds);

// Navigation callbacks (m2_callbacks.cpp)
void onNavigateClicked(GtkWidget* widget, ezgl::application* app);
void onWalkDriveClicked(GtkWidget* widget, ezgl::application* app);
void onClearPathClicked(GtkWidget* widget, ezgl::application* app);
void onHelpClicked(GtkWidget* widget, ezgl::application* app);
void clearNavigation(ezgl::application* app);
void showStreetNotFoundError(ezgl::application* app, const std::string& streetName);

// EZGL callbacks (m2_callbacks.cpp)
void initialSetup(ezgl::application* app, bool newWindow);
void actOnMousePress(ezgl::application* app, GdkEventButton* event,
                     double x, double y);
void actOnMouseMove(ezgl::application* app, GdkEventButton* event,
                    double x, double y);
void actOnKeyPress(ezgl::application* app, GdkEventKey* event, char* keyName);

// GTK callbacks (m2_callbacks.cpp)
void reloadMap(const std::string& newPath, ezgl::application* app);
void onLoadMapClicked(GtkWidget* widget, ezgl::application* app);
void onRefreshWeather(GtkWidget* widget, ezgl::application* app);
void onFindButtonClicked(GtkWidget* widget, ezgl::application* app);
void onNightModeToggled(GtkWidget* widget, ezgl::application* app);
void onTogglePOIs(GtkWidget* widget, ezgl::application* app);
void onToggleSubway(GtkWidget* widget, ezgl::application* app);

// Chat (m2_callbacks.cpp)
void onToggleChat(GtkWidget* widget, ezgl::application* app);
void onChatSend(GtkWidget* widget, ezgl::application* app);
void onChatEntryActivate(GtkWidget* widget, gpointer data);
void sendChatMessageAsync(const std::string& userMessage, ezgl::application* app);
std::string buildChatContext(const std::string& userMessage);
std::string extractGeminiText(const std::string& jsonResponse);
