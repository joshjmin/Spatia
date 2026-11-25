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

/*
 * m2.cpp
 *
 * Entry point for the M2 graphics layer.  Contains drawMap() which
 * configures EZGL settings, loads the CSS theme, and enters the GTK
 * event loop.  Also defines:
 *
 *   - Day and night ColorPalettes
 *   - The global AppState singleton (appState)
 *   - Coordinate conversion helpers (latLonToPoint, getZoomLevel)
 *   - Label collision detection (tryPlaceLabel)
 *   - Feature bounding-box culling (featureBBoxIntersectsViewport)
 *
 * The rest of M2 is split across:
 *   m2_shared.h        — shared types, constants, and declarations
 *   m2_precompute.cpp  — map-load-time precomputation
 *   m2_draw.cpp        — all rendering functions
 *   m2_callbacks.cpp   — EZGL / GTK event callbacks
 */

#include "m2.h"
#include "m2_shared.h"

// Color palettes (day and night)

const ColorPalette DAY_PALETTE = {
    ezgl::color(232, 232, 220),     // background
    ezgl::color(170, 211, 223),     // water
    ezgl::color(195, 228, 174),     // park
    ezgl::color(200, 233, 180),     // greenspace
    ezgl::color(180, 220, 160),     // golf course
    ezgl::color(255, 241, 186),     // beach
    ezgl::color(242, 239, 230),     // island
    ezgl::color(217, 213, 205),     // building
    ezgl::color(255, 191, 60),      // highway (orange-yellow)
    ezgl::color(210, 155, 40),      // highway outline
    ezgl::color(255, 255, 255),     // major road
    ezgl::color(190, 190, 190),     // major road outline
    ezgl::color(255, 255, 255),     // medium road
    ezgl::color(255, 255, 255),     // minor road
    ezgl::color(200, 200, 200),     // minor road outline
    ezgl::color(60, 60, 60),        // text
    ezgl::color(80, 80, 80),        // street text
    ezgl::color(230, 50, 50),       // highlight (red)
    ezgl::color(80, 50, 160),       // POI dot (purple)
    ezgl::color(80, 50, 160),       // POI text
    ezgl::color(50, 80, 150),       // one-way arrow (blue)
    ezgl::color(232, 232, 220),     // unknown feature
};

const ColorPalette NIGHT_PALETTE = {
    ezgl::color(25, 25, 35),        // background
    ezgl::color(25, 45, 70),        // water
    ezgl::color(25, 55, 30),        // park
    ezgl::color(28, 58, 33),        // greenspace
    ezgl::color(25, 50, 28),        // golf course
    ezgl::color(70, 65, 45),        // beach
    ezgl::color(35, 35, 45),        // island
    ezgl::color(45, 45, 55),        // building
    ezgl::color(180, 130, 30),      // highway
    ezgl::color(140, 100, 20),      // highway outline
    ezgl::color(80, 80, 90),        // major road
    ezgl::color(55, 55, 65),        // major road outline
    ezgl::color(70, 70, 80),        // medium road
    ezgl::color(55, 55, 65),        // minor road
    ezgl::color(40, 40, 50),        // minor road outline
    ezgl::color(180, 180, 190),     // text
    ezgl::color(160, 160, 170),     // street text
    ezgl::color(255, 80, 80),       // highlight
    ezgl::color(140, 110, 220),     // POI dot
    ezgl::color(160, 130, 230),     // POI text
    ezgl::color(100, 140, 220),     // one-way arrow
    ezgl::color(25, 25, 35),        // unknown feature
};

// Global app state — shared across all the m2_*.cpp files via m2_shared.h

AppState appState;

// Helper function that bundles the lon/lat projection into one call
ezgl::point2d latLonToPoint(LatLon pos) {
    return {lonToX(pos.longitude()), latToY(pos.latitude())};
}

// Buckets the visible width into one of our zoom levels
ZoomLevel getZoomLevel(double visibleWidthMeters) {
    if (visibleWidthMeters > ZOOM_FAR_THRESHOLD) {
        return ZOOM_FAR;
    }
    if (visibleWidthMeters > ZOOM_CITY_THRESHOLD) {
        return ZOOM_CITY;
    }
    if (visibleWidthMeters > ZOOM_DISTRICT_THRESHOLD) {
        return ZOOM_DISTRICT;
    }
    if (visibleWidthMeters > ZOOM_NEIGHBORHOOD_THRESHOLD) {
        return ZOOM_NEIGHBORHOOD;
    }
    if (visibleWidthMeters > ZOOM_BLOCK_THRESHOLD) {
        return ZOOM_BLOCK;
    }
    return ZOOM_CLOSE;
}

const ColorPalette& getPalette() {
    return appState.nightMode ? NIGHT_PALETTE : DAY_PALETTE;
}

// Tries to place a text label without overlapping any existing ones.
// Returns false if there's a collision, true if we successfully claimed the spot.
// Has to account for rotation since a tilted label takes up more AABB space.
bool tryPlaceLabel(ezgl::point2d center, const std::string& text,
                   double fontSizeWorld, double rotationDeg) {
    // rough estimate of how big the text will be
    double charWidth  = fontSizeWorld * CHAR_WIDTH_RATIO;
    double textWidth  = text.length() * charWidth;
    double textHeight = fontSizeWorld * CHAR_HEIGHT_RATIO;

    // figure out the AABB after rotation
    double rad  = rotationDeg * M_PI / 180.0;
    double cosR = std::abs(std::cos(rad));
    double sinR = std::abs(std::sin(rad));
    double bboxW = textWidth * cosR + textHeight * sinR;
    double bboxH = textWidth * sinR + textHeight * cosR;

    TextBBox newBox = {
        center.x - bboxW / 2.0, center.y - bboxH / 2.0,
        center.x + bboxW / 2.0, center.y + bboxH / 2.0
    };

    for (const auto& existing : appState.placedLabels) {
        if (newBox.overlaps(existing)) {
            return false;
        }
    }

    appState.placedLabels.push_back(newBox);
    return true;
}

// Quick check to see if the feature is on the screen
// if it's not, no point in drawing
// compares bounding boxes
bool featureBBoxIntersectsViewport(const FeatureDrawInfo& f,
                                    const ezgl::rectangle& visible) {
    if (f.points.empty()) {
        return false;
    }

    // find the extents of all the feature's points
    double fMinX = f.points[0].x, fMaxX = f.points[0].x;
    double fMinY = f.points[0].y, fMaxY = f.points[0].y;
    for (size_t i = 1; i < f.points.size(); ++i) {
        fMinX = std::min(fMinX, f.points[i].x);
        fMaxX = std::max(fMaxX, f.points[i].x);
        fMinY = std::min(fMinY, f.points[i].y);
        fMaxY = std::max(fMaxY, f.points[i].y);
    }

    return !(fMaxX < visible.left()  || fMinX > visible.right() ||
             fMaxY < visible.bottom() || fMinY > visible.top());
}

// Parses "#RRGGBB" hex strings into actual colors
// used for subway line colors that come from OSM as hex codes
ezgl::color parseHexColor(const std::string& hex) {
    if (hex.size() < 7 || hex[0] != '#') {
        return ezgl::color(100, 100, 100);
    }

    unsigned int r = 0, g = 0, b = 0;
    std::stringstream ss;
    ss << std::hex << hex.substr(1, 2); ss >> r; ss.clear();
    ss << std::hex << hex.substr(3, 2); ss >> g; ss.clear();
    ss << std::hex << hex.substr(5, 2); ss >> b;
    return ezgl::color(static_cast<uint8_t>(r),
                        static_cast<uint8_t>(g),
                        static_cast<uint8_t>(b));
}

// main entry point for the whole map UI. Sets up EZGL,
// hooks in our callbacks, and blocks until the user closes the window
void drawMap() {
    curl_global_init(CURL_GLOBAL_DEFAULT);  // initialize before any HTTP calls

    ezgl::application::settings settings;
    settings.main_ui_resource  = "libstreetmap/resources/main.ui";
    settings.window_identifier = "MainWindow";
    settings.canvas_identifier = "MainCanvas";

    ezgl::application application(settings);

    // start by showing the whole map
    ezgl::rectangle initialWorld(
        {lonToX(geoMinLon), latToY(geoMinLat)},
        {lonToX(geoMaxLon), latToY(geoMaxLat)}
    );

    application.add_canvas("MainCanvas", drawMainCanvas, initialWorld);
    application.run(initialSetup, actOnMousePress, actOnMouseMove, actOnKeyPress);

    curl_global_cleanup();  // clean up libcurl when the app exits
}