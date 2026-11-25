/*
 * m2_draw.cpp
 *
 * All rendering functions for the map.  drawMainCanvas() is the single
 * EZGL draw callback — it orchestrates every visual layer in order:
 *
 *   1. Features     — lakes, parks, rivers, buildings (filled polygons / lines)
 *   2. Streets      — 4-pass draw (minor → medium → major → highway), outline then fill
 *   3. One-way arrows — triangular direction indicators on one-way segments
 *   4. Street names — rotated labels with collision avoidance
 *   5. Subway       — route lines + station markers (togglable)
 *   6. POIs         — color-coded dots + labels at close zoom (togglable)
 *   7. Highlights   — red dots on selected intersections
 *   8. Weather      — bottom-right overlay box (fetched async from Open-Meteo)
 *   9. Scale bar    — top-left distance reference
 *
 * Also contains:
 *   fetchWeatherAsync()  — background HTTP request to Open-Meteo API
 *   WMO code mapping     — translates numeric weather codes to descriptions
 */
#include "m2_shared.h"

// Lakes, parks, rivers, buildings, etc. We draw these biggest-first
// (sorted during precompute) so small stuff like buildings ends up on top
void drawFeatures(ezgl::renderer* g, ZoomLevel zoom,
                  const ezgl::rectangle& visible, const ColorPalette& palette) {

    for (const auto& feat : appState.featureDrawOrder) {
        // tiny features are just noise when zoomed out
        if (zoom <= ZOOM_CITY && feat.area < SMALL_FEATURE_AREA && feat.isClosed) {
            continue;
        }
        // buildings only matter up close
        if (feat.type == BUILDING && zoom < ZOOM_NEIGHBORHOOD) {
            continue;
        }
        // skip anything off-screen
        if (!featureBBoxIntersectsViewport(feat, visible)) {
            continue;
        }

        // pick the right color for this feature type
        switch (feat.type) {
            case LAKE:
            case RIVER:
            case STREAM:
                g->set_color(palette.water);      break;
            case PARK:
                g->set_color(palette.park);       break;
            case GREENSPACE:
                g->set_color(palette.greenspace); break;
            case GOLFCOURSE:
                g->set_color(palette.golfCourse); break;
            case BEACH:
                g->set_color(palette.beach);      break;
            case ISLAND:
                g->set_color(palette.island);     break;
            case BUILDING:
                g->set_color(palette.building);   break;
            default:
                g->set_color(palette.unknown);    break;
        }

        // closed shapes get filled, open ones just draw as lines
        if (feat.isClosed && feat.points.size() >= 3) {
            g->fill_poly(feat.points);
        } else if (feat.points.size() >= 2) {
            // draw rivers a bit thicker than streams
            if (feat.type == RIVER || feat.type == STREAM) {
                g->set_line_width(feat.type == STREAM ? 1 : 2);
            } else {
                g->set_line_width(1);
            }
            for (size_t i = 0; i + 1 < feat.points.size(); ++i) {
                g->draw_line(feat.points[i], feat.points[i + 1]);
            }
        }
    }
}

// Draws one street segment, walks from the start intersection through
// any curve points and ends at the destination intersection
void drawStreetSegment(ezgl::renderer* g, StreetSegmentIdx seg,
                       const StreetSegmentInfo& info) {
    LatLon fromPos = intersectionPositions[info.from];
    ezgl::point2d prevPt = latLonToPoint(fromPos);

    // walk through curve points if the road bends
    for (int c = 0; c < info.numCurvePoints; ++c) {
        LatLon curvePos = getStreetSegmentCurvePoint(seg, c);
        ezgl::point2d curvePt = latLonToPoint(curvePos);
        g->draw_line(prevPt, curvePt);
        prevPt = curvePt;
    }

    ezgl::point2d toPt = latLonToPoint(intersectionPositions[info.to]);
    g->draw_line(prevPt, toPt);
}

// checks if a street segment is on screen by comparing its endpoint
// bounding box to the viewport 
bool segmentIsVisible(const StreetSegmentInfo& info, StreetSegmentIdx,
                      const ezgl::rectangle& visible) {
    ezgl::point2d fromPt = latLonToPoint(intersectionPositions[info.from]);
    ezgl::point2d toPt   = latLonToPoint(intersectionPositions[info.to]);

    double minX = std::min(fromPt.x, toPt.x);
    double maxX = std::max(fromPt.x, toPt.x);
    double minY = std::min(fromPt.y, toPt.y);
    double maxY = std::max(fromPt.y, toPt.y);

    return !(maxX < visible.left()  || minX > visible.right() ||
             maxY < visible.bottom() || minY > visible.top());
}

// draws the circular speed limit signs on highways/major roads while zoomed in
void drawSpeedLimitBadges(ezgl::renderer* g, const ezgl::rectangle& visible,
                           const ColorPalette& palette) {
    double worldPerPixel = visible.width() / 1200.0;
    double badgeRadius   = worldPerPixel * 11.0;

    g->set_font_size(10);
    g->set_horiz_justification(ezgl::justification::center);
    g->set_vert_justification(ezgl::justification::center);
    g->set_text_rotation(0);

    for (const auto& [seg, kmh] : appState.speedLimitKmh) {
        int tier = appState.segmentTier[seg];
        if (tier > TIER_MAJOR) {
            continue;
        }

        StreetSegmentInfo info = getStreetSegmentInfo(seg);
        if (!segmentIsVisible(info, seg, visible)) {
            continue;
        }

        ezgl::point2d fromPt = latLonToPoint(intersectionPositions[info.from]);
        ezgl::point2d toPt   = latLonToPoint(intersectionPositions[info.to]);
        ezgl::point2d mid    = {(fromPt.x + toPt.x) / 2.0,
                                 (fromPt.y + toPt.y) / 2.0};

        g->set_color(ezgl::WHITE);
        g->fill_arc(mid, badgeRadius, 0, 360);
        g->set_color(palette.minorRoadOutline);
        g->draw_arc(mid, badgeRadius, 0, 360);
        g->set_color(palette.text);
        g->draw_text(mid, std::to_string(kmh));
    }
}

// draw streets in 4 passes, least important first, so highways end up on top
// each pass does outline then fill
void drawStreets(ezgl::renderer* g, ZoomLevel zoom,
                 const ezgl::rectangle& visible, const ColorPalette& palette) {
    int numSegments = getNumStreetSegments();

    g->set_line_cap(ezgl::line_cap::round);   
    g->set_line_dash(ezgl::line_dash::none);

    // minor roads — only draw at neighborhood zoom and closer
    if (zoom >= ZOOM_NEIGHBORHOOD) {
        g->set_color(palette.minorRoadOutline);
        for (StreetSegmentIdx seg = 0; seg < numSegments; ++seg) {
            if (appState.segmentTier[seg] != TIER_MINOR) {
                continue;
            }
            StreetSegmentInfo info = getStreetSegmentInfo(seg);
            if (!segmentIsVisible(info, seg, visible)) {
                continue;
            }
            g->set_line_width(zoom >= ZOOM_CLOSE ? 4 : 2);
            drawStreetSegment(g, seg, info);
        }
        g->set_color(palette.minorRoad);
        for (StreetSegmentIdx seg = 0; seg < numSegments; ++seg) {
            if (appState.segmentTier[seg] != TIER_MINOR) {
                continue;
            }
            StreetSegmentInfo info = getStreetSegmentInfo(seg);
            if (!segmentIsVisible(info, seg, visible)) {
                continue;
            }
            g->set_line_width(zoom >= ZOOM_CLOSE ? 2 : 1);
            drawStreetSegment(g, seg, info);
        }
    }

    // medium (tertiary) roads — show up from district zoom
    if (zoom >= ZOOM_DISTRICT) {
        g->set_color(palette.minorRoadOutline);
        for (StreetSegmentIdx seg = 0; seg < numSegments; ++seg) {
            if (appState.segmentTier[seg] != TIER_MEDIUM) {
                continue;
            }
            StreetSegmentInfo info = getStreetSegmentInfo(seg);
            if (!segmentIsVisible(info, seg, visible)) {
                continue;
            }
            g->set_line_width((zoom >= ZOOM_CLOSE) ? 6 : (zoom >= ZOOM_BLOCK ? 4 : 3));
            drawStreetSegment(g, seg, info);
        }
        g->set_color(palette.mediumRoad);
        for (StreetSegmentIdx seg = 0; seg < numSegments; ++seg) {
            if (appState.segmentTier[seg] != TIER_MEDIUM) {
                continue;
            }
            StreetSegmentInfo info = getStreetSegmentInfo(seg);
            if (!segmentIsVisible(info, seg, visible)) {
                continue;
            }
            g->set_line_width((zoom >= ZOOM_CLOSE) ? 4 : (zoom >= ZOOM_BLOCK ? 2 : 1));
            drawStreetSegment(g, seg, info);
        }
    }

    // major roads — visible from city zoom
    if (zoom >= ZOOM_CITY) {
        g->set_color(palette.majorRoadOutline);
        for (StreetSegmentIdx seg = 0; seg < numSegments; ++seg) {
            if (appState.segmentTier[seg] != TIER_MAJOR) {
                continue;
            }
            StreetSegmentInfo info = getStreetSegmentInfo(seg);
            if (!segmentIsVisible(info, seg, visible)) {
                continue;
            }
            g->set_line_width((zoom >= ZOOM_CLOSE) ? 8 : (zoom >= ZOOM_BLOCK ? 6 : 4));
            drawStreetSegment(g, seg, info);
        }
        g->set_color(palette.majorRoad);
        for (StreetSegmentIdx seg = 0; seg < numSegments; ++seg) {
            if (appState.segmentTier[seg] != TIER_MAJOR) {
                continue;
            }
            StreetSegmentInfo info = getStreetSegmentInfo(seg);
            if (!segmentIsVisible(info, seg, visible)) {
                continue;
            }
            g->set_line_width((zoom >= ZOOM_CLOSE) ? 5 : (zoom >= ZOOM_BLOCK ? 4 : 2));
            drawStreetSegment(g, seg, info);
        }
    }

    // highways — always visible no matter how far out you are
    {
        g->set_color(palette.highwayOutline);
        for (StreetSegmentIdx seg = 0; seg < numSegments; ++seg) {
            if (appState.segmentTier[seg] != TIER_HIGHWAY) {
                continue;
            }
            StreetSegmentInfo info = getStreetSegmentInfo(seg);
            if (!segmentIsVisible(info, seg, visible)) {
                continue;
            }
            g->set_line_width((zoom >= ZOOM_CLOSE) ? 10 : (zoom >= ZOOM_BLOCK ? 8 : 5));
            drawStreetSegment(g, seg, info);
        }
        g->set_color(palette.highway);
        for (StreetSegmentIdx seg = 0; seg < numSegments; ++seg) {
            if (appState.segmentTier[seg] != TIER_HIGHWAY) {
                continue;
            }
            StreetSegmentInfo info = getStreetSegmentInfo(seg);
            if (!segmentIsVisible(info, seg, visible)) {
                continue;
            }
            g->set_line_width((zoom >= ZOOM_CLOSE) ? 7 : (zoom >= ZOOM_BLOCK ? 5 : 3));
            drawStreetSegment(g, seg, info);
        }
    }

    // throw on speed limit badges when zoomed in enough to read them
    if (zoom >= ZOOM_CLOSE) {
        drawSpeedLimitBadges(g, visible, palette);
    }
}

// Labels street names along the roads — rotated to follow the road angle,
// and spaced out so they don't overlap each other
void drawStreetNames(ezgl::renderer* g, ZoomLevel zoom,
                     const ezgl::rectangle& visible, const ColorPalette& palette) {
    g->set_color(palette.streetText);
    g->set_horiz_justification(ezgl::justification::center);
    g->set_vert_justification(ezgl::justification::center);

    // only label roads that are visible within the current zoom level
    int minTierToLabel = TIER_HIGHWAY;
    if (zoom >= ZOOM_CLOSE) {
        minTierToLabel = TIER_MINOR;
    } else if (zoom >= ZOOM_BLOCK) {
        minTierToLabel = TIER_MEDIUM;
    } else if (zoom >= ZOOM_DISTRICT) {
        minTierToLabel = TIER_MAJOR;
    }

    std::unordered_set<StreetIdx> labeledStreets;  // avoid duplicate labels

    double worldPerPixel = visible.width() / 1200.0;

    // bigger roads get slightly larger text
    double pixelFontSize;
    if (minTierToLabel <= TIER_HIGHWAY) {
        pixelFontSize = 11.0;
    } else if (minTierToLabel <= TIER_MAJOR) {
        pixelFontSize = 10.0;
    } else {
        pixelFontSize = 9.0;
    }

    // convert to world-space so the collision system can use it
    double fontSize = pixelFontSize * worldPerPixel;

    g->format_font("Noto Sans", ezgl::font_slant::normal, ezgl::font_weight::normal);
    g->set_font_size(pixelFontSize);

    int numSegments = getNumStreetSegments();

    for (StreetSegmentIdx seg = 0; seg < numSegments; ++seg) {
        int tier = appState.segmentTier[seg];
        if (tier > minTierToLabel) {
            continue;
        }

        StreetSegmentInfo info = getStreetSegmentInfo(seg);
        if (!segmentIsVisible(info, seg, visible)) {
            continue;
        }

        std::string streetName = getStreetName(info.streetID);
        if (streetName == "<unknown>" || streetName.empty()) {
            continue;
        }

        if (zoom < ZOOM_CLOSE && labeledStreets.count(info.streetID)) {
            continue;
        }

        ezgl::point2d fromPt = latLonToPoint(intersectionPositions[info.from]);
        ezgl::point2d toPt   = latLonToPoint(intersectionPositions[info.to]);

        // for curvy roads, grab the middle curve point
        // for straight ones, just the midpoint
        ezgl::point2d midPt, dirPt1, dirPt2;
        if (info.numCurvePoints > 0) {
            int midIdx = info.numCurvePoints / 2;
            midPt  = latLonToPoint(getStreetSegmentCurvePoint(seg, midIdx));
            dirPt1 = (midIdx > 0)
                ? latLonToPoint(getStreetSegmentCurvePoint(seg, midIdx - 1))
                : fromPt;
            dirPt2 = (midIdx + 1 < info.numCurvePoints)
                ? latLonToPoint(getStreetSegmentCurvePoint(seg, midIdx + 1))
                : toPt;
        } else {
            midPt  = {(fromPt.x + toPt.x) / 2.0, (fromPt.y + toPt.y) / 2.0};
            dirPt1 = fromPt;
            dirPt2 = toPt;
        }

        double dx = dirPt2.x - dirPt1.x;
        double dy = dirPt2.y - dirPt1.y;
        double angleDeg = std::atan2(dy, dx) * 180.0 / M_PI;
        // flip the label if it would be upside down
        if (angleDeg > 90.0) {
            angleDeg -= 180.0;
        }
        if (angleDeg < -90.0) {
            angleDeg += 180.0;
        }

        // ensure to not cram labels onto tiny segments
        double segLen = std::sqrt(dx * dx + dy * dy);
        if (segLen < fontSize * streetName.length() * CHAR_WIDTH_RATIO * 0.8) {
            continue;
        }

        if (!tryPlaceLabel(midPt, streetName, fontSize, angleDeg)) {
            continue;
        }

        g->set_text_rotation(angleDeg);
        g->draw_text(midPt, streetName, segLen, fontSize * 1.5);
        g->set_text_rotation(0);

        labeledStreets.insert(info.streetID);
    }
}

// triangular arrows on one-way streets so you can tell which direction traffic flows
// draw the arrow on the longest sub-segment
void drawOneWayArrows(ezgl::renderer* g, ZoomLevel zoom,
                      const ezgl::rectangle& visible, const ColorPalette& palette) {
    g->set_color(palette.oneWayArrow);
    int numSegments = getNumStreetSegments();

    double worldPerPixel = visible.width() / 1200.0;
    double arrowLen   = worldPerPixel * 8.0;
    double arrowWidth = worldPerPixel * 5.0;

    for (StreetSegmentIdx seg = 0; seg < numSegments; ++seg) {
        StreetSegmentInfo info = getStreetSegmentInfo(seg);
        if (!info.oneWay) {
            continue;
        }
        if (!segmentIsVisible(info, seg, visible)) {
            continue;
        }

        int tier = appState.segmentTier[seg];
        if (zoom < ZOOM_CLOSE && tier == TIER_MINOR) {
            continue;
        }
        if (zoom < ZOOM_BLOCK && tier == TIER_MEDIUM) {
            continue;
        }

        std::vector<ezgl::point2d> pts;
        pts.push_back(latLonToPoint(intersectionPositions[info.from]));
        for (int c = 0; c < info.numCurvePoints; ++c) {
            pts.push_back(latLonToPoint(getStreetSegmentCurvePoint(seg, c)));
        }
        pts.push_back(latLonToPoint(intersectionPositions[info.to]));

        // find the longest piece to put the arrow on
        double bestLen = 0;
        size_t bestIdx = 0;
        for (size_t i = 0; i + 1 < pts.size(); ++i) {
            double dx = pts[i + 1].x - pts[i].x;
            double dy = pts[i + 1].y - pts[i].y;
            double len = dx * dx + dy * dy;
            if (len > bestLen) {
                bestLen = len;
                bestIdx = i;
            }
        }
        if (bestLen < arrowLen * arrowLen) {
            continue;
        }

        // build the triangle from unit + perpendicular vectors
        ezgl::point2d p1 = pts[bestIdx], p2 = pts[bestIdx + 1];
        double dx  = p2.x - p1.x, dy = p2.y - p1.y;
        double len = std::sqrt(dx * dx + dy * dy);
        double ux = dx / len, uy = dy / len;
        double px = -uy,      py = ux;

        double mx = (p1.x + p2.x) / 2.0, my = (p1.y + p2.y) / 2.0;
        ezgl::point2d tip   = {mx + ux * arrowLen / 2.0, my + uy * arrowLen / 2.0};
        ezgl::point2d left  = {mx - ux * arrowLen / 2.0 + px * arrowWidth / 2.0,
                                my - uy * arrowLen / 2.0 + py * arrowWidth / 2.0};
        ezgl::point2d right = {mx - ux * arrowLen / 2.0 - px * arrowWidth / 2.0,
                                my - uy * arrowLen / 2.0 - py * arrowWidth / 2.0};
        g->fill_poly({tip, left, right});
    }
}

// Map-pin icon for each POI category — matches the colored pin icons in assets/.
// Pin tip at pt (the actual location), colored head circle above with a white
// inner circle and a category symbol character.
void drawPOIIcon(ezgl::renderer* g, const std::string& poiType,
                  ezgl::point2d pt, double r, const ColorPalette& /*palette*/) {
    ezgl::color pinColor;
    const char* symbol;

    // Category → color + symbol  (mirrors assets/ PNG icons)
    if (poiType == "restaurant" || poiType == "fast_food" || poiType == "cafe" ||
        poiType == "bar" || poiType == "pub" || poiType == "food_court" ||
        poiType == "ice_cream" || poiType == "bakery" || poiType == "biergarten") {
        pinColor = ezgl::color(220, 60, 60);        // red   — assets/red.png
        symbol = "F";                                // fork & knife
    } else if (poiType == "school" || poiType == "university" || poiType == "college" ||
               poiType == "library" || poiType == "kindergarten") {
        pinColor = ezgl::color(50, 120, 220);        // blue  — assets/blue.png
        symbol = "E";                                // education / graduation
    } else if (poiType == "hospital" || poiType == "clinic" || poiType == "pharmacy" ||
               poiType == "doctors" || poiType == "dentist" || poiType == "veterinary") {
        pinColor = ezgl::color(60, 180, 75);         // green — assets/green.png
        symbol = "+";                                // medical cross
    } else if (poiType == "bank" || poiType == "atm" || poiType == "post_office" ||
               poiType == "marketplace" || poiType == "supermarket" || poiType == "convenience" ||
               poiType == "shopping_mall" || poiType == "clothes" || poiType == "fuel") {
        pinColor = ezgl::color(240, 160, 30);        // orange — assets/orange.png
        symbol = "$";                                // money / shopping
    } else if (poiType == "parking" || poiType == "bus_station" || poiType == "taxi" ||
               poiType == "bicycle_parking" || poiType == "charging_station" ||
               poiType == "car_rental" || poiType == "ferry_terminal") {
        pinColor = ezgl::color(100, 100, 220);       // indigo — assets/indigo.png
        symbol = "T";                                // transport / bus
    } else if (poiType == "place_of_worship" || poiType == "theatre" || poiType == "cinema" ||
               poiType == "museum" || poiType == "arts_centre" || poiType == "nightclub" ||
               poiType == "community_centre" || poiType == "social_facility") {
        pinColor = ezgl::color(180, 80, 200);        // purple — assets/purple.png
        symbol = "C";                                // culture / theater
    } else if (poiType == "park" || poiType == "garden" || poiType == "playground" ||
               poiType == "sports_centre" || poiType == "stadium" || poiType == "pitch" ||
               poiType == "swimming_pool" || poiType == "fitness_centre") {
        pinColor = ezgl::color(30, 160, 100);        // teal  — assets/teal.png
        symbol = "P";                                // park / recreation
    } else {
        pinColor = ezgl::color(150, 150, 150);       // grey  — assets/grey.png
        symbol = "?";                                // unknown
    }

    // Pin dimensions (head radius slightly larger than old dot for visibility)
    double headR = r * 1.5;
    double pinH  = headR * 2.2;   // distance from tip to head centre
    ezgl::point2d head = {pt.x, pt.y + pinH};

    // Dark border ring for contrast against any background
    g->set_color(ezgl::color(40, 40, 40));
    g->fill_arc(head, headR * 1.12, 0, 360);

    // Colored triangle body connecting head to the pin tip
    g->set_color(pinColor);
    std::vector<ezgl::point2d> tri = {
        {head.x - headR * 0.6, head.y - headR * 0.3},
        {head.x + headR * 0.6, head.y - headR * 0.3},
        {pt.x, pt.y}
    };
    g->fill_poly(tri);

    // Colored head circle
    g->fill_arc(head, headR, 0, 360);

    // White inner circle
    g->set_color(ezgl::WHITE);
    g->fill_arc(head, headR * 0.62, 0, 360);

    // Category symbol centred in the white circle
    g->set_color(pinColor);
    g->set_horiz_justification(ezgl::justification::center);
    g->set_vert_justification(ezgl::justification::center);
    g->set_text_rotation(0);
    g->draw_text(head, symbol, headR * 1.2, headR * 1.2);
}

// Draws all POIs as map-pin icons. Names only appear at close zoom to
// avoid cluttering the map. Labels sit below the pin tip.
void drawPOIs(ezgl::renderer* g, ZoomLevel zoom,
              const ezgl::rectangle& visible, const ColorPalette& palette) {
    int numPOIs = getNumPointsOfInterest();

    double worldPerPixel = visible.width() / 1200.0;
    double pinRadius = worldPerPixel * 4.0;   // base radius for pin head
    double fontSize  = worldPerPixel * 8.0;

    g->set_font_size(8);
    g->set_text_rotation(0);
    g->set_horiz_justification(ezgl::justification::center);
    g->set_vert_justification(ezgl::justification::center);

    for (POIIdx poi = 0; poi < numPOIs; ++poi) {
        LatLon poiPos = getPOIPosition(poi);
        ezgl::point2d pt = latLonToPoint(poiPos);

        if (pt.x < visible.left() || pt.x > visible.right() ||
            pt.y < visible.bottom() || pt.y > visible.top()) {
            continue;
        }

        std::string poiType = getPOIType(poi);
        drawPOIIcon(g, poiType, pt, pinRadius, palette);

        // Label names below the pin tip when zoomed in enough to read them
        if (zoom >= ZOOM_CLOSE) {
            std::string poiName = getPOIName(poi);
            if (!poiName.empty()) {
                ezgl::point2d textPos = {pt.x, pt.y - fontSize * 1.2};
                if (tryPlaceLabel(textPos, poiName, fontSize, 0)) {
                    g->set_color(palette.poiText);
                    g->set_font_size(8);
                    g->draw_text(textPos, poiName, fontSize * poiName.length(),
                                 fontSize * 1.5);
                }
            }
        }
    }
}

// Subway route lines — each drawn in its official color from OSM data
void drawSubwayLines(ezgl::renderer* g, ZoomLevel zoom,
                     const ColorPalette& /*palette*/) {
    if (zoom < ZOOM_CITY) {
        return;  // way too small to see
    }
    g->set_line_dash(ezgl::line_dash::none);

    for (const auto& line : appState.subwayLines) {
        g->set_color(line.lineColor);
        g->set_line_width(3);
        for (const auto& path : line.paths) {
            for (size_t i = 0; i + 1 < path.size(); ++i) {
                g->draw_line(path[i], path[i + 1]);
            }
        }
    }
}

// Station markers — colored ring with a white center
// Names appear once you're zoomed in close enough
void drawSubwayStations(ezgl::renderer* g, ZoomLevel zoom,
                        const ColorPalette& palette) {
    if (zoom < ZOOM_DISTRICT) {
        return;
    }

    double worldPerPixel = g->get_visible_world().width() / 1200.0;
    double stationRadius = worldPerPixel * 5.0;

    for (const auto& line : appState.subwayLines) {
        for (const auto& station : line.stations) {
            g->set_color(line.lineColor);
            g->fill_arc(station.position, stationRadius, 0, 360);

            g->set_color(appState.nightMode
                ? ezgl::color(40, 40, 50)
                : ezgl::color(255, 255, 255));
            g->fill_arc(station.position, stationRadius * 0.6, 0, 360);

            if (zoom >= ZOOM_NEIGHBORHOOD && !station.name.empty()) {
                double fontSize = worldPerPixel * 9.0;
                ezgl::point2d textPos = {station.position.x,
                                          station.position.y + stationRadius * 2.5};
                if (tryPlaceLabel(textPos, station.name, fontSize, 0)) {
                    g->set_color(palette.text);
                    g->set_font_size(9);
                    g->set_text_rotation(0);
                    g->draw_text(textPos, station.name);
                }
            }
        }
    }
}

// curl write callback — same logic as the chat one, just appends data
static size_t curlWriteCallback(char* ptr, size_t size, size_t nmemb,
                                  std::string* data) {
    data->append(ptr, size * nmemb);
    return size * nmemb;
}

// Maps WMO weather codes to short human-readable descriptions
static std::string wmoCodeToDescription(int code) {
    switch (code) {
        case 0:  return "Clear sky";
        case 1:  return "Mainly clear";
        case 2:  return "Partly cloudy";
        case 3:  return "Overcast";
        case 45: case 48: return "Foggy";
        case 51: case 53: case 55: return "Drizzle";
        case 56: case 57: return "Freezing drizzle";
        case 61: case 63: case 65: return "Rain";
        case 66: case 67: return "Freezing rain";
        case 71: case 73: case 75: return "Snowfall";
        case 77: return "Snow grains";
        case 80: case 81: case 82: return "Rain showers";
        case 85: case 86: return "Snow showers";
        case 95: return "Thunderstorm";
        case 96: case 99: return "Thunderstorm w/ hail";
        default: return "Unknown";
    }
}

// Extracts a numeric value for "key" inside the "current_weather":{...} block
static double extractWeatherNumber(const std::string& json, const std::string& key) {
    // find the actual data block, not the units block
    size_t cwPos = json.find("\"current_weather\":{");
    if (cwPos == std::string::npos) return -999;

    std::string needle = "\"" + key + "\"";
    size_t pos = json.find(needle, cwPos + 19);
    if (pos == std::string::npos) return -999;
    pos = json.find(':', pos + needle.length());
    if (pos == std::string::npos) return -999;
    pos++; // skip ':'
    while (pos < json.length() && json[pos] == ' ') pos++;
    try {
        return std::stod(json.substr(pos));
    } catch (...) {
        return -999;
    }
}

// Fetches weather from Open-Meteo in a background thread so it does not freeze the UI
void fetchWeatherAsync(double lat, double lon) {
    std::thread([lat, lon]() {
        std::string url = "https://api.open-meteo.com/v1/forecast?latitude=" +
                          std::to_string(lat) + "&longitude=" + std::to_string(lon) +
                          "&current_weather=true";

        CURL* curl = curl_easy_init();
        if (!curl) {
            return;
        }

        std::string response;
        curl_easy_setopt(curl, CURLOPT_URL,            url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION,  curlWriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA,      &response);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT,          5L);
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER,  0L);
        curl_easy_setopt(curl, CURLOPT_USERAGENT,       "ECE297-Map/1.0");
        curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION,  1L);

        CURLcode res = curl_easy_perform(curl);
        curl_easy_cleanup(curl);

        std::lock_guard<std::mutex> lock(appState.weatherMutex);
        if (res == CURLE_OK && !response.empty()) {
            double temp = extractWeatherNumber(response, "temperature");
            int weatherCode = static_cast<int>(extractWeatherNumber(response, "weathercode"));
            double windSpeed = extractWeatherNumber(response, "windspeed");

            if (temp > -999) {
                std::ostringstream oss;
                oss << wmoCodeToDescription(weatherCode) << " "
                    << std::fixed << std::setprecision(1) << temp << "°C";
                if (windSpeed > -999) {
                    oss << ", wind " << std::setprecision(0) << windSpeed << " km/h";
                }
                appState.weatherInfo = oss.str();
            } else {
                appState.weatherInfo = "Weather unavailable";
            }
        } else {
            appState.weatherInfo = "Weather unavailable";
        }

        // GTK isn't thread-safe, so the redraw is scheduled on the main thread
        if (appState.ezglApp) {
            g_idle_add([](gpointer) -> gboolean {
                if (appState.ezglApp) {
                    appState.ezglApp->refresh_drawing();
                }
                return G_SOURCE_REMOVE;
            }, nullptr);
        }
    }).detach();
}

// Little weather box in the bottom-right corner
// use screen coords so it stays put no matter where you pan or zoom
void drawWeatherOverlay(ezgl::renderer* g, const ColorPalette& palette) {
    std::string info;
    {
        std::lock_guard<std::mutex> lock(appState.weatherMutex);
        info = appState.weatherInfo;
    }
    if (info.empty()) {
        return;
    }

    g->set_coordinate_system(ezgl::SCREEN);
    ezgl::rectangle screen = g->get_visible_screen();

    const double boxW = 240.0, boxH = 28.0, margin = 8.0;
    double bx = screen.right() - boxW - margin;
    double by = screen.top()   - boxH - margin;

    g->set_color(palette.building.red, palette.building.green,
                 palette.building.blue, 210);
    g->fill_rectangle({bx, by}, {bx + boxW, by + boxH});
    g->set_color(palette.minorRoadOutline);
    g->set_line_width(1);
    g->draw_rectangle({bx, by}, {bx + boxW, by + boxH});

    g->set_color(palette.text);
    g->set_font_size(10);
    g->format_font("Noto Sans", ezgl::font_slant::normal,
                   ezgl::font_weight::normal);
    g->set_horiz_justification(ezgl::justification::left);
    g->set_vert_justification(ezgl::justification::center);
    g->draw_text({bx + 8.0, by + boxH / 2.0}, "Weather: " + info);

    g->set_coordinate_system(ezgl::WORLD);
}

// Scale bar in the top-left (100m, 500m, 1km, etc.) 
void drawDistanceScale(ezgl::renderer* g, const ColorPalette& palette,
                        const ezgl::rectangle& visible) {
    g->set_coordinate_system(ezgl::SCREEN);
    ezgl::rectangle screen = g->get_visible_screen();

    double worldPerPixel = visible.width() / screen.width();
    double rawMeters = 150.0 * worldPerPixel;

    const double niceSteps[] =
        {10, 25, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 50000};
    double niceDist = niceSteps[0];
    for (double s : niceSteps) {
        if (s <= rawMeters) {
            niceDist = s;
        } else {
            break;
        }
    }

    double barPixels = niceDist / worldPerPixel;
    double bx = 12.0, by = screen.bottom() + 18.0;

    g->set_color(palette.text);
    g->set_line_width(2);
    g->draw_line({bx,             by - 6.0}, {bx,             by});
    g->draw_line({bx,             by},        {bx + barPixels, by});
    g->draw_line({bx + barPixels, by},        {bx + barPixels, by - 6.0});

    std::string label = (niceDist >= 1000.0)
        ? std::to_string(static_cast<int>(niceDist / 1000.0)) + " km"
        : std::to_string(static_cast<int>(niceDist))           + " m";

    g->set_font_size(9);
    g->format_font("Noto Sans", ezgl::font_slant::normal,
                   ezgl::font_weight::normal);
    g->set_horiz_justification(ezgl::justification::center);
    g->set_vert_justification(ezgl::justification::bottom);
    g->draw_text({bx + barPixels / 2.0, by - 8.0}, label);

    g->set_coordinate_system(ezgl::WORLD);
}

// Highlights for selected intersections: a red dot 
void drawHighlights(ezgl::renderer* g, const ColorPalette& palette) {
    if (appState.highlightedIntersections.empty()) {
        return;
    }

    double worldPerPixel  = g->get_visible_world().width() / 1200.0;
    double highlightRadius = worldPerPixel * 7.0;

    for (IntersectionIdx idx : appState.highlightedIntersections) {
        if (idx < 0 || idx >= static_cast<int>(intersectionPositions.size())) {
            continue;
        }

        ezgl::point2d pt = latLonToPoint(intersectionPositions[idx]);

        g->set_color(palette.highlight.red, palette.highlight.green,
                     palette.highlight.blue, 80);
        g->fill_arc(pt, highlightRadius * 1.8, 0, 360);

        g->set_color(palette.highlight);
        g->fill_arc(pt, highlightRadius, 0, 360);

        g->set_color(ezgl::WHITE);
        g->fill_arc(pt, highlightRadius * 0.35, 0, 360);
    }
}

// Draws a path segment with all its curve points in the given color
static void drawPathSegment(ezgl::renderer* g, StreetSegmentIdx seg) {
    StreetSegmentInfo info = getStreetSegmentInfo(seg);
    ezgl::point2d prevPt = latLonToPoint(intersectionPositions[info.from]);

    for (int c = 0; c < info.numCurvePoints; ++c) {
        ezgl::point2d curvePt = latLonToPoint(getStreetSegmentCurvePoint(seg, c));
        g->draw_line(prevPt, curvePt);
        prevPt = curvePt;
    }

    g->draw_line(prevPt, latLonToPoint(intersectionPositions[info.to]));
}

// Draws the current navigation path (walk in orange, drive in red)
void drawPath(ezgl::renderer* g, const ColorPalette& /*palette*/) {
    g->set_line_cap(ezgl::line_cap::round);
    g->set_line_dash(ezgl::line_dash::none);

    double worldPerPixel = g->get_visible_world().width() / 1200.0;

    // Draw walking path in orange (dashed)
    if (!appState.currentWalkPath.empty()) {
        g->set_color(ezgl::color(255, 140, 0));  // orange
        g->set_line_width(6);
        g->set_line_dash(ezgl::line_dash::asymmetric_5_3);
        for (StreetSegmentIdx seg : appState.currentWalkPath) {
            drawPathSegment(g, seg);
        }
        g->set_line_dash(ezgl::line_dash::none);
    }

    // Draw driving path in red (solid, outline then fill)
    if (!appState.currentDrivePath.empty()) {
        g->set_color(ezgl::color(160, 20, 20));  // dark red outline
        g->set_line_width(8);
        for (StreetSegmentIdx seg : appState.currentDrivePath) {
            drawPathSegment(g, seg);
        }
        g->set_color(ezgl::color(220, 55, 55));  // bright red fill
        g->set_line_width(5);
        for (StreetSegmentIdx seg : appState.currentDrivePath) {
            drawPathSegment(g, seg);
        }
    }

    // Annotate street names along the path
    auto annotatePath = [&](const std::vector<StreetSegmentIdx>& path,
                            ezgl::color textColor) {
        if (path.empty()) return;
        std::unordered_set<StreetIdx> labeledStreets;
        g->set_color(textColor);
        g->set_font_size(10);
        g->set_horiz_justification(ezgl::justification::center);
        g->set_vert_justification(ezgl::justification::center);

        for (size_t i = 0; i < path.size(); ++i) {
            StreetSegmentInfo info = getStreetSegmentInfo(path[i]);
            if (labeledStreets.count(info.streetID)) continue;

            std::string name = getStreetName(info.streetID);
            if (name.empty() || name == "<unknown>") continue;

            // Place label at midpoint of this segment
            ezgl::point2d fromPt = latLonToPoint(intersectionPositions[info.from]);
            ezgl::point2d toPt   = latLonToPoint(intersectionPositions[info.to]);
            ezgl::point2d mid = {(fromPt.x + toPt.x) / 2.0,
                                  (fromPt.y + toPt.y) / 2.0};

            double dx = toPt.x - fromPt.x;
            double dy = toPt.y - fromPt.y;
            double angleDeg = std::atan2(dy, dx) * 180.0 / M_PI;
            if (angleDeg > 90.0)  angleDeg -= 180.0;
            if (angleDeg < -90.0) angleDeg += 180.0;

            double segLen = std::sqrt(dx * dx + dy * dy);
            double fontSize = worldPerPixel * 9.0;
            if (segLen < fontSize * name.length() * 0.5) continue;

            if (tryPlaceLabel(mid, name, fontSize, angleDeg)) {
                g->set_text_rotation(angleDeg);
                g->draw_text(mid, name, segLen, fontSize * 1.5);
                g->set_text_rotation(0);
                labeledStreets.insert(info.streetID);
            }
        }
    };

    annotatePath(appState.currentWalkPath, ezgl::color(255, 220, 160));   // light orange
    annotatePath(appState.currentDrivePath, ezgl::color(255, 180, 180));  // light red

    // Determine start and end intersections from the path
    IntersectionIdx pathStart = -1;
    IntersectionIdx pathEnd = -1;

    const std::vector<StreetSegmentIdx>& startPath =
        !appState.currentWalkPath.empty() ? appState.currentWalkPath
                                          : appState.currentDrivePath;
    if (!startPath.empty()) {
        StreetSegmentInfo firstInfo = getStreetSegmentInfo(startPath[0]);
        if (startPath.size() > 1) {
            StreetSegmentInfo secondInfo = getStreetSegmentInfo(startPath[1]);
            if (firstInfo.from == secondInfo.from || firstInfo.from == secondInfo.to) {
                pathStart = firstInfo.to;
            } else {
                pathStart = firstInfo.from;
            }
        } else {
            pathStart = firstInfo.from;
        }
    }
    if (!appState.currentDrivePath.empty()) {
        StreetSegmentInfo lastInfo = getStreetSegmentInfo(
            appState.currentDrivePath.back());
        if (appState.currentDrivePath.size() > 1) {
            StreetSegmentInfo prevInfo = getStreetSegmentInfo(
                appState.currentDrivePath[appState.currentDrivePath.size() - 2]);
            if (lastInfo.to == prevInfo.from || lastInfo.to == prevInfo.to) {
                pathEnd = lastInfo.from;
            } else {
                pathEnd = lastInfo.to;
            }
        } else {
            pathEnd = lastInfo.to;
        }
    }

    // Use navStart/navEnd if set (during intermediate click)
    if (appState.navStart >= 0) pathStart = appState.navStart;
    if (appState.navEnd >= 0) pathEnd = appState.navEnd;

    // Draw start marker (green circle)
    if (pathStart >= 0) {
        double r = worldPerPixel * 10.0;
        ezgl::point2d startPt = latLonToPoint(intersectionPositions[pathStart]);
        g->set_color(ezgl::color(34, 180, 34));
        g->fill_arc(startPt, r, 0, 360);
        g->set_color(ezgl::WHITE);
        g->fill_arc(startPt, r * 0.4, 0, 360);
    }

    // Draw end marker (red circle)
    if (pathEnd >= 0) {
        double r = worldPerPixel * 10.0;
        ezgl::point2d endPt = latLonToPoint(intersectionPositions[pathEnd]);
        g->set_color(ezgl::color(220, 40, 40));
        g->fill_arc(endPt, r, 0, 360);
        g->set_color(ezgl::WHITE);
        g->fill_arc(endPt, r * 0.4, 0, 360);
    }
}

// Format seconds into a compact human-readable string
std::string formatTime(double seconds) {
    int total = static_cast<int>(seconds + 0.5);
    if (total < 60) {
        return std::to_string(total) + "s";
    } else if (total < 3600) {
        int m = total / 60;
        int s = total % 60;
        if (s == 0) return std::to_string(m) + "m";
        return std::to_string(m) + "m " + std::to_string(s) + "s";
    } else {
        int h = total / 3600;
        int m = (total % 3600) / 60;
        if (m == 0) return std::to_string(h) + "h";
        return std::to_string(h) + "h " + std::to_string(m) + "m";
    }
}

// Generates human-readable turn-by-turn directions with per-step estimated time.
// walkSpeed is only used when isWalking is true; turnPenalty adds time at street changes.
std::string generateDirections(const std::vector<StreetSegmentIdx>& path,
                                bool isWalking, double walkSpeed,
                                double turnPenalty) {
    if (path.empty()) return "";

    std::ostringstream dirs;
    std::string mode = isWalking ? "Walk" : "Drive";

    StreetSegmentInfo firstSeg = getStreetSegmentInfo(path[0]);
    std::string currentStreet = getStreetName(firstSeg.streetID);
    double currentDistance = segmentLengths[path[0]];
    // Accumulate travel time for the current step
    double currentTime = isWalking
        ? segmentLengths[path[0]] / walkSpeed
        : segmentTravelTimes[path[0]];
    int stepNum = 1;

    dirs << stepNum << ". " << mode << " along " << currentStreet;

    for (size_t i = 1; i < path.size(); ++i) {
        StreetSegmentInfo seg = getStreetSegmentInfo(path[i]);
        std::string nextStreet = getStreetName(seg.streetID);

        double segTime = isWalking
            ? segmentLengths[path[i]] / walkSpeed
            : segmentTravelTimes[path[i]];

        if (seg.streetID == firstSeg.streetID) {
            // Same street — accumulate distance and time
            currentDistance += segmentLengths[path[i]];
            currentTime += segTime;
            firstSeg = seg;
        } else {
            // Street changed — emit direction step with distance and time
            dirs << " for ";
            if (currentDistance >= 1000.0) {
                dirs << std::fixed << std::setprecision(1)
                     << currentDistance / 1000.0 << " km";
            } else {
                dirs << static_cast<int>(currentDistance) << " m";
            }
            dirs << " (~" << formatTime(currentTime) << ")\n";
            stepNum++;

            // Turn penalty when changing streets
            currentTime = segTime + turnPenalty;

            double angle = findStreetSegmentTurnAngle(path[i], path[i - 1]);
            std::string turnDir;
            if (angle > 0.5) {
                turnDir = "Turn left";
            } else if (angle < -0.5) {
                turnDir = "Turn right";
            } else {
                turnDir = "Continue";
            }

            dirs << stepNum << ". " << turnDir << " onto " << nextStreet;

            currentStreet = nextStreet;
            currentDistance = segmentLengths[path[i]];
            firstSeg = seg;
        }
    }

    // Final segment distance and time
    dirs << " for ";
    if (currentDistance >= 1000.0) {
        dirs << std::fixed << std::setprecision(1)
             << currentDistance / 1000.0 << " km";
    } else {
        dirs << static_cast<int>(currentDistance) << " m";
    }
    dirs << " (~" << formatTime(currentTime) << ")\n";

    return dirs.str();
}

// The main draw callback
void drawMainCanvas(ezgl::renderer* g) {
    const ColorPalette& palette = getPalette();
    ezgl::rectangle actualWorld = g->get_visible_world();
    ezgl::rectangle visible = actualWorld;

    // limits on max zoom and min zoom levels
    double w = visible.width();
    if (w > MAX_ZOOM_OUT_WIDTH || w < MIN_ZOOM_IN_WIDTH) {
        double targetW     = std::max(MIN_ZOOM_IN_WIDTH, std::min(w, MAX_ZOOM_OUT_WIDTH));
        double aspectRatio = visible.height() / w;
        double targetH     = targetW * aspectRatio;
        double cx = (actualWorld.left() + actualWorld.right()) / 2.0;
        double cy = (actualWorld.bottom() + actualWorld.top()) / 2.0;
        visible = ezgl::rectangle({cx - targetW / 2.0, cy - targetH / 2.0},
                                  {cx + targetW / 2.0, cy + targetH / 2.0});
        if (appState.ezglApp) {
            appState.ezglApp->change_canvas_world_coordinates("MainCanvas", visible);
        }
        // still draw this frame so we don't flash a blank screen
    }

    ZoomLevel zoom = getZoomLevel(visible.width());

    // clear last frame's label positions
    appState.placedLabels.clear();

    // paint the background first
    g->set_color(palette.background);
    g->fill_rectangle(actualWorld);

    // if it's still loading, just show a loading message
    if (appState.mapLoading || appState.segmentTier.empty()) {
        g->set_coordinate_system(ezgl::SCREEN);
        g->set_color(palette.text);
        g->set_font_size(16);
        g->set_horiz_justification(ezgl::justification::center);
        g->set_vert_justification(ezgl::justification::center);
        ezgl::rectangle screen = g->get_visible_screen();
        g->draw_text({screen.width() / 2.0, screen.height() / 2.0}, "Loading map...");
        g->set_coordinate_system(ezgl::WORLD);
        return;
    }

    drawFeatures(g, zoom, visible, palette);
    drawStreets(g, zoom, visible, palette);

    if (zoom >= ZOOM_BLOCK) {
        drawOneWayArrows(g, zoom, visible, palette);
    }
    if (zoom >= ZOOM_DISTRICT) {
        drawStreetNames(g, zoom, visible, palette);
    }

    if (appState.showSubway) {
        drawSubwayLines(g, zoom, palette);
        drawSubwayStations(g, zoom, palette);
    }

    if (appState.showPOIs && zoom >= ZOOM_BLOCK) {
        drawPOIs(g, zoom, visible, palette);
    }

    drawHighlights(g, palette);
    drawPath(g, palette);
    drawWeatherOverlay(g, palette);
    drawDistanceScale(g, palette, visible);
}
