/*
 * m2_precompute.cpp
 *
 * All map-load-time precomputation functions.  Called once from initialSetup()
 * (and again from reloadMap() when the user loads a different map).
 *
 *   precomputeSegmentTiers()    — classify each road segment into 4 tiers
 *   precomputeFeatureDrawOrder() — sort geographic features largest-first
 *   precomputeSubwayData()      — extract subway routes from OSM relations
 *   precomputeSpeedLimits()     — parse OSM maxspeed tags per segment
 *   setupAutocomplete()         — attach GtkEntryCompletion to search fields
 */

#include "m2_shared.h"

// Reads the OSM "highway" tag for every street segment's parent Way and assigns
// it to one of four tiers. Falls back to speed-limit-based classification when
// the OSM way is not in osmWayMap.

void precomputeSegmentTiers() {
    int numSegments = getNumStreetSegments();
    appState.segmentTier.resize(numSegments, TIER_MINOR);

    for (StreetSegmentIdx seg = 0; seg < numSegments; ++seg) {
        StreetSegmentInfo info = getStreetSegmentInfo(seg);

        // Try to find the OSM Way for this segment so we can read its tags
        auto wayIt = osmWayMap.find(info.wayOSMID);
        if (wayIt == osmWayMap.end()) {
            // Can't find the Way -> guess tier based on speed limit instead.
            // Thresholds are in m/s (the unit used by StreetSegmentInfo):
            // 27.0 m/s ≈ 97 km/h -> highway
            // 16.0 m/s ≈ 58 km/h -> major arterial
            // 11.0 m/s ≈ 40 km/h -> collector / medium road
            // anything slower -> minor / local road (default)
            if (info.speedLimit > 27.0f) {
                appState.segmentTier[seg] = TIER_HIGHWAY;
            } else if (info.speedLimit > 16.0f) {
                appState.segmentTier[seg] = TIER_MAJOR;
            } else if (info.speedLimit > 11.0f) {
                appState.segmentTier[seg] = TIER_MEDIUM;
            }
            continue;
        }

        // Look for the "highway" tag which tells us the road classification
        const OSMWay* way = wayIt->second;
        std::string highwayType;

        int tagCount = getTagCount(way);
        for (int t = 0; t < tagCount; ++t) {
            auto tag = getTagPair(way, t);
            if (tag.first == "highway") {
                highwayType = tag.second;
                break;
            }
        }

        // Map OSM highway values to our 4-tier system
        if (highwayType == "motorway"  || highwayType == "motorway_link" ||
            highwayType == "trunk"     || highwayType == "trunk_link") {
            appState.segmentTier[seg] = TIER_HIGHWAY;
        } else if (highwayType == "primary"   || highwayType == "primary_link" ||
                   highwayType == "secondary" || highwayType == "secondary_link") {
            appState.segmentTier[seg] = TIER_MAJOR;
        } else if (highwayType == "tertiary"  || highwayType == "tertiary_link") {
            appState.segmentTier[seg] = TIER_MEDIUM;
        } else {
            appState.segmentTier[seg] = TIER_MINOR;
        }
    }
}

// Converts every geographic feature to world coordinates and sorts them so that
// large closed polygons (lakes, parks) are drawn before small ones, and open
// features (rivers, streams) are drawn last.

void precomputeFeatureDrawOrder() {
    int numFeatures = getNumFeatures();
    appState.featureDrawOrder.clear();
    appState.featureDrawOrder.reserve(numFeatures);

    for (FeatureIdx feat = 0; feat < numFeatures; ++feat) {
        FeatureDrawInfo info;
        info.id   = feat;
        info.type = getFeatureType(feat);
        info.area = findFeatureArea(feat);  // from M1

        // Convert all feature points to world coordinates once
        int numPts = getNumFeaturePoints(feat);
        info.points.reserve(numPts);
        for (int p = 0; p < numPts; ++p) {
            info.points.push_back(latLonToPoint(getFeaturePoint(feat, p)));
        }

        // A feature is "closed" (a polygon) if its first and last LatLon points
        // are the same. We use an valu of  about0.1 mm instead of
        // exact equality to tolerate floating point rounding in the OSM data.
        // Features with fewer than 3 points can't form a polygon.
        if (numPts >= 3) {
            LatLon first = getFeaturePoint(feat, 0);
            LatLon last  = getFeaturePoint(feat, numPts - 1);
            info.isClosed = (std::abs(first.latitude()  - last.latitude())  < 1e-9 &&
                             std::abs(first.longitude() - last.longitude()) < 1e-9);
        } else {
            info.isClosed = false;
        }

        appState.featureDrawOrder.push_back(std::move(info));
    }

    // Sort so that large closed polygons (parks, lakes) are drawn first and
    // therefore appear beneath smaller features. Open polylines (rivers,
    // streams) are forced to the back of the closed-polygon group so they
    // always render on top of filled areas. 
    std::sort(appState.featureDrawOrder.begin(),
              appState.featureDrawOrder.end(),
              [](const FeatureDrawInfo& a, const FeatureDrawInfo& b) {
                  // Closed polygons always come before open polylines
                  if (a.isClosed && !b.isClosed) {
                      return true;
                  }
                  if (!a.isClosed && b.isClosed) {
                      return false;
                  }
                  // Within the same group, larger area drawn first
                  return a.area > b.area;
              });
}

// Scans OSM relations for route=subway entries and builds the SubwayLine list
// used by drawSubwayLines() and drawSubwayStations().

void precomputeSubwayData() {
    appState.subwayLines.clear();
    int numRelations = getNumberOfRelations();

    // Scan all OSM relations looking for subway routes
    for (int i = 0; i < numRelations; ++i) {
        const OSMRelation* rel = getRelationByIndex(i);

        // Check the tags to see if this relation is a subway route
        std::string routeType, lineName, colorStr;
        int tagCount = getTagCount(rel);
        for (int t = 0; t < tagCount; ++t) {
            auto tag = getTagPair(rel, t);
            if (tag.first == "route" && tag.second == "subway") {
                routeType = "subway";
            }
            if (tag.first == "name") {
                lineName = tag.second;
            }
            if (tag.first == "colour") {
                colorStr = tag.second;  //Hex color like #FF0000
            }
        }
        if (routeType != "subway") {
            continue;  // Not a subway, skip
        }

        // Build a SubwayLine entry; fall back to a default red if no colour tag
        SubwayLine line;
        line.name      = lineName;
        line.lineColor = colorStr.empty()
            ? ezgl::color(200, 50, 50)
            : parseHexColor(colorStr);

        // A subway relation contains Way members (the track paths) and
        // Node members (the stations). Process both.
        std::vector<TypedOSMID> members = getRelationMembers(rel);
        for (const auto& member : members) {
            if (member.type() == TypedOSMID::Way) {
                // Way members represent segments of the physical track

                auto wayIt = osmWayMap.find(member);
                if (wayIt == osmWayMap.end()) {
                    continue; // Way not in our OSM map (data gap), skip it
                }

                // Convert the ordered list of node IDs into world-space points
                const std::vector<OSMID>& nodes = getWayMembers(wayIt->second);
                std::vector<ezgl::point2d> path;
                path.reserve(nodes.size());
                for (const auto& nodeId : nodes) {
                    auto nodeIt = osmNodeById.find(nodeId);
                    if (nodeIt != osmNodeById.end()) {
                        path.push_back(latLonToPoint(getNodeCoords(nodeIt->second)));
                    }
                }

                // Only keep paths that have at least two points (drawable segments)
                if (path.size() >= 2) {
                    line.paths.push_back(std::move(path));
                }

            } else if (member.type() == TypedOSMID::Node) {
                // Node members represent individual subway stations

                auto nodeIt = osmNodeById.find(member);
                if (nodeIt == osmNodeById.end()) {
                    continue;  // Node not found in our OSM map, skip it
                }

                SubwayStation station;
                station.position = latLonToPoint(getNodeCoords(nodeIt->second));

                // Pull the station's display name from its OSM tags
                int nodeTags = getTagCount(nodeIt->second);
                for (int t = 0; t < nodeTags; ++t) {
                    auto tag = getTagPair(nodeIt->second, t);
                    if (tag.first == "name") {
                        station.name = tag.second;
                        break;
                    }
                }
                line.stations.push_back(std::move(station));
            }
        }

        // Only store the line if it has at least some drawable data
        if (!line.paths.empty() || !line.stations.empty()) {
            appState.subwayLines.push_back(std::move(line));
        }
    }
}

// Reads the OSM "maxspeed" tag from each segment's parent Way.  Falls back to
// StreetSegmentInfo.speedLimit (m/s) when the tag is absent.  Stores non-zero
// results in appState.speedLimitKmh for in-app display and M3 Dijkstra use.

void precomputeSpeedLimits() {
    appState.speedLimitKmh.clear();
    int numSegs = getNumStreetSegments();
    appState.speedLimitKmh.reserve(numSegs);

    for (StreetSegmentIdx seg = 0; seg < numSegs; ++seg) {
        StreetSegmentInfo info = getStreetSegmentInfo(seg);
        int kmh = 0;

        // First try: read the "maxspeed" tag from OSM
        auto wayIt = osmWayMap.find(info.wayOSMID);
        if (wayIt != osmWayMap.end()) {
            int tagCount = getTagCount(wayIt->second);
            for (int t = 0; t < tagCount; ++t) {
                auto tag = getTagPair(wayIt->second, t);
                if (tag.first == "maxspeed") {
                    try {
                        kmh = std::stoi(tag.second);
                        // Some OSM regions (e.g. the US, UK) tag speeds in mph.
                        // 1 mile = 1.60934 km, so multiply to convert to km/h.
                        if (tag.second.find("mph") != std::string::npos) {
                            kmh = static_cast<int>(kmh * 1.60934);
                        }
                    } catch (...) {
                        // OSM sometimes uses descriptive values instead of numbers
                        if (tag.second == "urban") {
                            kmh = 50;
                        } else if (tag.second == "rural") {
                            kmh = 90;
                        } else if (tag.second == "motorway") {
                            kmh = 100;
                        }
                    }
                    break;
                }
            }
        }

        // Fallback: StreetSegmentInfo.speedLimit is stored in m/s by the API.
        // Multiply by 3.6 to convert to km/h  (1 m/s = 3600 m/h = 3.6 km/h).
        if (kmh == 0 && info.speedLimit > 0) {
            kmh = static_cast<int>(info.speedLimit * 3.6f);
        }

        if (kmh > 0) {
            appState.speedLimitKmh[seg] = kmh;
        }
    }
}

// Builds a GtkListStore of all unique street names and attaches a
// GtkEntryCompletion widget to both search entry fields in the UI.

// Sets up autocomplete dropdowns on the two search entry fields.
// Collects all unique street names into a GtkListStore and attaches
// a GtkEntryCompletion to each field so the user gets suggestions as they type.
void setupAutocomplete(ezgl::application* app) {
    // Collect every unique street name (skip "<unknown>" placeholder names)
    std::set<std::string> uniqueNames;
    int numStreets = getNumStreets();
    for (int i = 0; i < numStreets; ++i) {
        std::string name = getStreetName(i);
        if (name != "<unknown>") {
            uniqueNames.insert(name);
        }
    }

    // Gtk_list_store_new(1, G_TYPE_STRING): create a store with exactly 1 column
    // of type string. The trailing -1 in gtk_list_store_set marks the end of
    GtkListStore* store = gtk_list_store_new(1, G_TYPE_STRING);
    GtkTreeIter iter;
    for (const auto& name : uniqueNames) {
        gtk_list_store_append(store, &iter);
        gtk_list_store_set(store, &iter, 0, name.c_str(), -1);
    }

    // Attach the completion widget to both search entries
    const char* entryIds[] = {"SearchEntry1", "SearchEntry2"};
    for (const char* entryId : entryIds) {
        GtkWidget* widget = app->find_widget(entryId);
        if (!widget) {
            continue;
        }

        GtkEntry* entry = GTK_ENTRY(widget);
        GtkEntryCompletion* completion = gtk_entry_completion_new();
        gtk_entry_completion_set_model(completion, GTK_TREE_MODEL(store));
        gtk_entry_completion_set_text_column(completion, 0);
        gtk_entry_completion_set_minimum_key_length(completion, 1);
        gtk_entry_completion_set_popup_completion(completion, TRUE);
        gtk_entry_set_completion(entry, completion);


        // When the user picks from the dropdown, set a flag so actOnMousePress
        // knows to ignore the click-through that GTK sends to the canvas.
        g_signal_connect(completion, "match-selected",
            G_CALLBACK(+[](GtkEntryCompletion*, GtkTreeModel*, GtkTreeIter*, gpointer) -> gboolean {
                appState.autocompleteJustSelected = true;
                return FALSE;  // let GTK fill the entry text as usual
            }), nullptr);


        g_object_unref(completion);
    }

    g_object_unref(store); // The completions hold their own refs
}
