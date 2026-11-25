/*
 * m2_callbacks.cpp
 *
 * EZGL event callbacks and GTK button handlers — all user interaction
 * lives here.  Nothing in this file draws directly; it mutates AppState
 * and then triggers redraws so m2_draw.cpp picks up the changes.
 *
 * Key functions:
 *   initialSetup()      — runs once after the EZGL window opens; creates
 *                          programmatic buttons, kicks off precomputation,
 *                          fetches weather, wires up autocomplete
 *   actOnMouseClick()   — finds the nearest intersection and highlights it
 *   actOnKeyPress()     — keyboard shortcuts (night mode, POIs, subway, etc.)
 *   onFindButton()      — intersection search from the two search entries
 *   onNightMode/onTogglePOIs/onToggleSubway — UI toggle callbacks
 *   onLoadMap()         — opens a file dialog, tears down old map, reloads
 *   onRefreshWeather()  — re-fetches weather for the current map centre
 *   onChatSend()        — sends user message + map context to Gemini API
 *   buildContextString() — assembles map metadata for AI chat context
 */

#include "m2_shared.h"

// Forward declarations for helpers used across this file
static void exitNavInputMode(ezgl::application* app);
static void updateNavButtonLabels();
static void handleNavTextInput(ezgl::application* app);
static void executePathfinding(ezgl::application* app,
                                IntersectionIdx start, IntersectionIdx end);

// Runs once after the EZGL window is created. Sets up the whole UI
// and kicks off all precomputation for the initially loaded map.
void initialSetup(ezgl::application* app, bool) {
    appState.ezglApp = app;

    // Load our custom CSS for the dark theme sidebar and chat panel
    GtkCssProvider* cssProvider = gtk_css_provider_new();
    GError* cssError = nullptr;
    gboolean cssLoaded = gtk_css_provider_load_from_path(
        cssProvider, "libstreetmap/resources/style.css", &cssError);
    if (cssLoaded) {
        // GTK_STYLE_PROVIDER_PRIORITY_APPLICATION (600) is high enough to
        // override the default theme but lower than inline style attributes,
        // so it won't clobber any per widget overrides we set elsewhere.
        gtk_style_context_add_provider_for_screen(
            gdk_screen_get_default(),
            GTK_STYLE_PROVIDER(cssProvider),
            GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);
    } else if (cssError) {
        g_printerr("Spatia: CSS load error: %s\n", cssError->message);
        g_error_free(cssError);
    }
    g_object_unref(cssProvider);

    // Do all the heavy lifting up front so drawing is fast.
    // These get called again in reloadMap() when switching cities.
    precomputeSegmentTiers();
    precomputeFeatureDrawOrder();
    precomputeSubwayData();
    precomputeSpeedLimits();

    // Fetch weather for the map centre. Averaging the bounding box extremes
    // gives a representative lat/lon without needing the actual centroid.
    double centerLat = (geoMinLat + geoMaxLat) / 2.0;
    double centerLon = (geoMinLon + geoMaxLon) / 2.0;
    fetchWeatherAsync(centerLat, centerLon);

    // Sidebar buttons —> the numbers are row positions in the GTK grid
    app->create_button("\u2605  Night Mode",      7,  onNightModeToggled);
    app->create_button("\u25CF  Toggle POIs",     8,  onTogglePOIs);
    app->create_button("\u29C9  Toggle Subway",   9,  onToggleSubway);
    app->create_button("\u2B1C  Load Map",        10, onLoadMapClicked);
    app->create_button("\u21BA  Weather",         11, onRefreshWeather);
    app->create_button("\u2709  Chat",            12, onToggleChat);
    app->create_button("\u279C  Navigate",        13, onNavigateClicked);
    app->create_button("\u26A1  Walk+Drive",      14, onWalkDriveClicked);
    app->create_button("\u2715  Clear Path",      15, onClearPathClicked);
    app->create_button("\u2753  Help",            16, onHelpClicked);

    // Grab button widget refs so we can toggle their labels later
    GtkWidget* innerGrid = GTK_WIDGET(app->get_object("InnerGrid"));
    if (innerGrid) {
        appState.navigateBtn  = gtk_grid_get_child_at(GTK_GRID(innerGrid), 0, 13);
        appState.walkDriveBtn = gtk_grid_get_child_at(GTK_GRID(innerGrid), 0, 14);
    }

    // Connect Find button from main.ui
    GtkWidget* findBtn = app->find_widget("FindButton");
    if (findBtn) {
        g_signal_connect(findBtn, "clicked", G_CALLBACK(onFindButtonClicked), app);
    }

    // Chat widget signals are wired lazily in onToggleChat —> if we try
    // to connect them here and the widgets don't exist, GTK prints warnings
    setupAutocomplete(app);

    app->update_message("Map loaded. Click an intersection or search for streets.");

    // Move focus to the canvas so the first map click works immediately.
    // Without this, GTK auto-focuses SearchEntry1 (first focusable widget).
    GtkWidget* canvas = GTK_WIDGET(app->get_object("MainCanvas"));
    if (canvas) {
        gtk_widget_grab_focus(canvas);
    }
}

// Navigation helpers

// Read walk parameters from the GUI entries.
// Ignores out of range values so a typo doesn't break pathfinding. 
// The previous valid value is kept instead. Walk speed range: (0.1, 20) km/h;
// Walk time limit range: (0, 7200) seconds (2 hours max).
static void readWalkParams() {
    if (appState.walkSpeedEntry) {
        const char* txt = gtk_entry_get_text(GTK_ENTRY(appState.walkSpeedEntry));
        double val = std::atof(txt);
        if (val > 0.1 && val < 20.0) {
            appState.walkSpeed = val;
        }
    }
    if (appState.walkLimitEntry) {
        const char* txt = gtk_entry_get_text(GTK_ENTRY(appState.walkLimitEntry));
        double val = std::atof(txt);
        if (val > 0.0 && val < 7200.0) {
            appState.walkTimeLimit = val;
        }
    }
}

// GTK "activate" callback for walk param entries -> reads values and unfocuses
static void onWalkParamEntryActivate(GtkWidget*, gpointer data) {
    readWalkParams();
    ezgl::application* app = static_cast<ezgl::application*>(data);
    GtkWidget* canvas = GTK_WIDGET(app->get_object("MainCanvas"));
    if (canvas) {
        gtk_widget_grab_focus(canvas);
    }
}

// Resolve directions panel widgets (called lazily on first use)
static void resolveDirectionsWidgets(ezgl::application* app) {
    if (appState.directionsPanel) {
        return;  // already resolved
    }
    appState.directionsPanel = GTK_WIDGET(app->get_object("DirectionsPanel"));
    appState.directionsTv    = GTK_WIDGET(app->get_object("DirectionsText"));
    appState.walkSpeedEntry  = GTK_WIDGET(app->get_object("WalkSpeedEntry"));
    appState.walkLimitEntry  = GTK_WIDGET(app->get_object("WalkLimitEntry"));
    appState.walkParamsBox   = GTK_WIDGET(app->get_object("WalkParamsBox"));

    // Pressing Enter in walk param entries applies the value and unfocuses
    if (appState.walkSpeedEntry) {
        g_signal_connect(appState.walkSpeedEntry, "activate",
            G_CALLBACK(onWalkParamEntryActivate), app);
    }
    if (appState.walkLimitEntry) {
        g_signal_connect(appState.walkLimitEntry, "activate",
            G_CALLBACK(onWalkParamEntryActivate), app);
    }
}

// Ensure styled text tags exist in a directions text buffer.
// Tags are shared across the buffer's lifetime via its GtkTextTagTable,
// so we only create them once. The lookup guards against double-creation
// which would print GTK warnings.
static void ensureDirectionsTags(GtkTextBuffer* buf) {
    GtkTextTagTable* table = gtk_text_buffer_get_tag_table(buf);
    if (gtk_text_tag_table_lookup(table, "step-num")) {
        return;  // already created
    }

    gtk_text_buffer_create_tag(buf, "step-num",
        "weight", PANGO_WEIGHT_BOLD,
        "foreground", "#5ba3d9", nullptr);
    gtk_text_buffer_create_tag(buf, "distance",
        "foreground", "#52c880",
        "weight", PANGO_WEIGHT_BOLD, nullptr);
    gtk_text_buffer_create_tag(buf, "time",
        "foreground", "#e0a0ff",
        "style", PANGO_STYLE_ITALIC, nullptr);
    gtk_text_buffer_create_tag(buf, "section-header",
        "weight", PANGO_WEIGHT_BOLD,
        "foreground", "#ff9c40",
        "scale", 1.1,
        "pixels-above-lines", 6,
        "pixels-below-lines", 4, nullptr);
    gtk_text_buffer_create_tag(buf, "driving-header",
        "weight", PANGO_WEIGHT_BOLD,
        "foreground", "#e63946",
        "scale", 1.1,
        "pixels-above-lines", 6,
        "pixels-below-lines", 4, nullptr);
    gtk_text_buffer_create_tag(buf, "normal",
        "foreground", "#cce0f4", nullptr);
}

// Show directions text in the GUI panel with rich formatting
static void showDirectionsInPanel(const std::string& dirs) {
    if (!appState.directionsPanel || !appState.directionsTv) {
        return;
    }

    if (dirs.empty()) {
        gtk_widget_set_visible(appState.directionsPanel, false);
        return;
    }

    gtk_widget_set_visible(appState.directionsPanel, true);
    GtkTextBuffer* buf = gtk_text_view_get_buffer(GTK_TEXT_VIEW(appState.directionsTv));
    ensureDirectionsTags(buf);

    gtk_text_buffer_set_text(buf, "", -1);
    GtkTextIter iter;
    gtk_text_buffer_get_end_iter(buf, &iter);

    // Parse directions line by line and apply styled tags
    std::istringstream stream(dirs);
    std::string line;
    while (std::getline(stream, line)) {
        if (line.empty()) {
            gtk_text_buffer_insert(buf, &iter, "\n", -1);
            continue;
        }

        // Section headers start with "---" (e.g. "--- Driving (Est. 4m) ---")
        if (line.size() >= 3 && line.substr(0, 3) == "---") {
            bool isDriving = line.find("Driving") != std::string::npos;
            gtk_text_buffer_insert_with_tags_by_name(buf, &iter,
                (line + "\n").c_str(), -1,
                isDriving ? "driving-header" : "section-header", nullptr);
            continue;
        }

        // Step lines like "1. Walk along Yonge Street for 200 m (~25s)"
        if (!line.empty() && std::isdigit(static_cast<unsigned char>(line[0]))) {
            size_t dotPos = line.find(". ");
            if (dotPos != std::string::npos) {
                // Step number in blue bold
                std::string stepNum = line.substr(0, dotPos + 2);
                gtk_text_buffer_insert_with_tags_by_name(buf, &iter,
                    stepNum.c_str(), -1, "step-num", nullptr);

                std::string rest = line.substr(dotPos + 2);
                // Split off " for 200 m (~25s)" into action, distance, time
                size_t forPos = rest.rfind(" for ");
                if (forPos != std::string::npos) {
                    std::string action = rest.substr(0, forPos);
                    std::string distAndTime = rest.substr(forPos);

                    // Separate time estimate "(~...)" from the distance
                    size_t tildePos = distAndTime.rfind("(~");
                    if (tildePos != std::string::npos) {
                        std::string distPart = distAndTime.substr(0, tildePos);
                        std::string timePart = distAndTime.substr(tildePos);
                        gtk_text_buffer_insert_with_tags_by_name(buf, &iter,
                            action.c_str(), -1, "normal", nullptr);
                        gtk_text_buffer_insert_with_tags_by_name(buf, &iter,
                            distPart.c_str(), -1, "distance", nullptr);
                        gtk_text_buffer_insert_with_tags_by_name(buf, &iter,
                            (timePart + "\n").c_str(), -1, "time", nullptr);
                    } else {
                        gtk_text_buffer_insert_with_tags_by_name(buf, &iter,
                            action.c_str(), -1, "normal", nullptr);
                        gtk_text_buffer_insert_with_tags_by_name(buf, &iter,
                            (distAndTime + "\n").c_str(), -1, "distance", nullptr);
                    }
                } else {
                    gtk_text_buffer_insert_with_tags_by_name(buf, &iter,
                        (rest + "\n").c_str(), -1, "normal", nullptr);
                }
                continue;
            }
        }

        // Fallback to plain text
        gtk_text_buffer_insert_with_tags_by_name(buf, &iter,
            (line + "\n").c_str(), -1, "normal", nullptr);
    }
}

// Left click -> find the closest intersection and highlight it.
// In navigation mode, first click = start, second click = destination + pathfind.
void actOnMousePress(ezgl::application* app, GdkEventButton* event,
                     double x, double y) {
    if (event->button != 1) {
        return;  // Only care about left clicks
    }

    // If the user just picked an item from the autocomplete dropdown,
    // GTK sends a click-through to the canvas
    if (appState.autocompleteJustSelected) {
        appState.autocompleteJustSelected = false;
        return;
    }

    // Ignore clicks when the user is interacting with search entries or other
    // input widgets. When an autocomplete dropdown is open, GTK passes the click
    // through to the canvas underneath, causing unwanted map interactions.
    GtkWidget* focusWidget = gtk_window_get_focus(
        GTK_WINDOW(app->get_object("MainWindow")));
    if (focusWidget && GTK_IS_ENTRY(focusWidget)) {
        GtkWidget* canvas = GTK_WIDGET(app->get_object("MainCanvas"));
        if (canvas) {
            gtk_widget_grab_focus(canvas);
        }
    }

    // Convert screen coords back to geographic
    LatLon clickPos = LatLon(static_cast<float>(yToLat(y)),
                             static_cast<float>(xToLon(x)));

    IntersectionIdx closest = findClosestIntersection(clickPos);
    if (closest < 0) {
        return;
    }

    LatLon intPos = intersectionPositions[closest];
    double dist   = findDistanceBetweenTwoPoints({clickPos, intPos});

    // Make the click threshold scale with zoom
    ezgl::rectangle visible      = app->get_renderer()->get_visible_world();
    double dynamicThreshold      = visible.width() / 50.0;
    dynamicThreshold             = std::max(dynamicThreshold, CLICK_THRESHOLD_METERS);
    if (dist > dynamicThreshold) {
        return;
    }

    appState.lastClickedIntersection = closest;

    // Navigation mode: click the map to set start/end intersections.
    // Works alongside text-input mode — clicking fills in the corresponding phase.
    if (appState.navigationMode) {
        resolveDirectionsWidgets(app);
        std::string intName = getIntersectionName(closest);

        // Determine which phase we're in. If text-input is active, use its phase;
        // otherwise fall back to the original navStart-based two-click logic.
        bool needsStart = false;
        bool needsEnd   = false;

        if (appState.navInputPhase == AppState::NAV_INPUT_START) {
            needsStart = true;
        } else if (appState.navInputPhase == AppState::NAV_INPUT_END) {
            needsEnd = true;
        } else if (appState.navStart < 0) {
            needsStart = true;
        } else {
            needsEnd = true;
        }

        if (needsStart) {
            // Set start intersection
            appState.navStart = closest;
            appState.textNavStart = closest;
            appState.navEnd = -1;
            appState.currentDrivePath.clear();
            appState.currentWalkPath.clear();
            appState.directionsText.clear();
            appState.highlightedIntersections.clear();
            appState.highlightedIntersections.insert(closest);

            // Advance to end-entry phase
            GtkWidget* entry1  = app->find_widget("SearchEntry1");
            GtkWidget* entry2  = app->find_widget("SearchEntry2");
            GtkWidget* label   = app->find_widget("SearchLabel");
            GtkWidget* findBtn = app->find_widget("FindButton");

            appState.navInputPhase = AppState::NAV_INPUT_END;
            if (label) {
                gtk_label_set_text(GTK_LABEL(label), "End Location:");
            }
            if (entry1) {
                gtk_entry_set_placeholder_text(GTK_ENTRY(entry1), "End Street 1");
                gtk_entry_set_text(GTK_ENTRY(entry1), "");
            }
            if (entry2) {
                gtk_entry_set_placeholder_text(GTK_ENTRY(entry2), "End Street 2");
                gtk_entry_set_text(GTK_ENTRY(entry2), "");
            }
            if (findBtn) {
                gtk_button_set_label(GTK_BUTTON(findBtn), "Find Route");
            }

            app->update_message("Start: " + intName +
                " | Enter destination streets or click the map.");
            app->refresh_drawing();
            return;
        }

        if (needsEnd) {
            // Set destination and run pathfinding
            appState.navEnd = closest;
            appState.textNavEnd = closest;
            appState.highlightedIntersections.insert(closest);

            IntersectionIdx startIdx = (appState.textNavStart >= 0)
                ? appState.textNavStart : appState.navStart;
            executePathfinding(app, startIdx, closest);

            // Reset for next query, stay in navigation mode
            appState.navStart = -1;
            appState.navEnd = -1;
            appState.navInputPhase = AppState::NAV_INPUT_START;

            GtkWidget* label   = app->find_widget("SearchLabel");
            GtkWidget* entry1  = app->find_widget("SearchEntry1");
            GtkWidget* entry2  = app->find_widget("SearchEntry2");
            GtkWidget* findBtn = app->find_widget("FindButton");
            if (label) {
                gtk_label_set_text(GTK_LABEL(label), "Start Location:");
            }
            if (entry1) {
                gtk_entry_set_placeholder_text(GTK_ENTRY(entry1), "Start Street 1");
                gtk_entry_set_text(GTK_ENTRY(entry1), "");
            }
            if (entry2) {
                gtk_entry_set_placeholder_text(GTK_ENTRY(entry2), "Start Street 2");
                gtk_entry_set_text(GTK_ENTRY(entry2), "");
            }
            if (findBtn) {
                gtk_button_set_label(GTK_BUTTON(findBtn), "Set Start");
            }
            return;
        }
    }

    // Normal mode (not navigating). Replace any previous highlight with this one.
    appState.highlightedIntersections.clear();
    appState.highlightedIntersections.insert(closest);

    // Auto-zoom when the view is very wide (> 2 km). Without this the
    // highlighted dot would be too tiny to see after clicking far out.
    ezgl::rectangle currentView = app->get_renderer()->get_visible_world();
    if (currentView.width() > 2000.0) {
        ezgl::point2d center = latLonToPoint(intPos);
        constexpr double halfSize = 300.0;
        app->change_canvas_world_coordinates("MainCanvas",
            ezgl::rectangle({center.x - halfSize, center.y - halfSize},
                            {center.x + halfSize, center.y + halfSize}));
    }

    // Build description string
    std::string intName = getIntersectionName(closest);
    std::string info    = "Intersection: " + intName;

    std::vector<StreetSegmentIdx> segments =
        findStreetSegmentsOfIntersection(closest);
    std::set<std::string> connectedStreets;
    for (auto segIdx : segments) {
        StreetSegmentInfo segInfo = getStreetSegmentInfo(segIdx);
        std::string name = getStreetName(segInfo.streetID);
        if (name != "<unknown>") {
            connectedStreets.insert(name);
        }
    }

    info += "\nConnected streets:";
    for (const auto& street : connectedStreets) {
        info += " " + street + ",";
    }
    if (!connectedStreets.empty()) {
        info.pop_back();
    }

    // Collect unique speed limits from all segments at this intersection.
    // Using a std::set deduplicates them (e.g. if two segments share the same limit).
    std::set<int> speeds;
    for (auto segIdx : segments) {
        auto it = appState.speedLimitKmh.find(segIdx);
        if (it != appState.speedLimitKmh.end()) {
            speeds.insert(it->second);
        }
    }
    if (!speeds.empty()) {
        info += "\nSpeed limits (km/h):";
        for (int s : speeds) {
            info += " " + std::to_string(s) + ",";
        }
        info.pop_back();  // remove trailing comma
    }

    app->create_popup_message("Intersection Info", info.c_str());
    app->refresh_drawing();
}

// Just shows coordinates in the status bar as the mouse moves around.
// Nice for debugging and seeing where you are on the map.
void actOnMouseMove(ezgl::application* app, GdkEventButton*, double x, double y) {
    if (!appState.highlightedIntersections.empty()) {
        return;  // keep the intersection info visible in the status bar
    }
    LatLon pos = LatLon(static_cast<float>(yToLat(y)),
                        static_cast<float>(xToLon(x)));
    std::ostringstream ss;
    // 5 decimal places gives ~1 m precision for latitude/longitude,
    // which is precise enough for a map UI without being visually noisy.
    ss << std::fixed << std::setprecision(5)
       << "Lat: " << pos.latitude() << "   Lon: " << pos.longitude();
    app->update_message(ss.str());
}

void actOnKeyPress(ezgl::application*, GdkEventKey*, char*) {
    // future use
}

// Tears down the current map and loads a brand new one from disk.
// Has to clear basically everything and redo all precomputation.
void reloadMap(const std::string& newPath, ezgl::application* app) {
    if (newPath.empty()) {
        return;
    }
    appState.mapLoading = true;
    app->update_message("Loading map: " + newPath + " ...");
    app->refresh_drawing();  // show "Loading..." before the heavy work starts

    // Wipe everything from the old map
    closeMap();
    appState.highlightedIntersections.clear();
    appState.lastClickedIntersection = -1;
    appState.navStart = -1;
    appState.navEnd = -1;
    appState.textNavStart = -1;
    appState.textNavEnd = -1;
    appState.navInputPhase = AppState::NAV_INPUT_NONE;
    appState.navigationMode = false;
    appState.currentDrivePath.clear();
    appState.currentWalkPath.clear();
    appState.directionsText.clear();
    if (appState.directionsPanel) {
        gtk_widget_set_visible(appState.directionsPanel, false);
    }
    // Restore search bar to normal mode
    exitNavInputMode(app);
    updateNavButtonLabels();
    appState.subwayLines.clear();
    appState.featureDrawOrder.clear();
    appState.segmentTier.clear();
    appState.speedLimitKmh.clear();
    // Clear weather and chat state under their respective mutexes.
    // Each piece of shared data has its own mutex to minimise lock contention
    // with the background fetch threads that write to these fields.
    {
        std::lock_guard<std::mutex> lock(appState.weatherMutex);
        appState.weatherInfo.clear();
    }
    {
        std::lock_guard<std::mutex> lock(appState.chatMutex);
        appState.chatHistory.clear();
        appState.chatResponse.clear();
        appState.chatWaiting = false;  // unblock the UI if a request was in flight
    }
    if (appState.chatHistoryTv) {
        GtkTextBuffer* buf = gtk_text_view_get_buffer(GTK_TEXT_VIEW(appState.chatHistoryTv));
        gtk_text_buffer_set_text(buf, "", -1);
    }
    appState.currentMapPath = newPath;

    bool ok = loadMap(newPath);
    if (!ok) {
        app->update_message("ERROR: Could not load map '" + newPath + "'.");
        app->create_popup_message("Load Error",
            ("Failed to load:\n" + newPath +
             "\n\nCheck the file path and try again.").c_str());
        appState.mapLoading = false;
        return;
    }

    // Redo all precomputation for the new map data
    precomputeSegmentTiers();
    precomputeFeatureDrawOrder();
    precomputeSubwayData();
    precomputeSpeedLimits();
    setupAutocomplete(app);

    // Fit the canvas to the new map's bounding box by converting the four
    // geographic extremes to world (Mercator) coordinates. The bottom left
    // corner uses (minLon, minLat) and the top right uses (maxLon, maxLat).
    ezgl::rectangle newWorld(
        {lonToX(geoMinLon), latToY(geoMinLat)},
        {lonToX(geoMaxLon), latToY(geoMaxLat)}
    );
    app->change_canvas_world_coordinates("MainCanvas", newWorld);

    appState.mapLoading = false;
    app->update_message("Loaded: " + newPath);

    // Fetch weather for new map centre
    double lat = (geoMinLat + geoMaxLat) / 2.0;
    double lon = (geoMinLon + geoMaxLon) / 2.0;
    fetchWeatherAsync(lat, lon);

    app->refresh_drawing();
}

// Opens a GTK file chooser dialog filtered to .streets.bin files.
// Defaults to the ECE297 maps directory on the teaching server.
void onLoadMapClicked(GtkWidget*, ezgl::application* app) {
    GtkWidget* mainWin = GTK_WIDGET(app->get_object("MainWindow"));

    GtkWidget* dialog = gtk_file_chooser_dialog_new(
        "Open Map File", GTK_WINDOW(mainWin),
        GTK_FILE_CHOOSER_ACTION_OPEN,
        "_Cancel", GTK_RESPONSE_CANCEL,
        "_Open",   GTK_RESPONSE_ACCEPT,
        nullptr);

    GtkFileFilter* filter = gtk_file_filter_new();
    gtk_file_filter_set_name(filter, "Map files (*.streets.bin)");
    gtk_file_filter_add_pattern(filter, "*.streets.bin");
    gtk_file_chooser_add_filter(GTK_FILE_CHOOSER(dialog), filter);
    gtk_file_chooser_set_current_folder(GTK_FILE_CHOOSER(dialog),
        "/cad2/ece297s/public/maps");

    if (gtk_dialog_run(GTK_DIALOG(dialog)) == GTK_RESPONSE_ACCEPT) {
        char* path = gtk_file_chooser_get_filename(GTK_FILE_CHOOSER(dialog));
        std::string mapPath(path);
        g_free(path);
        gtk_widget_destroy(dialog);
        reloadMap(mapPath, app);
    } else {
        gtk_widget_destroy(dialog);
    }
}


// Re fetch weather
void onRefreshWeather(GtkWidget*, ezgl::application* app) {
    {
        std::lock_guard<std::mutex> lock(appState.weatherMutex);
        appState.weatherInfo = "Fetching...";
    }
    app->refresh_drawing();
    fetchWeatherAsync((geoMinLat + geoMaxLat) / 2.0,
                      (geoMinLon + geoMaxLon) / 2.0);
    app->update_message("Refreshing weather data...");
}

// Reusable error popup when a street name can't be matched
void showStreetNotFoundError(ezgl::application* app, const std::string& streetName) {
    std::string msg = "No streets found matching \"" + streetName + "\".\n\n"
        "Tips:\n"
        "  - Check the spelling\n"
        "  - Try a shorter prefix (e.g. \"Yong\" for Yonge)\n"
        "  - Street names are case-insensitive";
    app->update_message("Street not found: \"" + streetName + "\"");
    app->create_popup_message("Street Not Found", msg.c_str());
}

// The main search feature —> user types two street names and we find
// where they intersect. Supports partial name matching.
// In navigation mode, this button handles text-based pathfinding input instead.
void onFindButtonClicked(GtkWidget*, ezgl::application* app) {
    // If in navigation text-input mode, delegate to the nav handler
    if (appState.navigationMode &&
        appState.navInputPhase != AppState::NAV_INPUT_NONE) {
        handleNavTextInput(app);
        return;
    }

    GtkWidget* entry1Widget = app->find_widget("SearchEntry1");
    GtkWidget* entry2Widget = app->find_widget("SearchEntry2");

    if (!entry1Widget || !entry2Widget) {
        app->update_message("Error: search entries not found.");
        return;
    }

    std::string street1 = gtk_entry_get_text(GTK_ENTRY(entry1Widget));
    std::string street2 = gtk_entry_get_text(GTK_ENTRY(entry2Widget));

    if (street1.empty() || street2.empty()) {
        app->update_message("Please enter two street names.");
        app->create_popup_message("Input Error",
            "Both street name fields must be filled in.\n"
            "Partial names are OK (e.g. \"Yong\" for Yonge St).");
        return;
    }

    std::vector<StreetIdx> ids1 = findStreetIdsFromPartialStreetName(street1);
    std::vector<StreetIdx> ids2 = findStreetIdsFromPartialStreetName(street2);

    if (ids1.empty()) {
        showStreetNotFoundError(app, street1);
        return;
    }
    if (ids2.empty()) {
        showStreetNotFoundError(app, street2);
        return;
    }

    // Try all combinations of partial matches until we find an intersection.
    // Usually the first match is the right one, but partial names might
    // match multiple streets (e.g. "King" -> King St, King Edward Ave, etc.)
    std::vector<IntersectionIdx> intersections;
    for (size_t i = 0; i < ids1.size() && intersections.empty(); ++i) {
        for (size_t j = 0; j < ids2.size() && intersections.empty(); ++j) {
            intersections = findIntersectionsOfTwoStreets(ids1[i], ids2[j]);
        }
    }

    if (intersections.empty()) {
        std::string name1 = getStreetName(ids1[0]);
        std::string name2 = getStreetName(ids2[0]);
        app->update_message("No intersections found between '" +
            name1 + "' and '" + name2 + "'.");
        std::string detail = "\"" + name1 + "\" and \"" + name2 +
            "\" do not intersect.";
        if (ids1.size() > 1 || ids2.size() > 1) {
            detail += "\n\nMultiple streets matched your input:";
            if (ids1.size() > 1) {
                detail += "\n  \"" + street1 + "\" matched " +
                    std::to_string(ids1.size()) + " streets (e.g. " +
                    getStreetName(ids1[0]) + ", " + getStreetName(ids1[1]) + ")";
            }
            if (ids2.size() > 1) {
                detail += "\n  \"" + street2 + "\" matched " +
                    std::to_string(ids2.size()) + " streets (e.g. " +
                    getStreetName(ids2[0]) + ", " + getStreetName(ids2[1]) + ")";
            }
            detail += "\n\nTry more specific names.";
        }
        app->create_popup_message("No Intersection", detail.c_str());
        return;
    }

    appState.highlightedIntersections.clear();
    for (auto id : intersections) {
        appState.highlightedIntersections.insert(id);
    }

    // Zoom to a 1000x1000 m box centred on the first found intersection
    LatLon pos = intersectionPositions[intersections[0]];
    ezgl::point2d center = latLonToPoint(pos);
    constexpr double halfSize = 500.0;
    app->change_canvas_world_coordinates("MainCanvas",
        ezgl::rectangle({center.x - halfSize, center.y - halfSize},
                        {center.x + halfSize, center.y + halfSize}));
    app->refresh_drawing();

    app->update_message("Found " + std::to_string(intersections.size()) +
        " intersection(s) of '" + getStreetName(ids1[0]) +
        "' & '" + getStreetName(ids2[0]) + "'.");
}

// Toggle callbacks
void onNightModeToggled(GtkWidget*, ezgl::application* app) {
    appState.nightMode = !appState.nightMode;
    app->refresh_drawing();
}

void onTogglePOIs(GtkWidget*, ezgl::application* app) {
    appState.showPOIs = !appState.showPOIs;
    app->refresh_drawing();
}

void onToggleSubway(GtkWidget*, ezgl::application* app) {
    appState.showSubway = !appState.showSubway;
    app->refresh_drawing();
}

// Navigation callbacks

// Switch the top search bar into navigation input mode
static void enterNavInputMode(ezgl::application* app) {
    GtkWidget* label = app->find_widget("SearchLabel");
    GtkWidget* entry1 = app->find_widget("SearchEntry1");
    GtkWidget* entry2 = app->find_widget("SearchEntry2");
    GtkWidget* findBtn = app->find_widget("FindButton");

    if (label) {
        gtk_label_set_text(GTK_LABEL(label), "Start Location:");
    }
    if (entry1) {
        gtk_entry_set_placeholder_text(GTK_ENTRY(entry1), "Start Street 1");
        gtk_entry_set_text(GTK_ENTRY(entry1), "");
    }
    if (entry2) {
        gtk_entry_set_placeholder_text(GTK_ENTRY(entry2), "Start Street 2");
        gtk_entry_set_text(GTK_ENTRY(entry2), "");
    }
    if (findBtn) {
        gtk_button_set_label(GTK_BUTTON(findBtn), "Set Start");
    }

    appState.navInputPhase = AppState::NAV_INPUT_START;
    appState.textNavStart = -1;
    appState.textNavEnd = -1;
}

// Restore the top search bar to normal "Find Intersection" mode
static void exitNavInputMode(ezgl::application* app) {
    GtkWidget* label = app->find_widget("SearchLabel");
    GtkWidget* entry1 = app->find_widget("SearchEntry1");
    GtkWidget* entry2 = app->find_widget("SearchEntry2");
    GtkWidget* findBtn = app->find_widget("FindButton");

    if (label) {
        gtk_label_set_text(GTK_LABEL(label), "Find Intersection:");
    }
    if (entry1) {
        gtk_entry_set_placeholder_text(GTK_ENTRY(entry1), "Street 1");
        gtk_entry_set_text(GTK_ENTRY(entry1), "");
    }
    if (entry2) {
        gtk_entry_set_placeholder_text(GTK_ENTRY(entry2), "Street 2");
        gtk_entry_set_text(GTK_ENTRY(entry2), "");
    }
    if (findBtn) {
        gtk_button_set_label(GTK_BUTTON(findBtn), "\u2315  Find");
    }

    appState.navInputPhase = AppState::NAV_INPUT_NONE;
    appState.textNavStart = -1;
    appState.textNavEnd = -1;
}

// Shared pathfinding logic used by both mouse-click and text input navigation
static void executePathfinding(ezgl::application* app,
                                IntersectionIdx start, IntersectionIdx end) {
    std::string startName = getIntersectionName(start);
    std::string endName   = getIntersectionName(end);

    resolveDirectionsWidgets(app);

    if (appState.walkDriveMode) {
        readWalkParams();
        auto [walkPath, drivePath] = findPathWithWalkToPickUp(
            start, end, appState.turnPenalty,
            appState.walkSpeed, appState.walkTimeLimit);

        appState.currentWalkPath  = walkPath;
        appState.currentDrivePath = drivePath;

        if (drivePath.empty() && walkPath.empty() && start != end) {
            app->update_message("No path found from " + startName + " to " + endName);
            showDirectionsInPanel("No path found between\n" + startName +
                "\nand\n" + endName + "\n\nTry different intersections.");
        } else {
            double driveTime = computePathTravelTime(appState.turnPenalty, drivePath);
            double walkTime  = computePathWalkingTime(walkPath, appState.walkSpeed,
                                                       appState.turnPenalty);
            std::string msg;
            if (!walkPath.empty())
                msg += "Walk: " + formatTime(walkTime) + " | ";
            msg += "Drive: " + formatTime(driveTime) + " | Total: "
                + formatTime(walkTime + driveTime);
            app->update_message(msg);

            std::string dirs;
            if (!walkPath.empty()) {
                dirs += "--- Walking (Est. " + formatTime(walkTime) + ") ---\n";
                dirs += generateDirections(walkPath, true,
                    appState.walkSpeed, appState.turnPenalty);
            }
            if (!drivePath.empty()) {
                dirs += "--- Driving (Est. " + formatTime(driveTime) + ") ---\n";
                dirs += generateDirections(drivePath, false,
                    appState.walkSpeed, appState.turnPenalty);
            }
            appState.directionsText = dirs;
            showDirectionsInPanel(dirs);
            std::cout << "\n" << dirs << std::endl;
        }
    } else {
        std::vector<StreetSegmentIdx> path = findPathBetweenIntersections(
            appState.turnPenalty, {start, end});

        appState.currentDrivePath = path;
        appState.currentWalkPath.clear();

        if (path.empty() && start != end) {
            app->update_message("No path found from " + startName + " to " + endName);
            showDirectionsInPanel("No path found between\n" + startName +
                "\nand\n" + endName + "\n\nTry different intersections.");
        } else {
            double travelTime = computePathTravelTime(appState.turnPenalty, path);
            std::ostringstream msg;
            msg << std::fixed << std::setprecision(1)
                << "Path found: " << formatTime(travelTime) << " ("
                << path.size() << " segments) from "
                << startName << " to " << endName;
            app->update_message(msg.str());

            std::string dirs = "--- Driving (Est. " + formatTime(travelTime) + ") ---\n";
            dirs += generateDirections(path, false,
                appState.walkSpeed, appState.turnPenalty);
            appState.directionsText = dirs;
            showDirectionsInPanel(dirs);
            std::cout << "\n" << dirs << std::endl;
        }
    }
    app->refresh_drawing();
}

// Resolve an intersection from two street names typed in the search bar.
// Returns the intersection index, or -1 on failure (with error popup shown).
static IntersectionIdx resolveIntersectionFromEntries(ezgl::application* app) {
    GtkWidget* entry1Widget = app->find_widget("SearchEntry1");
    GtkWidget* entry2Widget = app->find_widget("SearchEntry2");
    if (!entry1Widget || !entry2Widget) {
        return -1;
    }

    std::string street1 = gtk_entry_get_text(GTK_ENTRY(entry1Widget));
    std::string street2 = gtk_entry_get_text(GTK_ENTRY(entry2Widget));

    if (street1.empty() || street2.empty()) {
        app->update_message("Please enter two street names for the intersection.");
        app->create_popup_message("Input Error",
            "Both street name fields must be filled in.\n"
            "Partial names are OK (e.g. \"Yong\" for Yonge St).");
        return -1;
    }

    std::vector<StreetIdx> ids1 = findStreetIdsFromPartialStreetName(street1);
    std::vector<StreetIdx> ids2 = findStreetIdsFromPartialStreetName(street2);

    if (ids1.empty()) {
        showStreetNotFoundError(app, street1);
        return -1;
    }
    if (ids2.empty()) {
        showStreetNotFoundError(app, street2);
        return -1;
    }

    std::vector<IntersectionIdx> intersections;
    for (size_t i = 0; i < ids1.size() && intersections.empty(); ++i) {
        for (size_t j = 0; j < ids2.size() && intersections.empty(); ++j) {
            intersections = findIntersectionsOfTwoStreets(ids1[i], ids2[j]);
        }
    }

    if (intersections.empty()) {
        std::string name1 = getStreetName(ids1[0]);
        std::string name2 = getStreetName(ids2[0]);
        app->update_message("No intersection found between '" + name1 + "' and '" + name2 + "'.");
        app->create_popup_message("No Intersection",
            ("\"" + name1 + "\" and \"" + name2 + "\" do not intersect.\n"
             "Try different street names.").c_str());
        return -1;
    }
    return intersections[0];
}

// Handle the Find button press while in navigation text-input mode
static void handleNavTextInput(ezgl::application* app) {
    IntersectionIdx resolved = resolveIntersectionFromEntries(app);
    if (resolved < 0) {
        return;  // error already shown
    }

    GtkWidget* label   = app->find_widget("SearchLabel");
    GtkWidget* entry1  = app->find_widget("SearchEntry1");
    GtkWidget* entry2  = app->find_widget("SearchEntry2");
    GtkWidget* findBtn = app->find_widget("FindButton");

    if (appState.navInputPhase == AppState::NAV_INPUT_START) {
        // Save start intersection
        appState.textNavStart = resolved;
        appState.navStart = resolved;
        appState.highlightedIntersections.clear();
        appState.highlightedIntersections.insert(resolved);

        // Zoom to the start intersection so the user can see it
        LatLon startPos = intersectionPositions[resolved];
        ezgl::point2d center = latLonToPoint(startPos);
        constexpr double halfSize = 500.0;
        app->change_canvas_world_coordinates("MainCanvas",
            ezgl::rectangle({center.x - halfSize, center.y - halfSize},
                            {center.x + halfSize, center.y + halfSize}));

        // Switch UI to ask for end location
        appState.navInputPhase = AppState::NAV_INPUT_END;
        if (label) {
            gtk_label_set_text(GTK_LABEL(label), "End Location:");
        }
        if (entry1) {
            gtk_entry_set_placeholder_text(GTK_ENTRY(entry1), "End Street 1");
            gtk_entry_set_text(GTK_ENTRY(entry1), "");
        }
        if (entry2) {
            gtk_entry_set_placeholder_text(GTK_ENTRY(entry2), "End Street 2");
            gtk_entry_set_text(GTK_ENTRY(entry2), "");
        }
        if (findBtn) {
            gtk_button_set_label(GTK_BUTTON(findBtn), "Find Route");
        }

        app->update_message("Start set: " + getIntersectionName(resolved) +
            " | Now enter destination streets or click the map.");
        app->refresh_drawing();

    } else if (appState.navInputPhase == AppState::NAV_INPUT_END) {
        // Save end and run pathfinding
        appState.textNavEnd = resolved;
        appState.navEnd = resolved;
        appState.highlightedIntersections.insert(resolved);

        executePathfinding(app, appState.textNavStart, appState.textNavEnd);

        // Reset for next query but stay in navigation mode
        appState.navStart = -1;
        appState.navEnd = -1;
        appState.navInputPhase = AppState::NAV_INPUT_START;
        if (label) {
            gtk_label_set_text(GTK_LABEL(label), "Start Location:");
        }
        if (entry1) {
            gtk_entry_set_placeholder_text(GTK_ENTRY(entry1), "Start Street 1");
            gtk_entry_set_text(GTK_ENTRY(entry1), "");
        }
        if (entry2) {
            gtk_entry_set_placeholder_text(GTK_ENTRY(entry2), "Start Street 2");
            gtk_entry_set_text(GTK_ENTRY(entry2), "");
        }
        if (findBtn) {
            gtk_button_set_label(GTK_BUTTON(findBtn), "Set Start");
        }
    }
}

// Clears any active path and resets navigation state
void clearNavigation(ezgl::application* app) {
    appState.navStart = -1;
    appState.navEnd = -1;
    appState.textNavStart = -1;
    appState.textNavEnd = -1;
    appState.currentDrivePath.clear();
    appState.currentWalkPath.clear();
    appState.directionsText.clear();
    appState.highlightedIntersections.clear();
    if (appState.directionsPanel) {
        gtk_widget_set_visible(appState.directionsPanel, false);
    }
    app->refresh_drawing();
}

// Helper to update navigate button labels to reflect active state
static void updateNavButtonLabels() {
    if (appState.navigateBtn) {
        if (appState.navigationMode && !appState.walkDriveMode) {
            gtk_button_set_label(GTK_BUTTON(appState.navigateBtn),
                "\u279C  Navigate [ON]");
            gtk_style_context_add_class(
                gtk_widget_get_style_context(appState.navigateBtn), "nav-active");
        } else {
            gtk_button_set_label(GTK_BUTTON(appState.navigateBtn),
                "\u279C  Navigate");
            gtk_style_context_remove_class(
                gtk_widget_get_style_context(appState.navigateBtn), "nav-active");
        }
    }
    if (appState.walkDriveBtn) {
        if (appState.navigationMode && appState.walkDriveMode) {
            gtk_button_set_label(GTK_BUTTON(appState.walkDriveBtn),
                "\u26A1  Walk+Drive [ON]");
            gtk_style_context_add_class(
                gtk_widget_get_style_context(appState.walkDriveBtn), "nav-active");
        } else {
            gtk_button_set_label(GTK_BUTTON(appState.walkDriveBtn),
                "\u26A1  Walk+Drive");
            gtk_style_context_remove_class(
                gtk_widget_get_style_context(appState.walkDriveBtn), "nav-active");
        }
    }
}

// Toggle drive-only navigation mode.
// First click enables it and switches the search bar into nav-input mode.
// A second click on the same active mode turns navigation off entirely.
void onNavigateClicked(GtkWidget*, ezgl::application* app) {
    resolveDirectionsWidgets(app);
    if (appState.navigationMode && !appState.walkDriveMode) {
        appState.navigationMode = false;
        clearNavigation(app);
        exitNavInputMode(app);
        app->update_message("Navigation mode OFF.");
    } else {
        appState.navigationMode = true;
        appState.walkDriveMode = false;
        clearNavigation(app);
        enterNavInputMode(app);
        // Show directions panel with instructions
        if (appState.directionsPanel && appState.directionsTv) {
            GtkTextBuffer* buf = gtk_text_view_get_buffer(
                GTK_TEXT_VIEW(appState.directionsTv));
            gtk_text_buffer_set_text(buf,
                "Type two streets in the top bar and click Set Start,\n"
                "or click a start intersection on the map.\n"
                "Then enter/click a destination.", -1);
            gtk_widget_set_visible(appState.directionsPanel, true);
            if (appState.walkParamsBox) {
                gtk_widget_set_visible(appState.walkParamsBox, false);
            }
        }
        app->update_message("Navigate (Drive): Enter start streets in the top bar, or click the map.");
    }
    updateNavButtonLabels();
}

// Toggle walk+drive navigation mode.
// Walk+Drive finds a route where the user walks to a pickup point then drives.
// Toggling while already in Walk+Drive mode turns navigation off.
void onWalkDriveClicked(GtkWidget*, ezgl::application* app) {
    resolveDirectionsWidgets(app);
    readWalkParams();  // capture any edits the user made to speed/limit fields
    if (appState.navigationMode && appState.walkDriveMode) {
        appState.navigationMode = false;
        clearNavigation(app);
        exitNavInputMode(app);
        app->update_message("Walk+Drive mode OFF.");
    } else {
        appState.navigationMode = true;
        appState.walkDriveMode = true;
        clearNavigation(app);
        enterNavInputMode(app);
        // Show directions panel with walk params visible and instructions
        if (appState.directionsPanel && appState.directionsTv) {
            GtkTextBuffer* buf = gtk_text_view_get_buffer(
                GTK_TEXT_VIEW(appState.directionsTv));
            gtk_text_buffer_set_text(buf,
                "Set walking speed and time limit below,\n"
                "then enter start streets in the top bar\n"
                "or click a start intersection on the map.", -1);
            gtk_widget_set_visible(appState.directionsPanel, true);
            if (appState.walkParamsBox) {
                gtk_widget_set_visible(appState.walkParamsBox, true);
            }
        }
        app->update_message("Navigate (Walk+Drive): Set walk params, enter start streets or click the map.");
    }
    updateNavButtonLabels();
}

// Clear the current path from the map
void onClearPathClicked(GtkWidget*, ezgl::application* app) {
    appState.navigationMode = false;
    clearNavigation(app);
    exitNavInputMode(app);
    updateNavButtonLabels();
    app->update_message("Path cleared. Navigation mode OFF.");
}

// Show usage instructions in a richly formatted custom dialog
void onHelpClicked(GtkWidget*, ezgl::application* app) {
    GtkWidget* mainWin = GTK_WIDGET(app->get_object("MainWindow"));

    GtkWidget* dialog = gtk_dialog_new_with_buttons(
        "Spatia Help", GTK_WINDOW(mainWin),
        static_cast<GtkDialogFlags>(GTK_DIALOG_MODAL | GTK_DIALOG_DESTROY_WITH_PARENT),
        "Close", GTK_RESPONSE_ACCEPT,
        nullptr);
    gtk_widget_set_name(dialog, "HelpDialog");
    gtk_window_set_default_size(GTK_WINDOW(dialog), 500, 540);

    GtkWidget* content = gtk_dialog_get_content_area(GTK_DIALOG(dialog));

    GtkWidget* scrollWin = gtk_scrolled_window_new(nullptr, nullptr);
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scrollWin),
        GTK_POLICY_NEVER, GTK_POLICY_AUTOMATIC);

    GtkWidget* textView = gtk_text_view_new();
    gtk_text_view_set_editable(GTK_TEXT_VIEW(textView), FALSE);
    gtk_text_view_set_cursor_visible(GTK_TEXT_VIEW(textView), FALSE);
    gtk_text_view_set_wrap_mode(GTK_TEXT_VIEW(textView), GTK_WRAP_WORD);
    gtk_text_view_set_left_margin(GTK_TEXT_VIEW(textView), 18);
    gtk_text_view_set_right_margin(GTK_TEXT_VIEW(textView), 18);
    gtk_text_view_set_top_margin(GTK_TEXT_VIEW(textView), 14);
    gtk_text_view_set_bottom_margin(GTK_TEXT_VIEW(textView), 14);
    gtk_widget_set_name(textView, "HelpText");

    GtkTextBuffer* buf = gtk_text_view_get_buffer(GTK_TEXT_VIEW(textView));

    // Create styled tags for rich formatting
    gtk_text_buffer_create_tag(buf, "title",
        "weight", PANGO_WEIGHT_BOLD, "scale", 1.6,
        "foreground", "#5ba3d9",
        "pixels-below-lines", 10, nullptr);
    gtk_text_buffer_create_tag(buf, "section",
        "weight", PANGO_WEIGHT_BOLD, "scale", 1.15,
        "foreground", "#52c880",
        "pixels-above-lines", 14, "pixels-below-lines", 4, nullptr);
    gtk_text_buffer_create_tag(buf, "body",
        "foreground", "#cce0f4",
        "pixels-below-lines", 2, nullptr);
    gtk_text_buffer_create_tag(buf, "bullet",
        "foreground", "#8ab0ce",
        "left-margin", 28,
        "pixels-below-lines", 3, nullptr);
    gtk_text_buffer_create_tag(buf, "tip",
        "foreground", "#e8c840",
        "style", PANGO_STYLE_ITALIC,
        "left-margin", 28,
        "pixels-below-lines", 3, nullptr);

    GtkTextIter iter;
    gtk_text_buffer_get_start_iter(buf, &iter);

    // Helper lambda for inserting tagged text
    auto insert = [&](const char* text, const char* tag) {
        gtk_text_buffer_insert_with_tags_by_name(buf, &iter, text, -1, tag, nullptr);
    };

    insert("Spatia Map Explorer\n", "title");

    insert("\nSEARCH\n", "section");
    insert("Enter two street names in the top bar and click Find\n"
           "to locate their intersection on the map.\n", "bullet");
    insert("Partial names are supported (e.g. \"Yong\" for Yonge).\n", "bullet");
    insert("You can also type intersections directly for pathfinding\n"
           "when Navigate or Walk+Drive mode is active.\n", "bullet");

    insert("\nNAVIGATE (Drive Only)\n", "section");
    insert("1.  Click the Navigate button in the sidebar\n", "bullet");
    insert("2.  Type start streets in the top bar and click Set Start,\n"
           "    or click a start intersection on the map\n", "bullet");
    insert("3.  Type end streets and click Find Route,\n"
           "    or click a destination intersection on the map\n", "bullet");
    insert("The shortest driving route appears in red.\n", "bullet");

    insert("\nNAVIGATE (Walk + Drive)\n", "section");
    insert("1.  Click the Walk+Drive button\n", "bullet");
    insert("2.  Set walking speed and time limit in the\n"
           "    directions panel (top-right corner)\n", "bullet");
    insert("3.  Select start and destination (type or click)\n", "bullet");
    insert("Walking path shown as dashed orange,\n"
           "driving path shown as solid red.\n", "bullet");

    insert("\nOTHER FEATURES\n", "section");
    insert("Clear Path   -  Remove the current route\n", "bullet");
    insert("Night Mode   -  Toggle dark / light theme\n", "bullet");
    insert("Toggle POIs  -  Show / hide points of interest\n", "bullet");
    insert("Chat         -  Ask the AI about the map\n", "bullet");
    insert("Load Map     -  Open a different city map\n", "bullet");

    insert("\nTIPS\n", "section");
    insert("Directions appear in the top-right panel.\n", "tip");
    insert("Green marker = start,  Red marker = end.\n", "tip");
    insert("Scroll to zoom, drag to pan the map.\n", "tip");

    gtk_container_add(GTK_CONTAINER(scrollWin), textView);
    gtk_box_pack_start(GTK_BOX(content), scrollWin, TRUE, TRUE, 0);

    g_signal_connect_swapped(dialog, "response",
        G_CALLBACK(gtk_widget_destroy), dialog);

    gtk_widget_show_all(dialog);
}

// libcurl callback —> just appends received data to our response string
static size_t chatCurlWriteCallback(char* ptr, size_t size, size_t nmemb,
                                     std::string* data) {
    data->append(ptr, size * nmemb);
    return size * nmemb;
}

// Look up the chat GTK widgets once and cache them. We do this lazily
// because the widgets might not exist if main.ui hasn't been updated.
static void resolveChatWidgets(ezgl::application* app) {
    if (appState.chatWidgetsResolved) {
        return;
    }
    appState.chatWidgetsResolved = true;

    appState.chatPanel     = GTK_WIDGET(app->get_object("ChatPanel"));
    appState.chatSendBtn   = GTK_WIDGET(app->get_object("ChatSendButton"));
    appState.chatEntryW    = GTK_WIDGET(app->get_object("ChatEntry"));
    appState.chatHistoryTv = GTK_WIDGET(app->get_object("ChatHistory"));

    appState.chatAvailable = (appState.chatPanel != nullptr);

    // Wire up signals once
    if (appState.chatSendBtn) {
        g_signal_connect(appState.chatSendBtn, "clicked", G_CALLBACK(onChatSend), app);
    }
    if (appState.chatEntryW) {
        g_signal_connect(appState.chatEntryW, "activate", G_CALLBACK(onChatEntryActivate), app);
    }
}

// Toggle the AI chat panel. Lazily resolves widgets on first call.
// If ChatPanel doesn't exist in main.ui, shows an error and quits.
// When opening, grabs keyboard focus so the user can type immediately.
void onToggleChat(GtkWidget*, ezgl::application* app) {
    resolveChatWidgets(app);

    if (!appState.chatAvailable) {
        app->update_message("Chat panel not found — rebuild with updated main.ui");
        return;
    }

    appState.showChat = !appState.showChat;
    gtk_widget_set_visible(appState.chatPanel, appState.showChat);

    if (appState.showChat && appState.chatEntryW) {
        gtk_widget_grab_focus(appState.chatEntryW);
    }
}

// Builds the system prompt + context we send to Gemini along with the user's question.
// Includes map metadata, nearby POIs, weather, etc. so the AI can give useful answers.
std::string buildChatContext(const std::string& userMessage) {
    std::ostringstream ctx;

    ctx << "You are Spatia AI, a map assistant. IMPORTANT RULES:\n"
        << "1. ALWAYS list specific names from the nearby POI data below. Never say "
        << "'there are multiple' or 'many' — list them by name.\n"
        << "2. Include the POI type and distance when listing places.\n"
        << "3. If the user asks about a category (e.g. restaurants), filter the POI "
        << "list and show only matching entries with their names and distances.\n"
        << "4. Keep answers under 150 words but always include specific names.\n\n";

    if (!appState.currentMapPath.empty()) {
        ctx << "Loaded map: " << appState.currentMapPath << "\n";
    }

    ctx << "Map bounds: Lat [" << std::fixed << std::setprecision(2)
        << geoMinLat << ", " << geoMaxLat
        << "], Lon [" << geoMinLon << ", " << geoMaxLon << "]\n";
    ctx << "Total intersections: " << getNumIntersections()
        << ", streets: " << getNumStreets()
        << ", POIs: " << getNumPointsOfInterest() << "\n";

    {
        std::lock_guard<std::mutex> lock(appState.weatherMutex);
        if (!appState.weatherInfo.empty()) {
            ctx << "Current weather: " << appState.weatherInfo << "\n";
        }
    }

    // Try to figure out what location the user is asking about by
    // looking for street names in their message
    IntersectionIdx relevantIntersection = -1;

    // Split the message into words, strip punctuation
    std::vector<std::string> words;
    {
        std::istringstream iss(userMessage);
        std::string w;
        while (iss >> w) {
            // Strip common punctuation
            while (!w.empty() && (w.back() == '?' || w.back() == ',' ||
                   w.back() == '.' || w.back() == '!')) {
                w.pop_back();
            }
            if (!w.empty()) {
                words.push_back(w);
            }
        }
    }

    // Try single words and multi word combos (up to 3 words) to match
    // street names.
    std::vector<StreetIdx> foundStreets;
    std::set<StreetIdx> foundSet;
    for (size_t i = 0; i < words.size() && foundStreets.size() < 2; ++i) {
        for (int len = 1; len <= 3 && i + len <= words.size(); ++len) {
            std::string candidate;
            for (int k = 0; k < len; ++k) {
                if (k > 0) {
                    candidate += " ";
                }
                candidate += words[i + k];
            }
            if (candidate.length() < 3) {
                continue;  // skip short stopwords like "at", "on", "&"
            }
            std::vector<StreetIdx> matches = findStreetIdsFromPartialStreetName(candidate);
            // 1-20 results: specific enough to be a real street name
            if (matches.size() >= 1 && matches.size() <= 20) {
                for (auto sid : matches) {
                    if (foundSet.find(sid) == foundSet.end()) {
                        foundStreets.push_back(sid);
                        foundSet.insert(sid);
                        if (foundStreets.size() >= 2) {
                            break;
                        }
                    }
                }
                if (foundStreets.size() >= 2) {
                    break;
                }
                i += len - 1;  // advance past the multi word token we just consumed
                break;
            }
        }
    }

    // If we found two street names, look up where they cross
    if (foundStreets.size() >= 2) {
        std::vector<IntersectionIdx> isects =
            findIntersectionsOfTwoStreets(foundStreets[0], foundStreets[1]);
        if (!isects.empty()) {
            relevantIntersection = isects[0];
            ctx << "\nQueried intersection: " << getIntersectionName(relevantIntersection)
                << " (Lat " << getIntersectionPosition(relevantIntersection).latitude()
                << ", Lon " << getIntersectionPosition(relevantIntersection).longitude() << ")\n";
        }
    }

    // If no streets found in the message, use the last clicked intersection
    if (relevantIntersection < 0 && appState.lastClickedIntersection >= 0) {
        relevantIntersection = appState.lastClickedIntersection;
    }

    // Dump everything we know about this location into the context
    if (relevantIntersection >= 0) {
        IntersectionIdx idx = relevantIntersection;
        ctx << "Selected intersection: " << getIntersectionName(idx) << "\n";

        LatLon refPos = getIntersectionPosition(idx);
        ctx << "Position: Lat " << refPos.latitude()
            << ", Lon " << refPos.longitude() << "\n";

        // Streets at this intersection
        std::vector<StreetSegmentIdx> segs = findStreetSegmentsOfIntersection(idx);
        std::set<std::string> streets;
        for (auto s : segs) {
            std::string name = getStreetName(getStreetSegmentInfo(s).streetID);
            if (name != "<unknown>") {
                streets.insert(name);
            }
        }
        if (!streets.empty()) {
            std::string streetList;
            for (const auto& st : streets) {
                if (!streetList.empty()) {
                    streetList += ", ";
                }
                streetList += st;
            }
            ctx << "Connected streets: " << streetList << "\n";
        }

        // Scan all POIs and include those within 500 m of the reference
        // intersection. 500 m is roughly a 5 minute walking radius and
        // gives the AI enough context without overloading the prompt.
        // We cap at 30 entries to keep the prompt size manageable.
        int numPOIs = getNumPointsOfInterest();
        ctx << "\nNearby points of interest:\n";
        int poiCount = 0;
        for (int poi = 0; poi < numPOIs && poiCount < 30; ++poi) {
            LatLon poiPos = getPOIPosition(poi);
            double dist = findDistanceBetweenTwoPoints({refPos, poiPos});
            if (dist < 500.0) {
                ctx << "- " << getPOIName(poi) << " (" << getPOIType(poi)
                    << ", " << (int)dist << "m away)\n";
                ++poiCount;
            }
        }
        if (poiCount == 0) {
            ctx << "(none within 500m)\n";
        }
    }

    return ctx.str();
}

// JSON string extraction —> we don't want to retrieve in a whole JSON
// library just for this. Finds the next quoted string after startPos and
// handles the basic escape sequences (\", \\, \n, \t).
static std::string extractJsonString(const std::string& json, size_t startPos) {
    size_t pos = json.find('"', startPos);
    if (pos == std::string::npos) {
        return "";
    }
    pos++;

    std::string result;
    while (pos < json.length()) {
        char c = json[pos];
        if (c == '\\' && pos + 1 < json.length()) {
            char next = json[pos + 1];
            if (next == '"') {
                result += '"';
                pos += 2;
                continue;
            } else if (next == 'n') {
                result += '\n';
                pos += 2;
                continue;
            } else if (next == 't') {
                result += '\t';
                pos += 2;
                continue;
            } else if (next == '\\') {
                result += '\\';
                pos += 2;
                continue;
            } else {
                result += next;
                pos += 2;
                continue;
            }
        }
        if (c == '"') {
            break;
        }
        result += c;
        pos++;
    }
    return result;
}

// Pull the actual response text out of Gemini's JSON response.
// The structure is {"candidates":[{"content":{"parts":[{"text":"..."}]}}]}
std::string extractGeminiText(const std::string& json) {
    size_t textPos = json.find("\"text\"");
    if (textPos != std::string::npos) {
        std::string result = extractJsonString(json, textPos + 6);
        if (!result.empty()) {
            return result;
        }
    }

    // No "text" field — might be an error response from the API
    size_t msgPos = json.find("\"message\"");
    if (msgPos != std::string::npos) {
        std::string errMsg = extractJsonString(json, msgPos + 9);
        if (!errMsg.empty()) {
            return "API Error: " + errMsg;
        }
    }

    // Last resort: show the first 200 characters of the raw body for debugging
    return "Error: could not parse response. Raw: " + json.substr(0, 200);
}

// Fires off the Gemini API request in a background thread so the UI doesn't freeze.
// Same pattern as fetchWeatherAsync — detached thread + g_idle_add for GTK updates.
void sendChatMessageAsync(const std::string& userMessage, ezgl::application* /*app*/) {
    const char* apiKeyEnv = std::getenv("GEMINI_API_KEY");
    if (!apiKeyEnv || std::string(apiKeyEnv).empty()) {
        {
            std::lock_guard<std::mutex> lock(appState.chatMutex);
            appState.chatResponse = "Error: GEMINI_API_KEY environment variable not set.";
            appState.chatWaiting = false;
        }
        // g_idle_add() schedules the UI update on the GTK main loop even though
        // we're still on the main thread here. This keeps the error path
        // identical to the threaded success path so both update the chat
        // history widget through the same code. G_SOURCE_REMOVE ensures
        // the callback occurs only once.
        g_idle_add([](gpointer) -> gboolean {
            if (!appState.chatHistoryTv) {
                return G_SOURCE_REMOVE;
            }
            std::string resp;
            {
                std::lock_guard<std::mutex> lock(appState.chatMutex);
                resp = appState.chatResponse;
            }
            appState.chatHistory += "Bot: " + resp + "\n\n";
            GtkTextBuffer* buf = gtk_text_view_get_buffer(GTK_TEXT_VIEW(appState.chatHistoryTv));
            gtk_text_buffer_set_text(buf, appState.chatHistory.c_str(), -1);
            GtkTextIter end;
            gtk_text_buffer_get_end_iter(buf, &end);
            gtk_text_view_scroll_to_iter(GTK_TEXT_VIEW(appState.chatHistoryTv), &end, 0.0, FALSE, 0, 0);
            return G_SOURCE_REMOVE;
        }, nullptr);
        return;
    }

    std::string apiKey(apiKeyEnv);
    std::string context = buildChatContext(userMessage);

    {
        std::lock_guard<std::mutex> lock(appState.chatMutex);
        appState.chatWaiting = true;
    }

    std::thread([userMessage, apiKey, context]() {
        // Lambda to escape a string for safe embedding inside a JSON string
        // literal. Without this, quotes or backslashes in the user message
        // or the context block would produce malformed JSON, causing the API
        // to reject the request with a parse error.
        auto escapeJson = [](const std::string& s) -> std::string {
            std::string out;
            for (char c : s) {
                switch (c) {
                    case '"':  out += "\\\""; break;  // " -> \"
                    case '\\': out += "\\\\"; break;  // backslash
                    case '\n': out += "\\n";  break;  // newline -> \n
                    case '\r': out += "\\r";  break;  // CR      -> \r
                    case '\t': out += "\\t";  break;  // tab     -> \t
                    default:   out += c;      break;
                }
            }
            return out;
        };

        // Prepend the full conversation history so Gemini remembers earlier Q&As
        std::string history;
        {
            std::lock_guard<std::mutex> lock(appState.chatMutex);
            history = appState.chatHistory;
        }
        // Assemble the full prompt: system context + conversation history + new question.
        // Everything goes into a single "text" part because Gemini's free tier
        // doesn't support multi-turn message arrays in the same way as Chat completions.
        std::string fullPrompt = context;
        if (!history.empty()) {
            fullPrompt += "\nConversation so far:\n" + history;
        }
        fullPrompt += "\nUser question: " + userMessage;
        std::string requestBody =
            "{\"contents\":[{\"parts\":[{\"text\":\""
            + escapeJson(fullPrompt)
            + "\"}]}]}";

        std::string url =
            "https://generativelanguage.googleapis.com/v1beta/models/"
            "gemini-2.0-flash:generateContent?key=" + apiKey;

        CURL* curl = curl_easy_init();
        if (!curl) {
            std::lock_guard<std::mutex> lock(appState.chatMutex);
            appState.chatResponse = "Error: could not initialize HTTP client.";
            appState.chatWaiting = false;
            return;
        }

        std::string response;
        struct curl_slist* headers = nullptr;
        headers = curl_slist_append(headers, "Content-Type: application/json");

        curl_easy_setopt(curl, CURLOPT_URL,            url.c_str());
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS,     requestBody.c_str());
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER,     headers);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION,  chatCurlWriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA,      &response);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT,        15L);   // abort if no response in 15 s
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L);    // skip cert check on the teaching servers
        curl_easy_setopt(curl, CURLOPT_USERAGENT,      "ECE297-Spatia/1.0");
        curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);    // follow HTTP 3xx redirects

        CURLcode res = curl_easy_perform(curl);
        // Always free headers and clean up the handle, even on failure.
        // curl_easy_cleanup() must be called after curl_easy_perform() to
        // avoid a resource leak regardless of whether the request succeeded.
        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);

        {
            std::lock_guard<std::mutex> lock(appState.chatMutex);
            if (res == CURLE_OK && !response.empty()) {
                appState.chatResponse = extractGeminiText(response);
            } else {
                appState.chatResponse = "Error: API request failed.";
            }
            appState.chatWaiting = false;
        }

        if (appState.chatHistoryTv) {
            // g_idle_add() is important. This lambda is called from the
            // background curl thread, so the UI must update back
            // onto the GTK main loop thread. Touching GtkTextView from a
            // non-main thread causes errors. G_SOURCE_REMOVE tells GLib to run this
            // callback exactly once and then remove it from the main loop's list.
            g_idle_add([](gpointer) -> gboolean {
                if (!appState.chatHistoryTv) {
                    return G_SOURCE_REMOVE;
                }
                std::string resp;
                {
                    std::lock_guard<std::mutex> lock(appState.chatMutex);
                    resp = appState.chatResponse;
                }
                appState.chatHistory += "Bot: " + resp + "\n\n";
                GtkTextBuffer* buf = gtk_text_view_get_buffer(GTK_TEXT_VIEW(appState.chatHistoryTv));
                gtk_text_buffer_set_text(buf, appState.chatHistory.c_str(), -1);
                GtkTextIter end;
                gtk_text_buffer_get_end_iter(buf, &end);
                gtk_text_view_scroll_to_iter(GTK_TEXT_VIEW(appState.chatHistoryTv), &end, 0.0, FALSE, 0, 0);
                return G_SOURCE_REMOVE;
            }, nullptr);
        }
    // detach() makes this a fire and forget thread. The UI is unblocked
    // immediately; the thread posts its result back to the GTK main loop
    // via g_idle_add() once the API call completes.
    }).detach();
}

// Called when the user clicks Send or presses Enter in the chat entry
void onChatSend(GtkWidget*, ezgl::application* app) {
    if (!appState.chatEntryW) {
        return;
    }

    std::string userMsg = gtk_entry_get_text(GTK_ENTRY(appState.chatEntryW));
    if (userMsg.empty()) {
        return;
    }

    // Don't allow sending another message while we're waiting for a response
    {
        std::lock_guard<std::mutex> lock(appState.chatMutex);
        if (appState.chatWaiting) {
            app->update_message("Please wait for the current response...");
            return;
        }
    }

    appState.chatHistory += "You: " + userMsg + "\n";

    if (appState.chatHistoryTv) {
        // Show a "Thinking..." placeholder immediately so the user knows
        // their message was received while the API call is in flight.
        std::string displayText = appState.chatHistory + "Bot: Thinking...\n\n";
        GtkTextBuffer* buf = gtk_text_view_get_buffer(GTK_TEXT_VIEW(appState.chatHistoryTv));
        gtk_text_buffer_set_text(buf, displayText.c_str(), -1);
        // Scroll to the bottom so the new message is visible without manual scrolling.
        // gtk_text_buffer_get_end_iter() gives an iterator pointing past the last
        // character; gtk_text_view_scroll_to_iter() then brings it into view.
        GtkTextIter end;
        gtk_text_buffer_get_end_iter(buf, &end);
        gtk_text_view_scroll_to_iter(GTK_TEXT_VIEW(appState.chatHistoryTv), &end, 0.0, FALSE, 0, 0);
    }

    gtk_entry_set_text(GTK_ENTRY(appState.chatEntryW), "");
    sendChatMessageAsync(userMsg, app);
}

// GTK "activate" signal on the text entry = user pressed Enter
void onChatEntryActivate(GtkWidget*, gpointer data) {
    ezgl::application* app = static_cast<ezgl::application*>(data);
    onChatSend(nullptr, app);
}
