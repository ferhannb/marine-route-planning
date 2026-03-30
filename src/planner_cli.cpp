#include "planner.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>

namespace planner {
namespace {

void printUsage(const char* bin) {
    std::cout << "Kullanim:\n"
              << "  " << bin
              << " [--dataset <10m|50m|110m|/path/file.geojson>] [--grid-step <deg>]\n"
              << "    [--tss <path/tss.geojson>] [--bathymetry <path/bathy.geojson>] [--min-depth-m <m>]\n"
              << "    [--narrow-refine-step <deg>]\n"
              << "    [--corridor-lat <deg>] [--corridor-lon <deg>] [--clearance-m <m>]\n"
              << "    [--mesh-min-lat <deg>] [--mesh-max-lat <deg>] [--mesh-min-lon <deg>] [--mesh-max-lon <deg>]\n"
              << "    [--start-lat <deg>] [--start-lon <deg>] [--goal-lat <deg>] [--goal-lon <deg>]\n"
              << "    [--start-name <name>] [--goal-name <name>] [--svg <file.svg>] [--help]\n"
              << "    [--build-cache <file.bin>] [--use-cache <file.bin>] [--load-cache] [--config <file.ini>]\n";
}

bool finalizeMeshBoundsConfig(CliConfig& cfg) {
    if (cfg.narrow_refine_step_deg > 0.0 && cfg.narrow_refine_step_deg >= cfg.grid_step_deg) {
        std::cerr << "narrow_refine_step_deg, grid_step_deg degerinden kucuk olmali.\n";
        return false;
    }
    if (cfg.traffic.lane_preference_multiplier <= 0.0 || cfg.traffic.off_lane_multiplier <= 0.0) {
        std::cerr << "traffic lane/off-lane multiplier degerleri pozitif olmali.\n";
        return false;
    }
    if (cfg.traffic.separation_zone_penalty_km < 0.0 || cfg.traffic.boundary_crossing_penalty_km < 0.0) {
        std::cerr << "traffic ceza degerleri negatif olamaz.\n";
        return false;
    }
    if (cfg.traffic.lane_corridor_half_width_km <= 0.0) {
        std::cerr << "traffic lane_corridor_half_width_km pozitif olmali.\n";
        return false;
    }
    if (cfg.traffic.lane_endpoint_radius_km <= 0.0) {
        std::cerr << "traffic lane_endpoint_radius_km pozitif olmali.\n";
        return false;
    }
    if (cfg.traffic.starboard_boundary_bias < 0.0) {
        std::cerr << "traffic starboard_boundary_bias negatif olamaz.\n";
        return false;
    }
    int bounds_count = 0;
    bounds_count += cfg.mesh_min_lat_set ? 1 : 0;
    bounds_count += cfg.mesh_max_lat_set ? 1 : 0;
    bounds_count += cfg.mesh_min_lon_set ? 1 : 0;
    bounds_count += cfg.mesh_max_lon_set ? 1 : 0;

    if (bounds_count == 0) {
        cfg.mesh_bounds_override = false;
        return true;
    }
    if (bounds_count != 4) {
        std::cerr
            << "Mesh sabit sinirlari icin mesh.min_lat, mesh.max_lat, mesh.min_lon, mesh.max_lon birlikte verilmeli.\n";
        return false;
    }
    if (cfg.mesh_min_lat >= cfg.mesh_max_lat) {
        std::cerr << "mesh.min_lat, mesh.max_lat degerinden kucuk olmali.\n";
        return false;
    }
    if (cfg.mesh_min_lon >= cfg.mesh_max_lon) {
        std::cerr << "mesh.min_lon, mesh.max_lon degerinden kucuk olmali.\n";
        return false;
    }
    cfg.mesh_bounds_override = true;
    return true;
}

bool parseDoubleArg(const std::string& key, const std::string& value, double& out) {
    char* endptr = nullptr;
    const double v = std::strtod(value.c_str(), &endptr);
    if (endptr == value.c_str() || *endptr != '\0' || !std::isfinite(v)) {
        std::cerr << "Gecersiz " << key << " degeri: " << value << "\n";
        return false;
    }
    out = v;
    return true;
}

bool parseBoolArg(const std::string& key, const std::string& value, bool& out) {
    std::string v = value;
    std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (v == "1" || v == "true" || v == "yes" || v == "on") {
        out = true;
        return true;
    }
    if (v == "0" || v == "false" || v == "no" || v == "off") {
        out = false;
        return true;
    }
    std::cerr << "Gecersiz " << key << " degeri: " << value << "\n";
    return false;
}

std::string trimCopy(const std::string& s) {
    std::size_t b = 0;
    std::size_t e = s.size();
    while (b < e && std::isspace(static_cast<unsigned char>(s[b])) != 0) {
        ++b;
    }
    while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1])) != 0) {
        --e;
    }
    return s.substr(b, e - b);
}

std::string toLowerCopy(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

void applyDatasetValue(const std::string& value, CliConfig& cfg) {
    const std::string ds = value;
    if (ds == "10m") {
        cfg.land_geojson = "dataset/ne_10m_land.geojson";
    } else if (ds == "50m") {
        cfg.land_geojson = "dataset/ne_50m_land.geojson";
    } else if (ds == "110m") {
        cfg.land_geojson = "dataset/ne_110m_land.geojson";
    } else {
        cfg.land_geojson = ds;
    }
}

bool loadIniConfig(const std::string& ini_path, CliConfig& cfg) {
    std::ifstream in(ini_path);
    if (!in) {
        std::cerr << "INI acilamadi: " << ini_path << "\n";
        return false;
    }

    std::string section;
    std::string line;
    int line_no = 0;
    while (std::getline(in, line)) {
        line_no += 1;
        std::string t = trimCopy(line);
        if (t.empty() || t[0] == ';' || t[0] == '#') {
            continue;
        }
        if (t.front() == '[' && t.back() == ']') {
            section = toLowerCopy(trimCopy(t.substr(1, t.size() - 2)));
            continue;
        }

        const std::size_t eq_pos = t.find('=');
        if (eq_pos == std::string::npos) {
            std::cerr << "INI parse hatasi (" << ini_path << ":" << line_no << "): key=value bekleniyor.\n";
            return false;
        }

        const std::string key = toLowerCopy(trimCopy(t.substr(0, eq_pos)));
        std::string value = trimCopy(t.substr(eq_pos + 1));
        if (!value.empty() && (value.front() == '"' || value.front() == '\'')) {
            if (value.size() >= 2 && value.back() == value.front()) {
                value = value.substr(1, value.size() - 2);
            }
        } else {
            const std::size_t semicolon = value.find(';');
            const std::size_t hash = value.find('#');
            const std::size_t cut = std::min(semicolon, hash);
            if (cut != std::string::npos) {
                value = trimCopy(value.substr(0, cut));
            }
        }

        const std::string full_key = section.empty() ? key : (section + "." + key);
        auto parseIniDouble = [&](const std::string& name, double& out, bool positive_only, bool nonnegative_only) {
            if (!parseDoubleArg(name, value, out)) {
                return false;
            }
            if (positive_only && out <= 0.0) {
                std::cerr << "INI degeri pozitif olmali (" << name << "): " << value << "\n";
                return false;
            }
            if (nonnegative_only && out < 0.0) {
                std::cerr << "INI degeri negatif olamaz (" << name << "): " << value << "\n";
                return false;
            }
            return true;
        };

        if (full_key == "mesh.land_geojson" || full_key == "mesh.mesh_land_geojson" || key == "mesh_land_geojson") {
            cfg.mesh_land_geojson = value;
            continue;
        }
        if (full_key == "dataset.land_geojson" || (section.empty() && key == "land_geojson")) {
            applyDatasetValue(value, cfg);
            continue;
        }
        if (full_key == "tss.geojson" || full_key == "dataset.tss_geojson" || key == "tss_geojson") {
            cfg.tss_geojson = value;
            continue;
        }
        if (full_key == "bathymetry.geojson" || full_key == "dataset.bathymetry_geojson" ||
            key == "bathymetry_geojson") {
            cfg.bathymetry_geojson = value;
            continue;
        }
        if (full_key == "mesh.grid_step_deg" || key == "grid_step_deg") {
            if (!parseIniDouble("grid_step_deg", cfg.grid_step_deg, true, false)) {
                return false;
            }
            continue;
        }
        if (full_key == "mesh.narrow_refine_step_deg" || key == "narrow_refine_step_deg") {
            if (!parseIniDouble("narrow_refine_step_deg", cfg.narrow_refine_step_deg, true, false)) {
                return false;
            }
            continue;
        }
        if (full_key == "mesh.use_coastline_vertices" || key == "use_coastline_vertices") {
            if (!parseBoolArg("use_coastline_vertices", value, cfg.use_coastline_vertices_for_triangulation)) {
                return false;
            }
            continue;
        }
        if (full_key == "mesh.coastline_vertex_spacing_m" || key == "coastline_vertex_spacing_m") {
            if (!parseIniDouble("coastline_vertex_spacing_m", cfg.coastline_vertex_spacing_m, false, true)) {
                return false;
            }
            continue;
        }
        if (full_key == "mesh.corridor_lat_pad_deg" || key == "corridor_lat_pad_deg") {
            if (!parseIniDouble("corridor_lat_pad_deg", cfg.corridor_lat_pad, false, true)) {
                return false;
            }
            continue;
        }
        if (full_key == "mesh.corridor_lon_pad_deg" || key == "corridor_lon_pad_deg") {
            if (!parseIniDouble("corridor_lon_pad_deg", cfg.corridor_lon_pad, false, true)) {
                return false;
            }
            continue;
        }
        if (full_key == "mesh.min_lat" || key == "mesh_min_lat") {
            if (!parseIniDouble("mesh.min_lat", cfg.mesh_min_lat, false, false)) {
                return false;
            }
            cfg.mesh_min_lat_set = true;
            continue;
        }
        if (full_key == "mesh.max_lat" || key == "mesh_max_lat") {
            if (!parseIniDouble("mesh.max_lat", cfg.mesh_max_lat, false, false)) {
                return false;
            }
            cfg.mesh_max_lat_set = true;
            continue;
        }
        if (full_key == "mesh.min_lon" || key == "mesh_min_lon") {
            if (!parseIniDouble("mesh.min_lon", cfg.mesh_min_lon, false, false)) {
                return false;
            }
            cfg.mesh_min_lon_set = true;
            continue;
        }
        if (full_key == "mesh.max_lon" || key == "mesh_max_lon") {
            if (!parseIniDouble("mesh.max_lon", cfg.mesh_max_lon, false, false)) {
                return false;
            }
            cfg.mesh_max_lon_set = true;
            continue;
        }
        if (full_key == "safety.coastline_clearance_m" || key == "coastline_clearance_m") {
            if (!parseIniDouble("coastline_clearance_m", cfg.clearance_m, false, true)) {
                return false;
            }
            continue;
        }
        if (full_key == "safety.min_depth_m" || key == "min_depth_m") {
            if (!parseIniDouble("min_depth_m", cfg.min_depth_m, false, true)) {
                return false;
            }
            continue;
        }
        if (full_key == "traffic.enable_tss_routing" || key == "enable_tss_routing") {
            if (!parseBoolArg("enable_tss_routing", value, cfg.traffic.enable_tss_routing)) {
                return false;
            }
            continue;
        }
        if (full_key == "traffic.hard_tss_corridor" || key == "hard_tss_corridor") {
            if (!parseBoolArg("hard_tss_corridor", value, cfg.traffic.hard_tss_corridor)) {
                return false;
            }
            continue;
        }
        if (full_key == "traffic.directional_lane_selection" || key == "directional_lane_selection") {
            if (!parseBoolArg("directional_lane_selection", value, cfg.traffic.directional_lane_selection)) {
                return false;
            }
            continue;
        }
        if (full_key == "traffic.lane_preference_multiplier" || key == "lane_preference_multiplier") {
            if (!parseIniDouble("lane_preference_multiplier", cfg.traffic.lane_preference_multiplier, true, false)) {
                return false;
            }
            continue;
        }
        if (full_key == "traffic.off_lane_multiplier" || key == "off_lane_multiplier") {
            if (!parseIniDouble("off_lane_multiplier", cfg.traffic.off_lane_multiplier, true, false)) {
                return false;
            }
            continue;
        }
        if (full_key == "traffic.separation_zone_penalty_km" || key == "separation_zone_penalty_km") {
            if (!parseIniDouble("separation_zone_penalty_km", cfg.traffic.separation_zone_penalty_km, false, true)) {
                return false;
            }
            continue;
        }
        if (full_key == "traffic.boundary_crossing_penalty_km" || key == "boundary_crossing_penalty_km") {
            if (!parseIniDouble("boundary_crossing_penalty_km",
                                cfg.traffic.boundary_crossing_penalty_km,
                                false,
                                true)) {
                return false;
            }
            continue;
        }
        if (full_key == "traffic.lane_corridor_half_width_km" || key == "lane_corridor_half_width_km") {
            if (!parseIniDouble("lane_corridor_half_width_km", cfg.traffic.lane_corridor_half_width_km, true, false)) {
                return false;
            }
            continue;
        }
        if (full_key == "traffic.lane_endpoint_radius_km" || key == "lane_endpoint_radius_km") {
            if (!parseIniDouble("lane_endpoint_radius_km", cfg.traffic.lane_endpoint_radius_km, true, false)) {
                return false;
            }
            continue;
        }
        if (full_key == "traffic.starboard_boundary_bias" || key == "starboard_boundary_bias") {
            if (!parseIniDouble("starboard_boundary_bias", cfg.traffic.starboard_boundary_bias, false, true)) {
                return false;
            }
            continue;
        }
        if (full_key == "route.start_lat" || key == "start_lat") {
            if (!parseIniDouble("start_lat", cfg.start.lat_deg, false, false)) {
                return false;
            }
            continue;
        }
        if (full_key == "route.start_lon" || key == "start_lon") {
            if (!parseIniDouble("start_lon", cfg.start.lon_deg, false, false)) {
                return false;
            }
            continue;
        }
        if (full_key == "route.goal_lat" || key == "goal_lat") {
            if (!parseIniDouble("goal_lat", cfg.goal.lat_deg, false, false)) {
                return false;
            }
            continue;
        }
        if (full_key == "route.goal_lon" || key == "goal_lon") {
            if (!parseIniDouble("goal_lon", cfg.goal.lon_deg, false, false)) {
                return false;
            }
            continue;
        }
        if (full_key == "route.start_name" || key == "start_name") {
            cfg.start_name = value;
            continue;
        }
        if (full_key == "route.goal_name" || key == "goal_name") {
            cfg.goal_name = value;
            continue;
        }
        if (full_key == "cache.build_cache_file" || key == "build_cache_file") {
            cfg.build_cache_file = value;
            continue;
        }
        if (full_key == "cache.use_cache_file" || key == "use_cache_file") {
            cfg.use_cache_file = value;
            continue;
        }
        if (full_key == "cache.use_cache" || full_key == "cache.load_cache" || key == "use_cache" ||
            key == "load_cache") {
            if (!parseBoolArg("use_cache", value, cfg.use_cache_enabled)) {
                return false;
            }
            continue;
        }
        if (full_key == "output.svg_file" || key == "svg_file") {
            cfg.svg_file = value;
            continue;
        }
    }
    return true;
}

}  // namespace

bool parseCli(int argc, char** argv, CliConfig& cfg, bool& should_run) {
    should_run = true;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto needValue = [&](const char* name) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << name << " parametresi bir deger bekliyor.\n";
                return nullptr;
            }
            return argv[++i];
        };

        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            should_run = false;
            return true;
        } else if (arg == "--config") {
            const char* v = needValue("--config");
            if (!v) {
                return false;
            }
            if (!loadIniConfig(v, cfg)) {
                return false;
            }
        } else if (arg == "--dataset") {
            const char* v = needValue("--dataset");
            if (!v) {
                return false;
            }
            applyDatasetValue(v, cfg);
        } else if (arg == "--mesh-land-dataset") {
            const char* v = needValue("--mesh-land-dataset");
            if (!v) {
                return false;
            }
            cfg.mesh_land_geojson = v;
        } else if (arg == "--tss") {
            const char* v = needValue("--tss");
            if (!v) {
                return false;
            }
            cfg.tss_geojson = v;
        } else if (arg == "--bathymetry") {
            const char* v = needValue("--bathymetry");
            if (!v) {
                return false;
            }
            cfg.bathymetry_geojson = v;
        } else if (arg == "--min-depth-m") {
            const char* v = needValue("--min-depth-m");
            if (!v || !parseDoubleArg("--min-depth-m", v, cfg.min_depth_m) || cfg.min_depth_m < 0.0) {
                std::cerr << "--min-depth-m >= 0 olmali.\n";
                return false;
            }
        } else if (arg == "--grid-step") {
            const char* v = needValue("--grid-step");
            if (!v || !parseDoubleArg("--grid-step", v, cfg.grid_step_deg) || cfg.grid_step_deg <= 0.0) {
                std::cerr << "--grid-step > 0 olmali.\n";
                return false;
            }
        } else if (arg == "--narrow-refine-step") {
            const char* v = needValue("--narrow-refine-step");
            if (!v || !parseDoubleArg("--narrow-refine-step", v, cfg.narrow_refine_step_deg) ||
                cfg.narrow_refine_step_deg <= 0.0) {
                std::cerr << "--narrow-refine-step > 0 olmali.\n";
                return false;
            }
        } else if (arg == "--use-coastline-vertices") {
            cfg.use_coastline_vertices_for_triangulation = true;
        } else if (arg == "--coastline-vertex-spacing-m") {
            const char* v = needValue("--coastline-vertex-spacing-m");
            if (!v || !parseDoubleArg("--coastline-vertex-spacing-m", v, cfg.coastline_vertex_spacing_m) ||
                cfg.coastline_vertex_spacing_m < 0.0) {
                std::cerr << "--coastline-vertex-spacing-m >= 0 olmali.\n";
                return false;
            }
        } else if (arg == "--corridor-lat") {
            const char* v = needValue("--corridor-lat");
            if (!v || !parseDoubleArg("--corridor-lat", v, cfg.corridor_lat_pad) || cfg.corridor_lat_pad < 0.0) {
                std::cerr << "--corridor-lat >= 0 olmali.\n";
                return false;
            }
        } else if (arg == "--corridor-lon") {
            const char* v = needValue("--corridor-lon");
            if (!v || !parseDoubleArg("--corridor-lon", v, cfg.corridor_lon_pad) || cfg.corridor_lon_pad < 0.0) {
                std::cerr << "--corridor-lon >= 0 olmali.\n";
                return false;
            }
        } else if (arg == "--mesh-min-lat") {
            const char* v = needValue("--mesh-min-lat");
            if (!v || !parseDoubleArg("--mesh-min-lat", v, cfg.mesh_min_lat)) {
                return false;
            }
            cfg.mesh_min_lat_set = true;
        } else if (arg == "--mesh-max-lat") {
            const char* v = needValue("--mesh-max-lat");
            if (!v || !parseDoubleArg("--mesh-max-lat", v, cfg.mesh_max_lat)) {
                return false;
            }
            cfg.mesh_max_lat_set = true;
        } else if (arg == "--mesh-min-lon") {
            const char* v = needValue("--mesh-min-lon");
            if (!v || !parseDoubleArg("--mesh-min-lon", v, cfg.mesh_min_lon)) {
                return false;
            }
            cfg.mesh_min_lon_set = true;
        } else if (arg == "--mesh-max-lon") {
            const char* v = needValue("--mesh-max-lon");
            if (!v || !parseDoubleArg("--mesh-max-lon", v, cfg.mesh_max_lon)) {
                return false;
            }
            cfg.mesh_max_lon_set = true;
        } else if (arg == "--clearance-m") {
            const char* v = needValue("--clearance-m");
            if (!v || !parseDoubleArg("--clearance-m", v, cfg.clearance_m) || cfg.clearance_m < 0.0) {
                std::cerr << "--clearance-m >= 0 olmali.\n";
                return false;
            }
        } else if (arg == "--start-lat") {
            const char* v = needValue("--start-lat");
            if (!v || !parseDoubleArg("--start-lat", v, cfg.start.lat_deg)) {
                return false;
            }
        } else if (arg == "--start-lon") {
            const char* v = needValue("--start-lon");
            if (!v || !parseDoubleArg("--start-lon", v, cfg.start.lon_deg)) {
                return false;
            }
        } else if (arg == "--goal-lat") {
            const char* v = needValue("--goal-lat");
            if (!v || !parseDoubleArg("--goal-lat", v, cfg.goal.lat_deg)) {
                return false;
            }
        } else if (arg == "--goal-lon") {
            const char* v = needValue("--goal-lon");
            if (!v || !parseDoubleArg("--goal-lon", v, cfg.goal.lon_deg)) {
                return false;
            }
        } else if (arg == "--start-name") {
            const char* v = needValue("--start-name");
            if (!v) {
                return false;
            }
            cfg.start_name = v;
        } else if (arg == "--goal-name") {
            const char* v = needValue("--goal-name");
            if (!v) {
                return false;
            }
            cfg.goal_name = v;
        } else if (arg == "--svg") {
            const char* v = needValue("--svg");
            if (!v) {
                return false;
            }
            cfg.svg_file = v;
        } else if (arg == "--build-cache") {
            const char* v = needValue("--build-cache");
            if (!v) {
                return false;
            }
            cfg.build_cache_file = v;
        } else if (arg == "--use-cache") {
            const char* v = needValue("--use-cache");
            if (!v) {
                return false;
            }
            cfg.use_cache_file = v;
            cfg.use_cache_enabled = true;
        } else if (arg == "--load-cache") {
            cfg.use_cache_enabled = true;
        } else {
            std::cerr << "Bilinmeyen arguman: " << arg << "\n";
            printUsage(argv[0]);
            return false;
        }
    }
    return finalizeMeshBoundsConfig(cfg);
}

}  // namespace planner
