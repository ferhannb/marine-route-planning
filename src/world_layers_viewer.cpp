#include "planner.h"
#include "world_layers_support.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>

namespace {

std::string defaultBathymetryPath() {
    namespace fs = std::filesystem;
    const fs::path data_dir(worldlayers::resolveInputPath("dataset"));
    if (fs::exists(data_dir) && fs::is_directory(data_dir)) {
        for (const auto& entry : fs::directory_iterator(data_dir)) {
            if (!entry.is_regular_file()) {
                continue;
            }
            const std::string name = entry.path().filename().string();
            std::string lowered = name;
            std::transform(lowered.begin(),
                           lowered.end(),
                           lowered.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            if (lowered.find("gebco") != std::string::npos &&
                (lowered.size() >= 4 && lowered.substr(lowered.size() - 4) == ".asc")) {
                return entry.path().string();
            }
        }
    }
    return worldlayers::resolveInputPath("dataset/osm_bathymetry_istanbul_genoa.geojson");
}

struct ViewerConfig {
    std::string land_geojson = "dataset/ne_10m_land.geojson";
    std::string tss_geojson = "dataset/osm_tss_istanbul_genoa.geojson";
    std::string bathymetry_geojson = defaultBathymetryPath();
    std::string svg_file = "output/world_layers_overview.svg";

    bool tss_explicit = false;
    bool bathymetry_explicit = false;
    bool manual_bounds = false;
    bool min_lat_set = false;
    bool max_lat_set = false;
    bool min_lon_set = false;
    bool max_lon_set = false;

    double min_lat = -90.0;
    double max_lat = 90.0;
    double min_lon = -180.0;
    double max_lon = 180.0;
};

void printUsage(const char* bin) {
    std::cout << "Kullanim:\n"
              << "  " << bin << " [--dataset <10m|50m|110m|osm-local|osm-world-detailed|/path/land.geojson>]\n"
              << "    [--tss <path/tss.geojson>|--no-tss]\n"
              << "    [--bathymetry <path/bathy.geojson>|--no-bathymetry]\n"
              << "    [--svg <file.svg>]\n"
              << "    [--min-lat <deg> --max-lat <deg> --min-lon <deg> --max-lon <deg>]\n"
              << "    [--help]\n";
}

void applyDatasetValue(const std::string& value, ViewerConfig& cfg) {
    if (value == "10m") {
        cfg.land_geojson = "dataset/ne_10m_land.geojson";
    } else if (value == "50m") {
        cfg.land_geojson = "dataset/ne_50m_land.geojson";
    } else if (value == "110m") {
        cfg.land_geojson = "dataset/ne_110m_land.geojson";
    } else if (value == "osm-local" || value == "osm_detailed" || value == "osm-land") {
        cfg.land_geojson = "dataset/osm_land_istanbul_genoa.geojson";
    } else if (value == "osm-world-detailed" || value == "osm-world") {
        cfg.land_geojson = ".tmp_osm_land/land-polygons-split-4326/land-polygons-split-4326/land_polygons.shp";
    } else {
        cfg.land_geojson = value;
    }
}

bool parseDoubleArg(const std::string& key, const std::string& value, double& out_value) {
    char* endptr = nullptr;
    const double parsed = std::strtod(value.c_str(), &endptr);
    if (endptr == value.c_str() || *endptr != '\0' || !std::isfinite(parsed)) {
        std::cerr << "Gecersiz " << key << " degeri: " << value << "\n";
        return false;
    }
    out_value = parsed;
    return true;
}

bool parseArgs(int argc, char** argv, ViewerConfig& cfg, bool& should_run) {
    should_run = true;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto requireValue = [&](const char* name) -> const char* {
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
        }
        if (arg == "--dataset") {
            const char* value = requireValue("--dataset");
            if (!value) {
                return false;
            }
            applyDatasetValue(value, cfg);
            continue;
        }
        if (arg == "--tss") {
            const char* value = requireValue("--tss");
            if (!value) {
                return false;
            }
            cfg.tss_geojson = value;
            cfg.tss_explicit = true;
            continue;
        }
        if (arg == "--bathymetry") {
            const char* value = requireValue("--bathymetry");
            if (!value) {
                return false;
            }
            cfg.bathymetry_geojson = value;
            cfg.bathymetry_explicit = true;
            continue;
        }
        if (arg == "--no-tss") {
            cfg.tss_geojson.clear();
            cfg.tss_explicit = true;
            continue;
        }
        if (arg == "--no-bathymetry") {
            cfg.bathymetry_geojson.clear();
            cfg.bathymetry_explicit = true;
            continue;
        }
        if (arg == "--svg") {
            const char* value = requireValue("--svg");
            if (!value) {
                return false;
            }
            cfg.svg_file = value;
            continue;
        }
        if (arg == "--min-lat") {
            const char* value = requireValue("--min-lat");
            if (!value || !parseDoubleArg("min-lat", value, cfg.min_lat)) {
                return false;
            }
            cfg.min_lat_set = true;
            continue;
        }
        if (arg == "--max-lat") {
            const char* value = requireValue("--max-lat");
            if (!value || !parseDoubleArg("max-lat", value, cfg.max_lat)) {
                return false;
            }
            cfg.max_lat_set = true;
            continue;
        }
        if (arg == "--min-lon") {
            const char* value = requireValue("--min-lon");
            if (!value || !parseDoubleArg("min-lon", value, cfg.min_lon)) {
                return false;
            }
            cfg.min_lon_set = true;
            continue;
        }
        if (arg == "--max-lon") {
            const char* value = requireValue("--max-lon");
            if (!value || !parseDoubleArg("max-lon", value, cfg.max_lon)) {
                return false;
            }
            cfg.max_lon_set = true;
            continue;
        }

        std::cerr << "Bilinmeyen arguman: " << arg << "\n";
        printUsage(argv[0]);
        return false;
    }

    const int bounds_count = static_cast<int>(cfg.min_lat_set) + static_cast<int>(cfg.max_lat_set) +
                             static_cast<int>(cfg.min_lon_set) + static_cast<int>(cfg.max_lon_set);
    if (bounds_count != 0 && bounds_count != 4) {
        std::cerr << "Viewport siniri icin min/max lat/lon degerlerinin tamami verilmelidir.\n";
        return false;
    }
    if (bounds_count == 4) {
        if (cfg.min_lat >= cfg.max_lat || cfg.min_lon >= cfg.max_lon) {
            std::cerr << "Viewport sinirlari gecersiz.\n";
            return false;
        }
        cfg.manual_bounds = true;
    }

    return true;
}

}  // namespace

int main(int argc, char** argv) {
    ViewerConfig cfg;
    bool should_run = true;
    if (!parseArgs(argc, argv, cfg, should_run)) {
        return 1;
    }
    if (!should_run) {
        return 0;
    }

    worldlayers::Config app_cfg;
    app_cfg.land_geojson = cfg.land_geojson;
    app_cfg.tss_geojson = cfg.tss_geojson;
    app_cfg.bathymetry_geojson = cfg.bathymetry_geojson;
    app_cfg.svg_file = cfg.svg_file;
    app_cfg.allow_missing_tss = !cfg.tss_explicit;
    app_cfg.allow_missing_bathymetry = !cfg.bathymetry_explicit;
    app_cfg.bounds.enabled = cfg.manual_bounds;
    app_cfg.bounds.min_lat = cfg.min_lat;
    app_cfg.bounds.max_lat = cfg.max_lat;
    app_cfg.bounds.min_lon = cfg.min_lon;
    app_cfg.bounds.max_lon = cfg.max_lon;

    worldlayers::LoadedData data;
    std::string error_message;
    std::string warning_message;
    if (!worldlayers::loadData(app_cfg, data, error_message, warning_message)) {
        std::cerr << error_message << "\n";
        return 1;
    }
    if (!warning_message.empty()) {
        std::cerr << warning_message;
    }
    if (!worldlayers::renderSvg(app_cfg, data, error_message)) {
        std::cerr << error_message << "\n";
        return 1;
    }
    const auto summary = worldlayers::summarize(data);

    std::cout << "World layers viewer hazir.\n";
    std::cout << "Land dataset: " << cfg.land_geojson << "\n";
    if (!cfg.tss_geojson.empty()) {
        std::cout << "TSS dataset: " << cfg.tss_geojson << "\n";
    } else {
        std::cout << "TSS dataset: devre disi\n";
    }
    if (!cfg.bathymetry_geojson.empty()) {
        std::cout << "Bathymetry dataset: " << cfg.bathymetry_geojson << "\n";
    } else {
        std::cout << "Bathymetry dataset: devre disi\n";
    }
    if (cfg.manual_bounds) {
        std::cout << "Viewport lat: [" << cfg.min_lat << ", " << cfg.max_lat << "] lon: [" << cfg.min_lon << ", "
                  << cfg.max_lon << "]\n";
    } else {
        std::cout << "Viewport: tum yuklu katmanlar\n";
    }
    std::cout << "Land polygon sayisi: " << summary.land_polygon_count << "\n";
    std::cout << "TSS feature sayisi: " << summary.tss_feature_count << "\n";
    std::cout << "Bathymetry feature sayisi: " << summary.bathymetry_feature_count << "\n";
    std::cout << "SVG cikti: " << cfg.svg_file << "\n";

    if (!cfg.tss_explicit || !cfg.bathymetry_explicit) {
        std::cout << "Not: repo icindeki varsayilan TSS ve bathymetri dosyalari su anda global degil; yeni GeoJSON dosya "
                     "yollari verildiginde ayni executable ile dunya kapsamina gecilebilir.\n";
    }

    return 0;
}
