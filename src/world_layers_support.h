#pragma once

#include "planner.h"

#include <string>
#include <vector>

namespace worldlayers {

struct BoundsConfig {
    bool enabled = false;
    double min_lat = -90.0;
    double max_lat = 90.0;
    double min_lon = -180.0;
    double max_lon = 180.0;
};

struct Config {
    std::string land_geojson = "dataset/ne_10m_land.geojson";
    std::string tss_geojson = "dataset/osm_tss_istanbul_genoa.geojson";
    std::string bathymetry_geojson = "dataset/osm_bathymetry_istanbul_genoa.geojson";
    std::string buffer_zones_geojson;
    std::string svg_file = "output/world_layers_overview.svg";
    bool allow_missing_tss = true;
    bool allow_missing_bathymetry = true;
    bool allow_missing_buffer_zones = true;
    bool enable_buffer_zones = false;
    std::vector<double> buffer_offsets_m;
    BoundsConfig bounds;
};

struct LoadedData {
    std::vector<planner::LandPolygon> land;
    std::vector<planner::TssFeature> tss;
    std::vector<planner::BathymetryFeature> bathymetry;
    std::vector<planner::BufferZonePolygon> buffer_zones;
};

struct Summary {
    std::size_t land_polygon_count = 0;
    std::size_t tss_feature_count = 0;
    std::size_t bathymetry_feature_count = 0;
    std::size_t buffer_zone_polygon_count = 0;
};

std::string projectRootPath();
std::string resolveInputPath(const std::string& path);
std::string resolveOutputPath(const std::string& path);

bool loadData(const Config& cfg, LoadedData& out_data, std::string& error_message, std::string& warning_message);
bool renderSvg(const Config& cfg, const LoadedData& data, std::string& error_message);
Summary summarize(const LoadedData& data);

}  // namespace worldlayers
