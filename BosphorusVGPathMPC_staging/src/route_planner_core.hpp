#pragma once

#include "planner.h"

#include <string>
#include <vector>

namespace planner {

struct RoutePlan {
    CliConfig cfg;
    Projection projection{};
    double min_lat = 0.0;
    double max_lat = 0.0;
    double min_lon = 0.0;
    double max_lon = 0.0;
    double coastline_clearance_km = 0.0;
    NamedPoint start_point;
    NamedPoint goal_point;
    std::vector<NamedPoint> vertices;
    std::vector<Triangle> triangles;
    std::vector<LandPolygon> land;
    std::vector<LandPolygon> shallow_polygons;
    std::vector<TssFeature> tss;
    std::vector<BathymetryFeature> bathymetry;
    RouteResult route;
};

bool computeRoutePlan(const CliConfig& cfg, RoutePlan& out_plan, std::string* error = nullptr);
bool writeRouteCsv(const RoutePlan& plan, const std::string& file_path, std::string* error = nullptr);
void writeRouteSvg(const RoutePlan& plan, const std::string& file_path);

}  // namespace planner
