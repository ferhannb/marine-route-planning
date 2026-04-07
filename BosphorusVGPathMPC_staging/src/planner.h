#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace planner {

struct LatLon {
    double lat_deg;
    double lon_deg;
};

struct NamedPoint {
    std::string name;
    LatLon geo;
};

struct XY {
    double x;
    double y;
};

struct LandPolygon {
    std::vector<LatLon> vertices;
    double min_lat;
    double max_lat;
    double min_lon;
    double max_lon;
};

struct TssFeature {
    std::vector<LatLon> points;
    bool closed = false;
    std::string seamark_type;
    double min_lat = 0.0;
    double max_lat = 0.0;
    double min_lon = 0.0;
    double max_lon = 0.0;
};

struct BathymetryFeature {
    std::vector<LatLon> points;
    bool closed = false;
    double depth_m = std::numeric_limits<double>::quiet_NaN();
    double min_lat = 0.0;
    double max_lat = 0.0;
    double min_lon = 0.0;
    double max_lon = 0.0;
};

struct BufferZonePolygon {
    LandPolygon polygon;
    double inner_m = 0.0;
    double outer_m = 0.0;
};

struct LandSpatialIndex {
    double cell_deg = 1.0;
    std::unordered_map<std::int64_t, std::vector<int>> bins;
};

struct GridCell {
    double min_lat;
    double max_lat;
    double min_lon;
    double max_lon;
    bool fine = false;
};

struct Projection {
    double ref_lat_rad;
    double ref_lon_rad;
};

enum class TrafficFlowDirection : std::uint8_t {
    Unknown = 0,
    Northbound = 1,
    Southbound = 2,
};

struct TrafficRoutingConfig {
    bool enable_tss_routing = false;
    bool hard_tss_corridor = false;
    bool directional_lane_selection = true;
    double lane_preference_multiplier = 1.0;
    double off_lane_multiplier = 1.0;
    double separation_zone_penalty_km = 0.0;
    double boundary_crossing_penalty_km = 0.0;
    double lane_corridor_half_width_km = 1.0;
    double lane_endpoint_radius_km = 3.0;
    double starboard_boundary_bias = 0.2;
};

struct Triangle {
    int a;
    int b;
    int c;
    XY centroid;
};

struct TriangleTrafficData {
    TrafficRoutingConfig config;
    bool active = false;
    bool has_lane_polygons = false;
    bool has_lane_lines = false;
    TrafficFlowDirection flow_direction = TrafficFlowDirection::Unknown;
    std::vector<std::uint8_t> triangle_in_lane;
    std::vector<std::uint8_t> triangle_in_zone;
    std::vector<std::uint8_t> triangle_in_hard_corridor;
    std::vector<double> triangle_lane_distance_km;
    std::vector<std::vector<XY>> lane_polygons_xy;
    std::vector<std::vector<XY>> lane_lines_xy;
    std::vector<XY> lane_endpoint_points_xy;
    std::vector<std::vector<XY>> starboard_boundary_lines_xy;
    std::vector<std::vector<XY>> zone_polygons_xy;
    std::vector<std::vector<XY>> boundary_lines_xy;
};

struct NeighborPortal {
    int to;
    int va;
    int vb;
};

struct TriangleSearchResult {
    bool found = false;
    std::vector<int> triangle_sequence;
};

struct RouteResult {
    bool found = false;
    double total_km = 0.0;
    std::vector<LatLon> polyline;
    std::vector<int> triangle_sequence;
};

struct PlannerCacheData {
    std::string dataset_path;
    std::string mesh_dataset_path;
    std::string bathymetry_path;
    double min_lat = 0.0;
    double max_lat = 0.0;
    double min_lon = 0.0;
    double max_lon = 0.0;
    double grid_step_deg = 0.0;
    double narrow_refine_step_deg = 0.0;
    double clearance_m = 100.0;
    double min_depth_m = 0.0;
    bool use_coastline_vertices_for_triangulation = false;
    double coastline_vertex_spacing_m = 0.0;
    double outer_frame_margin_deg = 0.0;
    std::vector<LandPolygon> land_polygons;
    std::vector<NamedPoint> vertices;
    std::vector<Triangle> triangles;
};

struct CliConfig {
    std::string land_geojson = "dataset/ne_10m_land.geojson";
    std::string mesh_land_geojson;
    std::string tss_geojson = "dataset/osm_tss_istanbul_genoa.geojson";
    std::string bathymetry_geojson;
    double grid_step_deg = 0.5;
    double narrow_refine_step_deg = 0.0;
    double corridor_lat_pad = 6.0;
    double corridor_lon_pad = 8.0;
    double mesh_min_lat = 0.0;
    double mesh_max_lat = 0.0;
    double mesh_min_lon = 0.0;
    double mesh_max_lon = 0.0;
    bool mesh_min_lat_set = false;
    bool mesh_max_lat_set = false;
    bool mesh_min_lon_set = false;
    bool mesh_max_lon_set = false;
    bool mesh_bounds_override = false;
    double clearance_m = 100.0;
    double min_depth_m = 0.0;
    bool use_coastline_vertices_for_triangulation = false;
    double coastline_vertex_spacing_m = 0.0;
    TrafficRoutingConfig traffic;
    LatLon start = {41.0082, 28.9784};
    LatLon goal = {44.4056, 8.9463};
    std::string start_name = "Start";
    std::string goal_name = "Goal";
    std::string svg_file = "output/istanbul_genoa_route.svg";
    std::string build_cache_file;
    std::string use_cache_file;
    bool use_cache_enabled = false;
    bool rebuild_cache = false;
};

double degToRad(double deg);
double radToDeg(double rad);
double clampLatitude(double lat_deg);
double wrapToPi(double angle_rad);
double haversineKm(double lat1_deg, double lon1_deg, double lat2_deg, double lon2_deg);
double haversineKm(const LatLon& a, const LatLon& b);
double computeOuterFrameMarginDeg(double grid_step_deg, double narrow_refine_step_deg);
Projection makeProjection(double min_lat, double max_lat, double min_lon, double max_lon);
XY toXY(const LatLon& p, const Projection& proj);
LatLon toGeo(const XY& p, const Projection& proj);

bool bboxesOverlap(double min_lat_a,
                   double max_lat_a,
                   double min_lon_a,
                   double max_lon_a,
                   double min_lat_b,
                   double max_lat_b,
                   double min_lon_b,
                   double max_lon_b);
std::vector<LandPolygon> filterPolygonsByBounds(const std::vector<LandPolygon>& polygons,
                                                double min_lat,
                                                double max_lat,
                                                double min_lon,
                                                double max_lon);
LandSpatialIndex buildLandSpatialIndex(const std::vector<LandPolygon>& polygons, double cell_deg);
std::vector<TssFeature> filterTssByBounds(const std::vector<TssFeature>& features,
                                          double min_lat,
                                          double max_lat,
                                          double min_lon,
                                          double max_lon);
std::vector<BathymetryFeature> filterBathymetryByBounds(const std::vector<BathymetryFeature>& features,
                                                        double min_lat,
                                                        double max_lat,
                                                        double min_lon,
                                                        double max_lon);
std::vector<LandPolygon> buildShallowWaterPolygons(const std::vector<BathymetryFeature>& features,
                                                   double min_depth_m);
bool isOnLand(const LatLon& p,
              const std::vector<LandPolygon>& land_polygons,
              const LandSpatialIndex& land_index,
              double clearance_km);

bool loadLandPolygonsGeoJson(const std::string& file_path, std::vector<LandPolygon>& polygons_out);
bool loadLandPolygons(const std::string& file_path, std::vector<LandPolygon>& polygons_out);
bool loadLandPolygons(const std::string& file_path,
                      std::vector<LandPolygon>& polygons_out,
                      double min_lat,
                      double max_lat,
                      double min_lon,
                      double max_lon);
bool loadTssFeaturesGeoJson(const std::string& file_path, std::vector<TssFeature>& out_features);
bool loadBathymetryFeaturesGeoJson(const std::string& file_path, std::vector<BathymetryFeature>& out_features);
bool loadBathymetryFeatures(const std::string& file_path, std::vector<BathymetryFeature>& out_features);

bool edgeNavigable(const LatLon& a,
                   const LatLon& b,
                   const std::vector<LandPolygon>& land_polygons,
                   const LandSpatialIndex& land_index,
                   double clearance_km,
                   const std::vector<LandPolygon>& shallow_polygons,
                   const LandSpatialIndex& shallow_index);
LatLon nearestNavigablePoint(const LatLon& p,
                             const std::vector<LandPolygon>& land_polygons,
                             const LandSpatialIndex& land_index,
                             double clearance_km,
                             const std::vector<LandPolygon>& shallow_polygons,
                             const LandSpatialIndex& shallow_index);
std::vector<NamedPoint> generateOceanGrid(double min_lat,
                                          double max_lat,
                                          double min_lon,
                                          double max_lon,
                                          double spacing_deg,
                                          const std::vector<LandPolygon>& land_polygons,
                                          const LandSpatialIndex& land_index,
                                          double clearance_km,
                                          double narrow_refine_step_deg,
                                          const std::vector<LandPolygon>& shallow_polygons,
                                          const LandSpatialIndex& shallow_index);
std::vector<Triangle> buildDelaunay(const std::vector<XY>& input_pts);
std::vector<Triangle> filterNavigableTriangles(const std::vector<Triangle>& all_triangles,
                                               const std::vector<NamedPoint>& vertices,
                                               const std::vector<LandPolygon>& land_polygons,
                                               const LandSpatialIndex& land_index,
                                               double clearance_km,
                                               double min_lat,
                                               double max_lat,
                                               double min_lon,
                                               double max_lon,
                                               double outer_frame_margin_deg,
                                               const std::vector<LandPolygon>& shallow_polygons,
                                               const LandSpatialIndex& shallow_index);
TriangleTrafficData buildTriangleTrafficData(const std::vector<Triangle>& triangles,
                                             const Projection& proj,
                                             const std::vector<TssFeature>& tss_features,
                                             const LatLon& start,
                                             const LatLon& goal,
                                             const TrafficRoutingConfig& config);
bool xySegmentAllowedByTraffic(const XY& a, const XY& b, const TriangleTrafficData& traffic_data);
std::vector<std::vector<NeighborPortal>> buildTriangleAdjacency(const std::vector<Triangle>& triangles);
std::vector<int> buildVertexUsage(const std::vector<Triangle>& triangles, int vertex_count);
std::vector<std::pair<double, int>> rankUsableVerticesByDistance(const std::vector<NamedPoint>& vertices,
                                                                 const std::vector<int>& usage,
                                                                 const LatLon& target);
std::vector<int> buildVertexComponentIds(const std::vector<Triangle>& triangles,
                                         int vertex_count,
                                         const std::vector<int>& usage);
TriangleSearchResult searchTriangleSequence(const std::vector<Triangle>& triangles,
                                            const std::vector<std::vector<NeighborPortal>>& adj,
                                            int start_vertex,
                                            int goal_vertex,
                                            const std::vector<XY>& vertex_xy,
                                            const TriangleTrafficData* traffic_data = nullptr);
std::vector<LatLon> shortcutPath(const std::vector<LatLon>& raw,
                                 const std::vector<LandPolygon>& land_polygons,
                                 const LandSpatialIndex& land_index,
                                 double clearance_km,
                                 const std::vector<LandPolygon>& shallow_polygons,
                                 const LandSpatialIndex& shallow_index);
std::vector<LatLon> shortcutPathWithTraffic(const std::vector<LatLon>& raw,
                                            const Projection& proj,
                                            const std::vector<LandPolygon>& land_polygons,
                                            const LandSpatialIndex& land_index,
                                            double clearance_km,
                                            const std::vector<LandPolygon>& shallow_polygons,
                                            const LandSpatialIndex& shallow_index,
                                            const TriangleTrafficData& traffic_data);
std::vector<LatLon> buildRouteAlongTssCenterline(const LatLon& start,
                                                 const LatLon& goal,
                                                 const Projection& proj,
                                                 const std::vector<TssFeature>& tss_features,
                                                 const std::vector<LandPolygon>& land_polygons,
                                                 const LandSpatialIndex& land_index,
                                                 double clearance_km,
                                                 const std::vector<LandPolygon>& shallow_polygons,
                                                 const LandSpatialIndex& shallow_index);
double polylineLengthKm(const std::vector<LatLon>& line);
void printRoute(const RouteResult& route);

void writeSvgPlot(const std::string& file_path,
                  const std::vector<NamedPoint>& vertices,
                  const std::vector<Triangle>& triangles,
                  const std::vector<int>& tri_sequence,
                  const std::vector<LatLon>& route,
                  const std::vector<LandPolygon>& land_polygons,
                  const std::vector<TssFeature>& tss_features,
                  const std::vector<BathymetryFeature>& bathymetry_features,
                  const std::vector<BufferZonePolygon>& buffer_zones = {});
bool savePlannerCache(const std::string& file_path, const PlannerCacheData& cache);
bool loadPlannerCache(const std::string& file_path, PlannerCacheData& out_cache);
bool isPlannerCacheCompatible(const PlannerCacheData& cache,
                              const CliConfig& cfg,
                              double min_lat,
                              double max_lat,
                              double min_lon,
                              double max_lon,
                              double clearance_m);

bool parseCli(int argc, char** argv, CliConfig& cfg, bool& should_run);

}  // namespace planner
