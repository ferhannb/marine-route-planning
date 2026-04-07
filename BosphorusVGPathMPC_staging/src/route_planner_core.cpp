#include "route_planner_core.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <unordered_map>
#include <utility>

namespace planner {
namespace {

bool fail(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = message;
    }
    return false;
}

std::string meshDatasetPath(const CliConfig& cfg) {
    return cfg.mesh_land_geojson.empty() ? cfg.land_geojson : cfg.mesh_land_geojson;
}

void ensureParentDir(const std::string& file_path) {
    const std::filesystem::path path(file_path);
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
    }
}

}  // namespace

bool computeRoutePlan(const CliConfig& cfg, RoutePlan& out_plan, std::string* error) {
    out_plan = {};
    out_plan.cfg = cfg;

    std::vector<LandPolygon> all_land;
    if (!loadLandPolygons(cfg.land_geojson, all_land)) {
        return fail(error, "Dataset yok. Once su scripti calistirin: scripts/download_coastline_data.sh");
    }

    std::vector<LandPolygon> all_mesh_land = all_land;
    if (!cfg.mesh_land_geojson.empty()) {
        if (!loadLandPolygons(cfg.mesh_land_geojson, all_mesh_land)) {
            return fail(error, "Mesh dataset yuklenemedi: " + cfg.mesh_land_geojson);
        }
    }

    std::vector<TssFeature> all_tss;
    if (!cfg.tss_geojson.empty()) {
        if (!loadTssFeaturesGeoJson(cfg.tss_geojson, all_tss)) {
            std::cerr << "TSS katmani yuklenemedi, TSS olmadan devam ediliyor.\n";
            all_tss.clear();
        }
    }

    std::vector<BathymetryFeature> all_bathymetry;
    if (!cfg.bathymetry_geojson.empty()) {
        if (!loadBathymetryFeatures(cfg.bathymetry_geojson, all_bathymetry)) {
            return fail(error, "Bathymetry yuklenemedi: " + cfg.bathymetry_geojson);
        }
    }

    const auto all_land_index = buildLandSpatialIndex(all_land, 0.5);
    const auto all_shallow_polygons = buildShallowWaterPolygons(all_bathymetry, cfg.min_depth_m);
    const auto all_shallow_index = buildLandSpatialIndex(all_shallow_polygons, 0.5);
    const double coastline_clearance_km = cfg.clearance_m / 1000.0;

    std::cout << "Yuklenen kara poligonu sayisi: " << all_land.size() << "\n";
    if (!cfg.mesh_land_geojson.empty()) {
        std::cout << "Yuklenen mesh kara poligonu sayisi: " << all_mesh_land.size() << "\n";
    }
    std::cout << "Yuklenen TSS feature sayisi: " << all_tss.size() << "\n";
    std::cout << "Yuklenen bathymetri feature sayisi: " << all_bathymetry.size() << "\n";

    std::cout << "Dataset: " << cfg.land_geojson << "\n";
    if (!cfg.mesh_land_geojson.empty()) {
        std::cout << "Mesh dataset: " << cfg.mesh_land_geojson << "\n";
    }
    if (!cfg.bathymetry_geojson.empty()) {
        std::cout << "Bathymetry: " << cfg.bathymetry_geojson << "\n";
    }
    std::cout << "Grid step (deg): " << cfg.grid_step_deg << "\n";
    if (cfg.narrow_refine_step_deg > 0.0) {
        std::cout << "Narrow-water refine step (deg): " << cfg.narrow_refine_step_deg << "\n";
    }
    std::cout << "Corridor lat/lon pad (deg): " << cfg.corridor_lat_pad << " / " << cfg.corridor_lon_pad << "\n";
    std::cout << "Coastline clearance (m): " << (coastline_clearance_km * 1000.0) << "\n";
    if (cfg.min_depth_m > 0.0) {
        std::cout << "Min depth constraint (m): " << cfg.min_depth_m << "\n";
        std::cout << "Shallow polygon sayisi: " << all_shallow_polygons.size() << "\n";
    }
    if (cfg.min_depth_m > 0.0 && cfg.bathymetry_geojson.empty()) {
        std::cout << "Uyari: min_depth_m verildi ama bathymetri dosyasi tanimlanmadi; derinlik filtresi uygulanmayacak.\n";
    }

    NamedPoint start_point{cfg.start_name, cfg.start};
    NamedPoint goal_point{cfg.goal_name, cfg.goal};
    start_point.geo = nearestNavigablePoint(start_point.geo,
                                            all_land,
                                            all_land_index,
                                            coastline_clearance_km,
                                            all_shallow_polygons,
                                            all_shallow_index);
    goal_point.geo = nearestNavigablePoint(goal_point.geo,
                                           all_land,
                                           all_land_index,
                                           coastline_clearance_km,
                                           all_shallow_polygons,
                                           all_shallow_index);

    std::vector<NamedPoint> seed_vertices;
    double min_lat = 0.0;
    double max_lat = 0.0;
    double min_lon = 0.0;
    double max_lon = 0.0;
    if (cfg.mesh_bounds_override) {
        min_lat = clampLatitude(cfg.mesh_min_lat);
        max_lat = clampLatitude(cfg.mesh_max_lat);
        min_lon = std::max(-179.0, std::min(179.0, cfg.mesh_min_lon));
        max_lon = std::max(-179.0, std::min(179.0, cfg.mesh_max_lon));
        if (min_lat >= max_lat || min_lon >= max_lon) {
            return fail(error, "Mesh sinirlari gecersiz. mesh.min/max degerlerini kontrol edin.");
        }
        std::cout << "Mesh bounds mode: manual\n";
    } else {
        NamedPoint s0{"Seed-Istanbul", {41.0082, 28.9784}};
        NamedPoint s1{"Seed-Bosphorus-South", {40.97, 28.96}};
        NamedPoint s2{"Seed-Marmara-Mid", {40.74, 27.55}};
        NamedPoint s3{"Seed-Dardanelles-East", {40.16, 26.72}};
        NamedPoint s4{"Seed-Dardanelles-West", {40.05, 26.22}};
        NamedPoint s5{"Seed-Genoa", {44.4056, 8.9463}};

        s0.geo = nearestNavigablePoint(s0.geo, all_land, all_land_index, coastline_clearance_km, all_shallow_polygons,
                                       all_shallow_index);
        s1.geo = nearestNavigablePoint(s1.geo, all_land, all_land_index, coastline_clearance_km, all_shallow_polygons,
                                       all_shallow_index);
        s2.geo = nearestNavigablePoint(s2.geo, all_land, all_land_index, coastline_clearance_km, all_shallow_polygons,
                                       all_shallow_index);
        s3.geo = nearestNavigablePoint(s3.geo, all_land, all_land_index, coastline_clearance_km, all_shallow_polygons,
                                       all_shallow_index);
        s4.geo = nearestNavigablePoint(s4.geo, all_land, all_land_index, coastline_clearance_km, all_shallow_polygons,
                                       all_shallow_index);
        s5.geo = nearestNavigablePoint(s5.geo, all_land, all_land_index, coastline_clearance_km, all_shallow_polygons,
                                       all_shallow_index);
        seed_vertices = {s0, s1, s2, s3, s4, s5};

        min_lat = clampLatitude(std::min(s4.geo.lat_deg, s5.geo.lat_deg) - cfg.corridor_lat_pad);
        max_lat = clampLatitude(std::max(s4.geo.lat_deg, s5.geo.lat_deg) + cfg.corridor_lat_pad);
        min_lon = std::max(-179.0, std::min(s4.geo.lon_deg, s5.geo.lon_deg) - cfg.corridor_lon_pad);
        max_lon = std::min(179.0, std::max(s4.geo.lon_deg, s5.geo.lon_deg) + cfg.corridor_lon_pad);
        std::cout << "Mesh bounds mode: seed+pad\n";
    }

    auto land = filterPolygonsByBounds(all_land, min_lat, max_lat, min_lon, max_lon);
    auto mesh_land = filterPolygonsByBounds(all_mesh_land, min_lat, max_lat, min_lon, max_lon);
    auto land_index = buildLandSpatialIndex(land, 0.5);
    auto mesh_land_index = buildLandSpatialIndex(mesh_land, 0.5);
    auto tss = filterTssByBounds(all_tss, min_lat, max_lat, min_lon, max_lon);
    auto bathymetry = filterBathymetryByBounds(all_bathymetry, min_lat, max_lat, min_lon, max_lon);
    auto shallow_polygons = buildShallowWaterPolygons(bathymetry, cfg.min_depth_m);
    auto shallow_index = buildLandSpatialIndex(shallow_polygons, 0.5);

    std::cout << "Koridordeki kara poligonu: " << land.size() << "\n";
    std::cout << "Koridordeki mesh kara poligonu: " << mesh_land.size() << "\n";
    std::cout << "Koridordeki TSS feature: " << tss.size() << "\n";
    std::cout << "Koridordeki bathymetri feature: " << bathymetry.size() << "\n";
    if (cfg.min_depth_m > 0.0) {
        std::cout << "Koridordeki sig alan poligonu: " << shallow_polygons.size() << "\n";
    }
    std::cout << "Mesh bounds lat: [" << min_lat << ", " << max_lat << "] lon: [" << min_lon << ", " << max_lon
              << "]\n";

    auto pointInsideBounds = [&](const LatLon& p) {
        return p.lat_deg >= min_lat && p.lat_deg <= max_lat && p.lon_deg >= min_lon && p.lon_deg <= max_lon;
    };
    if (!pointInsideBounds(start_point.geo) || !pointInsideBounds(goal_point.geo)) {
        return fail(error,
                    "Start/goal mesh kapsami disinda. route/start-goal veya mesh.min/max sinirlarini guncelleyin.");
    }

    const Projection proj = makeProjection(min_lat, max_lat, min_lon, max_lon);
    std::vector<NamedPoint> vertices;
    std::vector<Triangle> triangles;
    std::vector<XY> vertex_xy;
    if (cfg.use_cache_enabled) {
        if (cfg.use_cache_file.empty()) {
            return fail(error, "Cache yukleme aktif ama --use-cache <file> verilmedi.");
        }
        PlannerCacheData cache;
        if (!loadPlannerCache(cfg.use_cache_file, cache)) {
            return fail(error, "Planner cache yuklenemedi: " + cfg.use_cache_file);
        }
        if (!isPlannerCacheCompatible(cache, cfg, min_lat, max_lat, min_lon, max_lon, coastline_clearance_km * 1000.0)) {
            return fail(error, "Planner cache ayarlari ile mevcut config uyusmuyor.");
        }
        vertices = std::move(cache.vertices);
        triangles = std::move(cache.triangles);
        std::cout << "Planner cache yuklendi: " << cfg.use_cache_file << "\n";
    } else {
        vertices = seed_vertices;
        auto grid = generateOceanGrid(min_lat,
                                      max_lat,
                                      min_lon,
                                      max_lon,
                                      cfg.grid_step_deg,
                                      mesh_land,
                                      mesh_land_index,
                                      coastline_clearance_km,
                                      cfg.narrow_refine_step_deg,
                                      shallow_polygons,
                                      shallow_index);
        std::cout << "Adaptive grid vertex sayisi: " << grid.size() << "\n";
        vertices.insert(vertices.end(), grid.begin(), grid.end());

        vertex_xy.reserve(vertices.size());
        for (const auto& v : vertices) {
            vertex_xy.push_back(toXY(v.geo, proj));
        }

        std::cout << "Delaunay triangulation basladi...\n";
        auto all_triangles = buildDelaunay(vertex_xy);
        std::cout << "Delaunay tamamlandi. Ham ucgen: " << all_triangles.size() << "\n";
        std::cout << "Gezilebilir ucgen filtreleme basladi...\n";
        triangles = filterNavigableTriangles(all_triangles,
                                             vertices,
                                             mesh_land,
                                             mesh_land_index,
                                             coastline_clearance_km,
                                             min_lat,
                                             max_lat,
                                             min_lon,
                                             max_lon,
                                             0.0,
                                             shallow_polygons,
                                             shallow_index);
        std::cout << "Gezilebilir ucgen filtreleme tamamlandi.\n";

        if (!cfg.build_cache_file.empty()) {
            PlannerCacheData cache;
            cache.dataset_path = cfg.land_geojson;
            cache.mesh_dataset_path = meshDatasetPath(cfg);
            cache.bathymetry_path = cfg.bathymetry_geojson;
            cache.min_lat = min_lat;
            cache.max_lat = max_lat;
            cache.min_lon = min_lon;
            cache.max_lon = max_lon;
            cache.grid_step_deg = cfg.grid_step_deg;
            cache.narrow_refine_step_deg = cfg.narrow_refine_step_deg;
            cache.clearance_m = coastline_clearance_km * 1000.0;
            cache.min_depth_m = cfg.min_depth_m;
            cache.use_coastline_vertices_for_triangulation = false;
            cache.coastline_vertex_spacing_m = 0.0;
            cache.outer_frame_margin_deg = 0.0;
            cache.land_polygons = mesh_land;
            cache.vertices = vertices;
            cache.triangles = triangles;

            ensureParentDir(cfg.build_cache_file);
            if (!savePlannerCache(cfg.build_cache_file, cache)) {
                return fail(error, "Planner cache yazilamadi: " + cfg.build_cache_file);
            }
            std::cout << "Planner cache yazildi: " << cfg.build_cache_file << "\n";
        }
    }

    if (vertex_xy.empty()) {
        vertex_xy.reserve(vertices.size());
        for (const auto& v : vertices) {
            vertex_xy.push_back(toXY(v.geo, proj));
        }
    }

    const auto traffic_data = buildTriangleTrafficData(triangles, proj, tss, start_point.geo, goal_point.geo, cfg.traffic);
    auto tri_adj = buildTriangleAdjacency(triangles);
    const auto vertex_usage = buildVertexUsage(triangles, static_cast<int>(vertices.size()));
    std::vector<int> vertex_traffic_usage(vertices.size(), 0);
    if (cfg.traffic.hard_tss_corridor && traffic_data.active) {
        for (size_t tri_idx = 0; tri_idx < triangles.size(); ++tri_idx) {
            if (tri_idx >= traffic_data.triangle_in_hard_corridor.size() ||
                traffic_data.triangle_in_hard_corridor[tri_idx] == 0) {
                continue;
            }
            const auto& tri = triangles[tri_idx];
            vertex_traffic_usage[tri.a] += 1;
            vertex_traffic_usage[tri.b] += 1;
            vertex_traffic_usage[tri.c] += 1;
        }
    }

    std::cout << "Toplam vertex: " << vertices.size() << "\n";
    std::cout << "Toplam ucgen: " << triangles.size() << "\n";
    if (cfg.traffic.enable_tss_routing) {
        std::cout << "TSS routing: " << (traffic_data.active ? "aktif" : "pasif/veri yetersiz") << "\n";
        if (traffic_data.active) {
            const auto lane_triangles =
                std::count(traffic_data.triangle_in_lane.begin(), traffic_data.triangle_in_lane.end(), 1U);
            const auto zone_triangles =
                std::count(traffic_data.triangle_in_zone.begin(), traffic_data.triangle_in_zone.end(), 1U);
            std::cout << "TSS lane triangle: " << lane_triangles << "\n";
            std::cout << "TSS zone triangle: " << zone_triangles << "\n";
        }
    }

    const auto start_ranked = rankUsableVerticesByDistance(vertices, vertex_usage, start_point.geo);
    const auto goal_ranked = rankUsableVerticesByDistance(vertices, vertex_usage, goal_point.geo);
    if (start_ranked.empty() || goal_ranked.empty()) {
        return fail(error, "Mesh uzerinde kullanilabilir baslangic/varis vertex'i bulunamadi.");
    }

    const auto vertex_comp = buildVertexComponentIds(triangles, static_cast<int>(vertices.size()), vertex_usage);
    constexpr int kMaxStartCandidates = 300;
    constexpr int kMaxGoalCandidates = 300;
    constexpr int kMaxGoalsPerComponent = 24;
    struct CandidatePair {
        int sv = -1;
        int gv = -1;
        double cost = std::numeric_limits<double>::infinity();
    };

    std::unordered_map<int, std::vector<std::pair<double, int>>> goals_by_comp;
    goals_by_comp.reserve(64);
    for (int i = 0; i < static_cast<int>(goal_ranked.size()) && i < kMaxGoalCandidates; ++i) {
        const auto& [dist, gv] = goal_ranked[i];
        if (!edgeNavigable(vertices[gv].geo,
                           goal_point.geo,
                           land,
                           land_index,
                           coastline_clearance_km,
                           shallow_polygons,
                           shallow_index)) {
            continue;
        }
        if (cfg.traffic.hard_tss_corridor && traffic_data.active &&
            (gv < 0 || gv >= static_cast<int>(vertex_traffic_usage.size()) || vertex_traffic_usage[gv] <= 0)) {
            continue;
        }
        const int comp = vertex_comp[gv];
        if (comp < 0) {
            continue;
        }
        auto& bucket = goals_by_comp[comp];
        if (static_cast<int>(bucket.size()) < kMaxGoalsPerComponent) {
            bucket.push_back({dist, gv});
        }
    }

    std::vector<CandidatePair> candidate_pairs;
    candidate_pairs.reserve(static_cast<std::size_t>(kMaxStartCandidates * 16));
    for (int i = 0; i < static_cast<int>(start_ranked.size()) && i < kMaxStartCandidates; ++i) {
        const auto& [start_dist, sv] = start_ranked[i];
        if (!edgeNavigable(start_point.geo,
                           vertices[sv].geo,
                           land,
                           land_index,
                           coastline_clearance_km,
                           shallow_polygons,
                           shallow_index)) {
            continue;
        }
        if (cfg.traffic.hard_tss_corridor && traffic_data.active &&
            (sv < 0 || sv >= static_cast<int>(vertex_traffic_usage.size()) || vertex_traffic_usage[sv] <= 0)) {
            continue;
        }
        const int comp = vertex_comp[sv];
        if (comp < 0) {
            continue;
        }
        const auto it = goals_by_comp.find(comp);
        if (it == goals_by_comp.end()) {
            continue;
        }
        for (const auto& [goal_dist, gv] : it->second) {
            candidate_pairs.push_back({sv, gv, start_dist + goal_dist});
        }
    }
    std::sort(candidate_pairs.begin(), candidate_pairs.end(), [](const CandidatePair& a, const CandidatePair& b) {
        if (a.cost != b.cost) {
            return a.cost < b.cost;
        }
        if (a.sv != b.sv) {
            return a.sv < b.sv;
        }
        return a.gv < b.gv;
    });

    int mesh_start_vertex = -1;
    int goal_vertex = -1;
    TriangleSearchResult tri_search;
    for (const auto& pair : candidate_pairs) {
        auto attempt = searchTriangleSequence(triangles,
                                              tri_adj,
                                              pair.sv,
                                              pair.gv,
                                              vertex_xy,
                                              traffic_data.active ? &traffic_data : nullptr);
        if (!attempt.found) {
            continue;
        }
        mesh_start_vertex = pair.sv;
        goal_vertex = pair.gv;
        tri_search = std::move(attempt);
        break;
    }

    RouteResult route;
    if (mesh_start_vertex < 0 || goal_vertex < 0) {
        std::cout << "Start/goal icin ayni baglantili mesh bolgesinde uygun vertex cift bulunamadi.\n";
    } else {
        std::cout << "Mesh start vertex: " << mesh_start_vertex << " (" << vertices[mesh_start_vertex].name << ")\n";
        std::cout << "Mesh goal vertex: " << goal_vertex << " (" << vertices[goal_vertex].name << ")\n";

        if (!tri_search.found) {
            std::cout << "Ucgen dizisi bulunamadi.\n";
        } else {
            route.found = true;
            route.triangle_sequence = tri_search.triangle_sequence;
            if (tri_search.triangle_sequence.empty()) {
                route.found = false;
                std::cout << "Refinement basarisiz.\n";
            } else {
                std::vector<LatLon> raw_mesh_line;
                raw_mesh_line.reserve(tri_search.triangle_sequence.size() + 4);
                raw_mesh_line.push_back(start_point.geo);
                raw_mesh_line.push_back(vertices[mesh_start_vertex].geo);
                for (int tri_id : tri_search.triangle_sequence) {
                    raw_mesh_line.push_back(toGeo(triangles[tri_id].centroid, proj));
                }
                raw_mesh_line.push_back(vertices[goal_vertex].geo);
                raw_mesh_line.push_back(goal_point.geo);

                if (traffic_data.active) {
                    route.polyline = buildRouteAlongTssCenterline(start_point.geo,
                                                                  goal_point.geo,
                                                                  proj,
                                                                  tss,
                                                                  land,
                                                                  land_index,
                                                                  coastline_clearance_km,
                                                                  shallow_polygons,
                                                                  shallow_index);
                    if (!route.polyline.empty()) {
                        std::cout << "Rota TSS centerline boyunca olusturuldu.\n";
                    } else {
                        route.polyline = shortcutPathWithTraffic(raw_mesh_line,
                                                                 proj,
                                                                 land,
                                                                 land_index,
                                                                 coastline_clearance_km,
                                                                 shallow_polygons,
                                                                 shallow_index,
                                                                 traffic_data);
                    }
                } else {
                    route.polyline = shortcutPath(raw_mesh_line,
                                                  land,
                                                  land_index,
                                                  coastline_clearance_km,
                                                  shallow_polygons,
                                                  shallow_index);
                }
                if (route.polyline.size() < 2) {
                    route.found = false;
                    std::cout << "Refinement basarisiz: karaya girmeden gecerli kisaltilmis rota bulunamadi.\n";
                } else {
                    route.total_km = polylineLengthKm(route.polyline);
                }
            }
        }
    }

    out_plan.projection = proj;
    out_plan.min_lat = min_lat;
    out_plan.max_lat = max_lat;
    out_plan.min_lon = min_lon;
    out_plan.max_lon = max_lon;
    out_plan.coastline_clearance_km = coastline_clearance_km;
    out_plan.start_point = start_point;
    out_plan.goal_point = goal_point;
    out_plan.vertices = std::move(vertices);
    out_plan.triangles = std::move(triangles);
    out_plan.land = std::move(land);
    out_plan.shallow_polygons = std::move(shallow_polygons);
    out_plan.tss = std::move(tss);
    out_plan.bathymetry = std::move(bathymetry);
    out_plan.route = std::move(route);
    return true;
}

bool writeRouteCsv(const RoutePlan& plan, const std::string& file_path, std::string* error) {
    ensureParentDir(file_path);
    std::ofstream out(file_path);
    if (!out.is_open()) {
        return fail(error, "Rota CSV acilamadi: " + file_path);
    }

    out << "idx,lat_deg,lon_deg,leg_km,cumulative_km,x_km,y_km\n";
    double cumulative_km = 0.0;
    for (size_t i = 0; i < plan.route.polyline.size(); ++i) {
        const auto& point = plan.route.polyline[i];
        const XY xy = toXY(point, plan.projection);
        const double leg_km = (i == 0) ? 0.0 : haversineKm(plan.route.polyline[i - 1], point);
        cumulative_km += leg_km;
        out << i << ',' << point.lat_deg << ',' << point.lon_deg << ',' << leg_km << ',' << cumulative_km << ','
            << xy.x << ',' << xy.y << '\n';
    }
    return true;
}

void writeRouteSvg(const RoutePlan& plan, const std::string& file_path) {
    ensureParentDir(file_path);
    writeSvgPlot(file_path,
                 plan.vertices,
                 plan.triangles,
                 plan.route.triangle_sequence,
                 plan.route.polyline,
                 plan.land,
                 plan.tss,
                 plan.bathymetry);
}

}  // namespace planner
