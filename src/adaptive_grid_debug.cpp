#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <queue>
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <rapidjson/document.h>
#include <rapidjson/error/en.h>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEps = 1e-9;

struct LatLon {
    double lat_deg;
    double lon_deg;
};

struct LandPolygon {
    std::vector<LatLon> vertices;
    double min_lat;
    double max_lat;
    double min_lon;
    double max_lon;
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

struct CliConfig {
    std::string land_geojson = "dataset/ne_10m_land.geojson";
    double grid_step_deg = 0.15;
    double corridor_lat_pad = 1.8;
    double corridor_lon_pad = 1.8;
    double clearance_m = 100.0;
    LatLon start{41.0082, 28.9784};  // Istanbul
    LatLon goal{38.4237, 27.1428};   // Izmir
    std::string route_label = "Istanbul - Izmir";
    std::string svg_file = "output/istanbul_izmir_adaptive_grid.svg";
    std::string build_cache_file;
    std::string use_cache_file;
    bool use_cache_enabled = false;
};

struct GraphCacheData {
    std::string dataset_path;
    double min_lat = 0.0;
    double max_lat = 0.0;
    double min_lon = 0.0;
    double max_lon = 0.0;
    double grid_step_deg = 0.0;
    double clearance_m = 100.0;
    std::vector<GridCell> cells;
    std::vector<std::pair<int, int>> edges;
};

double degToRad(double deg) {
    return deg * kPi / 180.0;
}

double clampLatitude(double lat_deg) {
    return std::max(-89.999, std::min(89.999, lat_deg));
}

double kmToLatDeg(double km) {
    return km / 111.32;
}

double kmToLonDeg(double km, double lat_deg) {
    const double cos_lat = std::max(0.2, std::cos(degToRad(clampLatitude(lat_deg))));
    return km / (111.32 * cos_lat);
}

bool bboxesOverlap(double min_lat_a,
                   double max_lat_a,
                   double min_lon_a,
                   double max_lon_a,
                   double min_lat_b,
                   double max_lat_b,
                   double min_lon_b,
                   double max_lon_b) {
    if (max_lat_a < min_lat_b || max_lat_b < min_lat_a) {
        return false;
    }
    if (max_lon_a < min_lon_b || max_lon_b < min_lon_a) {
        return false;
    }
    return true;
}

std::int64_t spatialBinKey(int lat_bin, int lon_bin) {
    return (static_cast<std::int64_t>(lat_bin) << 32) ^ static_cast<std::uint32_t>(lon_bin);
}

LandPolygon makeLandPolygon(std::vector<LatLon> vertices) {
    LandPolygon p;
    p.vertices = std::move(vertices);
    p.min_lat = std::numeric_limits<double>::infinity();
    p.max_lat = -std::numeric_limits<double>::infinity();
    p.min_lon = std::numeric_limits<double>::infinity();
    p.max_lon = -std::numeric_limits<double>::infinity();
    for (const auto& v : p.vertices) {
        p.min_lat = std::min(p.min_lat, v.lat_deg);
        p.max_lat = std::max(p.max_lat, v.lat_deg);
        p.min_lon = std::min(p.min_lon, v.lon_deg);
        p.max_lon = std::max(p.max_lon, v.lon_deg);
    }
    return p;
}

bool parseOuterRing(const rapidjson::Value& ring_arr, LandPolygon& out_polygon) {
    if (!ring_arr.IsArray() || ring_arr.Size() < 4) {
        return false;
    }

    std::vector<LatLon> vertices;
    vertices.reserve(ring_arr.Size());
    for (rapidjson::SizeType i = 0; i < ring_arr.Size(); ++i) {
        const auto& coord = ring_arr[i];
        if (!coord.IsArray() || coord.Size() < 2 || !coord[0].IsNumber() || !coord[1].IsNumber()) {
            return false;
        }
        vertices.push_back({coord[1].GetDouble(), coord[0].GetDouble()});
    }

    if (vertices.size() >= 2) {
        const auto& first = vertices.front();
        const auto& last = vertices.back();
        if (std::fabs(first.lat_deg - last.lat_deg) < 1e-12 && std::fabs(first.lon_deg - last.lon_deg) < 1e-12) {
            vertices.pop_back();
        }
    }

    if (vertices.size() < 3) {
        return false;
    }

    out_polygon = makeLandPolygon(std::move(vertices));
    return true;
}

bool loadLandPolygonsGeoJson(const std::string& file_path, std::vector<LandPolygon>& polygons_out) {
    std::ifstream in(file_path);
    if (!in) {
        std::cerr << "GeoJSON acilamadi: " << file_path << "\n";
        return false;
    }

    const std::string data((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    if (data.empty()) {
        std::cerr << "GeoJSON bos: " << file_path << "\n";
        return false;
    }

    rapidjson::Document doc;
    doc.Parse(data.c_str());
    if (doc.HasParseError()) {
        std::cerr << "GeoJSON parse hatasi: " << rapidjson::GetParseError_En(doc.GetParseError()) << " (offset "
                  << doc.GetErrorOffset() << ")\n";
        return false;
    }
    if (!doc.IsObject() || !doc.HasMember("features") || !doc["features"].IsArray()) {
        std::cerr << "GeoJSON FeatureCollection bekleniyor.\n";
        return false;
    }

    polygons_out.clear();
    const auto& features = doc["features"];
    for (rapidjson::SizeType i = 0; i < features.Size(); ++i) {
        const auto& feature = features[i];
        if (!feature.IsObject() || !feature.HasMember("geometry") || !feature["geometry"].IsObject()) {
            continue;
        }
        const auto& geom = feature["geometry"];
        if (!geom.HasMember("type") || !geom["type"].IsString() || !geom.HasMember("coordinates")) {
            continue;
        }

        const std::string type = geom["type"].GetString();
        const auto& coords = geom["coordinates"];
        if (type == "Polygon") {
            if (!coords.IsArray() || coords.Empty() || !coords[0].IsArray()) {
                continue;
            }
            LandPolygon poly;
            if (parseOuterRing(coords[0], poly)) {
                polygons_out.push_back(std::move(poly));
            }
        } else if (type == "MultiPolygon") {
            if (!coords.IsArray()) {
                continue;
            }
            for (rapidjson::SizeType p = 0; p < coords.Size(); ++p) {
                const auto& poly_coords = coords[p];
                if (!poly_coords.IsArray() || poly_coords.Empty() || !poly_coords[0].IsArray()) {
                    continue;
                }
                LandPolygon poly;
                if (parseOuterRing(poly_coords[0], poly)) {
                    polygons_out.push_back(std::move(poly));
                }
            }
        }
    }

    std::cout << "Yuklenen kara poligonu: " << polygons_out.size() << "\n";
    return !polygons_out.empty();
}

LandSpatialIndex buildLandSpatialIndex(const std::vector<LandPolygon>& polygons, double cell_deg) {
    LandSpatialIndex index;
    index.cell_deg = std::max(0.25, cell_deg);
    for (int i = 0; i < static_cast<int>(polygons.size()); ++i) {
        const auto& poly = polygons[i];
        const int lat_min_bin = static_cast<int>(std::floor(poly.min_lat / index.cell_deg));
        const int lat_max_bin = static_cast<int>(std::floor(poly.max_lat / index.cell_deg));
        const int lon_min_bin = static_cast<int>(std::floor(poly.min_lon / index.cell_deg));
        const int lon_max_bin = static_cast<int>(std::floor(poly.max_lon / index.cell_deg));
        for (int lat_bin = lat_min_bin; lat_bin <= lat_max_bin; ++lat_bin) {
            for (int lon_bin = lon_min_bin; lon_bin <= lon_max_bin; ++lon_bin) {
                index.bins[spatialBinKey(lat_bin, lon_bin)].push_back(i);
            }
        }
    }
    return index;
}

std::vector<int> candidatePolygonsForBBox(const LandSpatialIndex& index,
                                          double min_lat,
                                          double max_lat,
                                          double min_lon,
                                          double max_lon) {
    const int lat_min_bin = static_cast<int>(std::floor(min_lat / index.cell_deg));
    const int lat_max_bin = static_cast<int>(std::floor(max_lat / index.cell_deg));
    const int lon_min_bin = static_cast<int>(std::floor(min_lon / index.cell_deg));
    const int lon_max_bin = static_cast<int>(std::floor(max_lon / index.cell_deg));

    std::unordered_set<int> seen;
    std::vector<int> out;
    out.reserve(64);
    for (int lat_bin = lat_min_bin; lat_bin <= lat_max_bin; ++lat_bin) {
        for (int lon_bin = lon_min_bin; lon_bin <= lon_max_bin; ++lon_bin) {
            const auto it = index.bins.find(spatialBinKey(lat_bin, lon_bin));
            if (it == index.bins.end()) {
                continue;
            }
            for (int poly_id : it->second) {
                if (seen.insert(poly_id).second) {
                    out.push_back(poly_id);
                }
            }
        }
    }
    return out;
}

bool pointInLandPolygon(const LatLon& p, const LandPolygon& poly) {
    if (p.lat_deg < poly.min_lat || p.lat_deg > poly.max_lat || p.lon_deg < poly.min_lon || p.lon_deg > poly.max_lon) {
        return false;
    }

    bool inside = false;
    for (size_t i = 0, j = poly.vertices.size() - 1; i < poly.vertices.size(); j = i++) {
        const auto& a = poly.vertices[j];
        const auto& b = poly.vertices[i];
        const bool intersects = ((a.lat_deg > p.lat_deg) != (b.lat_deg > p.lat_deg)) &&
                                (p.lon_deg < (b.lon_deg - a.lon_deg) * (p.lat_deg - a.lat_deg) /
                                                     ((b.lat_deg - a.lat_deg) + kEps) +
                                                 a.lon_deg);
        if (intersects) {
            inside = !inside;
        }
    }
    return inside;
}

double pointToSegmentDistanceKm(double px, double py, double ax, double ay, double bx, double by) {
    const double abx = bx - ax;
    const double aby = by - ay;
    const double ab2 = abx * abx + aby * aby;
    if (ab2 < 1e-16) {
        const double dx = px - ax;
        const double dy = py - ay;
        return std::sqrt(dx * dx + dy * dy);
    }
    const double apx = px - ax;
    const double apy = py - ay;
    const double t = std::max(0.0, std::min(1.0, (apx * abx + apy * aby) / ab2));
    const double qx = ax + t * abx;
    const double qy = ay + t * aby;
    const double dx = px - qx;
    const double dy = py - qy;
    return std::sqrt(dx * dx + dy * dy);
}

double pointToLandBoundaryKm(const LatLon& p, const LandPolygon& poly) {
    const double lon_scale = 111.32 * std::max(0.2, std::cos(degToRad(p.lat_deg)));
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < poly.vertices.size(); ++i) {
        const auto& a = poly.vertices[i];
        const auto& b = poly.vertices[(i + 1) % poly.vertices.size()];
        const double ax = (a.lon_deg - p.lon_deg) * lon_scale;
        const double ay = (a.lat_deg - p.lat_deg) * 111.32;
        const double bx = (b.lon_deg - p.lon_deg) * lon_scale;
        const double by = (b.lat_deg - p.lat_deg) * 111.32;
        const double d = pointToSegmentDistanceKm(0.0, 0.0, ax, ay, bx, by);
        best = std::min(best, d);
    }
    return best;
}

bool pointInBufferedLandPolygon(const LatLon& p, const LandPolygon& poly, double clearance_km) {
    if (clearance_km > 0.0) {
        const double lat_margin = kmToLatDeg(clearance_km);
        const double lon_margin = kmToLonDeg(clearance_km, p.lat_deg);
        if (p.lat_deg < poly.min_lat - lat_margin || p.lat_deg > poly.max_lat + lat_margin ||
            p.lon_deg < poly.min_lon - lon_margin || p.lon_deg > poly.max_lon + lon_margin) {
            return false;
        }
    }

    if (pointInLandPolygon(p, poly)) {
        return true;
    }
    if (clearance_km <= 0.0) {
        return false;
    }
    return pointToLandBoundaryKm(p, poly) <= clearance_km;
}

bool isOnLand(const LatLon& p,
              const std::vector<LandPolygon>& land_polygons,
              const LandSpatialIndex& land_index,
              double clearance_km) {
    const double lat_margin = kmToLatDeg(clearance_km);
    const double lon_margin = kmToLonDeg(clearance_km, p.lat_deg);
    const auto candidate_ids = candidatePolygonsForBBox(
        land_index, p.lat_deg - lat_margin, p.lat_deg + lat_margin, p.lon_deg - lon_margin, p.lon_deg + lon_margin);
    for (int poly_id : candidate_ids) {
        if (pointInBufferedLandPolygon(p, land_polygons[poly_id], clearance_km)) {
            return true;
        }
    }
    return false;
}

LatLon nearestSeaPoint(const LatLon& p,
                       const std::vector<LandPolygon>& land_polygons,
                       const LandSpatialIndex& land_index,
                       double clearance_km) {
    if (!isOnLand(p, land_polygons, land_index, clearance_km)) {
        return p;
    }
    for (double radius_deg = 0.02; radius_deg <= 2.5; radius_deg += 0.02) {
        for (int bearing = 0; bearing < 360; bearing += 5) {
            const double br = degToRad(static_cast<double>(bearing));
            const double lat = clampLatitude(p.lat_deg + radius_deg * std::sin(br));
            const double cos_lat = std::max(0.2, std::cos(degToRad(lat)));
            const double lon = p.lon_deg + radius_deg * std::cos(br) / cos_lat;
            const LatLon c{lat, lon};
            if (!isOnLand(c, land_polygons, land_index, clearance_km)) {
                return c;
            }
        }
    }
    return p;
}

std::vector<LandPolygon> filterPolygonsByBounds(const std::vector<LandPolygon>& polygons,
                                                double min_lat,
                                                double max_lat,
                                                double min_lon,
                                                double max_lon) {
    std::vector<LandPolygon> filtered;
    filtered.reserve(polygons.size());
    for (const auto& poly : polygons) {
        if (bboxesOverlap(min_lat, max_lat, min_lon, max_lon, poly.min_lat, poly.max_lat, poly.min_lon, poly.max_lon)) {
            filtered.push_back(poly);
        }
    }
    return filtered;
}

std::vector<GridCell> generateAdaptiveGrid(double min_lat,
                                           double max_lat,
                                           double min_lon,
                                           double max_lon,
                                           double base_step_deg,
                                           const std::vector<LandPolygon>& land_polygons,
                                           const LandSpatialIndex& land_index,
                                           double clearance_km) {
    struct Cell {
        double min_lat;
        double max_lat;
        double min_lon;
        double max_lon;
        int depth;
    };

    std::vector<GridCell> cells;
    if (base_step_deg <= 0.0) {
        return cells;
    }

    std::unordered_map<std::uint64_t, bool> land_cache;
    land_cache.reserve(1 << 18);
    const auto quantizeDeg = [](double v) -> std::int32_t {
        constexpr double scale = 1e5;  // ~1.11 m per unit in latitude.
        return static_cast<std::int32_t>(std::llround(v * scale));
    };
    auto isOnLandCached = [&](const LatLon& p) {
        const std::uint32_t qlat = static_cast<std::uint32_t>(quantizeDeg(p.lat_deg));
        const std::uint32_t qlon = static_cast<std::uint32_t>(quantizeDeg(p.lon_deg));
        const std::uint64_t key = (static_cast<std::uint64_t>(qlat) << 32) | qlon;
        const auto it = land_cache.find(key);
        if (it != land_cache.end()) {
            return it->second;
        }
        const bool on_land = isOnLand(p, land_polygons, land_index, clearance_km);
        land_cache.emplace(key, on_land);
        return on_land;
    };

    const double min_cell_deg = base_step_deg;
    const int split_threshold = 2;  // PMR-like: split when obstacle edge count exceeds threshold.
    const int local_refine_extra_depth = 2;
    const double root_span = std::max(max_lat - min_lat, max_lon - min_lon);
    int max_depth = 0;
    for (double span = root_span; span > min_cell_deg && max_depth < 20; span *= 0.5) {
        max_depth += 1;
    }
    max_depth += 1;

    std::vector<Cell> stack;
    stack.reserve(4096);
    stack.push_back({min_lat, max_lat, min_lon, max_lon, 0});

    while (!stack.empty()) {
        const Cell cell = stack.back();
        stack.pop_back();

        const double lat_span = cell.max_lat - cell.min_lat;
        const double lon_span = cell.max_lon - cell.min_lon;
        if (lat_span <= 1e-12 || lon_span <= 1e-12) {
            continue;
        }
        const LatLon center{0.5 * (cell.min_lat + cell.max_lat), 0.5 * (cell.min_lon + cell.max_lon)};
        const bool center_on_land = isOnLandCached(center);

        const auto candidate_ids =
            candidatePolygonsForBBox(land_index, cell.min_lat, cell.max_lat, cell.min_lon, cell.max_lon);

        int edge_count = 0;
        for (int poly_id : candidate_ids) {
            const auto& poly = land_polygons[poly_id];
            if (!bboxesOverlap(cell.min_lat,
                               cell.max_lat,
                               cell.min_lon,
                               cell.max_lon,
                               poly.min_lat,
                               poly.max_lat,
                               poly.min_lon,
                               poly.max_lon)) {
                continue;
            }
            for (size_t i = 0; i < poly.vertices.size(); ++i) {
                const auto& a = poly.vertices[i];
                const auto& b = poly.vertices[(i + 1) % poly.vertices.size()];
                const double e_min_lat = std::min(a.lat_deg, b.lat_deg);
                const double e_max_lat = std::max(a.lat_deg, b.lat_deg);
                const double e_min_lon = std::min(a.lon_deg, b.lon_deg);
                const double e_max_lon = std::max(a.lon_deg, b.lon_deg);
                if (!bboxesOverlap(cell.min_lat, cell.max_lat, cell.min_lon, cell.max_lon, e_min_lat, e_max_lat, e_min_lon, e_max_lon)) {
                    continue;
                }
                edge_count += 1;
                if (edge_count > split_threshold) {
                    break;
                }
            }
            if (edge_count > split_threshold) {
                break;
            }
        }

        bool should_local_refine = false;
        if (!center_on_land && edge_count > 0 && cell.depth < (max_depth + local_refine_extra_depth)) {
            const double lat_km = lat_span * 111.32;
            const double lon_km = lon_span * 111.32 * std::max(0.2, std::cos(degToRad(center.lat_deg)));
            const double span_km = std::max(lat_km, lon_km);
            const double max_local_refine_span_km = std::max(15.0, min_cell_deg * 111.32 * 3.0);
            if (span_km <= max_local_refine_span_km) {
                const double probe_km = std::max(0.1, std::min(1.0, 0.35 * span_km));
                auto sampleLand = [&](double north_km, double east_km) {
                    const double lat = clampLatitude(center.lat_deg + kmToLatDeg(north_km));
                    const double lon = center.lon_deg + kmToLonDeg(east_km, center.lat_deg);
                    return isOnLandCached({lat, lon});
                };

                const bool north = sampleLand(+probe_km, 0.0);
                const bool south = sampleLand(-probe_km, 0.0);
                const bool east = sampleLand(0.0, +probe_km);
                const bool west = sampleLand(0.0, -probe_km);
                const bool ne = sampleLand(+probe_km, +probe_km);
                const bool nw = sampleLand(+probe_km, -probe_km);
                const bool se = sampleLand(-probe_km, +probe_km);
                const bool sw = sampleLand(-probe_km, -probe_km);
                const bool opposing_land = (north && south) || (east && west) || (ne && sw) || (nw && se);
                should_local_refine = opposing_land;
            }
        }

        const bool should_split = ((edge_count > split_threshold) && (cell.depth < max_depth)) || should_local_refine;
        if (!should_split) {
            if (!center_on_land) {
                const bool fine = std::max(lat_span, lon_span) <= (min_cell_deg * 1.5);
                cells.push_back({cell.min_lat, cell.max_lat, cell.min_lon, cell.max_lon, fine});
            }
            continue;
        }

        const double mid_lat = 0.5 * (cell.min_lat + cell.max_lat);
        const double mid_lon = 0.5 * (cell.min_lon + cell.max_lon);
        stack.push_back({cell.min_lat, mid_lat, cell.min_lon, mid_lon, cell.depth + 1});
        stack.push_back({cell.min_lat, mid_lat, mid_lon, cell.max_lon, cell.depth + 1});
        stack.push_back({mid_lat, cell.max_lat, cell.min_lon, mid_lon, cell.depth + 1});
        stack.push_back({mid_lat, cell.max_lat, mid_lon, cell.max_lon, cell.depth + 1});
    }

    return cells;
}

LatLon gridCellCenter(const GridCell& c) {
    return {0.5 * (c.min_lat + c.max_lat), 0.5 * (c.min_lon + c.max_lon)};
}

double haversineKm(const LatLon& a, const LatLon& b) {
    const double lat1 = degToRad(a.lat_deg);
    const double lon1 = degToRad(a.lon_deg);
    const double lat2 = degToRad(b.lat_deg);
    const double lon2 = degToRad(b.lon_deg);
    const double dlat = lat2 - lat1;
    const double dlon = lon2 - lon1;
    const double h = std::pow(std::sin(dlat / 2.0), 2.0) +
                     std::cos(lat1) * std::cos(lat2) * std::pow(std::sin(dlon / 2.0), 2.0);
    const double c = 2.0 * std::atan2(std::sqrt(h), std::sqrt(std::max(0.0, 1.0 - h)));
    return 6371.0088 * c;
}

std::pair<std::vector<std::vector<std::pair<int, double>>>, std::vector<std::pair<int, int>>> buildQuadtreeGraph(
    const std::vector<GridCell>& cells) {
    constexpr double tol = 1e-9;
    constexpr double boundary_tol = 1e-6;
    constexpr double key_eps = 1e-6;
    const auto overlapLen = [](double a0, double a1, double b0, double b1) {
        return std::min(a1, b1) - std::max(a0, b0);
    };
    const auto boundaryKey = [&](double v) -> std::int64_t {
        return static_cast<std::int64_t>(std::llround(v / key_eps));
    };

    std::vector<std::vector<std::pair<int, double>>> adj(cells.size());
    std::vector<std::pair<int, int>> edges;
    edges.reserve(cells.size() * 4);
    std::unordered_set<std::uint64_t> edge_seen;
    edge_seen.reserve(cells.size() * 8);

    std::unordered_map<std::int64_t, std::vector<int>> by_min_lon;
    std::unordered_map<std::int64_t, std::vector<int>> by_min_lat;
    by_min_lon.reserve(cells.size() * 2);
    by_min_lat.reserve(cells.size() * 2);

    std::vector<LatLon> centers(cells.size());
    for (int i = 0; i < static_cast<int>(cells.size()); ++i) {
        by_min_lon[boundaryKey(cells[i].min_lon)].push_back(i);
        by_min_lat[boundaryKey(cells[i].min_lat)].push_back(i);
        centers[i] = gridCellCenter(cells[i]);
    }

    auto addEdge = [&](int a, int b) {
        if (a == b) {
            return;
        }
        const int u = std::min(a, b);
        const int v = std::max(a, b);
        const std::uint64_t key = (static_cast<std::uint64_t>(static_cast<std::uint32_t>(u)) << 32) |
                                  static_cast<std::uint32_t>(v);
        if (!edge_seen.insert(key).second) {
            return;
        }
        const double w = haversineKm(centers[u], centers[v]);
        adj[u].push_back({v, w});
        adj[v].push_back({u, w});
        edges.push_back({u, v});
    };

    for (int i = 0; i < static_cast<int>(cells.size()); ++i) {
        const auto east_it = by_min_lon.find(boundaryKey(cells[i].max_lon));
        if (east_it != by_min_lon.end()) {
            for (int j : east_it->second) {
                if (std::fabs(cells[i].max_lon - cells[j].min_lon) > boundary_tol) {
                    continue;
                }
                if (overlapLen(cells[i].min_lat, cells[i].max_lat, cells[j].min_lat, cells[j].max_lat) <= tol) {
                    continue;
                }
                addEdge(i, j);
            }
        }

        const auto north_it = by_min_lat.find(boundaryKey(cells[i].max_lat));
        if (north_it == by_min_lat.end()) {
            continue;
        }
        for (int j : north_it->second) {
            if (std::fabs(cells[i].max_lat - cells[j].min_lat) > boundary_tol) {
                continue;
            }
            if (overlapLen(cells[i].min_lon, cells[i].max_lon, cells[j].min_lon, cells[j].max_lon) <= tol) {
                continue;
            }
            addEdge(i, j);
        }
    }
    return {adj, edges};
}

std::vector<std::vector<std::pair<int, double>>> buildAdjacencyFromEdges(
    const std::vector<GridCell>& cells,
    const std::vector<std::pair<int, int>>& edges) {
    std::vector<std::vector<std::pair<int, double>>> adj(cells.size());
    std::vector<LatLon> centers(cells.size());
    for (int i = 0; i < static_cast<int>(cells.size()); ++i) {
        centers[i] = gridCellCenter(cells[i]);
    }
    for (const auto& [u, v] : edges) {
        if (u < 0 || v < 0 || u >= static_cast<int>(cells.size()) || v >= static_cast<int>(cells.size()) || u == v) {
            continue;
        }
        const double w = haversineKm(centers[u], centers[v]);
        adj[u].push_back({v, w});
        adj[v].push_back({u, w});
    }
    return adj;
}

bool saveGraphCache(const std::string& file_path, const GraphCacheData& cache) {
    std::ofstream out(file_path, std::ios::binary);
    if (!out) {
        std::cerr << "Cache yazilamadi: " << file_path << "\n";
        return false;
    }

    const char magic[8] = {'V', 'G', 'P', 'C', 'A', 'C', 'H', 'E'};
    const std::uint32_t version = 1;
    const std::uint32_t dataset_len = static_cast<std::uint32_t>(cache.dataset_path.size());
    const std::uint64_t cell_count = static_cast<std::uint64_t>(cache.cells.size());
    const std::uint64_t edge_count = static_cast<std::uint64_t>(cache.edges.size());

    out.write(magic, sizeof(magic));
    out.write(reinterpret_cast<const char*>(&version), sizeof(version));
    out.write(reinterpret_cast<const char*>(&dataset_len), sizeof(dataset_len));
    out.write(cache.dataset_path.data(), dataset_len);
    out.write(reinterpret_cast<const char*>(&cache.min_lat), sizeof(cache.min_lat));
    out.write(reinterpret_cast<const char*>(&cache.max_lat), sizeof(cache.max_lat));
    out.write(reinterpret_cast<const char*>(&cache.min_lon), sizeof(cache.min_lon));
    out.write(reinterpret_cast<const char*>(&cache.max_lon), sizeof(cache.max_lon));
    out.write(reinterpret_cast<const char*>(&cache.grid_step_deg), sizeof(cache.grid_step_deg));
    out.write(reinterpret_cast<const char*>(&cache.clearance_m), sizeof(cache.clearance_m));
    out.write(reinterpret_cast<const char*>(&cell_count), sizeof(cell_count));
    out.write(reinterpret_cast<const char*>(&edge_count), sizeof(edge_count));

    for (const auto& c : cache.cells) {
        const std::uint8_t fine = c.fine ? 1 : 0;
        out.write(reinterpret_cast<const char*>(&c.min_lat), sizeof(c.min_lat));
        out.write(reinterpret_cast<const char*>(&c.max_lat), sizeof(c.max_lat));
        out.write(reinterpret_cast<const char*>(&c.min_lon), sizeof(c.min_lon));
        out.write(reinterpret_cast<const char*>(&c.max_lon), sizeof(c.max_lon));
        out.write(reinterpret_cast<const char*>(&fine), sizeof(fine));
    }

    for (const auto& [u, v] : cache.edges) {
        const std::int32_t uu = static_cast<std::int32_t>(u);
        const std::int32_t vv = static_cast<std::int32_t>(v);
        out.write(reinterpret_cast<const char*>(&uu), sizeof(uu));
        out.write(reinterpret_cast<const char*>(&vv), sizeof(vv));
    }

    if (!out.good()) {
        std::cerr << "Cache yazimi yarim kaldi: " << file_path << "\n";
        return false;
    }
    return true;
}

bool loadGraphCache(const std::string& file_path, GraphCacheData& out_cache) {
    std::ifstream in(file_path, std::ios::binary);
    if (!in) {
        std::cerr << "Cache acilamadi: " << file_path << "\n";
        return false;
    }

    char magic[8] = {};
    std::uint32_t version = 0;
    std::uint32_t dataset_len = 0;
    std::uint64_t cell_count = 0;
    std::uint64_t edge_count = 0;

    in.read(magic, sizeof(magic));
    in.read(reinterpret_cast<char*>(&version), sizeof(version));
    in.read(reinterpret_cast<char*>(&dataset_len), sizeof(dataset_len));
    if (!in.good()) {
        std::cerr << "Cache baslik okunamadi: " << file_path << "\n";
        return false;
    }

    const char expected_magic[8] = {'V', 'G', 'P', 'C', 'A', 'C', 'H', 'E'};
    if (std::memcmp(magic, expected_magic, sizeof(magic)) != 0 || version != 1) {
        std::cerr << "Cache formati uyumsuz: " << file_path << "\n";
        return false;
    }

    out_cache = GraphCacheData{};
    out_cache.dataset_path.resize(dataset_len);
    in.read(&out_cache.dataset_path[0], dataset_len);
    in.read(reinterpret_cast<char*>(&out_cache.min_lat), sizeof(out_cache.min_lat));
    in.read(reinterpret_cast<char*>(&out_cache.max_lat), sizeof(out_cache.max_lat));
    in.read(reinterpret_cast<char*>(&out_cache.min_lon), sizeof(out_cache.min_lon));
    in.read(reinterpret_cast<char*>(&out_cache.max_lon), sizeof(out_cache.max_lon));
    in.read(reinterpret_cast<char*>(&out_cache.grid_step_deg), sizeof(out_cache.grid_step_deg));
    in.read(reinterpret_cast<char*>(&out_cache.clearance_m), sizeof(out_cache.clearance_m));
    in.read(reinterpret_cast<char*>(&cell_count), sizeof(cell_count));
    in.read(reinterpret_cast<char*>(&edge_count), sizeof(edge_count));
    if (!in.good()) {
        std::cerr << "Cache metadata okunamadi: " << file_path << "\n";
        return false;
    }

    out_cache.cells.resize(static_cast<std::size_t>(cell_count));
    for (std::size_t i = 0; i < out_cache.cells.size(); ++i) {
        std::uint8_t fine = 0;
        in.read(reinterpret_cast<char*>(&out_cache.cells[i].min_lat), sizeof(out_cache.cells[i].min_lat));
        in.read(reinterpret_cast<char*>(&out_cache.cells[i].max_lat), sizeof(out_cache.cells[i].max_lat));
        in.read(reinterpret_cast<char*>(&out_cache.cells[i].min_lon), sizeof(out_cache.cells[i].min_lon));
        in.read(reinterpret_cast<char*>(&out_cache.cells[i].max_lon), sizeof(out_cache.cells[i].max_lon));
        in.read(reinterpret_cast<char*>(&fine), sizeof(fine));
        out_cache.cells[i].fine = (fine != 0);
    }

    out_cache.edges.resize(static_cast<std::size_t>(edge_count));
    for (std::size_t i = 0; i < out_cache.edges.size(); ++i) {
        std::int32_t u = -1;
        std::int32_t v = -1;
        in.read(reinterpret_cast<char*>(&u), sizeof(u));
        in.read(reinterpret_cast<char*>(&v), sizeof(v));
        out_cache.edges[i] = {static_cast<int>(u), static_cast<int>(v)};
    }

    if (!in.good()) {
        std::cerr << "Cache govdesi okunamadi: " << file_path << "\n";
        return false;
    }
    return true;
}

bool isCacheCompatible(const GraphCacheData& cache,
                       const CliConfig& cfg,
                       const LatLon& start_sea,
                       const LatLon& goal_sea) {
    auto almostEqual = [](double a, double b) { return std::fabs(a - b) <= 1e-9; };
    if (cache.dataset_path != cfg.land_geojson) {
        std::cerr << "Cache dataset uyusmuyor.\n";
        return false;
    }
    if (!almostEqual(cache.grid_step_deg, cfg.grid_step_deg)) {
        std::cerr << "Cache grid-step uyusmuyor.\n";
        return false;
    }
    if (!almostEqual(cache.clearance_m, cfg.clearance_m)) {
        std::cerr << "Cache clearance uyusmuyor.\n";
        return false;
    }
    auto inside = [&](const LatLon& p) {
        return p.lat_deg >= cache.min_lat && p.lat_deg <= cache.max_lat && p.lon_deg >= cache.min_lon &&
               p.lon_deg <= cache.max_lon;
    };
    if (!inside(start_sea) || !inside(goal_sea)) {
        std::cerr << "Start/goal cache kapsami disinda.\n";
        return false;
    }
    return true;
}

int nearestCellIndex(const std::vector<GridCell>& cells, const LatLon& p) {
    int best = -1;
    double best_d = std::numeric_limits<double>::infinity();
    for (int i = 0; i < static_cast<int>(cells.size()); ++i) {
        const LatLon c = gridCellCenter(cells[i]);
        const double d = haversineKm(c, p);
        if (d < best_d) {
            best_d = d;
            best = i;
        }
    }
    return best;
}

std::vector<int> astarPath(const std::vector<std::vector<std::pair<int, double>>>& adj,
                           const std::vector<GridCell>& cells,
                           int start,
                           int goal) {
    if (start < 0 || goal < 0 || start >= static_cast<int>(adj.size()) || goal >= static_cast<int>(adj.size()) ||
        cells.size() != adj.size()) {
        return {};
    }

    struct State {
        int node;
        double f_score;
        bool operator>(const State& other) const { return f_score > other.f_score; }
    };

    const LatLon goal_center = gridCellCenter(cells[goal]);
    auto heuristic = [&](int node) {
        return haversineKm(gridCellCenter(cells[node]), goal_center);
    };

    std::vector<double> g_cost(adj.size(), std::numeric_limits<double>::infinity());
    std::vector<int> parent(adj.size(), -1);
    std::priority_queue<State, std::vector<State>, std::greater<State>> pq;
    g_cost[start] = 0.0;
    pq.push({start, heuristic(start)});

    while (!pq.empty()) {
        const State cur = pq.top();
        pq.pop();
        const double expected_f = g_cost[cur.node] + heuristic(cur.node);
        if (cur.f_score > expected_f + 1e-9) {
            continue;
        }
        if (cur.node == goal) {
            break;
        }
        for (const auto& [to, w] : adj[cur.node]) {
            const double tentative_g = g_cost[cur.node] + w;
            if (tentative_g + 1e-9 < g_cost[to]) {
                g_cost[to] = tentative_g;
                parent[to] = cur.node;
                pq.push({to, tentative_g + heuristic(to)});
            }
        }
    }

    if (!std::isfinite(g_cost[goal])) {
        return {};
    }
    std::vector<int> path;
    for (int at = goal; at != -1; at = parent[at]) {
        path.push_back(at);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

bool isOnLandCachedPoint(const LatLon& p,
                         const std::vector<LandPolygon>& land_polygons,
                         const LandSpatialIndex& land_index,
                         double clearance_km,
                         std::unordered_map<std::uint64_t, bool>& cache) {
    const auto quantizeDeg = [](double v) -> std::int32_t {
        constexpr double scale = 1e5;  // ~1.11 m in latitude.
        return static_cast<std::int32_t>(std::llround(v * scale));
    };
    const std::uint32_t qlat = static_cast<std::uint32_t>(quantizeDeg(p.lat_deg));
    const std::uint32_t qlon = static_cast<std::uint32_t>(quantizeDeg(p.lon_deg));
    const std::uint64_t key = (static_cast<std::uint64_t>(qlat) << 32) | qlon;
    const auto it = cache.find(key);
    if (it != cache.end()) {
        return it->second;
    }
    const bool on_land = isOnLand(p, land_polygons, land_index, clearance_km);
    cache.emplace(key, on_land);
    return on_land;
}

bool hasLineOfSight(const LatLon& a,
                    const LatLon& b,
                    const std::vector<LandPolygon>& land_polygons,
                    const LandSpatialIndex& land_index,
                    double clearance_km,
                    std::unordered_map<std::uint64_t, bool>& cache) {
    const double distance_km = haversineKm(a, b);
    const int sample_count = std::max(1, static_cast<int>(std::ceil(distance_km / 0.2)));
    for (int i = 0; i <= sample_count; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(sample_count);
        const LatLon p{a.lat_deg + (b.lat_deg - a.lat_deg) * t, a.lon_deg + (b.lon_deg - a.lon_deg) * t};
        if (isOnLandCachedPoint(p, land_polygons, land_index, clearance_km, cache)) {
            return false;
        }
    }
    return true;
}

std::vector<int> refinePathWithLOS(const std::vector<int>& path,
                                   const std::vector<GridCell>& cells,
                                   const std::vector<LandPolygon>& land_polygons,
                                   const LandSpatialIndex& land_index,
                                   double clearance_km) {
    if (path.size() <= 2) {
        return path;
    }

    std::vector<LatLon> centers(cells.size());
    for (int i = 0; i < static_cast<int>(cells.size()); ++i) {
        centers[i] = gridCellCenter(cells[i]);
    }

    std::unordered_map<std::uint64_t, bool> cache;
    cache.reserve(1 << 14);

    std::vector<int> refined;
    refined.reserve(path.size());
    refined.push_back(path.front());

    std::size_t anchor = 0;
    while (anchor + 1 < path.size()) {
        std::size_t best = anchor + 1;
        for (std::size_t j = path.size() - 1; j > anchor; --j) {
            if (hasLineOfSight(centers[path[anchor]], centers[path[j]], land_polygons, land_index, clearance_km, cache)) {
                best = j;
                break;
            }
        }
        refined.push_back(path[best]);
        anchor = best;
    }

    return refined;
}

std::pair<std::vector<int>, std::vector<int>> connectedComponents(
    const std::vector<std::vector<std::pair<int, double>>>& adj) {
    std::vector<int> comp_id(adj.size(), -1);
    std::vector<int> comp_sizes;
    std::queue<int> q;
    int next_id = 0;

    for (int i = 0; i < static_cast<int>(adj.size()); ++i) {
        if (comp_id[i] != -1) {
            continue;
        }
        int size = 0;
        comp_id[i] = next_id;
        q.push(i);
        while (!q.empty()) {
            const int u = q.front();
            q.pop();
            size += 1;
            for (const auto& [v, _] : adj[u]) {
                if (comp_id[v] != -1) {
                    continue;
                }
                comp_id[v] = next_id;
                q.push(v);
            }
        }
        comp_sizes.push_back(size);
        next_id += 1;
    }
    return {comp_id, comp_sizes};
}

bool centerInsideBox(const GridCell& c,
                     double min_lat,
                     double max_lat,
                     double min_lon,
                     double max_lon) {
    const LatLon m = gridCellCenter(c);
    return (m.lat_deg >= min_lat && m.lat_deg <= max_lat && m.lon_deg >= min_lon && m.lon_deg <= max_lon);
}

void writeGridSvg(const std::string& file_path,
                  const std::vector<GridCell>& grid_cells,
                  const std::vector<LandPolygon>& land_polygons,
                  const LatLon& start,
                  const LatLon& goal,
                  const std::string& route_label,
                  const std::vector<std::pair<int, int>>& graph_edges,
                  const std::vector<int>& graph_path,
                  int coarse_count,
                  int fine_count) {
    constexpr double width = 1400.0;
    constexpr double height = 900.0;
    constexpr double margin = 50.0;

    double min_lon = std::numeric_limits<double>::infinity();
    double max_lon = -std::numeric_limits<double>::infinity();
    double min_lat = std::numeric_limits<double>::infinity();
    double max_lat = -std::numeric_limits<double>::infinity();

    auto extend = [&](double lat, double lon) {
        min_lat = std::min(min_lat, lat);
        max_lat = std::max(max_lat, lat);
        min_lon = std::min(min_lon, lon);
        max_lon = std::max(max_lon, lon);
    };

    for (const auto& c : grid_cells) {
        extend(c.min_lat, c.min_lon);
        extend(c.max_lat, c.max_lon);
    }
    extend(start.lat_deg, start.lon_deg);
    extend(goal.lat_deg, goal.lon_deg);

    const double lon_span = std::max(max_lon - min_lon, 1e-6);
    const double lat_span = std::max(max_lat - min_lat, 1e-6);
    min_lon -= lon_span * 0.08;
    max_lon += lon_span * 0.08;
    min_lat -= lat_span * 0.08;
    max_lat += lat_span * 0.08;

    const double drawable_w = width - 2.0 * margin;
    const double drawable_h = height - 2.0 * margin;
    const double sx = drawable_w / (max_lon - min_lon);
    const double sy = drawable_h / (max_lat - min_lat);
    const double scale = std::min(sx, sy);
    const double plot_w = (max_lon - min_lon) * scale;
    const double plot_h = (max_lat - min_lat) * scale;
    const double ox = margin + 0.5 * (drawable_w - plot_w);
    const double oy = margin + 0.5 * (drawable_h - plot_h);

    auto mapX = [&](double lon) { return ox + (lon - min_lon) * scale; };
    auto mapY = [&](double lat) { return oy + (max_lat - lat) * scale; };

    std::ofstream out(file_path);
    if (!out) {
        std::cerr << "SVG yazilamadi: " << file_path << "\n";
        return;
    }

    out << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width << "\" height=\"" << height
        << "\" viewBox=\"0 0 " << width << " " << height << "\">\n";
    out << "<rect x=\"0\" y=\"0\" width=\"" << width << "\" height=\"" << height << "\" fill=\"#f8fafc\"/>\n";
    out << "<rect x=\"" << ox << "\" y=\"" << oy << "\" width=\"" << plot_w << "\" height=\"" << plot_h
        << "\" fill=\"#eef2f7\" stroke=\"#94a3b8\" stroke-width=\"1\"/>\n";

    for (const auto& poly : land_polygons) {
        if (!bboxesOverlap(min_lat, max_lat, min_lon, max_lon, poly.min_lat, poly.max_lat, poly.min_lon, poly.max_lon)) {
            continue;
        }
        out << "<polygon points=\"";
        for (const auto& v : poly.vertices) {
            out << mapX(v.lon_deg) << "," << mapY(v.lat_deg) << " ";
        }
        out << "\" fill=\"#d97706\" fill-opacity=\"0.32\" stroke=\"#92400e\" stroke-width=\"0.7\"/>\n";
    }

    for (const auto& c : grid_cells) {
        const char* stroke = c.fine ? "#1d4ed8" : "#64748b";
        const char* fill = c.fine ? "#3b82f6" : "#94a3b8";
        const double opacity = c.fine ? 0.10 : 0.06;
        const double x = mapX(c.min_lon);
        const double y = mapY(c.max_lat);
        const double w = mapX(c.max_lon) - mapX(c.min_lon);
        const double h = mapY(c.min_lat) - mapY(c.max_lat);
        out << "<rect x=\"" << x << "\" y=\"" << y << "\" width=\"" << w << "\" height=\"" << h << "\" fill=\""
            << fill << "\" fill-opacity=\"" << opacity << "\" stroke=\"" << stroke << "\" stroke-width=\"0.6\"/>\n";
    }

    for (const auto& e : graph_edges) {
        const LatLon a = gridCellCenter(grid_cells[e.first]);
        const LatLon b = gridCellCenter(grid_cells[e.second]);
        out << "<line x1=\"" << mapX(a.lon_deg) << "\" y1=\"" << mapY(a.lat_deg) << "\" x2=\"" << mapX(b.lon_deg)
            << "\" y2=\"" << mapY(b.lat_deg) << "\" stroke=\"#cbd5e1\" stroke-width=\"0.45\"/>\n";
    }

    if (graph_path.size() >= 2) {
        out << "<polyline points=\"";
        for (int idx : graph_path) {
            const LatLon c = gridCellCenter(grid_cells[idx]);
            out << mapX(c.lon_deg) << "," << mapY(c.lat_deg) << " ";
        }
        out << "\" fill=\"none\" stroke=\"#0f172a\" stroke-width=\"2.2\" stroke-linecap=\"round\"/>\n";
    }

    out << "<circle cx=\"" << mapX(start.lon_deg) << "\" cy=\"" << mapY(start.lat_deg)
        << "\" r=\"5\" fill=\"#15803d\"/>\n";
    out << "<circle cx=\"" << mapX(goal.lon_deg) << "\" cy=\"" << mapY(goal.lat_deg)
        << "\" r=\"5\" fill=\"#b91c1c\"/>\n";

    out << "<rect x=\"24\" y=\"18\" width=\"690\" height=\"128\" rx=\"8\" fill=\"#ffffff\" stroke=\"#cbd5e1\"/>\n";
    out << "<text x=\"40\" y=\"45\" font-family=\"monospace\" font-size=\"18\" fill=\"#0f172a\">"
        << "Adaptive Grid Debug (" << route_label << ")</text>\n";
    out << "<text x=\"40\" y=\"70\" font-family=\"monospace\" font-size=\"14\" fill=\"#334155\">"
        << "Quadtree-based graph: light-gray edges over adaptive cells</text>\n";
    out << "<text x=\"40\" y=\"95\" font-family=\"monospace\" font-size=\"14\" fill=\"#334155\">"
        << "Cell count: total=" << grid_cells.size() << ", coarse=" << coarse_count << ", fine=" << fine_count
        << "</text>\n";
    out << "<text x=\"40\" y=\"120\" font-family=\"monospace\" font-size=\"14\" fill=\"#334155\">"
        << "Graph edges=" << graph_edges.size() << ", path nodes=" << graph_path.size() << "</text>\n";

    out << "</svg>\n";
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
    for (char& ch : s) {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    return s;
}

bool applyRoutePreset(const std::string& route, CliConfig& cfg) {
    if (route == "istanbul-izmir") {
        cfg.start = {41.0082, 28.9784};
        cfg.goal = {38.4237, 27.1428};
        cfg.route_label = "Istanbul - Izmir";
        cfg.svg_file = "output/istanbul_izmir_adaptive_grid.svg";
        return true;
    }
    if (route == "istanbul-genoa") {
        cfg.start = {41.0082, 28.9784};
        cfg.goal = {44.4056, 8.9463};
        cfg.route_label = "Istanbul - Genoa";
        cfg.svg_file = "output/istanbul_genoa_adaptive_grid.svg";
        return true;
    }
    if (route == "istanbul-mudanya") {
        cfg.start = {41.0082, 28.9784};
        cfg.goal = {40.3759, 28.8826};
        cfg.route_label = "Istanbul - Mudanya";
        cfg.svg_file = "output/istanbul_mudanya_adaptive_grid.svg";
        return true;
    }
    return false;
}

void printUsage(const char* bin) {
    std::cout << "Kullanim:\n"
              << "  " << bin
              << " [--dataset <10m|50m|110m|/path/file.geojson>] [--grid-step <deg>]\n"
              << "    [--corridor-lat <deg>] [--corridor-lon <deg>] [--clearance-m <m>]\n"
              << "    [--route <istanbul-izmir|istanbul-genoa|istanbul-mudanya>] [--help]\n"
              << "    [--build-cache <file.bin>] [--use-cache <file.bin>] [--load-cache] [--config <file.ini>]\n";
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
    const std::string v = toLowerCopy(trimCopy(value));
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

        if (full_key == "dataset.land_geojson" || key == "land_geojson") {
            cfg.land_geojson = value;
            continue;
        }
        if (full_key == "mesh.grid_step_deg" || key == "grid_step_deg") {
            if (!parseIniDouble("grid_step_deg", cfg.grid_step_deg, true, false)) {
                return false;
            }
            continue;
        }
        if (full_key == "mesh.corridor_lat_pad_deg" || full_key == "mesh.corridor_lat_pad" || key == "corridor_lat_pad_deg" ||
            key == "corridor_lat_pad") {
            if (!parseIniDouble("corridor_lat_pad", cfg.corridor_lat_pad, false, true)) {
                return false;
            }
            continue;
        }
        if (full_key == "mesh.corridor_lon_pad_deg" || full_key == "mesh.corridor_lon_pad" || key == "corridor_lon_pad_deg" ||
            key == "corridor_lon_pad") {
            if (!parseIniDouble("corridor_lon_pad", cfg.corridor_lon_pad, false, true)) {
                return false;
            }
            continue;
        }
        if (full_key == "safety.coastline_clearance_m" || full_key == "safety.clearance_m" || key == "coastline_clearance_m" ||
            key == "clearance_m") {
            if (!parseIniDouble("clearance_m", cfg.clearance_m, false, true)) {
                return false;
            }
            continue;
        }
        if (full_key == "route.preset" || full_key == "route.route" || key == "route") {
            const std::string route = toLowerCopy(value);
            if (!applyRoutePreset(route, cfg)) {
                std::cerr << "INI route bilinmiyor: " << value << "\n";
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
        if (full_key == "route.label" || full_key == "route.route_label" || key == "route_label") {
            cfg.route_label = value;
            continue;
        }
        if (full_key == "output.svg_file" || key == "svg_file") {
            cfg.svg_file = value;
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
        if (full_key == "cache.use_cache" || full_key == "cache.load_cache" || key == "use_cache" || key == "load_cache") {
            if (!parseBoolArg("use_cache", value, cfg.use_cache_enabled)) {
                return false;
            }
            continue;
        }
    }

    return true;
}

bool parseCli(int argc, char** argv, CliConfig& cfg, bool& should_run) {
    should_run = true;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto needValue = [&](const char* name) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << name << " deger bekliyor.\n";
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
            const char* v = needValue("--dataset");
            if (!v) {
                return false;
            }
            const std::string ds = v;
            if (ds == "10m") {
                cfg.land_geojson = "dataset/ne_10m_land.geojson";
            } else if (ds == "50m") {
                cfg.land_geojson = "dataset/ne_50m_land.geojson";
            } else if (ds == "110m") {
                cfg.land_geojson = "dataset/ne_110m_land.geojson";
            } else {
                cfg.land_geojson = ds;
            }
            continue;
        }
        if (arg == "--route") {
            const char* v = needValue("--route");
            if (!v) {
                return false;
            }
            const std::string route = toLowerCopy(v);
            if (!applyRoutePreset(route, cfg)) {
                std::cerr << "Bilinmeyen route: " << v << "\n";
                return false;
            }
            continue;
        }
        if (arg == "--grid-step") {
            const char* v = needValue("--grid-step");
            if (!v || !parseDoubleArg("--grid-step", v, cfg.grid_step_deg) || cfg.grid_step_deg <= 0.0) {
                return false;
            }
            continue;
        }
        if (arg == "--corridor-lat") {
            const char* v = needValue("--corridor-lat");
            if (!v || !parseDoubleArg("--corridor-lat", v, cfg.corridor_lat_pad) || cfg.corridor_lat_pad < 0.0) {
                return false;
            }
            continue;
        }
        if (arg == "--corridor-lon") {
            const char* v = needValue("--corridor-lon");
            if (!v || !parseDoubleArg("--corridor-lon", v, cfg.corridor_lon_pad) || cfg.corridor_lon_pad < 0.0) {
                return false;
            }
            continue;
        }
        if (arg == "--clearance-m") {
            const char* v = needValue("--clearance-m");
            if (!v || !parseDoubleArg("--clearance-m", v, cfg.clearance_m) || cfg.clearance_m < 0.0) {
                return false;
            }
            continue;
        }
        if (arg == "--build-cache") {
            const char* v = needValue("--build-cache");
            if (!v) {
                return false;
            }
            cfg.build_cache_file = v;
            continue;
        }
        if (arg == "--use-cache") {
            const char* v = needValue("--use-cache");
            if (!v) {
                return false;
            }
            cfg.use_cache_file = v;
            cfg.use_cache_enabled = true;
            continue;
        }
        if (arg == "--load-cache") {
            cfg.use_cache_enabled = true;
            continue;
        }
        if (arg == "--config") {
            const char* v = needValue("--config");
            if (!v) {
                return false;
            }
            if (!loadIniConfig(v, cfg)) {
                return false;
            }
            continue;
        }

        std::cerr << "Bilinmeyen arguman: " << arg << "\n";
        printUsage(argv[0]);
        return false;
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    CliConfig cfg;
    bool should_run = true;
    if (!parseCli(argc, argv, cfg, should_run)) {
        return 1;
    }
    if (!should_run) {
        return 0;
    }

    std::vector<LandPolygon> all_land;
    if (!loadLandPolygonsGeoJson(cfg.land_geojson, all_land)) {
        std::cerr << "Dataset yuklenemedi.\n";
        return 1;
    }

    const double clearance_km = cfg.clearance_m / 1000.0;
    const double min_lat = clampLatitude(std::min(cfg.start.lat_deg, cfg.goal.lat_deg) - cfg.corridor_lat_pad);
    const double max_lat = clampLatitude(std::max(cfg.start.lat_deg, cfg.goal.lat_deg) + cfg.corridor_lat_pad);
    const double min_lon = std::max(-179.0, std::min(cfg.start.lon_deg, cfg.goal.lon_deg) - cfg.corridor_lon_pad);
    const double max_lon = std::min(179.0, std::max(cfg.start.lon_deg, cfg.goal.lon_deg) + cfg.corridor_lon_pad);

    auto land = filterPolygonsByBounds(all_land, min_lat, max_lat, min_lon, max_lon);
    auto land_index = buildLandSpatialIndex(land, 0.5);
    const LatLon start_sea = nearestSeaPoint(cfg.start, land, land_index, clearance_km);
    const LatLon goal_sea = nearestSeaPoint(cfg.goal, land, land_index, clearance_km);

    std::vector<GridCell> grid_cells;
    std::vector<std::pair<int, int>> graph_edges;
    std::vector<std::vector<std::pair<int, double>>> graph_adj;

    if (cfg.use_cache_enabled) {
        if (cfg.use_cache_file.empty()) {
            std::cerr << "Cache yukleme aktif ama use_cache_file bos. (--use-cache <file> veya [cache] use_cache_file)\n";
            return 1;
        }
        GraphCacheData cache;
        if (!loadGraphCache(cfg.use_cache_file, cache)) {
            return 1;
        }
        if (!isCacheCompatible(cache, cfg, start_sea, goal_sea)) {
            return 1;
        }
        grid_cells = std::move(cache.cells);
        graph_edges = std::move(cache.edges);
        graph_adj = buildAdjacencyFromEdges(grid_cells, graph_edges);
        std::cout << "Cache yuklendi: " << cfg.use_cache_file << "\n";
    } else {
        grid_cells = generateAdaptiveGrid(min_lat, max_lat, min_lon, max_lon, cfg.grid_step_deg, land, land_index, clearance_km);
        std::tie(graph_adj, graph_edges) = buildQuadtreeGraph(grid_cells);

        if (!cfg.build_cache_file.empty()) {
            GraphCacheData cache;
            cache.dataset_path = cfg.land_geojson;
            cache.min_lat = min_lat;
            cache.max_lat = max_lat;
            cache.min_lon = min_lon;
            cache.max_lon = max_lon;
            cache.grid_step_deg = cfg.grid_step_deg;
            cache.clearance_m = cfg.clearance_m;
            cache.cells = grid_cells;
            cache.edges = graph_edges;

            const std::filesystem::path cache_path(cfg.build_cache_file);
            if (cache_path.has_parent_path()) {
                std::filesystem::create_directories(cache_path.parent_path());
            }
            if (saveGraphCache(cfg.build_cache_file, cache)) {
                std::cout << "Cache yazildi: " << cfg.build_cache_file << "\n";
            } else {
                return 1;
            }
        }
    }

    const int start_node = nearestCellIndex(grid_cells, start_sea);
    const int goal_node = nearestCellIndex(grid_cells, goal_sea);
    const auto astar_path = astarPath(graph_adj, grid_cells, start_node, goal_node);
    std::vector<int> refined_path = astar_path;
    if (!astar_path.empty()) {
        refined_path = refinePathWithLOS(astar_path, grid_cells, land, land_index, clearance_km);
    }

    int coarse_count = 0;
    int fine_count = 0;
    for (const auto& c : grid_cells) {
        if (c.fine) {
            fine_count += 1;
        } else {
            coarse_count += 1;
        }
    }

    std::cout << "Dataset: " << cfg.land_geojson << "\n";
    std::cout << "Grid step (fine): " << cfg.grid_step_deg << " deg\n";
    std::cout << "Route: " << cfg.route_label << "\n";
    std::cout << "Start sea: (" << start_sea.lat_deg << ", " << start_sea.lon_deg << ")\n";
    std::cout << "Goal sea: (" << goal_sea.lat_deg << ", " << goal_sea.lon_deg << ")\n";
    std::cout << "Corridor pad lat/lon: " << cfg.corridor_lat_pad << " / " << cfg.corridor_lon_pad << " deg\n";
    std::cout << "Clearance: " << cfg.clearance_m << " m\n";
    std::cout << "Grid hucreleri: total=" << grid_cells.size() << ", coarse=" << coarse_count
              << ", fine=" << fine_count << "\n";
    std::cout << "Quadtree-graph: nodes=" << grid_cells.size() << ", edges=" << graph_edges.size() << "\n";
    if (astar_path.empty()) {
        std::cout << "A* path: bulunamadi\n";
        const auto [comp_id, comp_sizes] = connectedComponents(graph_adj);
        if (start_node >= 0 && goal_node >= 0 && start_node < static_cast<int>(comp_id.size()) &&
            goal_node < static_cast<int>(comp_id.size())) {
            const int start_comp = comp_id[start_node];
            const int goal_comp = comp_id[goal_node];
            std::cout << "Bilesenler: start_comp=" << start_comp << " (size=" << comp_sizes[start_comp]
                      << "), goal_comp=" << goal_comp << " (size=" << comp_sizes[goal_comp] << ")\n";

            int start_comp_dardanelles = 0;
            int goal_comp_dardanelles = 0;
            int start_comp_bosphorus = 0;
            int goal_comp_bosphorus = 0;
            for (int i = 0; i < static_cast<int>(grid_cells.size()); ++i) {
                if (comp_id[i] == start_comp) {
                    if (centerInsideBox(grid_cells[i], 39.9, 40.6, 26.0, 27.0)) {
                        start_comp_dardanelles += 1;
                    }
                    if (centerInsideBox(grid_cells[i], 40.95, 41.3, 28.75, 29.35)) {
                        start_comp_bosphorus += 1;
                    }
                } else if (comp_id[i] == goal_comp) {
                    if (centerInsideBox(grid_cells[i], 39.9, 40.6, 26.0, 27.0)) {
                        goal_comp_dardanelles += 1;
                    }
                    if (centerInsideBox(grid_cells[i], 40.95, 41.3, 28.75, 29.35)) {
                        goal_comp_bosphorus += 1;
                    }
                }
            }

            std::cout << "Dardanelles kutusu (start/goal bileseni): " << start_comp_dardanelles << " / "
                      << goal_comp_dardanelles << "\n";
            std::cout << "Bosphorus kutusu (start/goal bileseni): " << start_comp_bosphorus << " / "
                      << goal_comp_bosphorus << "\n";
        }
    } else {
        std::cout << "A* path: bulundu (node sayisi=" << astar_path.size() << ")\n";
        std::cout << "LOS refine: " << astar_path.size() << " -> " << refined_path.size() << " node\n";
    }

    std::filesystem::create_directories("output");
    writeGridSvg(cfg.svg_file,
                 grid_cells,
                 land,
                 start_sea,
                 goal_sea,
                 cfg.route_label,
                 graph_edges,
                 refined_path,
                 coarse_count,
                 fine_count);
    std::cout << "SVG cikti: " << cfg.svg_file << "\n";

    return 0;
}
