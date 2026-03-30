#include "world_layers_support.h"

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <mutex>
#include <regex>
#include <sstream>
#include <string>
#include <unordered_map>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

namespace worldlayers {

namespace {

namespace bg = boost::geometry;

std::mutex g_land_cache_mutex;
std::unordered_map<std::string, std::vector<planner::LandPolygon>> g_land_cache;

using Point2 = bg::model::d2::point_xy<double>;
using Polygon2 = bg::model::polygon<Point2>;
using MultiPolygon2 = bg::model::multi_polygon<Polygon2>;

std::filesystem::path projectRootFsPath() {
#ifdef VGPP_PROJECT_ROOT
    return std::filesystem::path(VGPP_PROJECT_ROOT);
#else
    return std::filesystem::current_path();
#endif
}

std::filesystem::path resolvePath(const std::string& path, bool for_output) {
    if (path.empty()) {
        return {};
    }

    const std::filesystem::path raw(path);
    if (raw.is_absolute()) {
        return raw;
    }
    if (!for_output && std::filesystem::exists(raw)) {
        return raw;
    }

    const std::filesystem::path from_root = projectRootFsPath() / raw;
    if (for_output || std::filesystem::exists(from_root)) {
        return from_root;
    }

    const std::string raw_text = raw.generic_string();
    if (!for_output) {
        if (raw_text.rfind("data/", 0) == 0) {
            const std::filesystem::path dataset_alias = projectRootFsPath() / ("dataset/" + raw_text.substr(5));
            if (std::filesystem::exists(dataset_alias)) {
                return dataset_alias;
            }
        }
        if (raw_text.rfind("dataset/", 0) == 0) {
            const std::filesystem::path data_alias = projectRootFsPath() / ("data/" + raw_text.substr(8));
            if (std::filesystem::exists(data_alias)) {
                return data_alias;
            }
        }
    }
    return raw;
}

std::string makeBoundsCacheKey(const BoundsConfig& bounds) {
    std::ostringstream out;
    out.setf(std::ios::fixed);
    out.precision(6);
    out << (bounds.enabled ? 1 : 0) << "|" << bounds.min_lat << "|" << bounds.max_lat << "|" << bounds.min_lon << "|"
        << bounds.max_lon;
    return out.str();
}

std::string fileTimestampKey(const std::string& path) {
    std::error_code ec;
    const auto ts = std::filesystem::last_write_time(path, ec);
    if (ec) {
        return "no-ts";
    }
    return std::to_string(ts.time_since_epoch().count());
}

std::string hashedCacheName(const std::string& key, const char* prefix) {
    std::ostringstream out;
    out << prefix << "_" << std::hex << std::hash<std::string>{}(key) << ".bin";
    return out.str();
}

std::filesystem::path cacheDirectoryPath() {
    return projectRootFsPath() / ".cache" / "world_layers";
}

bool savePolygonVectorCache(const std::filesystem::path& cache_path,
                            const std::vector<planner::LandPolygon>& polygons) {
    std::error_code ec;
    std::filesystem::create_directories(cache_path.parent_path(), ec);
    if (ec) {
        return false;
    }

    std::ofstream out(cache_path, std::ios::binary);
    if (!out) {
        return false;
    }

    constexpr std::uint32_t kMagic = 0x574c5043U;
    constexpr std::uint32_t kVersion = 1U;
    const std::uint64_t poly_count = static_cast<std::uint64_t>(polygons.size());
    out.write(reinterpret_cast<const char*>(&kMagic), sizeof(kMagic));
    out.write(reinterpret_cast<const char*>(&kVersion), sizeof(kVersion));
    out.write(reinterpret_cast<const char*>(&poly_count), sizeof(poly_count));
    for (const auto& poly : polygons) {
        const std::uint64_t vertex_count = static_cast<std::uint64_t>(poly.vertices.size());
        out.write(reinterpret_cast<const char*>(&vertex_count), sizeof(vertex_count));
        out.write(reinterpret_cast<const char*>(&poly.min_lat), sizeof(poly.min_lat));
        out.write(reinterpret_cast<const char*>(&poly.max_lat), sizeof(poly.max_lat));
        out.write(reinterpret_cast<const char*>(&poly.min_lon), sizeof(poly.min_lon));
        out.write(reinterpret_cast<const char*>(&poly.max_lon), sizeof(poly.max_lon));
        for (const auto& vertex : poly.vertices) {
            out.write(reinterpret_cast<const char*>(&vertex.lat_deg), sizeof(vertex.lat_deg));
            out.write(reinterpret_cast<const char*>(&vertex.lon_deg), sizeof(vertex.lon_deg));
        }
    }
    return static_cast<bool>(out);
}

bool loadPolygonVectorCache(const std::filesystem::path& cache_path,
                            std::vector<planner::LandPolygon>& polygons) {
    std::ifstream in(cache_path, std::ios::binary);
    if (!in) {
        return false;
    }

    std::uint32_t magic = 0;
    std::uint32_t version = 0;
    std::uint64_t poly_count = 0;
    in.read(reinterpret_cast<char*>(&magic), sizeof(magic));
    in.read(reinterpret_cast<char*>(&version), sizeof(version));
    in.read(reinterpret_cast<char*>(&poly_count), sizeof(poly_count));
    if (!in || magic != 0x574c5043U || version != 1U) {
        return false;
    }

    polygons.clear();
    polygons.reserve(static_cast<std::size_t>(poly_count));
    for (std::uint64_t i = 0; i < poly_count; ++i) {
        planner::LandPolygon poly;
        std::uint64_t vertex_count = 0;
        in.read(reinterpret_cast<char*>(&vertex_count), sizeof(vertex_count));
        in.read(reinterpret_cast<char*>(&poly.min_lat), sizeof(poly.min_lat));
        in.read(reinterpret_cast<char*>(&poly.max_lat), sizeof(poly.max_lat));
        in.read(reinterpret_cast<char*>(&poly.min_lon), sizeof(poly.min_lon));
        in.read(reinterpret_cast<char*>(&poly.max_lon), sizeof(poly.max_lon));
        if (!in) {
            return false;
        }
        poly.vertices.resize(static_cast<std::size_t>(vertex_count));
        for (std::uint64_t j = 0; j < vertex_count; ++j) {
            in.read(reinterpret_cast<char*>(&poly.vertices[static_cast<std::size_t>(j)].lat_deg),
                    sizeof(poly.vertices[static_cast<std::size_t>(j)].lat_deg));
            in.read(reinterpret_cast<char*>(&poly.vertices[static_cast<std::size_t>(j)].lon_deg),
                    sizeof(poly.vertices[static_cast<std::size_t>(j)].lon_deg));
            if (!in) {
                return false;
            }
        }
        polygons.push_back(std::move(poly));
    }
    return true;
}

double effectiveSimplifyMeters(double offset_m) {
    return std::clamp(offset_m * 0.35, 5.0, 40.0);
}

planner::LandPolygon makeLandPolygon(std::vector<planner::LatLon> vertices) {
    planner::LandPolygon poly;
    poly.vertices = std::move(vertices);
    poly.min_lat = std::numeric_limits<double>::infinity();
    poly.max_lat = -std::numeric_limits<double>::infinity();
    poly.min_lon = std::numeric_limits<double>::infinity();
    poly.max_lon = -std::numeric_limits<double>::infinity();
    for (const auto& v : poly.vertices) {
        poly.min_lat = std::min(poly.min_lat, v.lat_deg);
        poly.max_lat = std::max(poly.max_lat, v.lat_deg);
        poly.min_lon = std::min(poly.min_lon, v.lon_deg);
        poly.max_lon = std::max(poly.max_lon, v.lon_deg);
    }
    return poly;
}

Polygon2 landToPolygon(const planner::LandPolygon& land, const planner::Projection& proj, double simplify_km) {
    Polygon2 poly;
    auto& outer = poly.outer();
    outer.reserve(land.vertices.size() + 1);
    for (const auto& v : land.vertices) {
        const auto xy = planner::toXY(v, proj);
        outer.emplace_back(xy.x, xy.y);
    }
    if (!outer.empty()) {
        outer.push_back(outer.front());
    }
    bg::correct(poly);

    if (simplify_km > 0.0) {
        Polygon2 simplified;
        bg::simplify(poly, simplified, simplify_km);
        bg::correct(simplified);
        if (simplified.outer().size() >= 4 && std::abs(bg::area(simplified)) > 1e-8 && bg::is_valid(simplified)) {
            poly = std::move(simplified);
        }
    }
    return poly;
}

MultiPolygon2 computeBuffer(const Polygon2& poly, double offset_km) {
    MultiPolygon2 buffered;
    bg::strategy::buffer::distance_symmetric<double> distance_strategy(offset_km);
    bg::strategy::buffer::side_straight side_strategy;
    bg::strategy::buffer::join_round join_strategy(12);
    bg::strategy::buffer::end_round end_strategy(12);
    bg::strategy::buffer::point_circle circle_strategy(12);
    bg::buffer(poly, buffered, distance_strategy, side_strategy, join_strategy, end_strategy, circle_strategy);
    return buffered;
}

MultiPolygon2 singlePolygonMulti(const Polygon2& poly) {
    MultiPolygon2 out;
    out.push_back(poly);
    return out;
}

std::vector<planner::BufferZonePolygon> buildBufferZones(const std::vector<planner::LandPolygon>& land,
                                                         std::vector<double> offsets_m) {
    std::sort(offsets_m.begin(), offsets_m.end());
    offsets_m.erase(std::remove_if(offsets_m.begin(), offsets_m.end(), [](double value) { return value <= 0.0; }),
                    offsets_m.end());
    offsets_m.erase(std::unique(offsets_m.begin(), offsets_m.end()), offsets_m.end());

    std::vector<planner::BufferZonePolygon> zones;
    if (offsets_m.empty()) {
        return zones;
    }

    zones.reserve(land.size() * offsets_m.size());
    for (const auto& source_poly : land) {
        const auto proj = planner::makeProjection(source_poly.min_lat,
                                                  source_poly.max_lat,
                                                  source_poly.min_lon,
                                                  source_poly.max_lon);
        for (std::size_t i = 0; i < offsets_m.size(); ++i) {
            const double inner_m = (i == 0) ? 0.0 : offsets_m[i - 1];
            const double outer_m = offsets_m[i];
            const Polygon2 local_poly =
                landToPolygon(source_poly, proj, effectiveSimplifyMeters(outer_m) / 1000.0);
            if (local_poly.outer().size() < 4 || std::abs(bg::area(local_poly)) <= 1e-8) {
                continue;
            }

            MultiPolygon2 outer = computeBuffer(local_poly, outer_m / 1000.0);
            MultiPolygon2 inner =
                inner_m > 0.0 ? computeBuffer(local_poly, inner_m / 1000.0) : singlePolygonMulti(local_poly);
            MultiPolygon2 band;
            bg::difference(outer, inner, band);

            for (const auto& piece : band) {
                if (piece.outer().size() < 4 || std::abs(bg::area(piece)) <= 1e-8) {
                    continue;
                }

                std::vector<planner::LatLon> vertices;
                vertices.reserve(piece.outer().size());
                for (std::size_t p = 0; p + 1 < piece.outer().size(); ++p) {
                    const auto& pt = piece.outer()[p];
                    vertices.push_back(planner::toGeo({pt.x(), pt.y()}, proj));
                }
                if (vertices.size() < 3) {
                    continue;
                }

                planner::BufferZonePolygon zone;
                zone.polygon = makeLandPolygon(std::move(vertices));
                zone.inner_m = inner_m;
                zone.outer_m = outer_m;
                zones.push_back(std::move(zone));
            }
        }
    }

    return zones;
}

std::pair<double, double> inferBufferBandMeters(const std::string& path) {
    const std::string name = std::filesystem::path(path).stem().string();

    {
        const std::regex sea_band_pattern(R"(.*_sea_band_(\d{3})_(\d{3})m$)");
        std::smatch match;
        if (std::regex_match(name, match, sea_band_pattern) && match.size() == 3) {
            return {std::stod(match[1].str()), std::stod(match[2].str())};
        }
    }

    {
        const std::regex buffer_pattern(R"(.*_buffer_(\d{3})m$)");
        std::smatch match;
        if (std::regex_match(name, match, buffer_pattern) && match.size() == 2) {
            return {0.0, std::stod(match[1].str())};
        }
    }

    return {0.0, 0.0};
}

bool loadBufferZoneDataset(const Config& cfg,
                           std::vector<planner::BufferZonePolygon>& out_zones,
                           std::string& error_message,
                           std::string& warning_message) {
    out_zones.clear();
    if (cfg.buffer_zones_geojson.empty()) {
        return true;
    }

    std::vector<planner::LandPolygon> polygons;
    const std::string path = resolveInputPath(cfg.buffer_zones_geojson);
    const bool loaded = cfg.bounds.enabled ? planner::loadLandPolygons(path,
                                                                       polygons,
                                                                       cfg.bounds.min_lat,
                                                                       cfg.bounds.max_lat,
                                                                       cfg.bounds.min_lon,
                                                                       cfg.bounds.max_lon)
                                           : planner::loadLandPolygons(path, polygons);
    if (!loaded) {
        if (!cfg.allow_missing_buffer_zones) {
            error_message = "Offset zone veri seti yuklenemedi: " + cfg.buffer_zones_geojson;
            return false;
        }
        warning_message += "Offset zone veri seti yuklenemedi; zon katmani kapatildi.\n";
        return true;
    }

    const auto [inner_m, outer_m] = inferBufferBandMeters(cfg.buffer_zones_geojson);
    out_zones.reserve(polygons.size());
    for (auto& polygon : polygons) {
        planner::BufferZonePolygon zone;
        zone.polygon = std::move(polygon);
        zone.inner_m = inner_m;
        zone.outer_m = outer_m;
        out_zones.push_back(std::move(zone));
    }
    return true;
}

}  // namespace

std::string projectRootPath() {
    return projectRootFsPath().string();
}

std::string resolveInputPath(const std::string& path) {
    return resolvePath(path, false).string();
}

std::string resolveOutputPath(const std::string& path) {
    return resolvePath(path, true).string();
}

bool loadData(const Config& cfg, LoadedData& out_data, std::string& error_message, std::string& warning_message) {
    error_message.clear();
    warning_message.clear();
    out_data = LoadedData{};

    const std::string land_path = resolveInputPath(cfg.land_geojson);
    const std::string tss_path = resolveInputPath(cfg.tss_geojson);
    const std::string bathymetry_path = resolveInputPath(cfg.bathymetry_geojson);
    const std::string land_cache_key =
        "v1|" + land_path + "|" + makeBoundsCacheKey(cfg.bounds) + "|" + fileTimestampKey(land_path);
    const std::filesystem::path land_cache_path = cacheDirectoryPath() / hashedCacheName(land_cache_key, "land");

    {
        std::lock_guard<std::mutex> lock(g_land_cache_mutex);
        const auto it = g_land_cache.find(land_cache_key);
        if (it != g_land_cache.end()) {
            out_data.land = it->second;
        }
    }
    if (out_data.land.empty()) {
        loadPolygonVectorCache(land_cache_path, out_data.land);
    }
    if (out_data.land.empty()) {
        const bool land_loaded = cfg.bounds.enabled
                                     ? planner::loadLandPolygons(land_path,
                                                                 out_data.land,
                                                                 cfg.bounds.min_lat,
                                                                 cfg.bounds.max_lat,
                                                                 cfg.bounds.min_lon,
                                                                 cfg.bounds.max_lon)
                                     : planner::loadLandPolygons(land_path, out_data.land);
        if (!land_loaded) {
            error_message = "Kara veri seti yuklenemedi: " + cfg.land_geojson;
            return false;
        }
        savePolygonVectorCache(land_cache_path, out_data.land);
        std::lock_guard<std::mutex> lock(g_land_cache_mutex);
        g_land_cache[land_cache_key] = out_data.land;
    }

    if (!cfg.tss_geojson.empty()) {
        if (!planner::loadTssFeaturesGeoJson(tss_path, out_data.tss)) {
            if (!cfg.allow_missing_tss) {
                error_message = "TSS veri seti yuklenemedi: " + cfg.tss_geojson;
                return false;
            }
            warning_message += "Varsayilan TSS katmani yuklenemedi; TSS kapatildi.\n";
            out_data.tss.clear();
        }
    }

    if (!cfg.bathymetry_geojson.empty()) {
        if (!planner::loadBathymetryFeatures(bathymetry_path, out_data.bathymetry)) {
            if (!cfg.allow_missing_bathymetry) {
                error_message = "Bathymetry veri seti yuklenemedi: " + cfg.bathymetry_geojson;
                return false;
            }
            warning_message += "Varsayilan bathymetry katmani yuklenemedi; bathymetry kapatildi.\n";
            out_data.bathymetry.clear();
        }
    }

    if (cfg.enable_buffer_zones && !cfg.buffer_zones_geojson.empty()) {
        if (!loadBufferZoneDataset(cfg, out_data.buffer_zones, error_message, warning_message)) {
            return false;
        }
    }

    if (cfg.bounds.enabled) {
        out_data.land = planner::filterPolygonsByBounds(out_data.land,
                                                        cfg.bounds.min_lat,
                                                        cfg.bounds.max_lat,
                                                        cfg.bounds.min_lon,
                                                        cfg.bounds.max_lon);
        out_data.tss = planner::filterTssByBounds(out_data.tss,
                                                  cfg.bounds.min_lat,
                                                  cfg.bounds.max_lat,
                                                  cfg.bounds.min_lon,
                                                  cfg.bounds.max_lon);
        out_data.bathymetry = planner::filterBathymetryByBounds(out_data.bathymetry,
                                                                cfg.bounds.min_lat,
                                                                cfg.bounds.max_lat,
                                                                cfg.bounds.min_lon,
                                                                cfg.bounds.max_lon);
        if (!out_data.buffer_zones.empty()) {
            std::vector<planner::BufferZonePolygon> filtered_zones;
            filtered_zones.reserve(out_data.buffer_zones.size());
            for (const auto& zone : out_data.buffer_zones) {
                if (planner::bboxesOverlap(cfg.bounds.min_lat,
                                           cfg.bounds.max_lat,
                                           cfg.bounds.min_lon,
                                           cfg.bounds.max_lon,
                                           zone.polygon.min_lat,
                                           zone.polygon.max_lat,
                                           zone.polygon.min_lon,
                                           zone.polygon.max_lon)) {
                    filtered_zones.push_back(zone);
                }
            }
            out_data.buffer_zones = std::move(filtered_zones);
        }
    }

    if (cfg.enable_buffer_zones && out_data.buffer_zones.empty() && !cfg.buffer_offsets_m.empty()) {
        out_data.buffer_zones = buildBufferZones(out_data.land, cfg.buffer_offsets_m);
    }

    return true;
}

bool renderSvg(const Config& cfg, const LoadedData& data, std::string& error_message) {
    error_message.clear();

    const std::filesystem::path svg_path(resolveOutputPath(cfg.svg_file));
    if (svg_path.has_parent_path()) {
        std::error_code ec;
        std::filesystem::create_directories(svg_path.parent_path(), ec);
        if (ec) {
            error_message = "SVG klasoru olusturulamadi: " + svg_path.parent_path().string();
            return false;
        }
    }

    planner::writeSvgPlot(svg_path.string(), {}, {}, {}, {}, data.land, data.tss, data.bathymetry, data.buffer_zones);
    return true;
}

Summary summarize(const LoadedData& data) {
    Summary summary;
    summary.land_polygon_count = data.land.size();
    summary.tss_feature_count = data.tss.size();
    summary.bathymetry_feature_count = data.bathymetry.size();
    summary.buffer_zone_polygon_count = data.buffer_zones.size();
    return summary;
}

}  // namespace worldlayers
