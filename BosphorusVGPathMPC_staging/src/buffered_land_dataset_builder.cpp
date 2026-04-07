#include "planner.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

namespace bg = boost::geometry;

namespace {

using Point2 = bg::model::d2::point_xy<double>;
using Polygon2 = bg::model::polygon<Point2>;
using MultiPolygon2 = bg::model::multi_polygon<Polygon2>;
using Box2 = bg::model::box<Point2>;

struct BuilderConfig {
    std::string input_path;
    std::string output_path;
    std::string output_dir;
    std::string band_output_path;
    std::string mesh_output_path;
    std::string basename;
    std::vector<double> offsets_m;
    bool single_band_mode = false;
    double band_inner_m = 0.0;
    double band_outer_m = 0.0;
    double simplify_m = -1.0;
    double mesh_post_simplify_m = -1.0;
    double mesh_min_area_m2 = 0.0;
    bool has_bounds = false;
    double min_lat = 0.0;
    double max_lat = 0.0;
    double min_lon = 0.0;
    double max_lon = 0.0;
};

struct BufferedFeature {
    planner::Projection projection{};
    MultiPolygon2 geometry;
    double inner_m = 0.0;
    double outer_m = 0.0;
    std::string zone = "buffered_land";
};

void printUsage(const char* bin) {
    std::cerr << "Kullanim:\n"
              << "  " << bin << " --input <land.geojson|land.shp> --offset-m <metre> --output <buffered.geojson>\n"
              << "    [--simplify-m <metre>] [--min-lat <deg> --max-lat <deg> --min-lon <deg> --max-lon <deg>]\n"
              << "\n"
              << "  " << bin << " --input <land.geojson|land.shp> --offsets-m <50,100,150,200>\n"
              << "    [--output-dir <klasor>] [--band-output <sea_bands.geojson>] [--basename <isim>]\n"
              << "    [--simplify-m <metre>] [--min-lat <deg> --max-lat <deg> --min-lon <deg> --max-lon <deg>]\n"
              << "\n"
              << "  " << bin << " --input <land.geojson|land.shp> --band-range-m <inner,outer> --band-output <sea_band.geojson>\n"
              << "    [--simplify-m <metre>] [--min-lat <deg> --max-lat <deg> --min-lon <deg> --max-lon <deg>]\n"
              << "\n"
              << "  " << bin << " --input <land.geojson|land.shp> --offset-m <metre> --mesh-output <mesh_land.geojson>\n"
              << "    [--simplify-m <metre>] [--mesh-post-simplify-m <metre>] [--mesh-min-area-m2 <m2>]\n"
              << "    [--min-lat <deg> --max-lat <deg> --min-lon <deg> --max-lon <deg>]\n"
              << "\n"
              << "Notlar:\n"
              << "  --output-dir verilirse her offset icin ayri kumulatif buffered-land GeoJSON uretir.\n"
              << "  --band-output verilirse 0-50 / 50-100 / ... gibi deniz bandlari tek bir GeoJSON icine yazilir.\n"
              << "  --mesh-output triangulation icin hole'suz ve filtrelenmis mesh land polygonu yazar.\n"
              << "  --offset-m 0 ile sadece simplify + hole strip + kucuk alan temizligi yapilabilir.\n"
              << "  --simplify-m: -1 otomatik, 0 kapali, >0 sabit tolerans.\n";
}

bool parseDouble(const char* text, double& out) {
    if (text == nullptr) {
        return false;
    }
    char* end = nullptr;
    out = std::strtod(text, &end);
    if (end == text || !std::isfinite(out)) {
        return false;
    }
    while (*end != '\0') {
        if (!std::isspace(static_cast<unsigned char>(*end))) {
            return false;
        }
        ++end;
    }
    return true;
}

std::vector<double> parseOffsetList(const std::string& text) {
    std::set<double> unique_offsets;
    std::stringstream ss(text);
    std::string token;
    while (std::getline(ss, token, ',')) {
        if (token.empty()) {
            continue;
        }
        double value = 0.0;
        if (!parseDouble(token.c_str(), value) || value <= 0.0) {
            throw std::runtime_error("Gecersiz offset listesi: " + text);
        }
        unique_offsets.insert(value);
    }
    return std::vector<double>(unique_offsets.begin(), unique_offsets.end());
}

std::pair<double, double> parseBandRange(const std::string& text) {
    std::stringstream ss(text);
    std::string inner_text;
    std::string outer_text;
    if (!std::getline(ss, inner_text, ',') || !std::getline(ss, outer_text, ',')) {
        throw std::runtime_error("Gecersiz band araligi: " + text);
    }

    double inner_m = 0.0;
    double outer_m = 0.0;
    if (!parseDouble(inner_text.c_str(), inner_m) || !parseDouble(outer_text.c_str(), outer_m) || inner_m < 0.0 ||
        outer_m <= inner_m) {
        throw std::runtime_error("Band araligi inner>=0 ve outer>inner olmali: " + text);
    }
    return {inner_m, outer_m};
}

double effectiveSimplifyMeters(double simplify_m, double offset_m) {
    if (simplify_m >= 0.0) {
        return simplify_m;
    }
    return std::clamp(offset_m * 0.35, 5.0, 40.0);
}

double effectiveMeshPostSimplifyMeters(double simplify_m, double offset_m) {
    if (simplify_m >= 0.0) {
        return simplify_m;
    }
    return std::clamp(offset_m * 0.4, 8.0, 60.0);
}

std::string defaultBasename(const std::string& input_path) {
    return std::filesystem::path(input_path).stem().string();
}

std::string formatMeters(double meters) {
    const int rounded = static_cast<int>(std::lround(meters));
    std::ostringstream oss;
    oss << std::setw(3) << std::setfill('0') << rounded << "m";
    return oss.str();
}

Polygon2 landToGeoPolygon(const planner::LandPolygon& land) {
    Polygon2 poly;
    auto& outer = poly.outer();
    outer.reserve(land.vertices.size() + 1);
    for (const auto& v : land.vertices) {
        outer.emplace_back(v.lon_deg, v.lat_deg);
    }
    if (!outer.empty()) {
        outer.push_back(outer.front());
    }
    bg::correct(poly);
    return poly;
}

planner::LandPolygon geoPolygonToLand(const Polygon2& poly) {
    planner::LandPolygon out;
    const auto& ring = poly.outer();
    if (ring.size() < 4) {
        return out;
    }
    out.vertices.reserve(ring.size() - 1);
    out.min_lat = std::numeric_limits<double>::infinity();
    out.max_lat = -std::numeric_limits<double>::infinity();
    out.min_lon = std::numeric_limits<double>::infinity();
    out.max_lon = -std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i + 1 < ring.size(); ++i) {
        const planner::LatLon geo{ring[i].y(), ring[i].x()};
        out.vertices.push_back(geo);
        out.min_lat = std::min(out.min_lat, geo.lat_deg);
        out.max_lat = std::max(out.max_lat, geo.lat_deg);
        out.min_lon = std::min(out.min_lon, geo.lon_deg);
        out.max_lon = std::max(out.max_lon, geo.lon_deg);
    }
    if (out.vertices.size() < 3) {
        out.vertices.clear();
    }
    return out;
}

std::vector<planner::LandPolygon> clipLandToBounds(const std::vector<planner::LandPolygon>& land,
                                                   double min_lat,
                                                   double max_lat,
                                                   double min_lon,
                                                   double max_lon) {
    const Box2 clip_box(Point2(min_lon, min_lat), Point2(max_lon, max_lat));
    std::vector<planner::LandPolygon> out;
    out.reserve(land.size());
    for (const auto& poly : land) {
        Polygon2 geo_poly = landToGeoPolygon(poly);
        if (geo_poly.outer().size() < 4 || std::abs(bg::area(geo_poly)) <= 1e-12) {
            continue;
        }
        MultiPolygon2 clipped;
        bg::intersection(geo_poly, clip_box, clipped);
        for (const auto& piece : clipped) {
            if (piece.outer().size() < 4 || std::abs(bg::area(piece)) <= 1e-12) {
                continue;
            }
            planner::LandPolygon clipped_land = geoPolygonToLand(piece);
            if (clipped_land.vertices.size() >= 3) {
                out.push_back(std::move(clipped_land));
            }
        }
    }
    return out;
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

MultiPolygon2 buildSeaBand(const Polygon2& base_poly, double inner_km, double outer_km) {
    MultiPolygon2 outer = computeBuffer(base_poly, outer_km);
    MultiPolygon2 inner = inner_km > 0.0 ? computeBuffer(base_poly, inner_km) : singlePolygonMulti(base_poly);
    MultiPolygon2 band;
    bg::difference(outer, inner, band);
    return band;
}

void filterEmptyGeometry(MultiPolygon2& geometry) {
    MultiPolygon2 filtered;
    for (const auto& poly : geometry) {
        if (poly.outer().size() < 4) {
            continue;
        }
        if (std::abs(bg::area(poly)) <= 1e-8) {
            continue;
        }
        filtered.push_back(poly);
    }
    geometry = std::move(filtered);
}

std::string featureProperties(const BufferedFeature& feature) {
    std::ostringstream oss;
    oss << "\"offset_m\":" << std::llround(feature.outer_m);
    if (feature.zone == "sea_band") {
        oss << ",\"inner_m\":" << std::llround(feature.inner_m)
            << ",\"outer_m\":" << std::llround(feature.outer_m)
            << ",\"zone\":\"sea_band\"";
    } else {
        oss << ",\"zone\":\"" << feature.zone << "\"";
    }
    return oss.str();
}

void writeRing(std::ostream& out, const bg::model::ring<Point2, true, true>& ring, const planner::Projection& proj) {
    out << "[";
    for (std::size_t i = 0; i < ring.size(); ++i) {
        const planner::LatLon geo = planner::toGeo({ring[i].x(), ring[i].y()}, proj);
        if (i > 0) {
            out << ",";
        }
        out << "[" << geo.lon_deg << "," << geo.lat_deg << "]";
    }
    out << "]";
}

void writePolygon(std::ostream& out, const Polygon2& poly, const planner::Projection& proj) {
    out << "[";
    writeRing(out, poly.outer(), proj);
    for (const auto& inner : poly.inners()) {
        out << ",";
        writeRing(out, inner, proj);
    }
    out << "]";
}

bool writeFeatureCollection(const std::string& file_path, const std::vector<BufferedFeature>& features) {
    std::filesystem::path output_path(file_path);
    if (output_path.has_parent_path()) {
        std::error_code ec;
        std::filesystem::create_directories(output_path.parent_path(), ec);
        if (ec) {
            std::cerr << "Buffered dataset klasoru olusturulamadi: " << output_path.parent_path() << "\n";
            return false;
        }
    }

    std::ofstream out(file_path);
    if (!out) {
        std::cerr << "Buffered dataset yazilamadi: " << file_path << "\n";
        return false;
    }

    out << std::fixed << std::setprecision(8);
    out << "{\"type\":\"FeatureCollection\",\"features\":[";
    bool first_feature = true;
    for (const auto& feature : features) {
        if (feature.geometry.empty()) {
            continue;
        }
        if (!first_feature) {
            out << ",";
        }
        first_feature = false;

        out << "{\"type\":\"Feature\",\"properties\":{" << featureProperties(feature) << "},\"geometry\":";
        if (feature.geometry.size() == 1U) {
            out << "{\"type\":\"Polygon\",\"coordinates\":";
            writePolygon(out, feature.geometry.front(), feature.projection);
            out << "}";
        } else {
            out << "{\"type\":\"MultiPolygon\",\"coordinates\":[";
            for (std::size_t i = 0; i < feature.geometry.size(); ++i) {
                if (i > 0) {
                    out << ",";
                }
                writePolygon(out, feature.geometry[i], feature.projection);
            }
            out << "]}";
        }
        out << "}";
    }
    out << "]}";
    return true;
}

void stripPolygonHoles(MultiPolygon2& geometry) {
    for (auto& poly : geometry) {
        poly.inners().clear();
        bg::correct(poly);
    }
}

void simplifyMultiPolygon(MultiPolygon2& geometry, double simplify_km) {
    if (simplify_km <= 0.0) {
        return;
    }

    MultiPolygon2 simplified_geometry;
    simplified_geometry.reserve(geometry.size());
    for (const auto& poly : geometry) {
        Polygon2 simplified;
        bg::simplify(poly, simplified, simplify_km);
        bg::correct(simplified);
        if (simplified.outer().size() >= 4 && std::abs(bg::area(simplified)) > 1e-8 && bg::is_valid(simplified)) {
            simplified_geometry.push_back(std::move(simplified));
        } else {
            simplified_geometry.push_back(poly);
        }
    }
    geometry = std::move(simplified_geometry);
}

void filterSmallArea(MultiPolygon2& geometry, double min_area_km2) {
    if (min_area_km2 <= 0.0) {
        return;
    }

    MultiPolygon2 filtered;
    filtered.reserve(geometry.size());
    for (const auto& poly : geometry) {
        if (std::abs(bg::area(poly)) >= min_area_km2) {
            filtered.push_back(poly);
        }
    }
    geometry = std::move(filtered);
}

BuilderConfig parseArgs(int argc, char** argv) {
    BuilderConfig cfg;

    auto needValue = [&](int& i, const char* arg) -> const char* {
        if (i + 1 >= argc) {
            throw std::runtime_error(std::string(arg) + " icin deger eksik.");
        }
        return argv[++i];
    };

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            std::exit(0);
        }
        if (arg == "--input") {
            cfg.input_path = needValue(i, "--input");
            continue;
        }
        if (arg == "--output") {
            cfg.output_path = needValue(i, "--output");
            continue;
        }
        if (arg == "--output-dir") {
            cfg.output_dir = needValue(i, "--output-dir");
            continue;
        }
        if (arg == "--band-output") {
            cfg.band_output_path = needValue(i, "--band-output");
            continue;
        }
        if (arg == "--mesh-output") {
            cfg.mesh_output_path = needValue(i, "--mesh-output");
            continue;
        }
        if (arg == "--basename") {
            cfg.basename = needValue(i, "--basename");
            continue;
        }
        if (arg == "--offset-m") {
            double offset_m = 0.0;
            if (!parseDouble(needValue(i, "--offset-m"), offset_m) || offset_m < 0.0) {
                throw std::runtime_error("coastline_offset_m >= 0 olmali.");
            }
            cfg.offsets_m = {offset_m};
            continue;
        }
        if (arg == "--offsets-m") {
            cfg.offsets_m = parseOffsetList(needValue(i, "--offsets-m"));
            continue;
        }
        if (arg == "--band-range-m") {
            const auto [inner_m, outer_m] = parseBandRange(needValue(i, "--band-range-m"));
            cfg.single_band_mode = true;
            cfg.band_inner_m = inner_m;
            cfg.band_outer_m = outer_m;
            continue;
        }
        if (arg == "--simplify-m") {
            if (!parseDouble(needValue(i, "--simplify-m"), cfg.simplify_m) || cfg.simplify_m < -1.0) {
                throw std::runtime_error("--simplify-m >= -1 olmali.");
            }
            continue;
        }
        if (arg == "--mesh-post-simplify-m") {
            if (!parseDouble(needValue(i, "--mesh-post-simplify-m"), cfg.mesh_post_simplify_m) ||
                cfg.mesh_post_simplify_m < -1.0) {
                throw std::runtime_error("--mesh-post-simplify-m >= -1 olmali.");
            }
            continue;
        }
        if (arg == "--mesh-min-area-m2") {
            if (!parseDouble(needValue(i, "--mesh-min-area-m2"), cfg.mesh_min_area_m2) || cfg.mesh_min_area_m2 < 0.0) {
                throw std::runtime_error("--mesh-min-area-m2 >= 0 olmali.");
            }
            continue;
        }
        if (arg == "--min-lat") {
            if (!parseDouble(needValue(i, "--min-lat"), cfg.min_lat)) {
                throw std::runtime_error("--min-lat gecersiz.");
            }
            cfg.has_bounds = true;
            continue;
        }
        if (arg == "--max-lat") {
            if (!parseDouble(needValue(i, "--max-lat"), cfg.max_lat)) {
                throw std::runtime_error("--max-lat gecersiz.");
            }
            cfg.has_bounds = true;
            continue;
        }
        if (arg == "--min-lon") {
            if (!parseDouble(needValue(i, "--min-lon"), cfg.min_lon)) {
                throw std::runtime_error("--min-lon gecersiz.");
            }
            cfg.has_bounds = true;
            continue;
        }
        if (arg == "--max-lon") {
            if (!parseDouble(needValue(i, "--max-lon"), cfg.max_lon)) {
                throw std::runtime_error("--max-lon gecersiz.");
            }
            cfg.has_bounds = true;
            continue;
        }
        throw std::runtime_error("Bilinmeyen arguman: " + arg);
    }

    if (cfg.input_path.empty()) {
        throw std::runtime_error("--input zorunlu.");
    }
    if (cfg.offsets_m.empty() && !cfg.single_band_mode) {
        throw std::runtime_error("--offset-m veya --offsets-m zorunlu.");
    }
    if (cfg.basename.empty()) {
        cfg.basename = defaultBasename(cfg.input_path);
    }
    if (cfg.has_bounds &&
        !(cfg.min_lat < cfg.max_lat && cfg.min_lon < cfg.max_lon)) {
        throw std::runtime_error("Bounds gecersiz: min degerler max degerlerden kucuk olmali.");
    }

    if (cfg.single_band_mode && !cfg.offsets_m.empty()) {
        throw std::runtime_error("--band-range-m ile --offset-m/--offsets-m birlikte kullanilamaz.");
    }
    if (!cfg.mesh_output_path.empty() && cfg.offsets_m.size() != 1U) {
        throw std::runtime_error("--mesh-output icin tek bir --offset-m kullanin.");
    }

    if (cfg.single_band_mode) {
        if (cfg.band_output_path.empty()) {
            throw std::runtime_error("--band-range-m icin --band-output zorunlu.");
        }
        return cfg;
    }

    const bool single_offset_mode = cfg.offsets_m.size() == 1U;
    if (single_offset_mode && cfg.output_path.empty() && cfg.output_dir.empty() && cfg.band_output_path.empty() &&
        cfg.mesh_output_path.empty()) {
        throw std::runtime_error("Tek offset icin --output veya --output-dir veya --band-output veya --mesh-output verilmeli.");
    }
    if (!single_offset_mode && cfg.output_dir.empty() && cfg.band_output_path.empty()) {
        throw std::runtime_error("Coklu offset icin --output-dir veya --band-output verilmeli.");
    }

    return cfg;
}

std::vector<planner::LandPolygon> loadInputLand(const BuilderConfig& cfg) {
    std::vector<planner::LandPolygon> land;
    const bool ok = planner::loadLandPolygons(cfg.input_path, land);
    if (!ok) {
        throw std::runtime_error("Buffered coastline veri seti yuklenemedi: " + cfg.input_path);
    }
    if (cfg.has_bounds) {
        land = clipLandToBounds(land, cfg.min_lat, cfg.max_lat, cfg.min_lon, cfg.max_lon);
    }
    if (land.empty()) {
        throw std::runtime_error("Buffered coastline veri seti bos.");
    }
    return land;
}

std::vector<BufferedFeature> buildBufferedLandDataset(const std::vector<planner::LandPolygon>& land,
                                                      double offset_m,
                                                      double simplify_m) {
    std::vector<BufferedFeature> out;
    out.reserve(land.size());

    const double simplify_km = effectiveSimplifyMeters(simplify_m, offset_m) / 1000.0;
    const double offset_km = offset_m / 1000.0;

    for (const auto& poly : land) {
        const auto proj = planner::makeProjection(poly.min_lat, poly.max_lat, poly.min_lon, poly.max_lon);
        Polygon2 local_poly = landToPolygon(poly, proj, simplify_km);
        if (local_poly.outer().size() < 4 || std::abs(bg::area(local_poly)) <= 1e-8) {
            continue;
        }

        MultiPolygon2 geometry = offset_km > 0.0 ? computeBuffer(local_poly, offset_km) : singlePolygonMulti(local_poly);
        filterEmptyGeometry(geometry);
        if (geometry.empty()) {
            continue;
        }

        BufferedFeature feature;
        feature.projection = proj;
        feature.geometry = std::move(geometry);
        feature.outer_m = offset_m;
        out.push_back(std::move(feature));
    }

    return out;
}

std::vector<BufferedFeature> buildSeaBandDataset(const std::vector<planner::LandPolygon>& land,
                                                 double inner_m,
                                                 double outer_m,
                                                 double simplify_m) {
    std::vector<BufferedFeature> out;
    out.reserve(land.size());

    const double simplify_km = effectiveSimplifyMeters(simplify_m, outer_m) / 1000.0;
    const double outer_km = outer_m / 1000.0;
    const double inner_km = inner_m / 1000.0;

    for (const auto& poly : land) {
        const auto proj = planner::makeProjection(poly.min_lat, poly.max_lat, poly.min_lon, poly.max_lon);
        Polygon2 local_poly = landToPolygon(poly, proj, simplify_km);
        if (local_poly.outer().size() < 4 || std::abs(bg::area(local_poly)) <= 1e-8) {
            continue;
        }

        MultiPolygon2 geometry = buildSeaBand(local_poly, inner_km, outer_km);
        filterEmptyGeometry(geometry);
        if (geometry.empty()) {
            continue;
        }

        BufferedFeature feature;
        feature.projection = proj;
        feature.geometry = std::move(geometry);
        feature.inner_m = inner_m;
        feature.outer_m = outer_m;
        feature.zone = "sea_band";
        out.push_back(std::move(feature));
    }

    return out;
}

std::vector<BufferedFeature> buildMeshLandDataset(const std::vector<planner::LandPolygon>& land,
                                                  double offset_m,
                                                  double simplify_m,
                                                  double mesh_post_simplify_m,
                                                  double mesh_min_area_m2) {
    std::vector<BufferedFeature> out;
    out.reserve(land.size());

    const double simplify_km = effectiveSimplifyMeters(simplify_m, offset_m) / 1000.0;
    const double offset_km = offset_m / 1000.0;
    const double post_simplify_km = effectiveMeshPostSimplifyMeters(mesh_post_simplify_m, offset_m) / 1000.0;
    const double min_area_km2 = mesh_min_area_m2 / 1'000'000.0;

    for (const auto& poly : land) {
        const auto proj = planner::makeProjection(poly.min_lat, poly.max_lat, poly.min_lon, poly.max_lon);
        Polygon2 local_poly = landToPolygon(poly, proj, simplify_km);
        if (local_poly.outer().size() < 4 || std::abs(bg::area(local_poly)) <= 1e-8) {
            continue;
        }

        MultiPolygon2 geometry = offset_km > 0.0 ? computeBuffer(local_poly, offset_km) : singlePolygonMulti(local_poly);
        stripPolygonHoles(geometry);
        simplifyMultiPolygon(geometry, post_simplify_km);
        filterEmptyGeometry(geometry);
        filterSmallArea(geometry, min_area_km2);
        filterEmptyGeometry(geometry);
        if (geometry.empty()) {
            continue;
        }

        BufferedFeature feature;
        feature.projection = proj;
        feature.geometry = std::move(geometry);
        feature.outer_m = offset_m;
        feature.zone = "mesh_buffered_land";
        out.push_back(std::move(feature));
    }

    return out;
}

std::string cumulativeOutputPath(const BuilderConfig& cfg, double offset_m) {
    if (!cfg.output_path.empty() && cfg.offsets_m.size() == 1U) {
        return cfg.output_path;
    }
    return (std::filesystem::path(cfg.output_dir) /
            (cfg.basename + "_buffer_" + formatMeters(offset_m) + ".geojson"))
        .string();
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const BuilderConfig cfg = parseArgs(argc, argv);
        const std::vector<planner::LandPolygon> land = loadInputLand(cfg);

        if (cfg.single_band_mode) {
            std::vector<BufferedFeature> sea_band =
                buildSeaBandDataset(land, cfg.band_inner_m, cfg.band_outer_m, cfg.simplify_m);
            if (sea_band.empty()) {
                throw std::runtime_error("Sea-band dataset bos olustu.");
            }
            if (!writeFeatureCollection(cfg.band_output_path, sea_band)) {
                throw std::runtime_error("Sea-band dataset yazilamadi: " + cfg.band_output_path);
            }
            std::cout << "Sea-band dataset hazir: " << cfg.band_output_path << "\n";
            return 0;
        }

        if (!cfg.output_dir.empty() || !cfg.output_path.empty()) {
            for (double offset_m : cfg.offsets_m) {
                std::vector<BufferedFeature> buffered = buildBufferedLandDataset(land, offset_m, cfg.simplify_m);
                if (buffered.empty()) {
                    throw std::runtime_error("Buffered dataset bos olustu.");
                }

                const std::string output_path = cumulativeOutputPath(cfg, offset_m);
                if (!writeFeatureCollection(output_path, buffered)) {
                    throw std::runtime_error("Buffered dataset yazilamadi: " + output_path);
                }
                std::cout << "Buffered dataset hazir: " << output_path << "\n";
            }
        }

        if (!cfg.band_output_path.empty()) {
            std::vector<BufferedFeature> sea_bands;
            for (std::size_t i = 0; i < cfg.offsets_m.size(); ++i) {
                const double inner_m = (i == 0) ? 0.0 : cfg.offsets_m[i - 1];
                const double outer_m = cfg.offsets_m[i];
                std::vector<BufferedFeature> band = buildSeaBandDataset(land, inner_m, outer_m, cfg.simplify_m);
                sea_bands.insert(sea_bands.end(), band.begin(), band.end());
            }
            if (sea_bands.empty()) {
                throw std::runtime_error("Sea-band dataset bos olustu.");
            }
            if (!writeFeatureCollection(cfg.band_output_path, sea_bands)) {
                throw std::runtime_error("Sea-band dataset yazilamadi: " + cfg.band_output_path);
            }
            std::cout << "Sea-band dataset hazir: " << cfg.band_output_path << "\n";
        }

        if (!cfg.mesh_output_path.empty()) {
            std::vector<BufferedFeature> mesh_buffered = buildMeshLandDataset(land,
                                                                              cfg.offsets_m.front(),
                                                                              cfg.simplify_m,
                                                                              cfg.mesh_post_simplify_m,
                                                                              cfg.mesh_min_area_m2);
            if (mesh_buffered.empty()) {
                throw std::runtime_error("Mesh buffered dataset bos olustu.");
            }
            if (!writeFeatureCollection(cfg.mesh_output_path, mesh_buffered)) {
                throw std::runtime_error("Mesh buffered dataset yazilamadi: " + cfg.mesh_output_path);
            }
            std::cout << "Mesh buffered dataset hazir: " << cfg.mesh_output_path << "\n";
        }
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << "\n";
        printUsage(argv[0]);
        return 1;
    }

    return 0;
}
