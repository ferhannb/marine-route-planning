#include "planner.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <rapidjson/document.h>
#include <rapidjson/error/en.h>
#include <unordered_set>

namespace planner {
namespace {

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
        if (std::fabs(first.lat_deg - last.lat_deg) < 1e-12 &&
            std::fabs(first.lon_deg - last.lon_deg) < 1e-12) {
            vertices.pop_back();
        }
    }
    if (vertices.size() < 3) {
        return false;
    }

    out_polygon = makeLandPolygon(std::move(vertices));
    return true;
}

bool parseLatLonCoord(const rapidjson::Value& coord, LatLon& out) {
    if (!coord.IsArray() || coord.Size() < 2 || !coord[0].IsNumber() || !coord[1].IsNumber()) {
        return false;
    }
    out = {coord[1].GetDouble(), coord[0].GetDouble()};
    return true;
}

bool parseLineStringCoords(const rapidjson::Value& arr, std::vector<LatLon>& points) {
    if (!arr.IsArray() || arr.Size() < 2) {
        return false;
    }
    points.clear();
    points.reserve(arr.Size());
    for (rapidjson::SizeType i = 0; i < arr.Size(); ++i) {
        LatLon p{};
        if (!parseLatLonCoord(arr[i], p)) {
            return false;
        }
        points.push_back(p);
    }
    return points.size() >= 2;
}

bool parseJsonNumberLike(const rapidjson::Value& value, double& out) {
    if (value.IsNumber()) {
        out = value.GetDouble();
        return std::isfinite(out);
    }
    if (!value.IsString()) {
        return false;
    }

    const char* text = value.GetString();
    char* endptr = nullptr;
    const double parsed = std::strtod(text, &endptr);
    if (endptr == text || !std::isfinite(parsed)) {
        return false;
    }
    while (*endptr != '\0') {
        if (!std::isspace(static_cast<unsigned char>(*endptr))) {
            return false;
        }
        ++endptr;
    }
    out = parsed;
    return true;
}

double normalizeDepthMeters(double depth_value) {
    return std::fabs(depth_value);
}

TssFeature makeTssFeature(std::vector<LatLon> points, bool closed, std::string seamark_type) {
    TssFeature f;
    f.points = std::move(points);
    f.closed = closed;
    f.seamark_type = std::move(seamark_type);
    f.min_lat = std::numeric_limits<double>::infinity();
    f.max_lat = -std::numeric_limits<double>::infinity();
    f.min_lon = std::numeric_limits<double>::infinity();
    f.max_lon = -std::numeric_limits<double>::infinity();
    for (const auto& p : f.points) {
        f.min_lat = std::min(f.min_lat, p.lat_deg);
        f.max_lat = std::max(f.max_lat, p.lat_deg);
        f.min_lon = std::min(f.min_lon, p.lon_deg);
        f.max_lon = std::max(f.max_lon, p.lon_deg);
    }
    return f;
}

BathymetryFeature makeBathymetryFeature(std::vector<LatLon> points, bool closed, double depth_m) {
    BathymetryFeature f;
    f.points = std::move(points);
    f.closed = closed;
    f.depth_m = depth_m;
    f.min_lat = std::numeric_limits<double>::infinity();
    f.max_lat = -std::numeric_limits<double>::infinity();
    f.min_lon = std::numeric_limits<double>::infinity();
    f.max_lon = -std::numeric_limits<double>::infinity();
    for (const auto& p : f.points) {
        f.min_lat = std::min(f.min_lat, p.lat_deg);
        f.max_lat = std::max(f.max_lat, p.lat_deg);
        f.min_lon = std::min(f.min_lon, p.lon_deg);
        f.max_lon = std::max(f.max_lon, p.lon_deg);
    }
    return f;
}

bool extractBathymetryDepthMeters(const rapidjson::Value& feature, double& out_depth_m) {
    if (!feature.HasMember("properties") || !feature["properties"].IsObject()) {
        return false;
    }

    const auto& props = feature["properties"];
    static const char* kDepthKeys[] = {
        "depth_m",
        "depth",
        "min_depth_m",
        "max_depth_m",
        "depth_min_m",
        "depth_max_m",
        "elevation",
    };

    for (const char* key : kDepthKeys) {
        if (!props.HasMember(key)) {
            continue;
        }
        double parsed = 0.0;
        if (parseJsonNumberLike(props[key], parsed)) {
            out_depth_m = normalizeDepthMeters(parsed);
            return true;
        }
    }
    return false;
}

std::uint64_t edgeKey(int a, int b) {
    const std::uint32_t u = static_cast<std::uint32_t>(std::min(a, b));
    const std::uint32_t v = static_cast<std::uint32_t>(std::max(a, b));
    return (static_cast<std::uint64_t>(u) << 32U) | v;
}

std::uint32_t readBe32(std::istream& in, bool& ok) {
    unsigned char bytes[4] = {};
    in.read(reinterpret_cast<char*>(bytes), sizeof(bytes));
    ok = in.good();
    if (!ok) {
        return 0;
    }
    return (static_cast<std::uint32_t>(bytes[0]) << 24U) | (static_cast<std::uint32_t>(bytes[1]) << 16U) |
           (static_cast<std::uint32_t>(bytes[2]) << 8U) | static_cast<std::uint32_t>(bytes[3]);
}

std::uint32_t readLe32(std::istream& in, bool& ok) {
    unsigned char bytes[4] = {};
    in.read(reinterpret_cast<char*>(bytes), sizeof(bytes));
    ok = in.good();
    if (!ok) {
        return 0;
    }
    return (static_cast<std::uint32_t>(bytes[3]) << 24U) | (static_cast<std::uint32_t>(bytes[2]) << 16U) |
           (static_cast<std::uint32_t>(bytes[1]) << 8U) | static_cast<std::uint32_t>(bytes[0]);
}

double readLe64(std::istream& in, bool& ok) {
    unsigned char bytes[8] = {};
    in.read(reinterpret_cast<char*>(bytes), sizeof(bytes));
    ok = in.good();
    if (!ok) {
        return 0.0;
    }
    std::uint64_t value = 0;
    for (int i = 7; i >= 0; --i) {
        value = (value << 8U) | static_cast<std::uint64_t>(bytes[i]);
    }
    double out = 0.0;
    std::memcpy(&out, &value, sizeof(out));
    return out;
}

double signedRingArea2(const std::vector<LatLon>& ring) {
    if (ring.size() < 3) {
        return 0.0;
    }
    double area2 = 0.0;
    for (size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++) {
        area2 += ring[j].lon_deg * ring[i].lat_deg - ring[i].lon_deg * ring[j].lat_deg;
    }
    return area2;
}

bool isExteriorRing(const std::vector<LatLon>& ring) {
    return signedRingArea2(ring) <= 0.0;
}

bool pathEndsWithInsensitive(const std::string& path, const std::string& suffix) {
    if (path.size() < suffix.size()) {
        return false;
    }
    for (size_t i = 0; i < suffix.size(); ++i) {
        const char a =
            static_cast<char>(std::tolower(static_cast<unsigned char>(path[path.size() - suffix.size() + i])));
        const char b = static_cast<char>(std::tolower(static_cast<unsigned char>(suffix[i])));
        if (a != b) {
            return false;
        }
    }
    return true;
}

bool parseAsciiBathymetryRaster(const std::string& file_path, std::vector<BathymetryFeature>& out_features) {
    std::ifstream in(file_path);
    if (!in) {
        std::cerr << "Bathymetri ASCII raster acilamadi: " << file_path << "\n";
        return false;
    }

    std::string key;
    int ncols = 0;
    int nrows = 0;
    double xll = 0.0;
    double yll = 0.0;
    double cellsize = 0.0;
    double nodata = 999999.0;
    bool x_is_center = false;
    bool y_is_center = false;

    auto readHeaderValue = [&](const char* expected, auto& out) {
        if (!(in >> key >> out)) {
            std::cerr << "Bathymetri ASCII header okunamadi: " << file_path << "\n";
            return false;
        }
        std::string lowered = key;
        std::transform(lowered.begin(),
                       lowered.end(),
                       lowered.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (lowered != expected) {
            std::cerr << "Bathymetri ASCII header beklenenden farkli: " << key << "\n";
            return false;
        }
        return true;
    };

    if (!readHeaderValue("ncols", ncols) || !readHeaderValue("nrows", nrows)) {
        return false;
    }
    if (!(in >> key >> xll)) {
        return false;
    }
    {
        std::string lowered = key;
        std::transform(lowered.begin(),
                       lowered.end(),
                       lowered.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (lowered == "xllcenter") {
            x_is_center = true;
        } else if (lowered != "xllcorner") {
            std::cerr << "Bathymetri ASCII xll header beklenenden farkli: " << key << "\n";
            return false;
        }
    }
    if (!(in >> key >> yll)) {
        return false;
    }
    {
        std::string lowered = key;
        std::transform(lowered.begin(),
                       lowered.end(),
                       lowered.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (lowered == "yllcenter") {
            y_is_center = true;
        } else if (lowered != "yllcorner") {
            std::cerr << "Bathymetri ASCII yll header beklenenden farkli: " << key << "\n";
            return false;
        }
    }
    if (!readHeaderValue("cellsize", cellsize)) {
        return false;
    }

    std::streampos after_core_header = in.tellg();
    if (in >> key) {
        std::string lowered = key;
        std::transform(lowered.begin(),
                       lowered.end(),
                       lowered.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (lowered == "nodata_value") {
            if (!(in >> nodata)) {
                std::cerr << "Bathymetri ASCII nodata degeri okunamadi: " << file_path << "\n";
                return false;
            }
        } else {
            in.clear();
            in.seekg(after_core_header);
        }
    } else {
        in.clear();
        in.seekg(after_core_header);
    }

    if (ncols <= 0 || nrows <= 0 || !std::isfinite(cellsize) || cellsize <= 0.0) {
        std::cerr << "Bathymetri ASCII header gecersiz: " << file_path << "\n";
        return false;
    }

    constexpr std::size_t kMaxOutputCells = 120000;
    const std::size_t total_cells = static_cast<std::size_t>(ncols) * static_cast<std::size_t>(nrows);
    int block = 1;
    while ((static_cast<std::size_t>((ncols + block - 1) / block) * static_cast<std::size_t>((nrows + block - 1) / block)) >
           kMaxOutputCells) {
        block += 1;
    }

    const int block_cols = (ncols + block - 1) / block;
    const int block_rows = (nrows + block - 1) / block;
    std::vector<double> sums(static_cast<std::size_t>(block_cols) * static_cast<std::size_t>(block_rows), 0.0);
    std::vector<int> counts(static_cast<std::size_t>(block_cols) * static_cast<std::size_t>(block_rows), 0);

    for (int row = 0; row < nrows; ++row) {
        for (int col = 0; col < ncols; ++col) {
            double value = nodata;
            if (!(in >> value)) {
                std::cerr << "Bathymetri ASCII hucre verisi okunamadi: " << file_path << "\n";
                return false;
            }
            if (!std::isfinite(value) || std::fabs(value - nodata) < 1e-12 || value >= 0.0) {
                continue;
            }
            const int br = row / block;
            const int bc = col / block;
            const std::size_t idx = static_cast<std::size_t>(br) * static_cast<std::size_t>(block_cols) + bc;
            sums[idx] += std::fabs(value);
            counts[idx] += 1;
        }
    }

    const double x_corner = x_is_center ? (xll - 0.5 * cellsize) : xll;
    const double y_corner = y_is_center ? (yll - 0.5 * cellsize) : yll;

    out_features.clear();
    out_features.reserve(kMaxOutputCells);
    for (int br = 0; br < block_rows; ++br) {
        const int row_start = br * block;
        const int row_end = std::min(nrows, row_start + block);
        for (int bc = 0; bc < block_cols; ++bc) {
            const std::size_t idx = static_cast<std::size_t>(br) * static_cast<std::size_t>(block_cols) + bc;
            if (counts[idx] == 0) {
                continue;
            }
            const int col_start = bc * block;
            const int col_end = std::min(ncols, col_start + block);

            const double min_lon = x_corner + static_cast<double>(col_start) * cellsize;
            const double max_lon = x_corner + static_cast<double>(col_end) * cellsize;
            const double max_lat = y_corner + static_cast<double>(nrows - row_start) * cellsize;
            const double min_lat = y_corner + static_cast<double>(nrows - row_end) * cellsize;

            std::vector<LatLon> ring = {
                {min_lat, min_lon},
                {min_lat, max_lon},
                {max_lat, max_lon},
                {max_lat, min_lon},
            };
            out_features.push_back(makeBathymetryFeature(std::move(ring), true, sums[idx] / counts[idx]));
        }
    }

    std::cout << "Yuklenen bathymetri feature sayisi: " << out_features.size() << "\n";
    if (block > 1) {
        std::cout << "Bathymetri ASCII raster block aggregate: " << block << "x" << block
                  << " (ham hucre: " << total_cells << ")\n";
    }
    return !out_features.empty();
}

bool loadLandPolygonsShapefile(const std::string& file_path,
                               std::vector<LandPolygon>& polygons_out,
                               bool use_bounds,
                               double min_lat,
                               double max_lat,
                               double min_lon,
                               double max_lon) {
    std::ifstream in(file_path, std::ios::binary);
    if (!in) {
        std::cerr << "Shapefile acilamadi: " << file_path << "\n";
        return false;
    }

    bool ok = true;
    const std::uint32_t file_code = readBe32(in, ok);
    if (!ok || file_code != 9994U) {
        std::cerr << "Gecersiz shapefile basligi: " << file_path << "\n";
        return false;
    }

    for (int i = 0; i < 5; ++i) {
        readBe32(in, ok);
    }
    readBe32(in, ok);
    readLe32(in, ok);
    readLe32(in, ok);
    readLe64(in, ok);
    readLe64(in, ok);
    readLe64(in, ok);
    readLe64(in, ok);
    in.seekg(100, std::ios::beg);
    if (!ok || !in.good()) {
        std::cerr << "Shapefile header okunamadi: " << file_path << "\n";
        return false;
    }

    polygons_out.clear();
    while (true) {
        const std::uint32_t record_number = readBe32(in, ok);
        if (!ok) {
            if (in.eof()) {
                break;
            }
            std::cerr << "Shapefile record basligi okunamadi: " << file_path << "\n";
            return false;
        }
        (void)record_number;
        const std::uint32_t content_words = readBe32(in, ok);
        if (!ok) {
            std::cerr << "Shapefile record uzunlugu okunamadi: " << file_path << "\n";
            return false;
        }
        const std::streampos content_start = in.tellg();
        const std::streampos content_end = content_start + static_cast<std::streamoff>(content_words) * 2;

        const std::uint32_t shape_type = readLe32(in, ok);
        if (!ok) {
            std::cerr << "Shapefile shape type okunamadi: " << file_path << "\n";
            return false;
        }
        if (shape_type == 0U) {
            in.seekg(content_end, std::ios::beg);
            continue;
        }
        if (shape_type != 5U && shape_type != 15U && shape_type != 25U) {
            in.seekg(content_end, std::ios::beg);
            continue;
        }

        const double shp_min_lon = readLe64(in, ok);
        const double shp_min_lat = readLe64(in, ok);
        const double shp_max_lon = readLe64(in, ok);
        const double shp_max_lat = readLe64(in, ok);
        const std::uint32_t num_parts = readLe32(in, ok);
        const std::uint32_t num_points = readLe32(in, ok);
        if (!ok || num_parts == 0U || num_points < 4U) {
            in.seekg(content_end, std::ios::beg);
            continue;
        }

        if (use_bounds &&
            !bboxesOverlap(min_lat, max_lat, min_lon, max_lon, shp_min_lat, shp_max_lat, shp_min_lon, shp_max_lon)) {
            in.seekg(content_end, std::ios::beg);
            continue;
        }

        std::vector<std::uint32_t> parts(num_parts);
        for (std::uint32_t i = 0; i < num_parts; ++i) {
            parts[i] = readLe32(in, ok);
        }
        if (!ok) {
            std::cerr << "Shapefile parts okunamadi: " << file_path << "\n";
            return false;
        }

        std::vector<LatLon> points;
        points.reserve(num_points);
        for (std::uint32_t i = 0; i < num_points; ++i) {
            const double lon = readLe64(in, ok);
            const double lat = readLe64(in, ok);
            if (!ok) {
                std::cerr << "Shapefile point verisi okunamadi: " << file_path << "\n";
                return false;
            }
            points.push_back({lat, lon});
        }

        for (std::uint32_t part_index = 0; part_index < num_parts; ++part_index) {
            const std::uint32_t start = parts[part_index];
            const std::uint32_t end = (part_index + 1U < num_parts) ? parts[part_index + 1U] : num_points;
            if (start >= end || end > num_points || (end - start) < 4U) {
                continue;
            }

            std::vector<LatLon> ring;
            ring.reserve(end - start);
            for (std::uint32_t i = start; i < end; ++i) {
                ring.push_back(points[i]);
            }
            if (ring.size() >= 2U) {
                const auto& first = ring.front();
                const auto& last = ring.back();
                if (std::fabs(first.lat_deg - last.lat_deg) < 1e-12 &&
                    std::fabs(first.lon_deg - last.lon_deg) < 1e-12) {
                    ring.pop_back();
                }
            }
            if (ring.size() < 3U || !isExteriorRing(ring)) {
                continue;
            }
            polygons_out.push_back(makeLandPolygon(std::move(ring)));
        }

        in.seekg(content_end, std::ios::beg);
    }

    std::cout << "Yuklenen kara poligonu sayisi: " << polygons_out.size() << "\n";
    return !polygons_out.empty();
}

}  // namespace

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
        std::cerr << "GeoJSON parse hatasi: " << rapidjson::GetParseError_En(doc.GetParseError())
                  << " (offset " << doc.GetErrorOffset() << ")\n";
        return false;
    }

    if (!doc.IsObject() || !doc.HasMember("features") || !doc["features"].IsArray()) {
        std::cerr << "GeoJSON formati beklenen FeatureCollection degil.\n";
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

    std::cout << "Yuklenen kara poligonu sayisi: " << polygons_out.size() << "\n";
    return !polygons_out.empty();
}

bool loadLandPolygons(const std::string& file_path, std::vector<LandPolygon>& polygons_out) {
    if (pathEndsWithInsensitive(file_path, ".shp")) {
        return loadLandPolygonsShapefile(file_path, polygons_out, false, 0.0, 0.0, 0.0, 0.0);
    }
    return loadLandPolygonsGeoJson(file_path, polygons_out);
}

bool loadLandPolygons(const std::string& file_path,
                      std::vector<LandPolygon>& polygons_out,
                      double min_lat,
                      double max_lat,
                      double min_lon,
                      double max_lon) {
    if (pathEndsWithInsensitive(file_path, ".shp")) {
        return loadLandPolygonsShapefile(file_path, polygons_out, true, min_lat, max_lat, min_lon, max_lon);
    }
    if (!loadLandPolygonsGeoJson(file_path, polygons_out)) {
        return false;
    }
    polygons_out = filterPolygonsByBounds(polygons_out, min_lat, max_lat, min_lon, max_lon);
    return !polygons_out.empty();
}

std::vector<TssFeature> filterTssByBounds(const std::vector<TssFeature>& features,
                                          double min_lat,
                                          double max_lat,
                                          double min_lon,
                                          double max_lon) {
    std::vector<TssFeature> out;
    out.reserve(features.size());
    for (const auto& f : features) {
        if (bboxesOverlap(min_lat, max_lat, min_lon, max_lon, f.min_lat, f.max_lat, f.min_lon, f.max_lon)) {
            out.push_back(f);
        }
    }
    return out;
}

bool loadTssFeaturesGeoJson(const std::string& file_path, std::vector<TssFeature>& out_features) {
    std::ifstream in(file_path);
    if (!in) {
        std::cerr << "TSS GeoJSON acilamadi: " << file_path << "\n";
        return false;
    }

    const std::string data((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    if (data.empty()) {
        std::cerr << "TSS GeoJSON bos: " << file_path << "\n";
        return false;
    }

    rapidjson::Document doc;
    doc.Parse(data.c_str());
    if (doc.HasParseError()) {
        std::cerr << "TSS GeoJSON parse hatasi: " << rapidjson::GetParseError_En(doc.GetParseError())
                  << " (offset " << doc.GetErrorOffset() << ")\n";
        return false;
    }

    if (!doc.IsObject() || !doc.HasMember("features") || !doc["features"].IsArray()) {
        std::cerr << "TSS GeoJSON formati beklenen FeatureCollection degil.\n";
        return false;
    }

    out_features.clear();
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

        std::string seamark_type = "tss";
        if (feature.HasMember("properties") && feature["properties"].IsObject()) {
            const auto& props = feature["properties"];
            if (props.HasMember("seamark:type") && props["seamark:type"].IsString()) {
                seamark_type = props["seamark:type"].GetString();
            } else if (props.HasMember("seamark_type") && props["seamark_type"].IsString()) {
                seamark_type = props["seamark_type"].GetString();
            }
        }

        const std::string type = geom["type"].GetString();
        const auto& coords = geom["coordinates"];
        if (type == "LineString") {
            std::vector<LatLon> pts;
            if (parseLineStringCoords(coords, pts)) {
                out_features.push_back(makeTssFeature(std::move(pts), false, seamark_type));
            }
        } else if (type == "MultiLineString") {
            if (!coords.IsArray()) {
                continue;
            }
            for (rapidjson::SizeType j = 0; j < coords.Size(); ++j) {
                std::vector<LatLon> pts;
                if (parseLineStringCoords(coords[j], pts)) {
                    out_features.push_back(makeTssFeature(std::move(pts), false, seamark_type));
                }
            }
        } else if (type == "Polygon") {
            if (!coords.IsArray() || coords.Empty()) {
                continue;
            }
            std::vector<LatLon> pts;
            if (parseLineStringCoords(coords[0], pts)) {
                if (pts.size() >= 2 && std::fabs(pts.front().lat_deg - pts.back().lat_deg) < 1e-12 &&
                    std::fabs(pts.front().lon_deg - pts.back().lon_deg) < 1e-12) {
                    pts.pop_back();
                }
                if (pts.size() >= 3) {
                    out_features.push_back(makeTssFeature(std::move(pts), true, seamark_type));
                }
            }
        } else if (type == "MultiPolygon") {
            if (!coords.IsArray()) {
                continue;
            }
            for (rapidjson::SizeType j = 0; j < coords.Size(); ++j) {
                const auto& poly = coords[j];
                if (!poly.IsArray() || poly.Empty()) {
                    continue;
                }
                std::vector<LatLon> pts;
                if (parseLineStringCoords(poly[0], pts)) {
                    if (pts.size() >= 2 && std::fabs(pts.front().lat_deg - pts.back().lat_deg) < 1e-12 &&
                        std::fabs(pts.front().lon_deg - pts.back().lon_deg) < 1e-12) {
                        pts.pop_back();
                    }
                    if (pts.size() >= 3) {
                        out_features.push_back(makeTssFeature(std::move(pts), true, seamark_type));
                    }
                }
            }
        }
    }

    std::cout << "Yuklenen TSS feature sayisi: " << out_features.size() << "\n";
    return true;
}

std::vector<BathymetryFeature> filterBathymetryByBounds(const std::vector<BathymetryFeature>& features,
                                                        double min_lat,
                                                        double max_lat,
                                                        double min_lon,
                                                        double max_lon) {
    std::vector<BathymetryFeature> out;
    out.reserve(features.size());
    for (const auto& f : features) {
        if (bboxesOverlap(min_lat, max_lat, min_lon, max_lon, f.min_lat, f.max_lat, f.min_lon, f.max_lon)) {
            out.push_back(f);
        }
    }
    return out;
}

bool loadBathymetryFeaturesGeoJson(const std::string& file_path, std::vector<BathymetryFeature>& out_features) {
    std::ifstream in(file_path);
    if (!in) {
        std::cerr << "Bathymetri GeoJSON acilamadi: " << file_path << "\n";
        return false;
    }

    const std::string data((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    if (data.empty()) {
        std::cerr << "Bathymetri GeoJSON bos: " << file_path << "\n";
        return false;
    }

    rapidjson::Document doc;
    doc.Parse(data.c_str());
    if (doc.HasParseError()) {
        std::cerr << "Bathymetri GeoJSON parse hatasi: " << rapidjson::GetParseError_En(doc.GetParseError())
                  << " (offset " << doc.GetErrorOffset() << ")\n";
        return false;
    }

    if (!doc.IsObject() || !doc.HasMember("features") || !doc["features"].IsArray()) {
        std::cerr << "Bathymetri GeoJSON formati beklenen FeatureCollection degil.\n";
        return false;
    }

    out_features.clear();
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

        double depth_m = std::numeric_limits<double>::quiet_NaN();
        extractBathymetryDepthMeters(feature, depth_m);

        const std::string type = geom["type"].GetString();
        const auto& coords = geom["coordinates"];
        if (type == "LineString") {
            std::vector<LatLon> pts;
            if (parseLineStringCoords(coords, pts)) {
                out_features.push_back(makeBathymetryFeature(std::move(pts), false, depth_m));
            }
        } else if (type == "MultiLineString") {
            if (!coords.IsArray()) {
                continue;
            }
            for (rapidjson::SizeType j = 0; j < coords.Size(); ++j) {
                std::vector<LatLon> pts;
                if (parseLineStringCoords(coords[j], pts)) {
                    out_features.push_back(makeBathymetryFeature(std::move(pts), false, depth_m));
                }
            }
        } else if (type == "Polygon") {
            if (!coords.IsArray() || coords.Empty()) {
                continue;
            }
            std::vector<LatLon> pts;
            if (parseLineStringCoords(coords[0], pts)) {
                if (pts.size() >= 2 && std::fabs(pts.front().lat_deg - pts.back().lat_deg) < 1e-12 &&
                    std::fabs(pts.front().lon_deg - pts.back().lon_deg) < 1e-12) {
                    pts.pop_back();
                }
                if (pts.size() >= 3) {
                    out_features.push_back(makeBathymetryFeature(std::move(pts), true, depth_m));
                }
            }
        } else if (type == "MultiPolygon") {
            if (!coords.IsArray()) {
                continue;
            }
            for (rapidjson::SizeType j = 0; j < coords.Size(); ++j) {
                const auto& poly = coords[j];
                if (!poly.IsArray() || poly.Empty()) {
                    continue;
                }
                std::vector<LatLon> pts;
                if (parseLineStringCoords(poly[0], pts)) {
                    if (pts.size() >= 2 && std::fabs(pts.front().lat_deg - pts.back().lat_deg) < 1e-12 &&
                        std::fabs(pts.front().lon_deg - pts.back().lon_deg) < 1e-12) {
                        pts.pop_back();
                    }
                    if (pts.size() >= 3) {
                        out_features.push_back(makeBathymetryFeature(std::move(pts), true, depth_m));
                    }
                }
            }
        }
    }

    std::cout << "Yuklenen bathymetri feature sayisi: " << out_features.size() << "\n";
    return true;
}

bool loadBathymetryFeatures(const std::string& file_path, std::vector<BathymetryFeature>& out_features) {
    if (pathEndsWithInsensitive(file_path, ".asc") || pathEndsWithInsensitive(file_path, ".ascii")) {
        return parseAsciiBathymetryRaster(file_path, out_features);
    }
    return loadBathymetryFeaturesGeoJson(file_path, out_features);
}

void writeSvgPlot(const std::string& file_path,
                  const std::vector<NamedPoint>& vertices,
                  const std::vector<Triangle>& triangles,
                  const std::vector<int>& tri_sequence,
                  const std::vector<LatLon>& route,
                  const std::vector<LandPolygon>& land_polygons,
                  const std::vector<TssFeature>& tss_features,
                  const std::vector<BathymetryFeature>& bathymetry_features,
                  const std::vector<BufferZonePolygon>& buffer_zones) {
    constexpr double width = 1400.0;
    constexpr double height = 900.0;
    constexpr double margin = 70.0;

    double min_lon = std::numeric_limits<double>::infinity();
    double max_lon = -std::numeric_limits<double>::infinity();
    double min_lat = std::numeric_limits<double>::infinity();
    double max_lat = -std::numeric_limits<double>::infinity();

    auto extendBounds = [&](double lat, double lon) {
        min_lon = std::min(min_lon, lon);
        max_lon = std::max(max_lon, lon);
        min_lat = std::min(min_lat, lat);
        max_lat = std::max(max_lat, lat);
    };

    for (const auto& p : vertices) {
        extendBounds(p.geo.lat_deg, p.geo.lon_deg);
    }
    for (const auto& p : route) {
        extendBounds(p.lat_deg, p.lon_deg);
    }
    for (const auto& poly : land_polygons) {
        extendBounds(poly.min_lat, poly.min_lon);
        extendBounds(poly.max_lat, poly.max_lon);
    }
    for (const auto& f : tss_features) {
        for (const auto& p : f.points) {
            extendBounds(p.lat_deg, p.lon_deg);
        }
    }
    for (const auto& f : bathymetry_features) {
        for (const auto& p : f.points) {
            extendBounds(p.lat_deg, p.lon_deg);
        }
    }
    for (const auto& zone : buffer_zones) {
        extendBounds(zone.polygon.min_lat, zone.polygon.min_lon);
        extendBounds(zone.polygon.max_lat, zone.polygon.max_lon);
    }

    const double lon_span = std::max(max_lon - min_lon, 1.0);
    const double lat_span = std::max(max_lat - min_lat, 1.0);
    min_lon -= lon_span * 0.06;
    max_lon += lon_span * 0.06;
    min_lat -= lat_span * 0.06;
    max_lat += lat_span * 0.06;

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

    const bool has_navigation_overlay = !triangles.empty() || !tri_sequence.empty() || !route.empty();
    const bool is_mpc_svg = file_path.find("mpc") != std::string::npos;
    const bool dense_land_dataset = land_polygons.size() > 5000;
    // Navigation SVG'lerinde triangulation ustunde gercek coastline'i gorebilmek icin
    // kara polygon vertex'lerini artik piksel bazinda seyreltmiyoruz.
    const double land_min_point_step_px = 0.0;
    const double land_min_area_px2 = 0.0;
    const double land_fill_opacity = has_navigation_overlay ? (dense_land_dataset ? 0.30 : 0.35) : 1.0;

    std::ofstream out(file_path);
    if (!out) {
        std::cerr << "SVG dosyasi olusturulamadi: " << file_path << "\n";
        return;
    }

    out << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width << "\" height=\"" << height
        << "\" viewBox=\"0 0 " << width << " " << height << "\">\n";
    out << "<rect x=\"0\" y=\"0\" width=\"" << width << "\" height=\"" << height
        << "\" fill=\"#f4f8fc\"/>\n";
    out << "<rect x=\"" << ox << "\" y=\"" << oy << "\" width=\"" << plot_w << "\" height=\"" << plot_h
        << "\" fill=\"#eef3f9\" stroke=\"#95a3b3\" stroke-width=\"1.2\"/>\n";

    auto bathymetryStroke = [](double depth_m) {
        if (!std::isfinite(depth_m)) {
            return std::string("#5c7cfa");
        }
        if (depth_m < 20.0) {
            return std::string("#74c0fc");
        }
        if (depth_m < 50.0) {
            return std::string("#4dabf7");
        }
        if (depth_m < 200.0) {
            return std::string("#228be6");
        }
        return std::string("#1c7ed6");
    };

    auto bathymetryFill = [](double depth_m) {
        if (!std::isfinite(depth_m)) {
            return std::string("#dbe4ff");
        }
        if (depth_m < 20.0) {
            return std::string("#d0ebff");
        }
        if (depth_m < 50.0) {
            return std::string("#a5d8ff");
        }
        if (depth_m < 200.0) {
            return std::string("#74c0fc");
        }
        return std::string("#4dabf7");
    };

    auto bufferZoneFill = [](double outer_m) {
        if (outer_m <= 50.0) {
            return std::string("#7bdff2");
        }
        if (outer_m <= 100.0) {
            return std::string("#58c4dd");
        }
        if (outer_m <= 150.0) {
            return std::string("#3fa7c4");
        }
        return std::string("#2d7f95");
    };

    for (const auto& bathy : bathymetry_features) {
        if (!bboxesOverlap(min_lat, max_lat, min_lon, max_lon, bathy.min_lat, bathy.max_lat, bathy.min_lon,
                           bathy.max_lon)) {
            continue;
        }

        const std::string stroke = bathymetryStroke(bathy.depth_m);
        const std::string fill = bathymetryFill(bathy.depth_m);
        if (bathy.closed && bathy.points.size() >= 3) {
            out << "<polygon points=\"";
            for (const auto& p : bathy.points) {
                out << mapX(p.lon_deg) << "," << mapY(p.lat_deg) << " ";
            }
            out << "\" fill=\"" << fill << "\" fill-opacity=\"0.16\" stroke=\"" << stroke
                << "\" stroke-opacity=\"0.60\" stroke-width=\"0.9\"/>\n";
        } else if (bathy.points.size() >= 2) {
            out << "<polyline points=\"";
            for (const auto& p : bathy.points) {
                out << mapX(p.lon_deg) << "," << mapY(p.lat_deg) << " ";
            }
            out << "\" fill=\"none\" stroke=\"" << stroke
                << "\" stroke-width=\"1.1\" stroke-opacity=\"0.72\"/>\n";
        }
    }

    for (const auto& zone : buffer_zones) {
        const auto& poly = zone.polygon;
        if (!bboxesOverlap(min_lat, max_lat, min_lon, max_lon, poly.min_lat, poly.max_lat, poly.min_lon,
                           poly.max_lon)) {
            continue;
        }

        out << "<polygon points=\"";
        for (const auto& v : poly.vertices) {
            out << mapX(v.lon_deg) << "," << mapY(v.lat_deg) << " ";
        }
        out << "\" fill=\"" << bufferZoneFill(zone.outer_m)
            << "\" fill-opacity=\"0.24\" stroke=\"none\"/>\n";
    }

    for (const auto& poly : land_polygons) {
        if (!bboxesOverlap(min_lat, max_lat, min_lon, max_lon, poly.min_lat, poly.max_lat, poly.min_lon,
                           poly.max_lon)) {
            continue;
        }
        std::vector<XY> svg_ring;
        svg_ring.reserve(poly.vertices.size());
        const double step2 = land_min_point_step_px * land_min_point_step_px;
        for (const auto& v : poly.vertices) {
            const XY cur{mapX(v.lon_deg), mapY(v.lat_deg)};
            if (!svg_ring.empty() && land_min_point_step_px > 0.0) {
                const double dx = cur.x - svg_ring.back().x;
                const double dy = cur.y - svg_ring.back().y;
                if (dx * dx + dy * dy < step2) {
                    continue;
                }
            }
            svg_ring.push_back(cur);
        }
        if (svg_ring.size() < 3) {
            continue;
        }

        if (land_min_area_px2 > 0.0) {
            double twice_area = 0.0;
            for (size_t i = 0, j = svg_ring.size() - 1; i < svg_ring.size(); j = i++) {
                twice_area += svg_ring[j].x * svg_ring[i].y - svg_ring[i].x * svg_ring[j].y;
            }
            if (std::abs(twice_area) * 0.5 < land_min_area_px2) {
                continue;
            }
        }

        out << "<polygon points=\"";
        for (const auto& p : svg_ring) {
            out << p.x << "," << p.y << " ";
        }
        out << "\" fill=\"#d17b49\" fill-opacity=\"" << land_fill_opacity << "\" stroke=\"none\"/>\n";
    }

    for (const auto& tss : tss_features) {
        if (!bboxesOverlap(min_lat, max_lat, min_lon, max_lon, tss.min_lat, tss.max_lat, tss.min_lon, tss.max_lon)) {
            continue;
        }

        std::string stroke = "#0b7285";
        std::string fill = "#99e9f2";
        std::string dash = "";
        double width_px = 1.8;
        if (tss.seamark_type.find("separation_lane") != std::string::npos) {
            stroke = "#0b7285";
            width_px = 2.2;
        } else if (tss.seamark_type.find("separation_boundary") != std::string::npos ||
                   tss.seamark_type.find("separation_line") != std::string::npos) {
            stroke = "#e67700";
            dash = " stroke-dasharray=\"7 5\"";
            width_px = 1.6;
        } else if (tss.seamark_type.find("separation_zone") != std::string::npos) {
            stroke = "#1971c2";
            fill = "#74c0fc";
        }

        if (tss.closed) {
            out << "<polygon points=\"";
            for (const auto& p : tss.points) {
                out << mapX(p.lon_deg) << "," << mapY(p.lat_deg) << " ";
            }
            out << "\" fill=\"" << fill << "\" fill-opacity=\"0.20\" stroke=\"" << stroke
                << "\" stroke-width=\"1.2\"" << dash << "/>\n";
        } else if (tss.points.size() >= 2) {
            out << "<polyline points=\"";
            for (const auto& p : tss.points) {
                out << mapX(p.lon_deg) << "," << mapY(p.lat_deg) << " ";
            }
            out << "\" fill=\"none\" stroke=\"" << stroke << "\" stroke-width=\"" << width_px << "\""
                << dash << "/>\n";
        }
    }

    std::unordered_set<std::uint64_t> drawn;
    for (const auto& t : triangles) {
        const std::pair<int, int> edges[3] = {{t.a, t.b}, {t.b, t.c}, {t.c, t.a}};
        for (const auto& e : edges) {
            const auto key = edgeKey(e.first, e.second);
            if (!drawn.insert(key).second) {
                continue;
            }
            const auto& a = vertices[e.first].geo;
            const auto& b = vertices[e.second].geo;
            out << "<line x1=\"" << mapX(a.lon_deg) << "\" y1=\"" << mapY(a.lat_deg) << "\" x2=\""
                << mapX(b.lon_deg) << "\" y2=\"" << mapY(b.lat_deg)
                << "\" stroke=\"#b5c0cc\" stroke-opacity=\"0.35\" stroke-width=\"0.8\"/>\n";
        }
    }

    for (int ti : tri_sequence) {
        const auto& t = triangles[ti];
        out << "<polygon points=\""
            << mapX(vertices[t.a].geo.lon_deg) << "," << mapY(vertices[t.a].geo.lat_deg) << " "
            << mapX(vertices[t.b].geo.lon_deg) << "," << mapY(vertices[t.b].geo.lat_deg) << " "
            << mapX(vertices[t.c].geo.lon_deg) << "," << mapY(vertices[t.c].geo.lat_deg)
            << "\" fill=\"#64a6ff\" fill-opacity=\"0.10\" stroke=\"none\"/>\n";
    }

    if (route.size() >= 2) {
        const char* route_stroke = is_mpc_svg ? "#111111" : "#084c97";
        const char* route_dash = is_mpc_svg ? " stroke-dasharray=\"10 7\"" : "";
        out << "<polyline points=\"";
        for (const auto& p : route) {
            out << mapX(p.lon_deg) << "," << mapY(p.lat_deg) << " ";
        }
        out << "\" fill=\"none\" stroke=\"" << route_stroke
            << "\" stroke-width=\"3.2\" stroke-opacity=\"0.72\" stroke-linecap=\"round\"" << route_dash << "/>\n";
    }

    if (!route.empty()) {
        const auto& s = route.front();
        const auto& g = route.back();
        out << "<circle cx=\"" << mapX(s.lon_deg) << "\" cy=\"" << mapY(s.lat_deg)
            << "\" r=\"6\" fill=\"#0a7f2e\"/>\n";
        out << "<circle cx=\"" << mapX(g.lon_deg) << "\" cy=\"" << mapY(g.lat_deg)
            << "\" r=\"6\" fill=\"#b42318\"/>\n";
    }

    const double legend_height = is_mpc_svg ? 132.0 : 120.0;
    out << "<rect x=\"22\" y=\"20\" width=\"760\" height=\"" << legend_height
        << "\" rx=\"8\" fill=\"#ffffff\" stroke=\"#9fb3c8\"/>\n";
    if (is_mpc_svg) {
        out << "<text x=\"38\" y=\"48\" font-family=\"monospace\" font-size=\"18\" fill=\"#102a43\">"
            << "Bosphorus MPC Tracking Output</text>\n";
        out << "<text x=\"38\" y=\"74\" font-family=\"monospace\" font-size=\"14\" fill=\"#334e68\">"
            << "Black dashed: MPC tracked route | Green/Red: start and finish</text>\n";
        out << "<text x=\"38\" y=\"100\" font-family=\"monospace\" font-size=\"14\" fill=\"#334e68\">"
            << "Gray: triangle mesh | Brown: land | Blue shades: bathymetry | Teal/Orange/Cyan: OSM TSS</text>\n";
    } else if (has_navigation_overlay) {
        out << "<text x=\"38\" y=\"48\" font-family=\"monospace\" font-size=\"18\" fill=\"#102a43\">"
            << "CDT-style Triangle Search + Funnel Refinement</text>\n";
        out << "<text x=\"38\" y=\"74\" font-family=\"monospace\" font-size=\"14\" fill=\"#334e68\">"
            << "Blue: refined route | Light-blue: selected triangle channel | Gray: triangle mesh</text>\n";
        out << "<text x=\"38\" y=\"100\" font-family=\"monospace\" font-size=\"14\" fill=\"#334e68\">"
            << (buffer_zones.empty() ? "Brown: land | Blue shades: bathymetry | Teal/Orange/Cyan: OSM TSS"
                                     : "Brown: land | Aqua bands: coast offset zones | Blue shades: bathymetry | Teal/Orange/Cyan: OSM TSS")
            << "</text>\n";
    } else {
        out << "<text x=\"38\" y=\"48\" font-family=\"monospace\" font-size=\"18\" fill=\"#102a43\">"
            << "World Coastline + TSS + Bathymetry Overview</text>\n";
        out << "<text x=\"38\" y=\"74\" font-family=\"monospace\" font-size=\"14\" fill=\"#334e68\">"
            << (buffer_zones.empty() ? "Brown: land polygons | Blue shades: bathymetry | Teal/Orange/Cyan: TSS"
                                     : "Brown: land polygons | Aqua bands: coast offset zones | Blue shades: bathymetry | Teal/Orange/Cyan: TSS")
            << "</text>\n";
        out << "<text x=\"38\" y=\"100\" font-family=\"monospace\" font-size=\"14\" fill=\"#334e68\">"
            << "Replace default TSS/bathymetry files with global GeoJSON layers when available</text>\n";
    }
    out << "</svg>\n";
}

bool savePlannerCache(const std::string& file_path, const PlannerCacheData& cache) {
    std::ofstream out(file_path, std::ios::binary);
    if (!out) {
        std::cerr << "Planner cache yazilamadi: " << file_path << "\n";
        return false;
    }

    auto writePolygonVector = [&](const std::vector<LandPolygon>& polygons) {
        const std::uint64_t poly_count = static_cast<std::uint64_t>(polygons.size());
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
    };

    const char magic[8] = {'R', 'P', 'C', 'A', 'C', 'H', 'E', '1'};
    const std::uint32_t version = 7;
    const std::uint32_t dataset_len = static_cast<std::uint32_t>(cache.dataset_path.size());
    const std::uint32_t mesh_dataset_len = static_cast<std::uint32_t>(cache.mesh_dataset_path.size());
    const std::uint32_t bathymetry_len = static_cast<std::uint32_t>(cache.bathymetry_path.size());
    const std::uint64_t vertex_count = static_cast<std::uint64_t>(cache.vertices.size());
    const std::uint64_t tri_count = static_cast<std::uint64_t>(cache.triangles.size());

    out.write(magic, sizeof(magic));
    out.write(reinterpret_cast<const char*>(&version), sizeof(version));
    out.write(reinterpret_cast<const char*>(&dataset_len), sizeof(dataset_len));
    out.write(cache.dataset_path.data(), dataset_len);
    out.write(reinterpret_cast<const char*>(&mesh_dataset_len), sizeof(mesh_dataset_len));
    out.write(cache.mesh_dataset_path.data(), mesh_dataset_len);
    out.write(reinterpret_cast<const char*>(&bathymetry_len), sizeof(bathymetry_len));
    out.write(cache.bathymetry_path.data(), bathymetry_len);
    out.write(reinterpret_cast<const char*>(&cache.min_lat), sizeof(cache.min_lat));
    out.write(reinterpret_cast<const char*>(&cache.max_lat), sizeof(cache.max_lat));
    out.write(reinterpret_cast<const char*>(&cache.min_lon), sizeof(cache.min_lon));
    out.write(reinterpret_cast<const char*>(&cache.max_lon), sizeof(cache.max_lon));
    out.write(reinterpret_cast<const char*>(&cache.grid_step_deg), sizeof(cache.grid_step_deg));
    out.write(reinterpret_cast<const char*>(&cache.narrow_refine_step_deg), sizeof(cache.narrow_refine_step_deg));
    out.write(reinterpret_cast<const char*>(&cache.clearance_m), sizeof(cache.clearance_m));
    out.write(reinterpret_cast<const char*>(&cache.min_depth_m), sizeof(cache.min_depth_m));
    out.write(reinterpret_cast<const char*>(&cache.use_coastline_vertices_for_triangulation),
              sizeof(cache.use_coastline_vertices_for_triangulation));
    out.write(reinterpret_cast<const char*>(&cache.coastline_vertex_spacing_m),
              sizeof(cache.coastline_vertex_spacing_m));
    out.write(reinterpret_cast<const char*>(&cache.outer_frame_margin_deg), sizeof(cache.outer_frame_margin_deg));
    out.write(reinterpret_cast<const char*>(&vertex_count), sizeof(vertex_count));
    out.write(reinterpret_cast<const char*>(&tri_count), sizeof(tri_count));

    for (const auto& v : cache.vertices) {
        const std::uint32_t name_len = static_cast<std::uint32_t>(v.name.size());
        out.write(reinterpret_cast<const char*>(&name_len), sizeof(name_len));
        out.write(v.name.data(), name_len);
        out.write(reinterpret_cast<const char*>(&v.geo.lat_deg), sizeof(v.geo.lat_deg));
        out.write(reinterpret_cast<const char*>(&v.geo.lon_deg), sizeof(v.geo.lon_deg));
    }

    for (const auto& t : cache.triangles) {
        const std::int32_t a = static_cast<std::int32_t>(t.a);
        const std::int32_t b = static_cast<std::int32_t>(t.b);
        const std::int32_t c = static_cast<std::int32_t>(t.c);
        out.write(reinterpret_cast<const char*>(&a), sizeof(a));
        out.write(reinterpret_cast<const char*>(&b), sizeof(b));
        out.write(reinterpret_cast<const char*>(&c), sizeof(c));
        out.write(reinterpret_cast<const char*>(&t.centroid.x), sizeof(t.centroid.x));
        out.write(reinterpret_cast<const char*>(&t.centroid.y), sizeof(t.centroid.y));
    }

    writePolygonVector(cache.land_polygons);

    if (!out.good()) {
        std::cerr << "Planner cache yazimi yarim kaldi: " << file_path << "\n";
        return false;
    }
    return true;
}

bool loadPlannerCache(const std::string& file_path, PlannerCacheData& out_cache) {
    std::ifstream in(file_path, std::ios::binary);
    if (!in) {
        std::cerr << "Planner cache acilamadi: " << file_path << "\n";
        return false;
    }

    char magic[8] = {};
    std::uint32_t version = 0;
    std::uint32_t dataset_len = 0;
    std::uint32_t bathymetry_len = 0;
    std::uint64_t vertex_count = 0;
    std::uint64_t tri_count = 0;

    in.read(magic, sizeof(magic));
    in.read(reinterpret_cast<char*>(&version), sizeof(version));
    in.read(reinterpret_cast<char*>(&dataset_len), sizeof(dataset_len));
    if (!in.good()) {
        std::cerr << "Planner cache baslik okunamadi: " << file_path << "\n";
        return false;
    }

    const char expected_magic[8] = {'R', 'P', 'C', 'A', 'C', 'H', 'E', '1'};
    if (std::memcmp(magic, expected_magic, sizeof(magic)) != 0 ||
        (version != 1 && version != 2 && version != 3 && version != 4 && version != 5 && version != 6 &&
         version != 7)) {
        std::cerr << "Planner cache formati uyumsuz: " << file_path << "\n";
        return false;
    }

    auto readPolygonVector = [&](std::vector<LandPolygon>& polygons) -> bool {
        std::uint64_t poly_count = 0;
        in.read(reinterpret_cast<char*>(&poly_count), sizeof(poly_count));
        if (!in.good()) {
            return false;
        }

        polygons.clear();
        polygons.reserve(static_cast<std::size_t>(poly_count));
        for (std::uint64_t i = 0; i < poly_count; ++i) {
            LandPolygon poly;
            std::uint64_t vertex_count = 0;
            in.read(reinterpret_cast<char*>(&vertex_count), sizeof(vertex_count));
            in.read(reinterpret_cast<char*>(&poly.min_lat), sizeof(poly.min_lat));
            in.read(reinterpret_cast<char*>(&poly.max_lat), sizeof(poly.max_lat));
            in.read(reinterpret_cast<char*>(&poly.min_lon), sizeof(poly.min_lon));
            in.read(reinterpret_cast<char*>(&poly.max_lon), sizeof(poly.max_lon));
            if (!in.good()) {
                return false;
            }

            poly.vertices.resize(static_cast<std::size_t>(vertex_count));
            for (std::uint64_t j = 0; j < vertex_count; ++j) {
                in.read(reinterpret_cast<char*>(&poly.vertices[static_cast<std::size_t>(j)].lat_deg),
                        sizeof(poly.vertices[static_cast<std::size_t>(j)].lat_deg));
                in.read(reinterpret_cast<char*>(&poly.vertices[static_cast<std::size_t>(j)].lon_deg),
                        sizeof(poly.vertices[static_cast<std::size_t>(j)].lon_deg));
                if (!in.good()) {
                    return false;
                }
            }
            polygons.push_back(std::move(poly));
        }
        return true;
    };

    out_cache = PlannerCacheData{};
    out_cache.dataset_path.resize(dataset_len);
    if (dataset_len > 0) {
        in.read(&out_cache.dataset_path[0], dataset_len);
    }
    if (version >= 5) {
        std::uint32_t mesh_dataset_len = 0;
        in.read(reinterpret_cast<char*>(&mesh_dataset_len), sizeof(mesh_dataset_len));
        out_cache.mesh_dataset_path.resize(mesh_dataset_len);
        if (mesh_dataset_len > 0) {
            in.read(&out_cache.mesh_dataset_path[0], mesh_dataset_len);
        }
    } else {
        out_cache.mesh_dataset_path = out_cache.dataset_path;
    }
    if (version >= 3) {
        in.read(reinterpret_cast<char*>(&bathymetry_len), sizeof(bathymetry_len));
        out_cache.bathymetry_path.resize(bathymetry_len);
        if (bathymetry_len > 0) {
            in.read(&out_cache.bathymetry_path[0], bathymetry_len);
        }
    }
    in.read(reinterpret_cast<char*>(&out_cache.min_lat), sizeof(out_cache.min_lat));
    in.read(reinterpret_cast<char*>(&out_cache.max_lat), sizeof(out_cache.max_lat));
    in.read(reinterpret_cast<char*>(&out_cache.min_lon), sizeof(out_cache.min_lon));
    in.read(reinterpret_cast<char*>(&out_cache.max_lon), sizeof(out_cache.max_lon));
    in.read(reinterpret_cast<char*>(&out_cache.grid_step_deg), sizeof(out_cache.grid_step_deg));
    if (version >= 2) {
        in.read(reinterpret_cast<char*>(&out_cache.narrow_refine_step_deg), sizeof(out_cache.narrow_refine_step_deg));
    }
    in.read(reinterpret_cast<char*>(&out_cache.clearance_m), sizeof(out_cache.clearance_m));
    if (version >= 3) {
        in.read(reinterpret_cast<char*>(&out_cache.min_depth_m), sizeof(out_cache.min_depth_m));
    }
    if (version >= 4) {
        in.read(reinterpret_cast<char*>(&out_cache.use_coastline_vertices_for_triangulation),
                sizeof(out_cache.use_coastline_vertices_for_triangulation));
        in.read(reinterpret_cast<char*>(&out_cache.coastline_vertex_spacing_m),
                sizeof(out_cache.coastline_vertex_spacing_m));
    }
    if (version >= 7) {
        in.read(reinterpret_cast<char*>(&out_cache.outer_frame_margin_deg), sizeof(out_cache.outer_frame_margin_deg));
    }
    in.read(reinterpret_cast<char*>(&vertex_count), sizeof(vertex_count));
    in.read(reinterpret_cast<char*>(&tri_count), sizeof(tri_count));
    if (!in.good()) {
        std::cerr << "Planner cache metadata okunamadi: " << file_path << "\n";
        return false;
    }

    out_cache.vertices.resize(static_cast<std::size_t>(vertex_count));
    for (std::size_t i = 0; i < out_cache.vertices.size(); ++i) {
        std::uint32_t name_len = 0;
        in.read(reinterpret_cast<char*>(&name_len), sizeof(name_len));
        out_cache.vertices[i].name.resize(name_len);
        if (name_len > 0) {
            in.read(&out_cache.vertices[i].name[0], name_len);
        }
        in.read(reinterpret_cast<char*>(&out_cache.vertices[i].geo.lat_deg), sizeof(out_cache.vertices[i].geo.lat_deg));
        in.read(reinterpret_cast<char*>(&out_cache.vertices[i].geo.lon_deg), sizeof(out_cache.vertices[i].geo.lon_deg));
    }

    out_cache.triangles.resize(static_cast<std::size_t>(tri_count));
    for (std::size_t i = 0; i < out_cache.triangles.size(); ++i) {
        std::int32_t a = -1;
        std::int32_t b = -1;
        std::int32_t c = -1;
        in.read(reinterpret_cast<char*>(&a), sizeof(a));
        in.read(reinterpret_cast<char*>(&b), sizeof(b));
        in.read(reinterpret_cast<char*>(&c), sizeof(c));
        in.read(reinterpret_cast<char*>(&out_cache.triangles[i].centroid.x), sizeof(out_cache.triangles[i].centroid.x));
        in.read(reinterpret_cast<char*>(&out_cache.triangles[i].centroid.y), sizeof(out_cache.triangles[i].centroid.y));
        out_cache.triangles[i].a = static_cast<int>(a);
        out_cache.triangles[i].b = static_cast<int>(b);
        out_cache.triangles[i].c = static_cast<int>(c);
    }

    if (version >= 6) {
        if (!readPolygonVector(out_cache.land_polygons)) {
            std::cerr << "Planner cache icindeki kara polygonlari okunamadi: " << file_path << "\n";
            return false;
        }
    }

    if (!in.good()) {
        std::cerr << "Planner cache govdesi okunamadi: " << file_path << "\n";
        return false;
    }
    return true;
}

bool isPlannerCacheCompatible(const PlannerCacheData& cache,
                              const CliConfig& cfg,
                              double min_lat,
                              double max_lat,
                              double min_lon,
                              double max_lon,
                              double clearance_m) {
    auto almostEqual = [](double a, double b) { return std::fabs(a - b) <= 1e-9; };
    const std::string expected_mesh_dataset = cfg.mesh_land_geojson.empty() ? cfg.land_geojson : cfg.mesh_land_geojson;
    if (cache.dataset_path != cfg.land_geojson) {
        std::cerr << "Planner cache dataset uyusmuyor.\n";
        return false;
    }
    if (cache.mesh_dataset_path != expected_mesh_dataset) {
        std::cerr << "Planner cache mesh dataset uyusmuyor.\n";
        return false;
    }
    if (cache.bathymetry_path != cfg.bathymetry_geojson) {
        std::cerr << "Planner cache bathymetri dataset uyusmuyor.\n";
        return false;
    }
    if (!almostEqual(cache.grid_step_deg, cfg.grid_step_deg)) {
        std::cerr << "Planner cache grid-step uyusmuyor.\n";
        return false;
    }
    if (!almostEqual(cache.narrow_refine_step_deg, cfg.narrow_refine_step_deg)) {
        std::cerr << "Planner cache narrow-refine-step uyusmuyor.\n";
        return false;
    }
    if (!almostEqual(cache.clearance_m, clearance_m)) {
        std::cerr << "Planner cache clearance uyusmuyor.\n";
        return false;
    }
    if (!almostEqual(cache.min_depth_m, cfg.min_depth_m)) {
        std::cerr << "Planner cache min-depth uyusmuyor.\n";
        return false;
    }
    if (cache.use_coastline_vertices_for_triangulation) {
        std::cerr << "Planner cache coastline-vertex modu uyusmuyor.\n";
        return false;
    }
    if (!almostEqual(cache.coastline_vertex_spacing_m, 0.0)) {
        std::cerr << "Planner cache coastline-vertex spacing uyusmuyor.\n";
        return false;
    }
    if (!almostEqual(cache.outer_frame_margin_deg, 0.0)) {
        std::cerr << "Planner cache outer-frame margin uyusmuyor.\n";
        return false;
    }
    if (!almostEqual(cache.min_lat, min_lat) || !almostEqual(cache.max_lat, max_lat) ||
        !almostEqual(cache.min_lon, min_lon) || !almostEqual(cache.max_lon, max_lon)) {
        std::cerr << "Planner cache corridor sinirlari uyusmuyor.\n";
        return false;
    }
    if (cache.vertices.size() < 6 || cache.triangles.empty()) {
        std::cerr << "Planner cache icerigi eksik.\n";
        return false;
    }
    return true;
}

}  // namespace planner
