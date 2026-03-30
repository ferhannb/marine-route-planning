#include "planner.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <queue>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

namespace planner {
namespace {

constexpr double kEarthRadiusKm = 6371.0088;
constexpr double kPi = 3.14159265358979323846;
constexpr double kEps = 1e-9;

double cross(const XY& a, const XY& b, const XY& c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

double triArea2(const XY& a, const XY& b, const XY& c) {
    return cross(a, b, c);
}

double dot(const XY& a, const XY& b) {
    return a.x * b.x + a.y * b.y;
}

double sqrNorm(const XY& v) {
    return dot(v, v);
}

double kmToLatDeg(double km) {
    return km / 111.32;
}

double kmToLonDeg(double km, double lat_deg) {
    const double cos_lat = std::max(0.2, std::cos(degToRad(clampLatitude(lat_deg))));
    return km / (111.32 * cos_lat);
}

bool almostEqualXY(const XY& a, const XY& b) {
    return std::fabs(a.x - b.x) < 1e-8 && std::fabs(a.y - b.y) < 1e-8;
}

bool onSegment(const XY& a, const XY& b, const XY& p) {
    if (std::fabs(cross(a, b, p)) > kEps) {
        return false;
    }
    return (std::min(a.x, b.x) - kEps <= p.x && p.x <= std::max(a.x, b.x) + kEps &&
            std::min(a.y, b.y) - kEps <= p.y && p.y <= std::max(a.y, b.y) + kEps);
}

int orientation(const XY& a, const XY& b, const XY& c) {
    const double value = cross(a, b, c);
    if (std::fabs(value) < kEps) {
        return 0;
    }
    return value > 0.0 ? 1 : -1;
}

bool segmentsIntersect(const XY& p1, const XY& q1, const XY& p2, const XY& q2) {
    const int o1 = orientation(p1, q1, p2);
    const int o2 = orientation(p1, q1, q2);
    const int o3 = orientation(p2, q2, p1);
    const int o4 = orientation(p2, q2, q1);

    if (o1 != o2 && o3 != o4) {
        return true;
    }
    if (o1 == 0 && onSegment(p1, q1, p2)) {
        return true;
    }
    if (o2 == 0 && onSegment(p1, q1, q2)) {
        return true;
    }
    if (o3 == 0 && onSegment(p2, q2, p1)) {
        return true;
    }
    if (o4 == 0 && onSegment(p2, q2, q1)) {
        return true;
    }
    return false;
}

double pointToSegmentDistanceKm(const XY& p, const XY& a, const XY& b) {
    const XY ab{b.x - a.x, b.y - a.y};
    const double ab2 = sqrNorm(ab);
    if (ab2 < 1e-16) {
        return std::sqrt(sqrNorm(XY{p.x - a.x, p.y - a.y}));
    }
    const XY ap{p.x - a.x, p.y - a.y};
    const double t = std::max(0.0, std::min(1.0, dot(ap, ab) / ab2));
    const XY proj{a.x + t * ab.x, a.y + t * ab.y};
    return std::sqrt(sqrNorm(XY{p.x - proj.x, p.y - proj.y}));
}

double segmentToSegmentDistanceKm(const XY& a1, const XY& a2, const XY& b1, const XY& b2) {
    if (segmentsIntersect(a1, a2, b1, b2)) {
        return 0.0;
    }
    const double d1 = pointToSegmentDistanceKm(a1, b1, b2);
    const double d2 = pointToSegmentDistanceKm(a2, b1, b2);
    const double d3 = pointToSegmentDistanceKm(b1, a1, a2);
    const double d4 = pointToSegmentDistanceKm(b2, a1, a2);
    return std::min(std::min(d1, d2), std::min(d3, d4));
}

bool pointInPolygon(const XY& p, const std::vector<XY>& poly) {
    if (poly.size() < 3) {
        return false;
    }
    bool inside = false;
    for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
        const XY& a = poly[j];
        const XY& b = poly[i];
        const bool intersects = ((a.y > p.y) != (b.y > p.y)) &&
                                (p.x < (b.x - a.x) * (p.y - a.y) / ((b.y - a.y) + kEps) + a.x);
        if (intersects) {
            inside = !inside;
        }
    }
    return inside;
}

std::int64_t spatialBinKey(int lat_bin, int lon_bin) {
    return (static_cast<std::int64_t>(lat_bin) << 32) ^ static_cast<std::uint32_t>(lon_bin);
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
    if (p.lat_deg < poly.min_lat || p.lat_deg > poly.max_lat || p.lon_deg < poly.min_lon ||
        p.lon_deg > poly.max_lon) {
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

double pointToLandBoundaryKm(const LatLon& p, const LandPolygon& poly) {
    const Projection local_proj{degToRad(p.lat_deg), degToRad(p.lon_deg)};
    const XY point_xy = toXY(p, local_proj);
    std::vector<XY> poly_xy;
    poly_xy.reserve(poly.vertices.size());
    for (const auto& v : poly.vertices) {
        poly_xy.push_back(toXY(v, local_proj));
    }

    if (poly_xy.empty()) {
        return std::numeric_limits<double>::infinity();
    }

    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < poly_xy.size(); ++i) {
        const XY& v1 = poly_xy[i];
        const XY& v2 = poly_xy[(i + 1) % poly_xy.size()];
        best = std::min(best, pointToSegmentDistanceKm(point_xy, v1, v2));
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

    if (p.lat_deg >= poly.min_lat && p.lat_deg <= poly.max_lat && p.lon_deg >= poly.min_lon &&
        p.lon_deg <= poly.max_lon && pointInLandPolygon(p, poly)) {
        return true;
    }

    if (clearance_km <= 0.0) {
        return false;
    }
    return pointToLandBoundaryKm(p, poly) <= clearance_km;
}

bool segmentIntersectsLandPolygon(const LatLon& a, const LatLon& b, const LandPolygon& poly, double clearance_km) {
    const double min_lat = std::min(a.lat_deg, b.lat_deg);
    const double max_lat = std::max(a.lat_deg, b.lat_deg);
    const double min_lon = std::min(a.lon_deg, b.lon_deg);
    const double max_lon = std::max(a.lon_deg, b.lon_deg);
    const double lat_margin = kmToLatDeg(clearance_km);
    const double lon_margin = kmToLonDeg(clearance_km, 0.5 * (a.lat_deg + b.lat_deg));
    if (!bboxesOverlap(min_lat - lat_margin,
                       max_lat + lat_margin,
                       min_lon - lon_margin,
                       max_lon + lon_margin,
                       poly.min_lat,
                       poly.max_lat,
                       poly.min_lon,
                       poly.max_lon)) {
        return false;
    }

    const Projection local_proj{degToRad(0.5 * (a.lat_deg + b.lat_deg)),
                                degToRad(0.5 * (a.lon_deg + b.lon_deg))};
    const XY a_xy = toXY(a, local_proj);
    const XY b_xy = toXY(b, local_proj);
    std::vector<XY> poly_xy;
    poly_xy.reserve(poly.vertices.size());
    for (const auto& v : poly.vertices) {
        poly_xy.push_back(toXY(v, local_proj));
    }

    if (pointInPolygon(a_xy, poly_xy) || pointInPolygon(b_xy, poly_xy)) {
        return true;
    }

    for (size_t i = 0; i < poly_xy.size(); ++i) {
        const XY& p1 = poly_xy[i];
        const XY& p2 = poly_xy[(i + 1) % poly_xy.size()];
        if (segmentsIntersect(a_xy, b_xy, p1, p2)) {
            return true;
        }
    }

    if (clearance_km <= 0.0) {
        return false;
    }

    double min_dist = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < poly_xy.size(); ++i) {
        const XY& p1 = poly_xy[i];
        const XY& p2 = poly_xy[(i + 1) % poly_xy.size()];
        min_dist = std::min(min_dist, segmentToSegmentDistanceKm(a_xy, b_xy, p1, p2));
        if (min_dist <= clearance_km) {
            return true;
        }
    }

    return false;
}

bool isOnLandImpl(const LatLon& p,
                  const std::vector<LandPolygon>& land_polygons,
                  const LandSpatialIndex& land_index,
                  double clearance_km = 0.0) {
    const double lat_margin = kmToLatDeg(clearance_km);
    const double lon_margin = kmToLonDeg(clearance_km, p.lat_deg);
    const auto candidate_ids = candidatePolygonsForBBox(land_index,
                                                        p.lat_deg - lat_margin,
                                                        p.lat_deg + lat_margin,
                                                        p.lon_deg - lon_margin,
                                                        p.lon_deg + lon_margin);
    for (int poly_id : candidate_ids) {
        if (pointInBufferedLandPolygon(p, land_polygons[poly_id], clearance_km)) {
            return true;
        }
    }
    return false;
}

bool edgeCrossesLand(const LatLon& a,
                     const LatLon& b,
                     const std::vector<LandPolygon>& land_polygons,
                     const LandSpatialIndex& land_index,
                     double clearance_km = 0.0) {
    const double min_lat = std::min(a.lat_deg, b.lat_deg);
    const double max_lat = std::max(a.lat_deg, b.lat_deg);
    const double min_lon = std::min(a.lon_deg, b.lon_deg);
    const double max_lon = std::max(a.lon_deg, b.lon_deg);
    const double lat_margin = kmToLatDeg(clearance_km);
    const double lon_margin = kmToLonDeg(clearance_km, 0.5 * (a.lat_deg + b.lat_deg));
    const auto candidate_ids = candidatePolygonsForBBox(land_index,
                                                        min_lat - lat_margin,
                                                        max_lat + lat_margin,
                                                        min_lon - lon_margin,
                                                        max_lon + lon_margin);

    for (int poly_id : candidate_ids) {
        if (segmentIntersectsLandPolygon(a, b, land_polygons[poly_id], clearance_km)) {
            return true;
        }
    }

    const double seg_len_km = haversineKm(a, b);
    if (seg_len_km > 10.0) {
        const int sample_count = std::min(64, std::max(2, static_cast<int>(std::ceil(seg_len_km / 8.0))));
        for (int i = 1; i < sample_count; ++i) {
            const double t = static_cast<double>(i) / static_cast<double>(sample_count);
            const LatLon p{
                a.lat_deg + (b.lat_deg - a.lat_deg) * t,
                a.lon_deg + (b.lon_deg - a.lon_deg) * t,
            };
            if (isOnLandImpl(p, land_polygons, land_index, clearance_km)) {
                return true;
            }
        }
    }
    return false;
}

LatLon nearestSeaPoint(const LatLon& p,
                       const std::vector<LandPolygon>& land_polygons,
                       const LandSpatialIndex& land_index,
                       double clearance_km = 0.0) {
    if (!isOnLandImpl(p, land_polygons, land_index, clearance_km)) {
        return p;
    }

    for (double radius_deg = 0.05; radius_deg <= 3.0; radius_deg += 0.05) {
        for (int bearing = 0; bearing < 360; bearing += 5) {
            const double br = degToRad(static_cast<double>(bearing));
            const double lat = clampLatitude(p.lat_deg + radius_deg * std::sin(br));
            const double cos_lat = std::max(0.2, std::cos(degToRad(lat)));
            const double lon = p.lon_deg + radius_deg * std::cos(br) / cos_lat;
            const LatLon candidate{lat, lon};
            if (!isOnLandImpl(candidate, land_polygons, land_index, clearance_km)) {
                return candidate;
            }
        }
    }

    return p;
}

bool isNavigablePointImpl(const LatLon& p,
                          const std::vector<LandPolygon>& land_polygons,
                          const LandSpatialIndex& land_index,
                          double clearance_km,
                          const std::vector<LandPolygon>& shallow_polygons,
                          const LandSpatialIndex& shallow_index) {
    if (isOnLandImpl(p, land_polygons, land_index, clearance_km)) {
        return false;
    }
    if (!shallow_polygons.empty() && isOnLandImpl(p, shallow_polygons, shallow_index, 0.0)) {
        return false;
    }
    return true;
}

bool pointInsideInnerBounds(const LatLon& p,
                            double min_lat,
                            double max_lat,
                            double min_lon,
                            double max_lon,
                            double margin_deg) {
    return p.lat_deg >= (min_lat + margin_deg) && p.lat_deg <= (max_lat - margin_deg) &&
           p.lon_deg >= (min_lon + margin_deg) && p.lon_deg <= (max_lon - margin_deg);
}

bool seamarkTypeContains(const std::string& seamark_type, const char* token) {
    return seamark_type.find(token) != std::string::npos;
}

bool segmentIntersectsPolyline(const XY& a, const XY& b, const std::vector<XY>& line) {
    if (line.size() < 2) {
        return false;
    }
    for (size_t i = 1; i < line.size(); ++i) {
        if (segmentsIntersect(a, b, line[i - 1], line[i])) {
            return true;
        }
    }
    return false;
}

double pointToPolylineDistanceKm(const XY& p, const std::vector<XY>& line) {
    if (line.size() < 2) {
        return std::numeric_limits<double>::infinity();
    }
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 1; i < line.size(); ++i) {
        best = std::min(best, pointToSegmentDistanceKm(p, line[i - 1], line[i]));
    }
    return best;
}

bool segmentIntersectsPolygon(const XY& a, const XY& b, const std::vector<XY>& poly) {
    if (poly.size() < 3) {
        return false;
    }
    if (pointInPolygon(a, poly) || pointInPolygon(b, poly)) {
        return true;
    }
    for (size_t i = 0; i < poly.size(); ++i) {
        if (segmentsIntersect(a, b, poly[i], poly[(i + 1) % poly.size()])) {
            return true;
        }
    }
    return false;
}

bool pointInsideTrafficLane(const XY& p, const TriangleTrafficData& traffic_data) {
    if (traffic_data.has_lane_polygons) {
        for (const auto& lane : traffic_data.lane_polygons_xy) {
            if (pointInPolygon(p, lane)) {
                return true;
            }
        }
    }
    for (const auto& endpoint : traffic_data.lane_endpoint_points_xy) {
        const double dx = p.x - endpoint.x;
        const double dy = p.y - endpoint.y;
        if (std::sqrt(dx * dx + dy * dy) <= traffic_data.config.lane_endpoint_radius_km) {
            return true;
        }
    }
    if (traffic_data.has_lane_lines) {
        for (const auto& lane_line : traffic_data.lane_lines_xy) {
            if (pointToPolylineDistanceKm(p, lane_line) <= traffic_data.config.lane_corridor_half_width_km) {
                return true;
            }
        }
        return false;
    }
    return true;
}

bool xySegmentAllowedByTrafficImpl(const XY& a, const XY& b, const TriangleTrafficData& traffic_data) {
    if (!traffic_data.active || !traffic_data.config.hard_tss_corridor) {
        return true;
    }
    if (!pointInsideTrafficLane(a, traffic_data) || !pointInsideTrafficLane(b, traffic_data)) {
        return false;
    }
    for (const auto& zone : traffic_data.zone_polygons_xy) {
        if (segmentIntersectsPolygon(a, b, zone)) {
            return false;
        }
    }

    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double seg_len_km = std::sqrt(dx * dx + dy * dy);
    const int sample_count = std::max(2, std::min(64, static_cast<int>(std::ceil(seg_len_km / 0.25))));
    for (int i = 1; i < sample_count; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(sample_count);
        const XY p{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t};
        if (!pointInsideTrafficLane(p, traffic_data)) {
            return false;
        }
    }
    return true;
}

std::uint64_t edgeKey(int a, int b) {
    const std::uint32_t u = static_cast<std::uint32_t>(std::min(a, b));
    const std::uint32_t v = static_cast<std::uint32_t>(std::max(a, b));
    return (static_cast<std::uint64_t>(u) << 32U) | v;
}

struct TriRaw {
    int a;
    int b;
    int c;
};

void ensureCCW(TriRaw& t, const std::vector<XY>& pts) {
    if (cross(pts[t.a], pts[t.b], pts[t.c]) < 0.0) {
        std::swap(t.b, t.c);
    }
}

bool pointInCircumcircle(const TriRaw& t, const std::vector<XY>& pts, const XY& p) {
    const XY& a = pts[t.a];
    const XY& b = pts[t.b];
    const XY& c = pts[t.c];

    const double ax = a.x - p.x;
    const double ay = a.y - p.y;
    const double bx = b.x - p.x;
    const double by = b.y - p.y;
    const double cx = c.x - p.x;
    const double cy = c.y - p.y;

    const double det = (ax * ax + ay * ay) * (bx * cy - cx * by) -
                       (bx * bx + by * by) * (ax * cy - cx * ay) +
                       (cx * cx + cy * cy) * (ax * by - bx * ay);

    return det > 1e-10;
}

double xyDistanceKm(const XY& a, const XY& b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

std::vector<int> trianglesUsingVertex(const std::vector<Triangle>& triangles, int vertex_idx) {
    std::vector<int> out;
    out.reserve(32);
    for (int i = 0; i < static_cast<int>(triangles.size()); ++i) {
        const auto& t = triangles[i];
        if (t.a == vertex_idx || t.b == vertex_idx || t.c == vertex_idx) {
            out.push_back(i);
        }
    }
    return out;
}

bool triangleNavigable(const Triangle& tri,
                       const std::vector<NamedPoint>& vertices,
                       const std::vector<LandPolygon>& land_polygons,
                       const LandSpatialIndex& land_index,
                       double clearance_km,
                       const std::vector<LandPolygon>& shallow_polygons,
                       const LandSpatialIndex& shallow_index) {
    const LatLon centroid_geo{
        (vertices[tri.a].geo.lat_deg + vertices[tri.b].geo.lat_deg + vertices[tri.c].geo.lat_deg) / 3.0,
        (vertices[tri.a].geo.lon_deg + vertices[tri.b].geo.lon_deg + vertices[tri.c].geo.lon_deg) / 3.0,
    };

    if (!isNavigablePointImpl(centroid_geo, land_polygons, land_index, clearance_km, shallow_polygons,
                              shallow_index)) {
        return false;
    }

    const auto edgeLooksSea = [&](int i, int j) {
        const LatLon& a = vertices[i].geo;
        const LatLon& b = vertices[j].geo;
        const LatLon m1{(2.0 * a.lat_deg + b.lat_deg) / 3.0, (2.0 * a.lon_deg + b.lon_deg) / 3.0};
        const LatLon m2{(a.lat_deg + 2.0 * b.lat_deg) / 3.0, (a.lon_deg + 2.0 * b.lon_deg) / 3.0};
        if (!isNavigablePointImpl(m1, land_polygons, land_index, clearance_km, shallow_polygons, shallow_index)) {
            return false;
        }
        if (!isNavigablePointImpl(m2, land_polygons, land_index, clearance_km, shallow_polygons, shallow_index)) {
            return false;
        }
        return true;
    };

    if (!edgeLooksSea(tri.a, tri.b) || !edgeLooksSea(tri.b, tri.c) || !edgeLooksSea(tri.c, tri.a)) {
        return false;
    }

    return true;
}

bool findSharedPortal(const std::vector<NeighborPortal>& neighbors, int to_tri, int& va, int& vb) {
    for (const auto& nb : neighbors) {
        if (nb.to == to_tri) {
            va = nb.va;
            vb = nb.vb;
            return true;
        }
    }
    return false;
}

}  // namespace

double degToRad(double deg) {
    return deg * kPi / 180.0;
}

double radToDeg(double rad) {
    return rad * 180.0 / kPi;
}

double clampLatitude(double lat_deg) {
    return std::max(-89.999, std::min(89.999, lat_deg));
}

double wrapToPi(double angle_rad) {
    while (angle_rad > kPi) {
        angle_rad -= 2.0 * kPi;
    }
    while (angle_rad < -kPi) {
        angle_rad += 2.0 * kPi;
    }
    return angle_rad;
}

double haversineKm(double lat1_deg, double lon1_deg, double lat2_deg, double lon2_deg) {
    const double lat1 = degToRad(lat1_deg);
    const double lon1 = degToRad(lon1_deg);
    const double lat2 = degToRad(lat2_deg);
    const double lon2 = degToRad(lon2_deg);

    const double dlat = lat2 - lat1;
    const double dlon = wrapToPi(lon2 - lon1);
    const double h = std::pow(std::sin(dlat / 2.0), 2.0) +
                     std::cos(lat1) * std::cos(lat2) * std::pow(std::sin(dlon / 2.0), 2.0);
    const double c = 2.0 * std::atan2(std::sqrt(h), std::sqrt(1.0 - h));
    return kEarthRadiusKm * c;
}

double haversineKm(const LatLon& a, const LatLon& b) {
    return haversineKm(a.lat_deg, a.lon_deg, b.lat_deg, b.lon_deg);
}

double computeOuterFrameMarginDeg(double grid_step_deg, double narrow_refine_step_deg) {
    (void)grid_step_deg;
    (void)narrow_refine_step_deg;
    return 0.0;
}

Projection makeProjection(double min_lat, double max_lat, double min_lon, double max_lon) {
    const double ref_lat = 0.5 * (min_lat + max_lat);
    const double ref_lon = 0.5 * (min_lon + max_lon);
    return {degToRad(ref_lat), degToRad(ref_lon)};
}

XY toXY(const LatLon& p, const Projection& proj) {
    const double lat = degToRad(p.lat_deg);
    const double lon = degToRad(p.lon_deg);
    const double dlon = wrapToPi(lon - proj.ref_lon_rad);
    return {
        kEarthRadiusKm * dlon * std::cos(proj.ref_lat_rad),
        kEarthRadiusKm * (lat - proj.ref_lat_rad),
    };
}

LatLon toGeo(const XY& p, const Projection& proj) {
    const double lat = proj.ref_lat_rad + p.y / kEarthRadiusKm;
    const double lon = proj.ref_lon_rad + p.x / (kEarthRadiusKm * std::cos(proj.ref_lat_rad));
    return {radToDeg(lat), radToDeg(lon)};
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

std::vector<LandPolygon> filterPolygonsByBounds(const std::vector<LandPolygon>& polygons,
                                                double min_lat,
                                                double max_lat,
                                                double min_lon,
                                                double max_lon) {
    std::vector<LandPolygon> filtered;
    filtered.reserve(polygons.size());
    for (const auto& poly : polygons) {
        if (bboxesOverlap(min_lat, max_lat, min_lon, max_lon, poly.min_lat, poly.max_lat, poly.min_lon,
                          poly.max_lon)) {
            filtered.push_back(poly);
        }
    }
    return filtered;
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

std::vector<LandPolygon> buildShallowWaterPolygons(const std::vector<BathymetryFeature>& features, double min_depth_m) {
    std::vector<LandPolygon> out;
    if (min_depth_m <= 0.0) {
        return out;
    }

    out.reserve(features.size());
    for (const auto& f : features) {
        if (!f.closed || f.points.size() < 3 || !std::isfinite(f.depth_m) || f.depth_m >= min_depth_m) {
            continue;
        }
        LandPolygon p;
        p.vertices = f.points;
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
        out.push_back(std::move(p));
    }
    return out;
}

bool isOnLand(const LatLon& p,
              const std::vector<LandPolygon>& land_polygons,
              const LandSpatialIndex& land_index,
              double clearance_km) {
    return isOnLandImpl(p, land_polygons, land_index, clearance_km);
}

bool edgeNavigable(const LatLon& a,
                   const LatLon& b,
                   const std::vector<LandPolygon>& land_polygons,
                   const LandSpatialIndex& land_index,
                   double clearance_km,
                   const std::vector<LandPolygon>& shallow_polygons,
                   const LandSpatialIndex& shallow_index) {
    if (edgeCrossesLand(a, b, land_polygons, land_index, clearance_km)) {
        return false;
    }
    if (!shallow_polygons.empty() && edgeCrossesLand(a, b, shallow_polygons, shallow_index, 0.0)) {
        return false;
    }
    return true;
}

LatLon nearestNavigablePoint(const LatLon& p,
                             const std::vector<LandPolygon>& land_polygons,
                             const LandSpatialIndex& land_index,
                             double clearance_km,
                             const std::vector<LandPolygon>& shallow_polygons,
                             const LandSpatialIndex& shallow_index) {
    if (isNavigablePointImpl(p, land_polygons, land_index, clearance_km, shallow_polygons, shallow_index)) {
        return p;
    }

    for (double radius_deg = 0.05; radius_deg <= 3.0; radius_deg += 0.05) {
        for (int bearing = 0; bearing < 360; bearing += 5) {
            const double br = degToRad(static_cast<double>(bearing));
            const double lat = clampLatitude(p.lat_deg + radius_deg * std::sin(br));
            const double cos_lat = std::max(0.2, std::cos(degToRad(lat)));
            const double lon = p.lon_deg + radius_deg * std::cos(br) / cos_lat;
            const LatLon candidate{lat, lon};
            if (isNavigablePointImpl(candidate, land_polygons, land_index, clearance_km, shallow_polygons,
                                     shallow_index)) {
                return candidate;
            }
        }
    }

    return nearestSeaPoint(p, land_polygons, land_index, clearance_km);
}

std::vector<GridCell> generateAdaptiveGridCells(double min_lat,
                                                double max_lat,
                                                double min_lon,
                                                double max_lon,
                                                double base_step_deg,
                                                const std::vector<LandPolygon>& land_polygons,
                                                const LandSpatialIndex& land_index,
                                                double clearance_km,
                                                double narrow_refine_step_deg) {
    struct Cell {
        double min_lat;
        double max_lat;
        double min_lon;
        double max_lon;
        int depth;
    };

    std::vector<GridCell> cells;
    if (base_step_deg <= 0.0 || max_lat <= min_lat || max_lon <= min_lon) {
        return cells;
    }

    std::unordered_map<std::uint64_t, bool> land_cache;
    land_cache.reserve(1 << 18);
    const auto quantizeDeg = [](double v) -> std::int32_t {
        constexpr double scale = 1e5;
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
        const bool on_land = isOnLandImpl(p, land_polygons, land_index, clearance_km);
        land_cache.emplace(key, on_land);
        return on_land;
    };

    const double min_cell_deg = base_step_deg;
    const int split_threshold = 2;
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
                if (!bboxesOverlap(cell.min_lat, cell.max_lat, cell.min_lon, cell.max_lon, e_min_lat, e_max_lat,
                                   e_min_lon, e_max_lon)) {
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
                                          const LandSpatialIndex& shallow_index) {
    std::vector<NamedPoint> points;
    const auto cells = generateAdaptiveGridCells(min_lat,
                                                 max_lat,
                                                 min_lon,
                                                 max_lon,
                                                 spacing_deg,
                                                 land_polygons,
                                                 land_index,
                                                 clearance_km,
                                                 narrow_refine_step_deg);
    points.reserve(cells.size());
    int id = 0;
    for (const auto& c : cells) {
        const LatLon center{0.5 * (c.min_lat + c.max_lat), 0.5 * (c.min_lon + c.max_lon)};
        if (!isNavigablePointImpl(center, land_polygons, land_index, clearance_km, shallow_polygons, shallow_index)) {
            continue;
        }
        std::ostringstream oss;
        oss << "G" << id++;
        points.push_back({oss.str(), center});
    }
    return points;
}

std::vector<Triangle> buildDelaunay(const std::vector<XY>& input_pts) {
    if (input_pts.size() < 3) {
        return {};
    }

    std::vector<XY> pts = input_pts;
    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();

    for (const auto& p : pts) {
        min_x = std::min(min_x, p.x);
        max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, p.y);
        max_y = std::max(max_y, p.y);
    }

    const double dx = max_x - min_x;
    const double dy = max_y - min_y;
    const double delta = std::max(dx, dy) * 30.0;
    const XY center{0.5 * (min_x + max_x), 0.5 * (min_y + max_y)};

    const int n = static_cast<int>(pts.size());
    const int s0 = n;
    const int s1 = n + 1;
    const int s2 = n + 2;
    pts.push_back({center.x - 2.0 * delta, center.y - delta});
    pts.push_back({center.x, center.y + 2.0 * delta});
    pts.push_back({center.x + 2.0 * delta, center.y - delta});

    std::vector<TriRaw> tris;
    tris.push_back({s0, s1, s2});
    ensureCCW(tris.back(), pts);

    for (int i = 0; i < n; ++i) {
        std::vector<int> bad;
        bad.reserve(64);
        for (int ti = 0; ti < static_cast<int>(tris.size()); ++ti) {
            if (pointInCircumcircle(tris[ti], pts, pts[i])) {
                bad.push_back(ti);
            }
        }

        std::unordered_set<int> bad_set(bad.begin(), bad.end());
        std::unordered_map<std::uint64_t, std::pair<int, int>> boundary_edges;
        std::unordered_map<std::uint64_t, int> edge_counts;

        for (int ti : bad) {
            const TriRaw& t = tris[ti];
            const std::pair<int, int> edges[3] = {{t.a, t.b}, {t.b, t.c}, {t.c, t.a}};
            for (const auto& e : edges) {
                const auto key = edgeKey(e.first, e.second);
                edge_counts[key] += 1;
                boundary_edges[key] = e;
            }
        }

        std::vector<TriRaw> kept;
        kept.reserve(tris.size() + boundary_edges.size());
        for (int ti = 0; ti < static_cast<int>(tris.size()); ++ti) {
            if (bad_set.find(ti) == bad_set.end()) {
                kept.push_back(tris[ti]);
            }
        }

        for (const auto& kv : edge_counts) {
            if (kv.second != 1) {
                continue;
            }
            const auto e = boundary_edges[kv.first];
            TriRaw nt{e.first, e.second, i};
            ensureCCW(nt, pts);
            kept.push_back(nt);
        }

        tris.swap(kept);
    }

    std::vector<Triangle> out;
    out.reserve(tris.size());
    std::unordered_set<std::uint64_t> unique_tri;

    for (const auto& t : tris) {
        if (t.a >= n || t.b >= n || t.c >= n) {
            continue;
        }
        int v[3] = {t.a, t.b, t.c};
        std::sort(v, v + 3);
        const std::uint64_t key = (static_cast<std::uint64_t>(v[0]) << 42U) |
                                  (static_cast<std::uint64_t>(v[1]) << 21U) |
                                  static_cast<std::uint64_t>(v[2]);
        if (!unique_tri.insert(key).second) {
            continue;
        }

        const XY c{(pts[t.a].x + pts[t.b].x + pts[t.c].x) / 3.0,
                   (pts[t.a].y + pts[t.b].y + pts[t.c].y) / 3.0};
        out.push_back({t.a, t.b, t.c, c});
    }

    return out;
}

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
                                               const LandSpatialIndex& shallow_index) {
    (void)min_lat;
    (void)max_lat;
    (void)min_lon;
    (void)max_lon;
    (void)outer_frame_margin_deg;
    std::vector<Triangle> out;
    out.reserve(all_triangles.size());
    for (const auto& tri : all_triangles) {
        if (triangleNavigable(tri, vertices, land_polygons, land_index, clearance_km, shallow_polygons,
                              shallow_index)) {
            out.push_back(tri);
        }
    }
    return out;
}

std::vector<std::vector<NeighborPortal>> buildTriangleAdjacency(const std::vector<Triangle>& triangles) {
    std::vector<std::vector<NeighborPortal>> adj(triangles.size());
    std::unordered_map<std::uint64_t, std::pair<int, std::pair<int, int>>> edge_owner;

    for (int ti = 0; ti < static_cast<int>(triangles.size()); ++ti) {
        const Triangle& t = triangles[ti];
        const std::pair<int, int> edges[3] = {{t.a, t.b}, {t.b, t.c}, {t.c, t.a}};

        for (const auto& e : edges) {
            const auto key = edgeKey(e.first, e.second);
            const auto it = edge_owner.find(key);
            if (it == edge_owner.end()) {
                edge_owner[key] = {ti, {e.first, e.second}};
            } else {
                const int tj = it->second.first;
                const int va = it->second.second.first;
                const int vb = it->second.second.second;
                adj[ti].push_back({tj, va, vb});
                adj[tj].push_back({ti, va, vb});
            }
        }
    }

    return adj;
}

std::vector<int> buildVertexUsage(const std::vector<Triangle>& triangles, int vertex_count) {
    std::vector<int> usage(vertex_count, 0);
    for (const auto& t : triangles) {
        usage[t.a] += 1;
        usage[t.b] += 1;
        usage[t.c] += 1;
    }
    return usage;
}

std::vector<std::pair<double, int>> rankUsableVerticesByDistance(const std::vector<NamedPoint>& vertices,
                                                                 const std::vector<int>& usage,
                                                                 const LatLon& target) {
    std::vector<std::pair<double, int>> ranked;
    ranked.reserve(vertices.size());
    for (int i = 0; i < static_cast<int>(vertices.size()); ++i) {
        if (usage[i] <= 0) {
            continue;
        }
        ranked.push_back({haversineKm(vertices[i].geo, target), i});
    }
    std::sort(ranked.begin(), ranked.end(), [](const auto& a, const auto& b) {
        if (a.first != b.first) {
            return a.first < b.first;
        }
        return a.second < b.second;
    });
    return ranked;
}

std::vector<int> buildVertexComponentIds(const std::vector<Triangle>& triangles,
                                         int vertex_count,
                                         const std::vector<int>& usage) {
    std::vector<std::vector<int>> adj(vertex_count);
    for (const auto& t : triangles) {
        adj[t.a].push_back(t.b);
        adj[t.a].push_back(t.c);
        adj[t.b].push_back(t.a);
        adj[t.b].push_back(t.c);
        adj[t.c].push_back(t.a);
        adj[t.c].push_back(t.b);
    }

    std::vector<int> comp_id(vertex_count, -1);
    int next_comp = 0;
    std::vector<int> stack;
    for (int v = 0; v < vertex_count; ++v) {
        if (usage[v] <= 0 || comp_id[v] >= 0) {
            continue;
        }
        stack.clear();
        stack.push_back(v);
        comp_id[v] = next_comp;
        while (!stack.empty()) {
            const int cur = stack.back();
            stack.pop_back();
            for (int nb : adj[cur]) {
                if (nb < 0 || nb >= vertex_count || usage[nb] <= 0 || comp_id[nb] >= 0) {
                    continue;
                }
                comp_id[nb] = next_comp;
                stack.push_back(nb);
            }
        }
        next_comp += 1;
    }
    return comp_id;
}

std::vector<LatLon> shortcutPath(const std::vector<LatLon>& raw,
                                 const std::vector<LandPolygon>& land_polygons,
                                 const LandSpatialIndex& land_index,
                                 double clearance_km,
                                 const std::vector<LandPolygon>& shallow_polygons,
                                 const LandSpatialIndex& shallow_index) {
    if (raw.size() <= 2) {
        return raw;
    }

    std::vector<LatLon> out;
    out.reserve(raw.size());
    int i = 0;
    out.push_back(raw.front());

    while (i < static_cast<int>(raw.size()) - 1) {
        int best = -1;
        for (int j = static_cast<int>(raw.size()) - 1; j > i; --j) {
            if (edgeNavigable(raw[i], raw[j], land_polygons, land_index, clearance_km, shallow_polygons,
                              shallow_index)) {
                best = j;
                break;
            }
        }
        if (best < 0) {
            return {};
        }
        out.push_back(raw[best]);
        i = best;
    }

    return out;
}

double polylineLengthKm(const std::vector<LatLon>& line) {
    if (line.size() < 2) {
        return 0.0;
    }
    double total = 0.0;
    for (size_t i = 1; i < line.size(); ++i) {
        total += haversineKm(line[i - 1], line[i]);
    }
    return total;
}

void printRoute(const RouteResult& route) {
    if (!route.found) {
        std::cout << "Rota bulunamadi.\n";
        return;
    }

    std::cout << std::fixed << std::setprecision(1);
    std::cout << "Toplam rota: " << route.total_km << " km\n";
    std::cout << "Polyline noktasi: " << route.polyline.size() << "\n";
    for (size_t i = 0; i < route.polyline.size(); ++i) {
        const auto& p = route.polyline[i];
        std::cout << "  " << (i + 1) << ". (" << p.lat_deg << ", " << p.lon_deg << ")";
        if (i > 0) {
            std::cout << " leg=" << haversineKm(route.polyline[i - 1], route.polyline[i]) << " km";
        }
        std::cout << "\n";
    }
}

}  // namespace planner
