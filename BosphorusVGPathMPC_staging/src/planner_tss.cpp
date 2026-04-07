#include "planner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_set>

namespace planner {
namespace {

constexpr double kEps = 1e-9;

double cross(const XY& a, const XY& b, const XY& c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

double dot(const XY& a, const XY& b) {
    return a.x * b.x + a.y * b.y;
}

double sqrNorm(const XY& v) {
    return dot(v, v);
}

double xyDistanceKm(const XY& a, const XY& b) {
    return std::sqrt(sqrNorm(XY{a.x - b.x, a.y - b.y}));
}

bool onSegment(const XY& a, const XY& b, const XY& p) {
    if (std::fabs(cross(a, b, p)) > kEps) {
        return false;
    }
    return p.x >= std::min(a.x, b.x) - kEps && p.x <= std::max(a.x, b.x) + kEps &&
           p.y >= std::min(a.y, b.y) - kEps && p.y <= std::max(a.y, b.y) + kEps;
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
        return xyDistanceKm(p, a);
    }

    const XY ap{p.x - a.x, p.y - a.y};
    const double t = std::max(0.0, std::min(1.0, dot(ap, ab) / ab2));
    const XY proj{a.x + t * ab.x, a.y + t * ab.y};
    return xyDistanceKm(p, proj);
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

double pointToPolygonDistanceKm(const XY& p, const std::vector<XY>& poly) {
    if (poly.size() < 3) {
        return std::numeric_limits<double>::infinity();
    }
    if (pointInPolygon(p, poly)) {
        return 0.0;
    }

    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < poly.size(); ++i) {
        best = std::min(best, pointToSegmentDistanceKm(p, poly[i], poly[(i + 1) % poly.size()]));
    }
    return best;
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

bool seamarkTypeContains(const std::string& seamark_type, const char* token) {
    return seamark_type.find(token) != std::string::npos;
}

TrafficFlowDirection inferVoyageFlowDirection(const XY& start_xy, const XY& goal_xy) {
    const double dx = goal_xy.x - start_xy.x;
    const double dy = goal_xy.y - start_xy.y;
    if (std::fabs(dy) < std::max(1.5, std::fabs(dx) * 0.2)) {
        return TrafficFlowDirection::Unknown;
    }
    return dy >= 0.0 ? TrafficFlowDirection::Northbound : TrafficFlowDirection::Southbound;
}

TrafficFlowDirection classifyLaneFlowDirection(const std::vector<XY>& line_xy) {
    if (line_xy.size() < 2) {
        return TrafficFlowDirection::Unknown;
    }

    const double dx = line_xy.back().x - line_xy.front().x;
    const double dy = line_xy.back().y - line_xy.front().y;
    if (std::fabs(dy) < std::max(0.25, std::fabs(dx) * 0.35)) {
        return TrafficFlowDirection::Unknown;
    }
    return dy >= 0.0 ? TrafficFlowDirection::Northbound : TrafficFlowDirection::Southbound;
}

bool flowDirectionMatches(TrafficFlowDirection voyage_flow, TrafficFlowDirection lane_flow) {
    return voyage_flow == TrafficFlowDirection::Unknown || lane_flow == TrafficFlowDirection::Unknown ||
           voyage_flow == lane_flow;
}

double starboardBoundaryPenaltyKm(const XY& a, const XY& b, const TriangleTrafficData& traffic_data);

bool hasLaneGeometry(const TriangleTrafficData& traffic_data) {
    return traffic_data.has_lane_polygons || traffic_data.has_lane_lines ||
           !traffic_data.lane_endpoint_points_xy.empty();
}

bool pointInsideTrafficLaneImpl(const XY& p, const TriangleTrafficData& traffic_data) {
    if (!hasLaneGeometry(traffic_data)) {
        return false;
    }

    if (traffic_data.has_lane_polygons) {
        for (const auto& lane : traffic_data.lane_polygons_xy) {
            if (pointInPolygon(p, lane)) {
                return true;
            }
        }
    }

    for (const auto& endpoint : traffic_data.lane_endpoint_points_xy) {
        if (xyDistanceKm(p, endpoint) <= traffic_data.config.lane_endpoint_radius_km) {
            return true;
        }
    }

    if (traffic_data.has_lane_lines) {
        for (const auto& lane_line : traffic_data.lane_lines_xy) {
            if (pointToPolylineDistanceKm(p, lane_line) <= traffic_data.config.lane_corridor_half_width_km) {
                return true;
            }
        }
    }

    return false;
}

bool pointInsideTrafficZone(const XY& p, const TriangleTrafficData& traffic_data) {
    for (const auto& zone : traffic_data.zone_polygons_xy) {
        if (pointInPolygon(p, zone)) {
            return true;
        }
    }
    return false;
}

double pointToLaneDistanceKm(const XY& p, const TriangleTrafficData& traffic_data) {
    double best = std::numeric_limits<double>::infinity();

    for (const auto& lane : traffic_data.lane_polygons_xy) {
        best = std::min(best, pointToPolygonDistanceKm(p, lane));
    }
    for (const auto& line : traffic_data.lane_lines_xy) {
        best = std::min(best, pointToPolylineDistanceKm(p, line));
    }
    for (const auto& endpoint : traffic_data.lane_endpoint_points_xy) {
        best = std::min(best, xyDistanceKm(p, endpoint));
    }

    return best;
}

bool segmentCrossesBoundary(const XY& a, const XY& b, const TriangleTrafficData& traffic_data, int* crossings = nullptr) {
    bool hit = false;
    int count = 0;
    for (const auto& boundary : traffic_data.boundary_lines_xy) {
        if (segmentIntersectsPolyline(a, b, boundary)) {
            hit = true;
            count += 1;
        }
    }
    if (crossings != nullptr) {
        *crossings = count;
    }
    return hit;
}

bool segmentHitsZone(const XY& a, const XY& b, const TriangleTrafficData& traffic_data) {
    for (const auto& zone : traffic_data.zone_polygons_xy) {
        if (segmentIntersectsPolygon(a, b, zone)) {
            return true;
        }
    }
    return false;
}

bool xySegmentAllowedByTrafficImpl(const XY& a, const XY& b, const TriangleTrafficData& traffic_data) {
    if (!traffic_data.active || !traffic_data.config.hard_tss_corridor) {
        return true;
    }
    if (!hasLaneGeometry(traffic_data)) {
        return true;
    }
    if (!pointInsideTrafficLaneImpl(a, traffic_data) || !pointInsideTrafficLaneImpl(b, traffic_data)) {
        return false;
    }
    if (segmentHitsZone(a, b, traffic_data)) {
        return false;
    }

    const double seg_len_km = xyDistanceKm(a, b);
    const int sample_count = std::max(2, std::min(64, static_cast<int>(std::ceil(seg_len_km / 0.25))));
    for (int i = 1; i < sample_count; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(sample_count);
        const XY p{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t};
        if (!pointInsideTrafficLaneImpl(p, traffic_data) || pointInsideTrafficZone(p, traffic_data)) {
            return false;
        }
    }
    return true;
}

double segmentTrafficCost(const XY& a, const XY& b, const TriangleTrafficData& traffic_data) {
    const double seg_len_km = xyDistanceKm(a, b);
    if (!traffic_data.active || seg_len_km <= 0.0) {
        return seg_len_km;
    }
    if (traffic_data.config.hard_tss_corridor && !xySegmentAllowedByTrafficImpl(a, b, traffic_data)) {
        return std::numeric_limits<double>::infinity();
    }

    double multiplier = 1.0;
    if (hasLaneGeometry(traffic_data)) {
        const int sample_count = std::max(3, std::min(96, static_cast<int>(std::ceil(seg_len_km / 0.25)) + 1));
        int lane_hits = 0;
        for (int i = 0; i < sample_count; ++i) {
            const double t = sample_count == 1 ? 0.0 : static_cast<double>(i) / static_cast<double>(sample_count - 1);
            const XY p{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t};
            if (pointInsideTrafficLaneImpl(p, traffic_data)) {
                lane_hits += 1;
            }
        }
        const double lane_ratio = static_cast<double>(lane_hits) / static_cast<double>(sample_count);
        const double off_ratio = 1.0 - lane_ratio;
        multiplier = lane_ratio * traffic_data.config.lane_preference_multiplier +
                     off_ratio * traffic_data.config.off_lane_multiplier;
    }

    double cost = seg_len_km * multiplier;
    if (segmentHitsZone(a, b, traffic_data)) {
        cost += traffic_data.config.separation_zone_penalty_km;
    }
    if (traffic_data.flow_direction != TrafficFlowDirection::Unknown) {
        cost += starboardBoundaryPenaltyKm(a, b, traffic_data);
    }
    int boundary_crossings = 0;
    segmentCrossesBoundary(a, b, traffic_data, &boundary_crossings);
    if (boundary_crossings > 0) {
        cost += static_cast<double>(boundary_crossings) * traffic_data.config.boundary_crossing_penalty_km;
    }
    return cost;
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

bool triangleInsideHardCorridor(const TriangleTrafficData& traffic_data, int tri_idx) {
    return tri_idx >= 0 && tri_idx < static_cast<int>(traffic_data.triangle_in_hard_corridor.size()) &&
           traffic_data.triangle_in_hard_corridor[tri_idx] != 0;
}

struct PolylineProjection {
    XY point{};
    std::size_t seg_idx = 0;
    double seg_t = 0.0;
    double distance_km = std::numeric_limits<double>::infinity();
    double along_km = 0.0;
};

bool almostSamePoint(const XY& a, const XY& b) {
    return xyDistanceKm(a, b) <= 1e-6;
}

PolylineProjection projectPointOntoPolyline(const XY& p, const std::vector<XY>& line) {
    PolylineProjection best;
    if (line.empty()) {
        return best;
    }
    if (line.size() == 1) {
        best.point = line.front();
        best.distance_km = xyDistanceKm(p, line.front());
        return best;
    }

    double prefix_km = 0.0;
    for (std::size_t i = 1; i < line.size(); ++i) {
        const XY& a = line[i - 1];
        const XY& b = line[i];
        const XY ab{b.x - a.x, b.y - a.y};
        const double ab2 = sqrNorm(ab);
        double t = 0.0;
        if (ab2 > 1e-16) {
            const XY ap{p.x - a.x, p.y - a.y};
            t = std::max(0.0, std::min(1.0, dot(ap, ab) / ab2));
        }
        const XY proj{a.x + t * ab.x, a.y + t * ab.y};
        const double dist = xyDistanceKm(p, proj);
        if (dist + 1e-9 < best.distance_km) {
            best.point = proj;
            best.seg_idx = i - 1;
            best.seg_t = t;
            best.distance_km = dist;
            best.along_km = prefix_km + xyDistanceKm(a, proj);
        }
        prefix_km += xyDistanceKm(a, b);
    }
    return best;
}

std::vector<XY> extractPolylineSlice(const std::vector<XY>& line,
                                     const PolylineProjection& start_proj,
                                     const PolylineProjection& goal_proj) {
    std::vector<XY> out;
    if (line.size() < 2 || goal_proj.along_km + 1e-6 < start_proj.along_km) {
        return out;
    }

    out.push_back(start_proj.point);
    for (std::size_t i = start_proj.seg_idx + 1; i <= goal_proj.seg_idx && i < line.size(); ++i) {
        if (!almostSamePoint(out.back(), line[i])) {
            out.push_back(line[i]);
        }
    }
    if (out.empty() || !almostSamePoint(out.back(), goal_proj.point)) {
        out.push_back(goal_proj.point);
    }
    return out;
}

std::vector<XY> reverseLine(const std::vector<XY>& line) {
    std::vector<XY> out = line;
    std::reverse(out.begin(), out.end());
    return out;
}

XY polylineTangentAtVertex(const std::vector<XY>& line, std::size_t idx) {
    if (line.size() < 2) {
        return XY{};
    }
    if (idx == 0) {
        return XY{line[1].x - line[0].x, line[1].y - line[0].y};
    }
    if (idx + 1 >= line.size()) {
        return XY{line.back().x - line[line.size() - 2].x, line.back().y - line[line.size() - 2].y};
    }
    return XY{line[idx + 1].x - line[idx - 1].x, line[idx + 1].y - line[idx - 1].y};
}

std::vector<std::vector<XY>> selectStarboardBoundaryLines(const std::vector<std::vector<XY>>& directional_lane_lines,
                                                          const std::vector<std::vector<XY>>& outer_boundary_lines) {
    std::vector<std::vector<XY>> out;
    if (directional_lane_lines.empty() || outer_boundary_lines.empty()) {
        return out;
    }

    std::vector<std::size_t> used_indices;
    used_indices.reserve(directional_lane_lines.size());
    for (const auto& lane_line : directional_lane_lines) {
        if (lane_line.size() < 2) {
            continue;
        }

        const std::size_t mid_idx = lane_line.size() / 2;
        const XY lane_mid = lane_line[mid_idx];
        const XY tangent = polylineTangentAtVertex(lane_line, mid_idx);
        if (sqrNorm(tangent) < 1e-12) {
            continue;
        }

        int best_starboard_idx = -1;
        double best_starboard_dist = std::numeric_limits<double>::infinity();
        int best_any_idx = -1;
        double best_any_dist = std::numeric_limits<double>::infinity();
        for (std::size_t boundary_idx = 0; boundary_idx < outer_boundary_lines.size(); ++boundary_idx) {
            const auto& boundary = outer_boundary_lines[boundary_idx];
            const auto proj = projectPointOntoPolyline(lane_mid, boundary);
            if (!std::isfinite(proj.distance_km)) {
                continue;
            }

            if (proj.distance_km + 1e-9 < best_any_dist) {
                best_any_idx = static_cast<int>(boundary_idx);
                best_any_dist = proj.distance_km;
            }

            const XY to_boundary{proj.point.x - lane_mid.x, proj.point.y - lane_mid.y};
            const double signed_side = tangent.x * to_boundary.y - tangent.y * to_boundary.x;
            if (signed_side < -1e-6 && proj.distance_km + 1e-9 < best_starboard_dist) {
                best_starboard_idx = static_cast<int>(boundary_idx);
                best_starboard_dist = proj.distance_km;
            }
        }

        const int chosen_idx = best_starboard_idx >= 0 ? best_starboard_idx : best_any_idx;
        if (chosen_idx < 0) {
            continue;
        }

        const auto duplicate = std::find(used_indices.begin(), used_indices.end(), static_cast<std::size_t>(chosen_idx));
        if (duplicate != used_indices.end()) {
            continue;
        }
        used_indices.push_back(static_cast<std::size_t>(chosen_idx));
        out.push_back(outer_boundary_lines[chosen_idx]);
    }

    return out;
}

double starboardBoundaryPenaltyKm(const XY& a, const XY& b, const TriangleTrafficData& traffic_data) {
    if (traffic_data.starboard_boundary_lines_xy.empty() || traffic_data.config.starboard_boundary_bias <= 0.0) {
        return 0.0;
    }

    const double seg_len_km = xyDistanceKm(a, b);
    if (seg_len_km <= 0.0) {
        return 0.0;
    }

    const int sample_count = std::max(3, std::min(96, static_cast<int>(std::ceil(seg_len_km / 0.25)) + 1));
    double distance_sum_km = 0.0;
    int sample_hits = 0;
    for (int i = 0; i < sample_count; ++i) {
        const double t = sample_count == 1 ? 0.0 : static_cast<double>(i) / static_cast<double>(sample_count - 1);
        const XY p{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t};
        if (!pointInsideTrafficLaneImpl(p, traffic_data)) {
            continue;
        }

        double best = std::numeric_limits<double>::infinity();
        for (const auto& boundary : traffic_data.starboard_boundary_lines_xy) {
            best = std::min(best, pointToPolylineDistanceKm(p, boundary));
        }
        if (std::isfinite(best)) {
            distance_sum_km += best;
            sample_hits += 1;
        }
    }

    if (sample_hits == 0) {
        return 0.0;
    }

    const double avg_distance_km = distance_sum_km / static_cast<double>(sample_hits);
    const double width_scale_km = std::max(0.25, traffic_data.config.lane_corridor_half_width_km);
    return seg_len_km * (avg_distance_km / width_scale_km) * traffic_data.config.starboard_boundary_bias;
}

double transitionHeuristicKm(const std::vector<Triangle>& triangles,
                             int tri_idx,
                             int goal_vertex,
                             const std::vector<XY>& vertex_xy,
                             const TriangleTrafficData* traffic_data) {
    double multiplier = 1.0;
    if (traffic_data != nullptr && traffic_data->active) {
        multiplier = std::min(traffic_data->config.lane_preference_multiplier,
                              traffic_data->config.off_lane_multiplier);
    }
    return xyDistanceKm(triangles[tri_idx].centroid, vertex_xy[goal_vertex]) * multiplier;
}

double transitionCost(const XY& a, const XY& b, const TriangleTrafficData* traffic_data) {
    if (traffic_data == nullptr) {
        return xyDistanceKm(a, b);
    }
    return segmentTrafficCost(a, b, *traffic_data);
}

}  // namespace

TriangleTrafficData buildTriangleTrafficData(const std::vector<Triangle>& triangles,
                                             const Projection& proj,
                                             const std::vector<TssFeature>& tss_features,
                                             const LatLon& start,
                                             const LatLon& goal,
                                             const TrafficRoutingConfig& config) {
    TriangleTrafficData data;
    data.config = config;
    data.triangle_in_lane.assign(triangles.size(), 0);
    data.triangle_in_zone.assign(triangles.size(), 0);
    data.triangle_in_hard_corridor.assign(triangles.size(), 0);
    data.triangle_lane_distance_km.assign(triangles.size(), std::numeric_limits<double>::infinity());

    if (!config.enable_tss_routing || triangles.empty() || tss_features.empty()) {
        return data;
    }

    data.flow_direction =
        config.directional_lane_selection ? inferVoyageFlowDirection(toXY(start, proj), toXY(goal, proj))
                                          : TrafficFlowDirection::Unknown;

    std::vector<std::vector<XY>> fallback_lane_polygons_xy;
    std::vector<std::vector<XY>> fallback_lane_lines_xy;
    std::vector<XY> fallback_lane_endpoint_points_xy;
    std::vector<std::vector<XY>> directional_lane_lines_xy;
    std::vector<std::vector<XY>> outer_boundary_lines_xy;

    for (const auto& feature : tss_features) {
        if (feature.points.size() < 2) {
            continue;
        }

        std::vector<XY> points_xy;
        points_xy.reserve(feature.points.size());
        for (const auto& p : feature.points) {
            points_xy.push_back(toXY(p, proj));
        }

        if (seamarkTypeContains(feature.seamark_type, "separation_lane")) {
            if (feature.closed && points_xy.size() >= 3) {
                fallback_lane_polygons_xy.push_back(points_xy);
                data.lane_polygons_xy.push_back(points_xy);
            } else if (points_xy.size() >= 2) {
                fallback_lane_endpoint_points_xy.push_back(points_xy.front());
                fallback_lane_endpoint_points_xy.push_back(points_xy.back());
                fallback_lane_lines_xy.push_back(points_xy);

                const TrafficFlowDirection lane_flow = classifyLaneFlowDirection(points_xy);
                if (data.flow_direction != TrafficFlowDirection::Unknown && lane_flow == data.flow_direction) {
                    directional_lane_lines_xy.push_back(points_xy);
                }
                if (flowDirectionMatches(data.flow_direction, lane_flow)) {
                    data.lane_endpoint_points_xy.push_back(points_xy.front());
                    data.lane_endpoint_points_xy.push_back(points_xy.back());
                    data.lane_lines_xy.push_back(points_xy);
                }
            }
            continue;
        }

        if (seamarkTypeContains(feature.seamark_type, "separation_zone")) {
            if (feature.closed && points_xy.size() >= 3) {
                data.zone_polygons_xy.push_back(points_xy);
            }
            continue;
        }

        if (seamarkTypeContains(feature.seamark_type, "separation_boundary") ||
            seamarkTypeContains(feature.seamark_type, "separation_line")) {
            if (points_xy.size() >= 2) {
                data.boundary_lines_xy.push_back(points_xy);
                if (seamarkTypeContains(feature.seamark_type, "separation_boundary")) {
                    outer_boundary_lines_xy.push_back(points_xy);
                }
            }
        }
    }

    if (!fallback_lane_lines_xy.empty() && data.lane_lines_xy.empty()) {
        if (data.lane_polygons_xy.empty()) {
            data.lane_polygons_xy = fallback_lane_polygons_xy;
        }
        data.lane_lines_xy = fallback_lane_lines_xy;
        data.lane_endpoint_points_xy = fallback_lane_endpoint_points_xy;
        directional_lane_lines_xy.clear();
        data.flow_direction = TrafficFlowDirection::Unknown;
    }

    data.starboard_boundary_lines_xy = selectStarboardBoundaryLines(directional_lane_lines_xy, outer_boundary_lines_xy);
    data.has_lane_polygons = !data.lane_polygons_xy.empty();
    data.has_lane_lines = !data.lane_lines_xy.empty();
    data.active = data.has_lane_polygons || data.has_lane_lines || !data.zone_polygons_xy.empty() ||
                  !data.boundary_lines_xy.empty();

    if (!data.active) {
        return data;
    }

    for (size_t tri_idx = 0; tri_idx < triangles.size(); ++tri_idx) {
        const XY& centroid = triangles[tri_idx].centroid;
        const bool in_lane = pointInsideTrafficLaneImpl(centroid, data);
        const bool in_zone = pointInsideTrafficZone(centroid, data);

        data.triangle_in_lane[tri_idx] = in_lane ? 1U : 0U;
        data.triangle_in_zone[tri_idx] = in_zone ? 1U : 0U;
        if (hasLaneGeometry(data) && in_lane && !in_zone) {
            data.triangle_in_hard_corridor[tri_idx] = 1U;
        }
        data.triangle_lane_distance_km[tri_idx] = pointToLaneDistanceKm(centroid, data);
    }

    return data;
}

bool xySegmentAllowedByTraffic(const XY& a, const XY& b, const TriangleTrafficData& traffic_data) {
    return xySegmentAllowedByTrafficImpl(a, b, traffic_data);
}

TriangleSearchResult searchTriangleSequence(const std::vector<Triangle>& triangles,
                                            const std::vector<std::vector<NeighborPortal>>& adj,
                                            int start_vertex,
                                            int goal_vertex,
                                            const std::vector<XY>& vertex_xy,
                                            const TriangleTrafficData* traffic_data) {
    TriangleSearchResult out;

    auto start_tris = trianglesUsingVertex(triangles, start_vertex);
    auto goal_tris = trianglesUsingVertex(triangles, goal_vertex);
    if (start_tris.empty() || goal_tris.empty()) {
        return out;
    }

    const bool enforce_hard_corridor =
        traffic_data != nullptr && traffic_data->active && traffic_data->config.hard_tss_corridor &&
        std::any_of(traffic_data->triangle_in_hard_corridor.begin(),
                    traffic_data->triangle_in_hard_corridor.end(),
                    [](std::uint8_t value) { return value != 0; });

    if (enforce_hard_corridor) {
        start_tris.erase(std::remove_if(start_tris.begin(),
                                        start_tris.end(),
                                        [&](int tri_idx) { return !triangleInsideHardCorridor(*traffic_data, tri_idx); }),
                         start_tris.end());
        goal_tris.erase(std::remove_if(goal_tris.begin(),
                                       goal_tris.end(),
                                       [&](int tri_idx) { return !triangleInsideHardCorridor(*traffic_data, tri_idx); }),
                        goal_tris.end());
        if (start_tris.empty() || goal_tris.empty()) {
            return out;
        }
    }

    std::unordered_set<int> goal_set(goal_tris.begin(), goal_tris.end());

    struct State {
        int tri = -1;
        double f = std::numeric_limits<double>::infinity();
        bool operator>(const State& other) const { return f > other.f; }
    };

    const int n = static_cast<int>(triangles.size());
    std::vector<double> g(n, std::numeric_limits<double>::infinity());
    std::vector<int> parent(n, -1);
    std::priority_queue<State, std::vector<State>, std::greater<State>> pq;

    for (int s : start_tris) {
        const double start_cost = transitionCost(vertex_xy[start_vertex], triangles[s].centroid, traffic_data);
        if (!std::isfinite(start_cost)) {
            continue;
        }
        g[s] = start_cost;
        pq.push({s, start_cost + transitionHeuristicKm(triangles, s, goal_vertex, vertex_xy, traffic_data)});
    }

    int reached = -1;
    while (!pq.empty()) {
        const State cur = pq.top();
        pq.pop();

        const double expected_f =
            g[cur.tri] + transitionHeuristicKm(triangles, cur.tri, goal_vertex, vertex_xy, traffic_data);
        if (cur.f > expected_f + 1e-6) {
            continue;
        }

        if (goal_set.find(cur.tri) != goal_set.end()) {
            reached = cur.tri;
            break;
        }

        for (const auto& nb : adj[cur.tri]) {
            if (enforce_hard_corridor && !triangleInsideHardCorridor(*traffic_data, nb.to)) {
                continue;
            }

            const double step_cost = transitionCost(triangles[cur.tri].centroid, triangles[nb.to].centroid, traffic_data);
            if (!std::isfinite(step_cost)) {
                continue;
            }

            const double tentative = g[cur.tri] + step_cost;
            if (tentative + 1e-6 < g[nb.to]) {
                g[nb.to] = tentative;
                parent[nb.to] = cur.tri;
                pq.push({nb.to,
                         tentative + transitionHeuristicKm(triangles, nb.to, goal_vertex, vertex_xy, traffic_data)});
            }
        }
    }

    if (reached < 0) {
        return out;
    }

    out.found = true;
    for (int at = reached; at != -1; at = parent[at]) {
        out.triangle_sequence.push_back(at);
    }
    std::reverse(out.triangle_sequence.begin(), out.triangle_sequence.end());
    return out;
}

std::vector<LatLon> shortcutPathWithTraffic(const std::vector<LatLon>& raw,
                                            const Projection& proj,
                                            const std::vector<LandPolygon>& land_polygons,
                                            const LandSpatialIndex& land_index,
                                            double clearance_km,
                                            const std::vector<LandPolygon>& shallow_polygons,
                                            const LandSpatialIndex& shallow_index,
                                            const TriangleTrafficData& traffic_data) {
    if (raw.size() <= 2) {
        return raw;
    }

    std::vector<XY> raw_xy;
    raw_xy.reserve(raw.size());
    for (const auto& p : raw) {
        raw_xy.push_back(toXY(p, proj));
    }

    std::vector<LatLon> out;
    out.reserve(raw.size());
    out.push_back(raw.front());

    int i = 0;
    while (i < static_cast<int>(raw.size()) - 1) {
        int best = -1;
        for (int j = static_cast<int>(raw.size()) - 1; j > i; --j) {
            if (!edgeNavigable(raw[i],
                               raw[j],
                               land_polygons,
                               land_index,
                               clearance_km,
                               shallow_polygons,
                               shallow_index)) {
                continue;
            }

            const double direct_cost = segmentTrafficCost(raw_xy[i], raw_xy[j], traffic_data);
            if (!std::isfinite(direct_cost)) {
                continue;
            }

            double chain_cost = 0.0;
            bool chain_ok = true;
            for (int k = i + 1; k <= j; ++k) {
                const double leg_cost = segmentTrafficCost(raw_xy[k - 1], raw_xy[k], traffic_data);
                if (!std::isfinite(leg_cost)) {
                    chain_ok = false;
                    break;
                }
                chain_cost += leg_cost;
            }
            if (!chain_ok) {
                continue;
            }

            if (direct_cost <= chain_cost + 1e-6) {
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

std::vector<LatLon> buildRouteAlongTssCenterline(const LatLon& start,
                                                 const LatLon& goal,
                                                 const Projection& proj,
                                                 const std::vector<TssFeature>& tss_features,
                                                 const std::vector<LandPolygon>& land_polygons,
                                                 const LandSpatialIndex& land_index,
                                                 double clearance_km,
                                                 const std::vector<LandPolygon>& shallow_polygons,
                                                 const LandSpatialIndex& shallow_index) {
    const XY start_xy = toXY(start, proj);
    const XY goal_xy = toXY(goal, proj);
    const TrafficFlowDirection voyage_flow = inferVoyageFlowDirection(start_xy, goal_xy);

    struct LaneCandidate {
        std::vector<XY> line_xy;
        PolylineProjection start_proj;
        PolylineProjection goal_proj;
        double score = std::numeric_limits<double>::infinity();
    };

    LaneCandidate best;
    auto tryCandidate = [&](const std::vector<XY>& line_xy) {
        const auto start_proj = projectPointOntoPolyline(start_xy, line_xy);
        const auto goal_proj = projectPointOntoPolyline(goal_xy, line_xy);
        if (goal_proj.along_km + 1e-6 < start_proj.along_km) {
            return;
        }

        const double along_span_km = goal_proj.along_km - start_proj.along_km;
        const double score = start_proj.distance_km * 3.0 + goal_proj.distance_km * 3.0 + along_span_km;
        if (score + 1e-6 < best.score) {
            best.line_xy = line_xy;
            best.start_proj = start_proj;
            best.goal_proj = goal_proj;
            best.score = score;
        }
    };

    auto scanLanes = [&](bool strict_directional) {
        for (const auto& feature : tss_features) {
            if (feature.closed || feature.points.size() < 2 ||
                !seamarkTypeContains(feature.seamark_type, "separation_lane")) {
                continue;
            }

            std::vector<XY> base_line;
            base_line.reserve(feature.points.size());
            for (const auto& p : feature.points) {
                base_line.push_back(toXY(p, proj));
            }

            const TrafficFlowDirection lane_flow = classifyLaneFlowDirection(base_line);
            if (strict_directional) {
                if (voyage_flow == TrafficFlowDirection::Unknown || lane_flow != voyage_flow) {
                    continue;
                }
                tryCandidate(base_line);
                continue;
            }

            if (voyage_flow != TrafficFlowDirection::Unknown && lane_flow == voyage_flow) {
                tryCandidate(base_line);
                continue;
            }

            tryCandidate(base_line);
            tryCandidate(reverseLine(base_line));
        }
    };

    scanLanes(true);
    if (!std::isfinite(best.score)) {
        scanLanes(false);
    }

    if (!std::isfinite(best.score)) {
        return {};
    }

    const auto line_slice = extractPolylineSlice(best.line_xy, best.start_proj, best.goal_proj);
    if (line_slice.size() < 2) {
        return {};
    }

    std::vector<LatLon> route;
    route.reserve(line_slice.size() + 2);
    route.push_back(start);
    for (const auto& p : line_slice) {
        route.push_back(toGeo(p, proj));
    }
    route.push_back(goal);

    std::vector<LatLon> deduped;
    deduped.reserve(route.size());
    for (const auto& p : route) {
        if (!deduped.empty() && haversineKm(deduped.back(), p) < 0.01) {
            continue;
        }
        deduped.push_back(p);
    }

    if (deduped.size() < 2) {
        return {};
    }

    for (std::size_t i = 1; i < deduped.size(); ++i) {
        if (!edgeNavigable(deduped[i - 1],
                           deduped[i],
                           land_polygons,
                           land_index,
                           clearance_km,
                           shallow_polygons,
                           shallow_index)) {
            return {};
        }
    }

    return deduped;
}

}  // namespace planner
