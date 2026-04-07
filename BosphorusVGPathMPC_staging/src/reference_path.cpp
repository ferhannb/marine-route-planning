#include "reference_path.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace {

double wrapToPiLocal(double angle_rad) {
    return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

void ensureParentDir(const std::string& file_path) {
    const std::filesystem::path path(file_path);
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
    }
}

bool fail(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = message;
    }
    return false;
}

}  // namespace

std::vector<ReferenceSample> buildUniformReferenceSamples(const std::vector<planner::LatLon>& route,
                                                          const planner::Projection& projection,
                                                          double step_km,
                                                          LocalReferenceFrame* out_frame) {
    if (route.size() < 2) {
        return {};
    }

    if (step_km <= 0.0) {
        throw std::runtime_error("reference step_km must be > 0");
    }

    std::vector<planner::XY> absolute_xy;
    absolute_xy.reserve(route.size());
    for (const auto& point : route) {
        absolute_xy.push_back(planner::toXY(point, projection));
    }

    const planner::XY origin_xy = absolute_xy.front();
    if (out_frame != nullptr) {
        out_frame->projection = projection;
        out_frame->origin_xy = origin_xy;
    }

    std::vector<planner::XY> local_xy;
    local_xy.reserve(absolute_xy.size());
    for (const auto& point : absolute_xy) {
        local_xy.push_back({point.x - origin_xy.x, point.y - origin_xy.y});
    }

    std::vector<double> segment_length_km;
    segment_length_km.reserve(local_xy.size() - 1);
    std::vector<double> cumulative_km(local_xy.size(), 0.0);
    for (size_t i = 1; i < local_xy.size(); ++i) {
        const double dx = local_xy[i].x - local_xy[i - 1].x;
        const double dy = local_xy[i].y - local_xy[i - 1].y;
        const double length = std::hypot(dx, dy);
        segment_length_km.push_back(length);
        cumulative_km[i] = cumulative_km[i - 1] + length;
    }

    const double total_km = cumulative_km.back();
    if (total_km <= 0.0) {
        return {};
    }

    std::vector<ReferenceSample> samples;
    auto pushSampleAt = [&](double s_km) {
        size_t seg_idx = 0;
        while (seg_idx + 1 < cumulative_km.size() && cumulative_km[seg_idx + 1] < s_km) {
            ++seg_idx;
        }
        if (seg_idx >= segment_length_km.size()) {
            seg_idx = segment_length_km.size() - 1;
        }

        const double seg_start = cumulative_km[seg_idx];
        const double seg_len = std::max(segment_length_km[seg_idx], 1e-9);
        const double ratio = std::clamp((s_km - seg_start) / seg_len, 0.0, 1.0);
        const auto& p0 = local_xy[seg_idx];
        const auto& p1 = local_xy[seg_idx + 1];
        const double x = p0.x + ratio * (p1.x - p0.x);
        const double y = p0.y + ratio * (p1.y - p0.y);
        const double heading = std::atan2(p1.y - p0.y, p1.x - p0.x);
        const planner::LatLon geo = planner::toGeo({x + origin_xy.x, y + origin_xy.y}, projection);
        samples.push_back({s_km, x, y, heading, 0.0, geo});
    };

    for (double s = 0.0; s < total_km; s += step_km) {
        pushSampleAt(s);
    }

    const auto& last_local = local_xy.back();
    const double last_heading = samples.empty() ? 0.0 : samples.back().heading_rad;
    if (samples.empty() || std::abs(samples.back().s_km - total_km) > 1e-9) {
        samples.push_back({total_km, last_local.x, last_local.y, last_heading, 0.0, route.back()});
    } else {
        samples.back() = {total_km, last_local.x, last_local.y, last_heading, 0.0, route.back()};
    }

    if (samples.size() == 1) {
        samples.front().curvature_per_km = 0.0;
        return samples;
    }

    for (size_t i = 0; i < samples.size(); ++i) {
        if (i == 0) {
            const double ds = std::max(samples[1].s_km - samples[0].s_km, 1e-9);
            samples[i].curvature_per_km =
                wrapToPiLocal(samples[1].heading_rad - samples[0].heading_rad) / ds;
        } else if (i + 1 == samples.size()) {
            const double ds = std::max(samples[i].s_km - samples[i - 1].s_km, 1e-9);
            samples[i].curvature_per_km =
                wrapToPiLocal(samples[i].heading_rad - samples[i - 1].heading_rad) / ds;
        } else {
            const double ds = std::max(samples[i + 1].s_km - samples[i - 1].s_km, 1e-9);
            samples[i].curvature_per_km =
                wrapToPiLocal(samples[i + 1].heading_rad - samples[i - 1].heading_rad) / ds;
        }
    }

    return samples;
}

std::vector<Waypoint> buildWaypointsFromReferenceSamples(const std::vector<ReferenceSample>& samples,
                                                         double waypoint_tol_km,
                                                         std::optional<double> w_wp,
                                                         std::optional<double> hit_scale) {
    std::vector<Waypoint> waypoints;
    if (samples.size() < 2) {
        return waypoints;
    }

    waypoints.reserve(samples.size() - 1);
    for (size_t i = 1; i < samples.size(); ++i) {
        Waypoint wp;
        wp.X = samples[i].x_km;
        wp.Y = samples[i].y_km;
        wp.psig = samples[i].heading_rad;
        wp.Kf = samples[i].curvature_per_km;
        wp.tol = waypoint_tol_km;
        wp.use_Kf = true;
        wp.w_wp = w_wp;
        wp.hit_scale = hit_scale;
        waypoints.push_back(wp);
    }
    return waypoints;
}

State4 buildInitialStateFromReference(const std::vector<ReferenceSample>& samples) {
    if (samples.empty()) {
        return {};
    }
    return State4{samples.front().x_km,
                  samples.front().y_km,
                  samples.front().heading_rad,
                  samples.front().curvature_per_km};
}

planner::LatLon localToGeo(const planner::XY& local_xy, const LocalReferenceFrame& frame) {
    return planner::toGeo({local_xy.x + frame.origin_xy.x, local_xy.y + frame.origin_xy.y}, frame.projection);
}

std::vector<planner::LatLon> recedingLogToGeoPolyline(const RecedingLog& log, const LocalReferenceFrame& frame) {
    std::vector<planner::LatLon> route;
    route.reserve(log.traj.size());
    for (const auto& sample : log.traj) {
        route.push_back(localToGeo({sample.first, sample.second}, frame));
    }
    return route;
}

bool writeReferenceSamplesCsv(const std::vector<ReferenceSample>& samples,
                              const std::string& file_path,
                              std::string* error) {
    ensureParentDir(file_path);
    std::ofstream out(file_path);
    if (!out.is_open()) {
        return fail(error, "Reference CSV acilamadi: " + file_path);
    }

    out << "idx,s_km,x_km,y_km,heading_rad,curvature_per_km,lat_deg,lon_deg\n";
    for (size_t i = 0; i < samples.size(); ++i) {
        const auto& sample = samples[i];
        out << i << ',' << sample.s_km << ',' << sample.x_km << ',' << sample.y_km << ',' << sample.heading_rad
            << ',' << sample.curvature_per_km << ',' << sample.geo.lat_deg << ',' << sample.geo.lon_deg << '\n';
    }
    return true;
}

bool writeMpcGeoCsv(const RecedingLog& log,
                    const LocalReferenceFrame& frame,
                    const std::string& file_path,
                    std::string* error) {
    ensureParentDir(file_path);
    std::ofstream out(file_path);
    if (!out.is_open()) {
        return fail(error, "MPC geo CSV acilamadi: " + file_path);
    }

    out << "step,x_km,y_km,lat_deg,lon_deg,psi,K,Kcmd,ds,wp_index\n";
    for (size_t i = 0; i < log.traj.size(); ++i) {
        const planner::XY local_xy{log.traj[i].first, log.traj[i].second};
        const auto geo = localToGeo(local_xy, frame);
        const double psi = (i < log.psi.size()) ? log.psi[i] : 0.0;
        const double K = (i < log.K.size()) ? log.K[i] : 0.0;
        const double Kcmd = (i > 0 && (i - 1) < log.Kcmd.size()) ? log.Kcmd[i - 1] : 0.0;
        const double ds = (i > 0 && (i - 1) < log.ds.size()) ? log.ds[i - 1] : 0.0;
        const int wp_index = (i < log.wp_index.size()) ? log.wp_index[i] : -1;
        out << i << ',' << local_xy.x << ',' << local_xy.y << ',' << geo.lat_deg << ',' << geo.lon_deg << ','
            << psi << ',' << K << ',' << Kcmd << ',' << ds << ',' << wp_index << '\n';
    }
    return true;
}
