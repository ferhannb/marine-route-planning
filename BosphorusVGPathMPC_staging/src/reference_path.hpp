#pragma once

#include "planner.h"
#include "rota_optimal_ds.hpp"

#include <optional>
#include <string>
#include <vector>

struct LocalReferenceFrame {
    planner::Projection projection{};
    planner::XY origin_xy{};
};

struct ReferenceSample {
    double s_km = 0.0;
    double x_km = 0.0;
    double y_km = 0.0;
    double heading_rad = 0.0;
    double curvature_per_km = 0.0;
    planner::LatLon geo{};
};

std::vector<ReferenceSample> buildUniformReferenceSamples(const std::vector<planner::LatLon>& route,
                                                          const planner::Projection& projection,
                                                          double step_km,
                                                          LocalReferenceFrame* out_frame = nullptr);
std::vector<Waypoint> buildWaypointsFromReferenceSamples(const std::vector<ReferenceSample>& samples,
                                                         double waypoint_tol_km,
                                                         std::optional<double> w_wp = std::nullopt,
                                                         std::optional<double> hit_scale = std::nullopt);
State4 buildInitialStateFromReference(const std::vector<ReferenceSample>& samples);
planner::LatLon localToGeo(const planner::XY& local_xy, const LocalReferenceFrame& frame);
std::vector<planner::LatLon> recedingLogToGeoPolyline(const RecedingLog& log, const LocalReferenceFrame& frame);
bool writeReferenceSamplesCsv(const std::vector<ReferenceSample>& samples,
                              const std::string& file_path,
                              std::string* error = nullptr);
bool writeMpcGeoCsv(const RecedingLog& log,
                    const LocalReferenceFrame& frame,
                    const std::string& file_path,
                    std::string* error = nullptr);
