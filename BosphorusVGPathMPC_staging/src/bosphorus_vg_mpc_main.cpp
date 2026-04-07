#include "reference_path.hpp"
#include "route_planner_core.hpp"
#include "rota_optimal_ds.hpp"
#include "scenario_parser.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct Options {
    std::string route_config = "config/route_planner_bosphorus_mesh.ini";
    std::string mpc_scenario = "scenarios/bosphorus_tracking.ini";
    double reference_step_km = 0.20;
    double waypoint_tol_km = 0.15;
    std::string route_csv = "output/bosphorus_route.csv";
    std::string reference_csv = "output/bosphorus_reference.csv";
    std::string mpc_log_csv = "output/bosphorus_mpc_log.csv";
    std::string mpc_geo_csv = "output/bosphorus_mpc_geo.csv";
    std::string route_svg = "output/bosphorus_reference.svg";
    std::string mpc_svg = "output/bosphorus_mpc.svg";
};

void printHelp(const char* bin) {
    std::cout << "Usage: " << bin << " [options]\n"
              << "Options:\n"
              << "  --route-config <file>      Route planner config ini\n"
              << "  --mpc-scenario <file>      MPC tuning scenario ini\n"
              << "  --reference-step-km <v>    Uniform reference sampling step in km\n"
              << "  --waypoint-tol-km <v>      Dense waypoint reach tolerance in km\n"
              << "  --route-csv <file>         Geo route export CSV\n"
              << "  --reference-csv <file>     Uniform reference CSV\n"
              << "  --mpc-log-csv <file>       Raw MPC receding log CSV\n"
              << "  --mpc-geo-csv <file>       MPC trajectory geo CSV\n"
              << "  --route-svg <file>         Reference route SVG output\n"
              << "  --mpc-svg <file>           MPC trajectory SVG output\n"
              << "  --help                     Show this help\n";
}

bool parseArgs(int argc, char** argv, Options& options) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help") {
            printHelp(argv[0]);
            return false;
        }
        if (arg == "--route-config" && i + 1 < argc) {
            options.route_config = argv[++i];
            continue;
        }
        if (arg == "--mpc-scenario" && i + 1 < argc) {
            options.mpc_scenario = argv[++i];
            continue;
        }
        if (arg == "--reference-step-km" && i + 1 < argc) {
            options.reference_step_km = std::stod(argv[++i]);
            continue;
        }
        if (arg == "--waypoint-tol-km" && i + 1 < argc) {
            options.waypoint_tol_km = std::stod(argv[++i]);
            continue;
        }
        if (arg == "--route-csv" && i + 1 < argc) {
            options.route_csv = argv[++i];
            continue;
        }
        if (arg == "--reference-csv" && i + 1 < argc) {
            options.reference_csv = argv[++i];
            continue;
        }
        if (arg == "--mpc-log-csv" && i + 1 < argc) {
            options.mpc_log_csv = argv[++i];
            continue;
        }
        if (arg == "--mpc-geo-csv" && i + 1 < argc) {
            options.mpc_geo_csv = argv[++i];
            continue;
        }
        if (arg == "--route-svg" && i + 1 < argc) {
            options.route_svg = argv[++i];
            continue;
        }
        if (arg == "--mpc-svg" && i + 1 < argc) {
            options.mpc_svg = argv[++i];
            continue;
        }
        throw std::runtime_error("Unknown or incomplete argument: " + arg);
    }
    return true;
}

planner::CliConfig loadRouteConfig(const std::string& config_path) {
    planner::CliConfig cfg;
    bool should_run = true;

    std::vector<std::string> args = {"bosphorus_vg_mpc", "--config", config_path};
    std::vector<char*> argv;
    argv.reserve(args.size());
    for (auto& arg : args) {
        argv.push_back(arg.data());
    }

    if (!planner::parseCli(static_cast<int>(argv.size()), argv.data(), cfg, should_run)) {
        throw std::runtime_error("Route config parse edilemedi: " + config_path);
    }
    if (!should_run) {
        throw std::runtime_error("Route config execution iptal edildi: " + config_path);
    }
    return cfg;
}

double wrapAngle(double angle_rad) {
    return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

int findNearestForwardSample(const std::vector<ReferenceSample>& samples,
                             const State4& state,
                             int current_index,
                             int search_ahead) {
    if (samples.empty()) {
        return 0;
    }
    const int start = std::clamp(current_index, 0, static_cast<int>(samples.size()) - 1);
    const int stop = std::clamp(current_index + search_ahead, start, static_cast<int>(samples.size()) - 1);

    int best_index = start;
    double best_dist_sq = std::numeric_limits<double>::infinity();
    for (int idx = start; idx <= stop; ++idx) {
        const double dx = state.x - samples[idx].x_km;
        const double dy = state.y - samples[idx].y_km;
        const double dist_sq = dx * dx + dy * dy;
        if (dist_sq < best_dist_sq) {
            best_dist_sq = dist_sq;
            best_index = idx;
        }
    }
    return best_index;
}

RecedingLog trackReferenceSamples(MPCNumericClothoidCost& mpc,
                                  const std::vector<ReferenceSample>& samples,
                                  const ScenarioSpec& scenario,
                                  double tracking_tol_km) {
    RecedingLog log;
    if (samples.empty()) {
        return log;
    }

    const int last_index = static_cast<int>(samples.size()) - 1;
    const int goal_ahead = std::max(4, scenario.cfg.N);
    const int hit_ahead = std::max(2, scenario.cfg.N / 2);
    const int search_ahead = std::max(goal_ahead * 3, 24);

    log.start = scenario.initial_state;
    log.goal = State4{samples.back().x_km, samples.back().y_km, samples.back().heading_rad, 0.0};
    log.waypoints = buildWaypointsFromReferenceSamples(samples, tracking_tol_km);

    State4 state = scenario.initial_state;
    log.traj.push_back({state.x, state.y});
    log.psi.push_back(state.psi);
    log.K.push_back(state.K);
    log.detour_wp_x.push_back(std::numeric_limits<double>::quiet_NaN());
    log.detour_wp_y.push_back(std::numeric_limits<double>::quiet_NaN());
    log.detour_kf.push_back(std::numeric_limits<double>::quiet_NaN());
    log.detour_obs_idx.push_back(-1);
    log.wp_index.push_back(0);

    int current_index = 0;
    std::optional<double> ds_seed = std::nullopt;

    for (int iter = 0; iter < scenario.opts.max_iters; ++iter) {
        current_index = findNearestForwardSample(samples, state, current_index, search_ahead);

        const int goal_index = std::min(current_index + goal_ahead, last_index);
        const int hit_index = std::min(current_index + hit_ahead, last_index);
        const bool is_final_window = goal_index >= last_index;

        const auto& goal_sample = samples[goal_index];
        const auto& hit_sample = samples[hit_index];
        const double goal_k = scenario.opts.use_wp_kf ? goal_sample.curvature_per_km : scenario.opts.kf_fallback;
        const double hit_k = scenario.opts.use_wp_kf ? hit_sample.curvature_per_km : scenario.opts.kf_fallback;
        const double term_scale = is_final_window ? scenario.opts.term_scale_final : scenario.opts.term_scale_intermediate;
        const double w_wp = is_final_window ? scenario.opts.w_wp_final : scenario.opts.w_wp_intermediate;
        const double hit_scale = is_final_window ? 0.0 : scenario.opts.hit_scale_intermediate;

        const auto t0 = std::chrono::steady_clock::now();
        StepOutput step = mpc.mpc_step(state,
                                       State4{goal_sample.x_km, goal_sample.y_km, goal_sample.heading_rad, goal_k},
                                       term_scale,
                                       w_wp,
                                       goal_sample.x_km,
                                       goal_sample.y_km,
                                       hit_scale,
                                       hit_sample.x_km,
                                       hit_sample.y_km,
                                       hit_sample.heading_rad,
                                       hit_k,
                                       ds_seed);
        const auto t1 = std::chrono::steady_clock::now();
        log.solve_time_s.push_back(std::chrono::duration<double>(t1 - t0).count());

        state = step.state;
        ds_seed = step.ds0;

        log.traj.push_back({state.x, state.y});
        log.psi.push_back(state.psi);
        log.K.push_back(state.K);
        log.Kcmd.push_back(step.Kcmd0);
        log.ds.push_back(step.ds0);
        log.detour_wp_x.push_back(std::numeric_limits<double>::quiet_NaN());
        log.detour_wp_y.push_back(std::numeric_limits<double>::quiet_NaN());
        log.detour_kf.push_back(std::numeric_limits<double>::quiet_NaN());
        log.detour_obs_idx.push_back(-1);
        log.wp_index.push_back(current_index);

        const double dx_goal = state.x - samples.back().x_km;
        const double dy_goal = state.y - samples.back().y_km;
        const double goal_dist = std::hypot(dx_goal, dy_goal);
        const double heading_err = wrapAngle(state.psi - samples.back().heading_rad);
        if (goal_dist <= tracking_tol_km &&
            (!scenario.opts.use_heading_gate || std::abs(heading_err) <= scenario.opts.tol_psi)) {
            current_index = last_index;
            break;
        }
    }

    log.active_wp = current_index;
    if (!log.solve_time_s.empty()) {
        double total = 0.0;
        for (double value : log.solve_time_s) {
            total += value;
        }
        log.mean_solve_time_s = total / static_cast<double>(log.solve_time_s.size());
    }
    return log;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        Options options;
        if (!parseArgs(argc, argv, options)) {
            return 0;
        }

        const auto route_cfg = loadRouteConfig(options.route_config);
        planner::RoutePlan plan;
        std::string error;
        if (!planner::computeRoutePlan(route_cfg, plan, &error)) {
            throw std::runtime_error(error);
        }
        if (!plan.route.found || plan.route.polyline.size() < 2) {
            throw std::runtime_error("Boğaz hattı için geçerli bir referans rota üretilemedi.");
        }

        planner::printRoute(plan.route);
        if (!planner::writeRouteCsv(plan, options.route_csv, &error)) {
            throw std::runtime_error(error);
        }
        planner::writeRouteSvg(plan, options.route_svg);

        ScenarioSpec scenario = load_scenario_ini(options.mpc_scenario);

        LocalReferenceFrame frame;
        const auto samples =
            buildUniformReferenceSamples(plan.route.polyline, plan.projection, options.reference_step_km, &frame);
        if (samples.size() < 2) {
            throw std::runtime_error("Referans rota yeterli sayida uniform ornek uretmedi.");
        }

        scenario.initial_state = buildInitialStateFromReference(samples);
        scenario.waypoints = buildWaypointsFromReferenceSamples(samples, options.waypoint_tol_km);

        if (!writeReferenceSamplesCsv(samples, options.reference_csv, &error)) {
            throw std::runtime_error(error);
        }

        MPCNumericClothoidCost mpc(scenario.cfg);
        RecedingLog log = trackReferenceSamples(mpc, samples, scenario, options.waypoint_tol_km);
        mpc.write_log_csv(log, options.mpc_log_csv);

        if (!writeMpcGeoCsv(log, frame, options.mpc_geo_csv, &error)) {
            throw std::runtime_error(error);
        }

        auto mpc_geo_polyline = recedingLogToGeoPolyline(log, frame);
        planner::RoutePlan mpc_plan = plan;
        mpc_plan.route.found = !mpc_geo_polyline.empty();
        mpc_plan.route.polyline = std::move(mpc_geo_polyline);
        mpc_plan.route.total_km = planner::polylineLengthKm(mpc_plan.route.polyline);
        mpc_plan.route.triangle_sequence.clear();
        planner::writeRouteSvg(mpc_plan, options.mpc_svg);

        std::cout << "Reference route CSV: " << options.route_csv << "\n";
        std::cout << "Uniform reference CSV: " << options.reference_csv << "\n";
        std::cout << "MPC log CSV: " << options.mpc_log_csv << "\n";
        std::cout << "MPC geo CSV: " << options.mpc_geo_csv << "\n";
        std::cout << "Reference SVG: " << options.route_svg << "\n";
        std::cout << "MPC SVG: " << options.mpc_svg << "\n";
        std::cout << "MPC average solve time [s]: " << log.mean_solve_time_s << "\n";
        std::cout << "MPC active waypoint index: " << log.active_wp << " / " << scenario.waypoints.size() << "\n";
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }

    return 0;
}
