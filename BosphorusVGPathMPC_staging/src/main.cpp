#include "route_planner_core.hpp"

#include <iostream>

int main(int argc, char** argv) {
    using namespace planner;

    CliConfig cfg;
    bool should_run = true;
    if (!parseCli(argc, argv, cfg, should_run)) {
        return 1;
    }
    if (!should_run) {
        return 0;
    }

    RoutePlan plan;
    std::string error;
    if (!computeRoutePlan(cfg, plan, &error)) {
        std::cerr << error << "\n";
        return 1;
    }

    printRoute(plan.route);
    writeRouteSvg(plan, cfg.svg_file);
    std::cout << "Start (sea): (" << plan.start_point.geo.lat_deg << ", " << plan.start_point.geo.lon_deg << ")\n";
    std::cout << "Goal (sea): (" << plan.goal_point.geo.lat_deg << ", " << plan.goal_point.geo.lon_deg << ")\n";
    std::cout << "Plot dosyasi: " << cfg.svg_file << "\n";
    return plan.route.found ? 0 : 1;
}
