#include "Output.hpp"
#include "vsim/Serialization.hpp"
#include <nlohmann/json.hpp>
#include <iomanip>
#include <iostream>
#include <locale>
#include <stdexcept>

namespace vsim::app {
CsvOutput::CsvOutput(const std::filesystem::path& path) {
    if (path.empty()) return;
    createParentDirectories(path);
    stream_.open(path);
    if (!stream_) throw std::runtime_error("Cannot write CSV: " + path.string());
    stream_.imbue(std::locale::classic());
    stream_ << "tick,time,id,lane,active,x,y,yaw,velocity,acceleration,steering_angle,lateral_velocity,yaw_rate,lateral_acceleration,tire_utilization,grip_limited,acceleration_command,steering_command\n" << std::setprecision(17);
}
void CsvOutput::sample(const Simulation& sim) {
    if (!stream_.is_open()) return;
    for (const auto& v : sim.world().vehicles())
        stream_ << sim.clock().tick() << ',' << sim.clock().time() << ',' << v.id << ',' << v.lane
            << ',' << v.active << ',' << v.state.x << ',' << v.state.y << ',' << v.state.yaw
            << ',' << v.state.velocity << ',' << v.state.acceleration << ',' << v.state.steering_angle
            << ',' << v.state.lateral_velocity << ',' << v.state.yaw_rate << ',' << v.state.lateral_acceleration
            << ',' << v.state.tire_utilization << ',' << v.state.grip_limited << ',' << v.control.acceleration << ',' << v.control.steering_angle << '\n';
    if (!stream_) throw std::runtime_error("Failed writing CSV");
}
void CsvOutput::flush() {
    if (!stream_.is_open()) return;
    stream_.flush();
    if (!stream_) throw std::runtime_error("Failed flushing CSV");
}
void writeReport(const std::filesystem::path& path, const Simulation& sim, double wall_seconds) {
    if (path.empty()) return;
    nlohmann::json states = nlohmann::json::array();
    for (const auto& v : sim.world().vehicles())
        states.push_back({{"id", v.id}, {"lane", v.lane}, {"active", v.active},
            {"x", v.state.x}, {"y", v.state.y}, {"yaw", v.state.yaw}, {"velocity", v.state.velocity},
            {"acceleration", v.state.acceleration}, {"steering_angle", v.state.steering_angle},
            {"route", v.route}, {"route_index", v.route_index}, {"layer", v.layer}, {"junction_admitted", v.junction_admitted},
            {"lateral_velocity",v.state.lateral_velocity},{"yaw_rate",v.state.yaw_rate},{"lateral_acceleration",v.state.lateral_acceleration},
            {"tire_utilization",v.state.tire_utilization},{"grip_limited",v.state.grip_limited},
            {"acceleration_command",v.control.acceleration},{"steering_command",v.control.steering_angle},
            {"vehicle_config",v.config}});
    nlohmann::json report{
        {"version", VSIM_VERSION}, {"scenario", sim.scenario().name}, {"seed", sim.scenario().seed},
        {"ticks", sim.clock().tick()}, {"time_step", sim.clock().timeStep()}, {"simulated_seconds", sim.clock().time()},
        {"wall_seconds", wall_seconds}, {"real_time_factor", wall_seconds > 0.0 ? sim.clock().time() / wall_seconds : 0.0},
        {"vehicles", sim.world().vehicles().size()}, {"active", sim.world().activeCount()},
        {"completed", sim.world().vehicles().size() - sim.world().activeCount()},
        {"collision_events", sim.collisionEvents()}, {"peak_collision_pairs", sim.peakCollisionPairs()},
        {"state_hash", hexString(sim.stateHash())}, {"final_states", states}
    };
    if (sim.world().road().imported) {
        const auto& road = sim.world().road(); const auto& info = road.import_info;
        report["map"] = {{"format", "OSM XML"}, {"source", "OpenStreetMap contributors"},
            {"attribution_url", "https://www.openstreetmap.org/copyright"},
            {"projection", "WGS84 ECEF to local east/north tangent plane"},
            {"origin_latitude", info.origin_latitude}, {"origin_longitude", info.origin_longitude},
            {"nodes", info.nodes}, {"ways", info.ways}, {"road_segments", info.road_segments},
            {"road_lanes", info.road_lanes}, {"connectors", info.connectors},
            {"junctions", road.junctions.size()}, {"skipped_ways", info.skipped_ways}, {"warnings", info.warnings}};
    }
    createParentDirectories(path);
    std::ofstream stream(path);
    if (!stream) throw std::runtime_error("Cannot write report: " + path.string());
    stream << report.dump(2) << '\n';
    stream.flush();
    if (!stream) throw std::runtime_error("Failed writing report");
}
void printSummary(const Simulation& sim, double wall_seconds) {
    std::cout << "Vehicle Simulator v" VSIM_VERSION " | " << sim.scenario().name << '\n'
        << "Seed: " << sim.scenario().seed << " | Steps: " << sim.clock().tick()
        << " | dt: " << sim.clock().timeStep() << " s\n"
        << "Vehicles: " << sim.world().vehicles().size() << " | Active: " << sim.world().activeCount()
        << " | Completed: " << sim.world().vehicles().size() - sim.world().activeCount() << '\n'
        << "Collision events: " << sim.collisionEvents() << " | Peak pairs: " << sim.peakCollisionPairs() << '\n'
        << std::fixed << std::setprecision(3) << "Simulated: " << sim.clock().time()
        << " s | Wall: " << wall_seconds << " s | Real-time factor: "
        << (wall_seconds > 0.0 ? sim.clock().time() / wall_seconds : 0.0) << "x\n"
        << "State hash: " << hexString(sim.stateHash()) << '\n';
    if (sim.world().vehicles().size() == 1) {
        const auto& v = sim.world().vehicles().front();
        std::cout << "Final state: x=" << v.state.x << " m, y=" << v.state.y
                  << " m, yaw=" << v.state.yaw << " rad, v=" << v.state.velocity << " m/s\n";
    }
}
}
