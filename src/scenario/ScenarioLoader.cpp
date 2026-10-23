#include "vsim/Scenario.hpp"
#include "vsim/Dynamics.hpp"
#include "vsim/World.hpp"
#include "vsim/Simulation.hpp"
#include "vsim/Serialization.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <random>
#include <stdexcept>

namespace vsim {
namespace {
using Json = nlohmann::json;
double number(const Json& j, const char* key, double fallback) {
    const double value = j.value(key, fallback);
    if (!std::isfinite(value)) throw std::invalid_argument(std::string(key) + " must be finite");
    return value;
}
std::uint32_t unsignedNumber(const Json& j, const char* key, std::uint32_t fallback) {
    if (!j.contains(key)) return fallback;
    const auto& value = j.at(key);
    if (!value.is_number_integer() || value.get<double>() < 0.0
        || value.get<double>() > static_cast<double>(UINT32_MAX))
        throw std::invalid_argument(std::string(key) + " must be an unsigned 32-bit integer");
    return value.get<std::uint32_t>();
}
VehicleConfig configFrom(const Json& j) {
    auto c=j.get<VehicleConfig>();
    validate(c);
    return c;
}
Vehicle spawn(const RoadNetwork& road, const VehicleConfig& config, EntityId id,
              LaneId lane_id, double progress, double speed, double desired_speed) {
    const auto& lane = road.lane(lane_id);
    if (!std::isfinite(progress) || progress < 0.0 || progress >= lane.length())
        throw std::invalid_argument("Spawn progress must be within the lane");
    const auto sample = lane.sample(progress);
    Vehicle v;
    v.id = id; v.lane = lane_id; v.config = config; v.desired_speed = desired_speed;
    v.state.x = sample.position.x; v.state.y = sample.position.y; v.state.yaw = sample.yaw; v.state.velocity = speed;
    return v;
}
}

Scenario loadScenario(const std::filesystem::path& path, const ScenarioOverrides& overrides) {
    try {
        std::ifstream stream(path);
        if (!stream) throw std::runtime_error("Cannot open scenario");
        const Json j = Json::parse(stream);
        if (!j.is_object()) throw std::invalid_argument("Scenario root must be an object");
        Scenario s;
        s.name = j.value("name", s.name);
        s.seed = overrides.seed.value_or(unsignedNumber(j, "seed", s.seed));
        s.time_step = number(j, "time_step", s.time_step);
        s.duration = number(j, "duration", s.duration);
        SimulationClock check_clock(s.time_step);
        if (s.duration <= 0.0) throw std::invalid_argument("Duration must be positive");
        const auto map = j.value("map", Json::object());
        if (map.value("type", std::string("four_way_intersection")) != "four_way_intersection")
            throw std::invalid_argument("v0.1 supports map type four_way_intersection");
        s.road = RoadNetwork::intersection(number(map, "arm_length", 240.0), number(map, "lane_width", 3.5));
        const auto traffic = j.value("traffic", Json::object());
        s.traffic.enabled = traffic.value("enabled", true);
        s.traffic.green_seconds = number(traffic, "green_seconds", 12.0);
        s.traffic.clearance_seconds = number(traffic, "clearance_seconds", 2.0);
        s.traffic.minimum_gap = number(traffic, "minimum_gap", 2.0);
        s.traffic.time_headway = number(traffic, "time_headway", 1.3);
        validate(s.traffic);
        const auto config = configFrom(j.value("vehicle_config", Json::object()));
        if (j.contains("fleet") && j.contains("vehicles"))
            throw std::invalid_argument("Use either fleet or explicit vehicles");
        if (j.contains("fleet")) {
            const auto& fleet = j.at("fleet");
            const std::size_t count = overrides.vehicle_count.value_or(unsignedNumber(fleet, "count", 100));
            if (count == 0 || count > 1000) throw std::invalid_argument("Fleet count must be in [1,1000]");
            const double spacing = number(fleet, "spacing", 9.0);
            const double desired = number(fleet, "desired_speed", 12.0);
            const double speed = number(fleet, "initial_speed", 0.0);
            if (spacing < config.length + s.traffic.minimum_gap)
                throw std::invalid_argument("Fleet spacing is smaller than vehicle length plus minimum gap");
            std::mt19937 random(s.seed);
            for (std::size_t i = 0; i < count; ++i) {
                const double progress = s.road.stopLine() - config.length / 2.0 - 8.0
                                      - static_cast<double>(i / 4) * spacing;
                if (progress < config.length / 2.0)
                    throw std::invalid_argument("Fleet does not fit on this map; increase arm_length or reduce count/spacing");
                const double factor = 0.90 + 0.10 * static_cast<double>(random()) / 4294967295.0;
                s.vehicles.push_back(spawn(s.road, config, static_cast<EntityId>(i + 1),
                    static_cast<LaneId>(i % 4), progress, speed, desired * factor));
            }
        } else {
            if (overrides.vehicle_count) throw std::invalid_argument("--vehicles requires a fleet scenario");
            if (!j.contains("vehicles") || !j.at("vehicles").is_array() || j.at("vehicles").empty()
                || j.at("vehicles").size() > 1000)
                throw std::invalid_argument("Scenario requires 1..1000 vehicles or a fleet");
            for (const auto& item : j.at("vehicles")) {
                const auto& lane = s.road.lane(item.at("lane").get<std::string>());
                auto v = spawn(s.road, config, unsignedNumber(item, "id", 0), lane.id,
                    number(item, "progress", 0.0), number(item, "speed", 0.0), number(item, "desired_speed", 12.0));
                v.automatic = item.value("automatic", true);
                v.state.lateral_velocity=number(item,"lateral_velocity",0);
                v.state.yaw_rate=number(item,"yaw_rate",0);
                v.state.steering_angle=number(item,"steering_angle",0);
                if (item.contains("position")) {
                    const auto& position = item.at("position");
                    v.state.x = position.at("x").get<double>();
                    v.state.y = position.at("y").get<double>();
                    v.state.yaw = number(position, "yaw", v.state.yaw);
                }
                const auto control = item.value("control", Json::object());
                v.control = {number(control, "acceleration", 0.0), number(control, "steering_angle", 0.0)};
                s.vehicles.push_back(v);
            }
        }
        World check_world(s);
        return s;
    } catch (const std::exception& error) {
        throw std::runtime_error("Scenario '" + path.string() + "': " + error.what());
    }
}
}
