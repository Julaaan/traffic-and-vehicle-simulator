#pragma once
#include "vsim/Simulation.hpp"
#include <nlohmann/json.hpp>

namespace vsim {
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Vec2, x, y)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(VehicleControl, acceleration, steering_angle)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(TimedControl, time, control)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ActuatorState, initialized, time, acceleration, steering_angle, history)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(VehicleState, x, y, yaw, velocity, acceleration, steering_angle,
    lateral_velocity, yaw_rate, lateral_acceleration, tire_utilization, grip_limited, actuators)
void to_json(nlohmann::json& j, const DynamicsModel& model);
void from_json(const nlohmann::json& j, DynamicsModel& model);
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(VehicleConfig, length, width, wheelbase, max_speed, max_acceleration,
    max_deceleration, max_steering, dynamics_model, mass, yaw_inertia, front_axle_distance, front_cornering_stiffness,
    rear_cornering_stiffness, friction_coefficient, tire_grip, acceleration_time_constant, steering_time_constant,
    acceleration_delay, steering_delay, max_steering_rate)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Vehicle, id, lane, state, config, control, desired_speed, automatic, active,
    route, route_index, layer, stopped_time, junction_admitted)
void to_json(nlohmann::json& j, const Lane& lane);
void from_json(const nlohmann::json& j, Lane& lane);
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Junction, id, position, radius, traffic_lights, incoming, outgoing, connectors)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ImportInfo, nodes, ways, road_segments, road_lanes, connectors, skipped_ways,
    origin_latitude, origin_longitude, warnings)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(RoadNetwork, arm_length, lane_width, lanes, imported, junctions, import_info)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(TrafficConfig, enabled, green_seconds, clearance_seconds, minimum_gap, time_headway)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Scenario, name, seed, time_step, duration, road, traffic, vehicles)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(CollisionPair, first, second)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(SimulationSnapshot, tick, collision_events, peak_collision_pairs, vehicles, collisions)
VehicleConfig loadVehicleProfile(const std::filesystem::path& path);
void validateOutputPaths(const std::vector<std::filesystem::path>& outputs, const std::vector<std::filesystem::path>& inputs = {});
}
