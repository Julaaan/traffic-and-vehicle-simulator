#pragma once

#include <cmath>
#include <cstdint>
#include <deque>
#include <numbers>
#include <vector>

namespace vsim {
using EntityId = std::uint32_t;
using LaneId = std::uint32_t;
using OsmId = std::int64_t;
inline constexpr double pi = std::numbers::pi_v<double>;

struct Vec2 {
    double x{};
    double y{};
    Vec2 operator+(Vec2 rhs) const { return {x + rhs.x, y + rhs.y}; }
    Vec2 operator-(Vec2 rhs) const { return {x - rhs.x, y - rhs.y}; }
    Vec2 operator*(double factor) const { return {x * factor, y * factor}; }
};
inline double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
inline double cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }
inline double norm(Vec2 a) { return std::hypot(a.x, a.y); }
inline double wrapAngle(double angle) { return std::remainder(angle, 2.0 * pi); }
inline bool finite(Vec2 p) { return std::isfinite(p.x) && std::isfinite(p.y); }

struct VehicleControl {
    double acceleration{};
    double steering_angle{};
};
struct RecordedControl { EntityId id{}; bool automatic{}; VehicleControl control; };
struct TimedControl { double time{}; VehicleControl control; };
struct ActuatorState {
    bool initialized{};
    double time{}, acceleration{}, steering_angle{};
    std::deque<TimedControl> history;
};
struct VehicleState {
    double x{};
    double y{};
    double yaw{};
    double velocity{};
    double acceleration{};
    double steering_angle{};
    double lateral_velocity{};
    double yaw_rate{};
    double lateral_acceleration{};
    double tire_utilization{};
    bool grip_limited{};
    ActuatorState actuators;
    Vec2 position() const { return {x, y}; }
};

enum class DynamicsModel { Kinematic, DynamicBicycle };
struct VehicleConfig {
    double length{4.5};
    double width{1.8};
    double wheelbase{2.7};
    double max_speed{25.0};
    double max_acceleration{2.5};
    double max_deceleration{6.0};
    double max_steering{0.6};
    DynamicsModel dynamics_model{DynamicsModel::Kinematic};
    double mass{1500.0}, yaw_inertia{2500.0}, front_axle_distance{1.2};
    double front_cornering_stiffness{70000.0}, rear_cornering_stiffness{80000.0};
    double friction_coefficient{1.0};
    bool tire_grip{};
    double acceleration_time_constant{}, steering_time_constant{};
    double acceleration_delay{}, steering_delay{}, max_steering_rate{};
};

struct Vehicle {
    EntityId id{};
    LaneId lane{};
    VehicleState state;
    VehicleConfig config;
    VehicleControl control;
    double desired_speed{12.0};
    bool automatic{true};
    bool active{true};
    std::vector<LaneId> route;
    std::size_t route_index{};
    int layer{};
    double stopped_time{};
    bool junction_admitted{};
};
}
