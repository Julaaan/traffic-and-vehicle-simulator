#include "vsim/Simulation.hpp"
#include <algorithm>
#include <bit>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <utility>

namespace vsim {
std::string hexString(std::uint64_t value) {
    char buffer[17];
    std::snprintf(buffer, sizeof(buffer), "%016llx", static_cast<unsigned long long>(value));
    return buffer;
}
SimulationClock::SimulationClock(double dt) : dt_(dt) {
    if (!std::isfinite(dt) || dt < 0.0001 || dt > 0.1)
        throw std::invalid_argument("Time step must be in [0.0001, 0.1] seconds");
}
void SimulationClock::advance() {
    if (tick_ == std::numeric_limits<std::uint64_t>::max()) throw std::overflow_error("Simulation clock overflow");
    ++tick_;
}
Simulation::Simulation(Scenario scenario)
    : initial_(std::move(scenario)), world_(initial_), clock_(initial_.time_step),
      collision_events_(world_.collisions().size()), peak_collision_pairs_(world_.collisions().size()),
      previous_collisions_(world_.collisions()) {}

void Simulation::step(std::span<const RecordedControl> recorded, StepProfile* profile) {
    world_.step(clock_.timeStep(), clock_.time(), recorded, profile);
    clock_.advance();
    const auto& current = world_.collisions();
    for (const auto& collision : current)
        if (std::find(previous_collisions_.begin(), previous_collisions_.end(), collision) == previous_collisions_.end())
            ++collision_events_;
    peak_collision_pairs_ = std::max(peak_collision_pairs_, current.size());
    previous_collisions_ = current;
}
SimulationSnapshot Simulation::snapshot() const {
    return {clock_.tick(),collision_events_,peak_collision_pairs_,world_.vehicles(),world_.collisions()};
}
void Simulation::restore(const SimulationSnapshot& s) {
    if (s.tick>1000000000ULL || s.collision_events<s.collisions.size() || s.peak_collision_pairs<s.collisions.size())
        throw std::invalid_argument("Invalid simulation snapshot counters");
    world_.restore(s.vehicles,s.collisions); clock_.restore(s.tick);
    collision_events_=s.collision_events; peak_collision_pairs_=s.peak_collision_pairs; previous_collisions_=s.collisions;
}
void Simulation::reset() {
    world_ = World(initial_);
    clock_ = SimulationClock(initial_.time_step);
    collision_events_ = world_.collisions().size();
    peak_collision_pairs_ = world_.collisions().size();
    previous_collisions_ = world_.collisions();
}
std::uint64_t Simulation::stateHash() const {
    std::uint64_t hash = 14695981039346656037ULL;
    const auto append = [&hash](std::uint64_t value) {
        for (unsigned byte = 0; byte < 8; ++byte) {
            hash ^= (value >> (byte * 8U)) & 0xffU;
            hash *= 1099511628211ULL;
        }
    };
    append(clock_.tick());
    append(std::bit_cast<std::uint64_t>(clock_.timeStep()));
    append(world_.vehicles().size());
    append(collision_events_); append(peak_collision_pairs_);
    for (const auto& p : world_.collisions()) { append(p.first); append(p.second); }
    for (const auto& v : world_.vehicles()) {
        append(v.id); append(v.lane); append(v.active ? 1U : 0U);
        for (double value : {v.state.x, v.state.y, v.state.yaw, v.state.velocity,
                            v.state.acceleration, v.state.steering_angle})
            append(std::bit_cast<std::uint64_t>(value));
        append(v.route_index); append(static_cast<std::uint64_t>(v.layer)); append(v.junction_admitted);
        append(v.automatic); append(v.route.size()); for (auto lane : v.route) append(lane);
        for (double value : {v.desired_speed,v.stopped_time,v.control.acceleration,v.control.steering_angle,
            v.state.lateral_velocity,v.state.yaw_rate,v.state.lateral_acceleration,v.state.tire_utilization})
            append(std::bit_cast<std::uint64_t>(value));
        append(v.state.grip_limited);
        const auto& c=v.config; append(static_cast<std::uint64_t>(c.dynamics_model)); append(c.tire_grip);
        for (double value : {c.length,c.width,c.wheelbase,c.max_speed,c.max_acceleration,c.max_deceleration,c.max_steering,
            c.mass,c.yaw_inertia,c.front_axle_distance,c.front_cornering_stiffness,c.rear_cornering_stiffness,c.friction_coefficient,
            c.acceleration_time_constant,c.steering_time_constant,c.acceleration_delay,c.steering_delay,c.max_steering_rate})
            append(std::bit_cast<std::uint64_t>(value));
        const auto& m=v.state.actuators; append(m.initialized);
        for (double value : {m.time,m.acceleration,m.steering_angle}) append(std::bit_cast<std::uint64_t>(value));
        append(m.history.size());
        for (const auto& h : m.history) for (double value : {h.time,h.control.acceleration,h.control.steering_angle})
            append(std::bit_cast<std::uint64_t>(value));
    }
    return hash;
}
}
