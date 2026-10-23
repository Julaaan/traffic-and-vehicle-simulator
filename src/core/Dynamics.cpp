#include "vsim/Dynamics.hpp"
#include <algorithm>
#include <array>
#include <stdexcept>

namespace vsim {
namespace {
constexpr double gravity = 9.80665;
void constantKinematics(VehicleState& s, const VehicleConfig& c, double a, double steering, double dt) {
    const double initial_v = s.velocity;
    const double final_v = std::clamp(initial_v + a * dt, 0.0, c.max_speed);
    double moving_time = dt;
    if (a > 0.0 && initial_v + a * dt > c.max_speed) moving_time = (c.max_speed - initial_v) / a;
    else if (a < 0.0 && initial_v + a * dt < 0.0) moving_time = -initial_v / a;
    const double distance = initial_v * moving_time + 0.5 * a * moving_time * moving_time + final_v * (dt - moving_time);
    double curvature = std::tan(steering) / c.wheelbase;
    s.grip_limited = false;
    if (c.tire_grip) {
        const double budget = std::sqrt(std::max(0.0, std::pow(c.friction_coefficient * gravity, 2) - a*a));
        const double speed = std::max(initial_v, final_v);
        const double limit = budget / std::max(0.01, speed * speed);
        s.grip_limited = std::abs(curvature) > limit;
        curvature = std::clamp(curvature, -limit, limit);
    }
    const double yaw_delta = distance * curvature, half_turn = yaw_delta / 2.0;
    const double sinc = std::abs(half_turn) < 1e-8 ? 1.0 - half_turn*half_turn / 6.0 : std::sin(half_turn) / half_turn;
    s.x += distance * sinc * std::cos(s.yaw + half_turn);
    s.y += distance * sinc * std::sin(s.yaw + half_turn);
    s.yaw = wrapAngle(s.yaw + yaw_delta); s.velocity = final_v;
    s.acceleration = (final_v - initial_v) / dt; s.steering_angle = steering;
    s.yaw_rate = final_v * curvature;
    s.lateral_velocity = (c.wheelbase - c.front_axle_distance) * s.yaw_rate;
    s.lateral_acceleration = final_v * s.yaw_rate;
    s.tire_utilization = std::hypot(s.acceleration, s.lateral_acceleration) / (c.friction_coefficient * gravity);
}
double response(double current, double target, double tau, double rate, double dt) {
    double next = tau == 0.0 ? target : current + (target - current) * -std::expm1(-dt / tau);
    if (rate > 0.0) next = std::clamp(next, current - rate * dt, current + rate * dt);
    return next;
}
VehicleControl delayed(const ActuatorState& memory, const VehicleConfig& c, double time) {
    VehicleControl result;
    for (const auto& entry : memory.history) {
        if (entry.time <= time - c.acceleration_delay + 1e-12) result.acceleration = entry.control.acceleration;
        if (entry.time <= time - c.steering_delay + 1e-12) result.steering_angle = entry.control.steering_angle;
    }
    return result;
}
struct Forces { double front{}, rear{}, utilization{}; bool limited{}; };
Forces forces(double v, double lateral, double r, double steering, double a, const VehicleConfig& c) {
    const double lf = c.front_axle_distance, lr = c.wheelbase - lf;
    const double capacity = c.friction_coefficient * c.mass * gravity;
    const double lateral_budget = std::sqrt(std::max(0.0, capacity*capacity - std::pow(c.mass * a, 2)));
    const double front_limit = lateral_budget * lr / c.wheelbase, rear_limit = lateral_budget * lf / c.wheelbase;
    const double front = c.front_cornering_stiffness * (steering - (lateral + lf * r) / std::max(2.0, v));
    const double rear = c.rear_cornering_stiffness * (-(lateral - lr * r) / std::max(2.0, v));
    Forces f{std::clamp(front, -front_limit, front_limit), std::clamp(rear, -rear_limit, rear_limit),
        0.0, std::abs(front) > front_limit + 1e-9 || std::abs(rear) > rear_limit + 1e-9};
    const double front_load = capacity * lr / c.wheelbase, rear_load = capacity * lf / c.wheelbase;
    f.utilization = std::max(std::hypot(c.mass*a*lr/c.wheelbase, f.front) / front_load,
        std::hypot(c.mass*a*lf/c.wheelbase, f.rear) / rear_load);
    return f;
}
void dynamicMotion(VehicleState& s, const VehicleConfig& c, double a, double steering, double h) {
    const double old_v = s.velocity, lr = c.wheelbase - c.front_axle_distance;
    const double final_v = std::clamp(old_v + a*h, 0.0, c.max_speed);
    if (std::max(old_v, final_v) <= 2.0) {
        auto low = c; low.tire_grip = true;
        constantKinematics(s, low, a, steering, h); return;
    }
    using Full = std::array<double, 5>;
    const auto derivative = [&](const Full& q, double t) {
        const double v = std::clamp(old_v + a*t, 0.0, c.max_speed);
        const auto f = forces(v, q[3], q[4], steering, a, c);
        const double blend = std::clamp((v - 2.0) / 1.0, 0.0, 1.0);
        const double kin_r = v * std::tan(steering) / c.wheelbase, kin_vy = lr * kin_r;
        const double dvy = (f.front * std::cos(steering) + f.rear) / c.mass - v*q[4];
        const double dr = (c.front_axle_distance*f.front*std::cos(steering) - lr*f.rear) / c.yaw_inertia;
        const double rear_vy = q[3] - lr*q[4];
        return Full{v*std::cos(q[2]) - rear_vy*std::sin(q[2]), v*std::sin(q[2]) + rear_vy*std::cos(q[2]), q[4],
            blend*dvy + (1.0-blend)*(kin_vy-q[3])*10.0, blend*dr + (1.0-blend)*(kin_r-q[4])*10.0};
    };
    const auto shifted = [](Full q, const Full& k, double step) { for (std::size_t i=0; i<q.size(); ++i) q[i] += k[i]*step; return q; };
    Full q{s.x,s.y,s.yaw,s.lateral_velocity,s.yaw_rate};
    const auto k1=derivative(q,0), k2=derivative(shifted(q,k1,h/2),h/2);
    const auto k3=derivative(shifted(q,k2,h/2),h/2), k4=derivative(shifted(q,k3,h),h);
    for (std::size_t i=0; i<q.size(); ++i) q[i] += h*(k1[i]+2*k2[i]+2*k3[i]+k4[i])/6;
    s.x=q[0]; s.y=q[1]; s.yaw=wrapAngle(q[2]); s.lateral_velocity=q[3]; s.yaw_rate=q[4]; s.velocity=final_v;
    s.acceleration=(final_v-old_v)/h; s.steering_angle=steering;
    const auto f=forces(final_v,s.lateral_velocity,s.yaw_rate,steering,a,c);
    s.lateral_acceleration=(f.front*std::cos(steering)+f.rear)/c.mass;
    s.tire_utilization=f.utilization; s.grip_limited=f.limited;
}
}
const char* dynamicsModelName(DynamicsModel model) {
    switch (model) { case DynamicsModel::Kinematic: return "kinematic"; case DynamicsModel::DynamicBicycle: return "dynamic_bicycle"; }
    throw std::invalid_argument("Unknown dynamics model");
}
void validate(const VehicleConfig& c) {
    for (double value : {c.length,c.width,c.wheelbase,c.max_speed,c.max_acceleration,c.max_deceleration,c.max_steering,
        c.mass,c.yaw_inertia,c.front_axle_distance,c.front_cornering_stiffness,c.rear_cornering_stiffness,c.friction_coefficient})
        if (!std::isfinite(value) || value <= 0.0) throw std::invalid_argument("Vehicle dimensions and limits must be finite and positive");
    if (c.max_steering >= pi/2 || c.wheelbase > c.length || c.front_axle_distance >= c.wheelbase
        || c.friction_coefficient > 3 || c.mass < 100 || c.yaw_inertia < 100
        || c.front_cornering_stiffness > 1e6 || c.rear_cornering_stiffness > 1e6)
        throw std::invalid_argument("Invalid vehicle geometry or dynamics parameters");
    for (double value : {c.acceleration_time_constant,c.steering_time_constant,c.acceleration_delay,c.steering_delay,c.max_steering_rate})
        if (!std::isfinite(value) || value < 0 || value > 10) throw std::invalid_argument("Actuator parameters must be in [0,10]");
    dynamicsModelName(c.dynamics_model);
}
void validate(const VehicleState& s, const VehicleConfig& c) {
    if (!finite(s.position()) || !std::isfinite(s.yaw) || !std::isfinite(s.velocity)
        || !std::isfinite(s.acceleration) || !std::isfinite(s.steering_angle) || !std::isfinite(s.lateral_velocity)
        || !std::isfinite(s.yaw_rate) || !std::isfinite(s.lateral_acceleration) || !std::isfinite(s.tire_utilization)
        || s.velocity < 0 || s.velocity > c.max_speed || s.tire_utilization < 0)
        throw std::invalid_argument("Invalid vehicle state or velocity outside limits");
    const auto& m=s.actuators;
    if (!std::isfinite(m.time) || m.time < 0 || !std::isfinite(m.acceleration) || !std::isfinite(m.steering_angle) || m.history.size()>100002)
        throw std::invalid_argument("Invalid actuator state");
    double previous=-1;
    for (const auto& entry : m.history) {
        if (!std::isfinite(entry.time) || entry.time < 0 || entry.time < previous || entry.time > m.time + 1e-9)
            throw std::invalid_argument("Invalid actuator command history");
        validate(entry.control); previous=entry.time;
    }
}
void validate(const VehicleControl& control) {
    if (!std::isfinite(control.acceleration) || !std::isfinite(control.steering_angle)) throw std::invalid_argument("Vehicle control must be finite");
}
void integrate(VehicleState& s, const VehicleConfig& c, VehicleControl input, double dt) {
    validate(c); validate(s,c); validate(input);
    if (!std::isfinite(dt) || dt <= 0 || dt > 10) throw std::invalid_argument("Time step must be in (0,10]");
    input.acceleration=std::clamp(input.acceleration,-c.max_deceleration,c.max_acceleration);
    input.steering_angle=std::clamp(input.steering_angle,-c.max_steering,c.max_steering);
    auto& m=s.actuators;
    if (!m.initialized) { m.initialized=true; m.acceleration=s.acceleration; m.steering_angle=s.steering_angle; }
    const bool instant=c.acceleration_time_constant==0 && c.steering_time_constant==0
        && c.acceleration_delay==0 && c.steering_delay==0 && c.max_steering_rate==0;
    if (instant && c.dynamics_model==DynamicsModel::Kinematic && !c.tire_grip) {
        constantKinematics(s,c,input.acceleration,input.steering_angle,dt);
        m.acceleration=input.acceleration; m.steering_angle=input.steering_angle; m.time+=dt; return;
    }
    m.history.push_back({m.time,input});
    const double start=m.time, initial_v=s.velocity;
    double elapsed=0;
    const double lr=c.wheelbase-c.front_axle_distance;
    const double stable_step=std::min({0.005, 0.2*c.mass/(c.front_cornering_stiffness+c.rear_cornering_stiffness),
        0.2*c.yaw_inertia/(c.front_cornering_stiffness*c.front_axle_distance*c.front_axle_distance+c.rear_cornering_stiffness*lr*lr)});
    bool limited=false;
    while (elapsed < dt-1e-12) {
        double h=std::min(stable_step,dt-elapsed);
        for (const auto& entry : m.history) for (double delay : {c.acceleration_delay,c.steering_delay}) {
            const double arrival=entry.time+delay-(start+elapsed);
            if (arrival > 1e-12 && arrival < h) h=arrival;
        }
        const auto target=delayed(m,c,start+elapsed);
        double a=response(m.acceleration,target.acceleration,c.acceleration_time_constant,0,h/2);
        const double steering=response(m.steering_angle,target.steering_angle,c.steering_time_constant,c.max_steering_rate,h/2);
        if (c.tire_grip || c.dynamics_model==DynamicsModel::DynamicBicycle) {
            const double bound=c.friction_coefficient*gravity;
            limited=limited || std::abs(a)>bound; a=std::clamp(a,-bound,bound);
        }
        if (c.dynamics_model==DynamicsModel::DynamicBicycle) dynamicMotion(s,c,a,steering,h);
        else constantKinematics(s,c,a,steering,h);
        limited=limited || s.grip_limited;
        m.acceleration=response(m.acceleration,target.acceleration,c.acceleration_time_constant,0,h);
        m.steering_angle=response(m.steering_angle,target.steering_angle,c.steering_time_constant,c.max_steering_rate,h);
        elapsed+=h;
    }
    s.acceleration=(s.velocity-initial_v)/dt; s.steering_angle=m.steering_angle; s.grip_limited=limited;
    m.time=start+dt;
    const double oldest=m.time-std::max(c.acceleration_delay,c.steering_delay);
    while (m.history.size()>1 && m.history[1].time <= oldest + 1e-12) m.history.pop_front();
    validate(s,c);
}
}
