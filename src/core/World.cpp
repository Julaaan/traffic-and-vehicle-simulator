#include "vsim/World.hpp"
#include "vsim/Dynamics.hpp"
#include <algorithm>
#include <chrono>
#include <limits>
#include <optional>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace vsim {
StepProfile& StepProfile::operator+=(const StepProfile& other) {
    projection += other.projection; admission += other.admission; control += other.control;
    integration += other.integration; transition += other.transition; collision += other.collision;
    steps += other.steps;
    return *this;
}

World::World(const Scenario& scenario)
    : road_(scenario.road), traffic_(scenario.traffic), vehicles_(scenario.vehicles) {
    validate(traffic_);
    std::unordered_set<LaneId> lane_ids;
    lane_lengths_.reserve(road_.lanes.size());
    for (const auto& lane : road_.lanes) {
        if (!lane_ids.insert(lane.id).second) throw std::invalid_argument("Lane ids must be unique");
        if (lane.centerline.size() < 2 || !std::isfinite(lane.width) || lane.width <= 0.0
            || !std::isfinite(lane.speed_limit) || lane.speed_limit <= 0.0)
            throw std::invalid_argument("Invalid lane geometry or limits");
        for (Vec2 p : lane.centerline) if (!finite(p)) throw std::invalid_argument("Lane coordinates must be finite");
        lane_lengths_.push_back(lane.length());
        if (lane_lengths_.back() <= 0.0) throw std::invalid_argument("Lane must have positive length");
        for (LaneId next : lane.successors) road_.lane(next);
        if (lane.junction && *lane.junction >= road_.junctions.size()) throw std::invalid_argument("Invalid junction reference");
        if (lane.connector && (!lane.junction || lane.successors.size()!=1 || road_.lane(lane.successors.front()).connector))
            throw std::invalid_argument("Connector requires one outgoing road and a junction");
    }
    std::sort(vehicles_.begin(), vehicles_.end(), [](const Vehicle& a, const Vehicle& b) { return a.id < b.id; });
    EntityId previous = 0;
    for (auto& vehicle : vehicles_) {
        if (vehicle.id == 0 || vehicle.id == previous)
            throw std::invalid_argument("Vehicle ids must be unique and positive");
        previous = vehicle.id;
        road_.lane(vehicle.lane);
        validate(vehicle.config);
        validate(vehicle.state, vehicle.config);
        validate(vehicle.control);
        if (!std::isfinite(vehicle.desired_speed) || vehicle.desired_speed <= 0.0
            || vehicle.desired_speed > vehicle.config.max_speed)
            throw std::invalid_argument("Desired speed must be positive and within vehicle limits");
        if (!std::isfinite(vehicle.stopped_time) || vehicle.stopped_time<0)
            throw std::invalid_argument("Stopped time must be finite and nonnegative");
        vehicle.state.yaw = wrapAngle(vehicle.state.yaw);
        if (!vehicle.route.empty()) {
            if (vehicle.route_index >= vehicle.route.size() || vehicle.route[vehicle.route_index] != vehicle.lane)
                throw std::invalid_argument("Vehicle route does not match current lane");
            for (std::size_t i = 1; i < vehicle.route.size(); ++i) {
                const auto& previous_lane = road_.lane(vehicle.route[i - 1]);
                if (std::find(previous_lane.successors.begin(), previous_lane.successors.end(), vehicle.route[i]) == previous_lane.successors.end())
                    throw std::invalid_argument("Vehicle route contains disconnected lanes");
            }
        }
        if (road_.imported) vehicle.layer = road_.lane(vehicle.lane).layer;
    }
    collisions_ = findCollisions(vehicles_);
}

double World::length(const Lane& lane) const {
    return lane_lengths_[static_cast<std::size_t>(&lane - road_.lanes.data())];
}

std::vector<bool> World::networkAdmissions(const std::vector<LaneProjection>& projections, double time) {
    std::vector<bool> allowed(vehicles_.size(), true);
    std::vector<std::vector<LaneId>> occupied(road_.junctions.size());
    struct Candidate { std::size_t vehicle{}, junction{}; LaneId connector{}; double arrival{}; bool commit{}; };
    std::vector<Candidate> candidates;
    for (std::size_t i = 0; i < vehicles_.size(); ++i) {
        const auto& v = vehicles_[i];
        if (!v.active) continue;
        const auto& lane = road_.lane(v.lane);
        if (lane.connector && lane.junction) occupied[*lane.junction].push_back(lane.id);
        if (!lane.connector && v.route_index > 0 && !v.route.empty()) {
            const auto& previous = road_.lane(v.route[v.route_index - 1]);
            if (previous.connector && previous.junction && projections[i].progress < v.config.length / 2.0 + 2.0)
                occupied[*previous.junction].push_back(previous.id);
        }
        if (lane.connector || v.route.empty() || v.route_index + 1 >= v.route.size()) continue;
        const auto& next = road_.lane(v.route[v.route_index + 1]);
        if (!next.connector || !next.junction) continue;
        const double gap = length(lane) - 0.5 - projections[i].progress - v.config.length / 2.0;
        if (v.junction_admitted || gap <= 0.0) { occupied[*next.junction].push_back(next.id); continue; }
        allowed[i] = false;
        if (gap > 35.0 || !greenFor(road_, lane.id, time, traffic_) || (lane.stop_sign && v.stopped_time < 1.0)) continue;
        bool exit_clear = true;
        const LaneId exit = next.successors.front();
        for (std::size_t j = 0; j < vehicles_.size(); ++j)
            if (j != i && vehicles_[j].active && vehicles_[j].lane == exit
                && projections[j].progress < v.config.length + traffic_.minimum_gap + vehicles_[j].config.length / 2.0)
                exit_clear = false;
        if (!exit_clear) continue;
        const int road_priority = lane.priority - (lane.give_way || lane.stop_sign ? 2 : 0);
        const double commit_distance = std::max(10.0, v.state.velocity * v.state.velocity / (2.0 * v.config.max_deceleration)
            + traffic_.minimum_gap + 5.0);
        candidates.push_back({i, *next.junction, next.id,
            gap / (v.state.velocity + 3.0) - 0.1 * road_priority, gap <= commit_distance});
    }
    std::sort(candidates.begin(), candidates.end(), [&](const auto& a, const auto& b) {
        if (a.arrival != b.arrival) return a.arrival < b.arrival;
        return vehicles_[a.vehicle].id < vehicles_[b.vehicle].id;
    });
    for (const auto& c : candidates) {
        const auto& claims = occupied[c.junction];
        if (std::any_of(claims.begin(), claims.end(), [&](LaneId other) { return other != c.connector; })) continue;
        allowed[c.vehicle] = true;
        if (c.commit) vehicles_[c.vehicle].junction_admitted = true;
        occupied[c.junction].push_back(c.connector);
    }
    return allowed;
}

VehicleControl World::networkControl(std::size_t index, const std::vector<LaneProjection>& projections, bool may_enter) const {
    const auto& v = vehicles_[index]; const auto& lane = road_.lane(v.lane);
    const double s = projections[index].progress, speed = v.state.velocity;
    double desired_speed = std::min(v.desired_speed, lane.speed_limit);
    const double lookahead = std::clamp(2.5 + speed * 0.4, 3.0, 12.0);
    LaneSample target = lane.sample(s + lookahead);
    std::vector<std::pair<LaneId, double>> route_ahead;
    if (!v.route.empty()) {
        double remaining = s + lookahead;
        double to_next = length(lane) - s;
        for (std::size_t r = v.route_index; r < v.route.size(); ++r) {
            const auto& next = road_.lane(v.route[r]);
            if (remaining <= length(next) || r + 1 == v.route.size()) { target = next.sample(remaining); break; }
            remaining -= length(next);
        }
        for (std::size_t r = v.route_index + 1; r < v.route.size() && to_next < 60.0; ++r) {
            const auto& next = road_.lane(v.route[r]);
            desired_speed = std::min(desired_speed, std::sqrt(next.speed_limit * next.speed_limit + 4.0 * std::max(0.0, to_next - 3.0)));
            to_next += length(next);
        }
        double offset = length(lane) - s;
        for (std::size_t r = v.route_index + 1; r < v.route.size() && offset < 150.0; ++r) {
            route_ahead.emplace_back(v.route[r], offset);
            offset += length(road_.lane(v.route[r]));
        }
    }
    const auto routeOffset = [&route_ahead](LaneId id) -> std::optional<double> {
        for (const auto& [lane_id, offset] : route_ahead) if (lane_id == id) return offset;
        return std::nullopt;
    };
    double gap = std::numeric_limits<double>::infinity(), leader_speed = speed;
    for (std::size_t j = 0; j < vehicles_.size(); ++j) {
        const auto& other = vehicles_[j];
        if (j == index || !other.active) continue;
        double ahead = -1.0;
        if (other.lane == v.lane) ahead = projections[j].progress - s;
        else if (!v.route.empty()) {
            if (const auto offset = routeOffset(other.lane)) ahead = *offset + projections[j].progress;
            const auto& other_lane = road_.lane(other.lane);
            if (ahead < 0.0 && other_lane.connector && !other_lane.successors.empty())
                if (const auto offset = routeOffset(other_lane.successors.front()))
                    ahead = *offset + projections[j].progress - length(other_lane);
        }
        const double candidate = ahead - (v.config.length + other.config.length) / 2.0;
        if (ahead > 0.0 && candidate < gap) { gap = candidate; leader_speed = other.state.velocity; }
    }
    if (!may_enter && !lane.connector) {
        const double to_line = length(lane) - 0.5 - s - v.config.length / 2.0;
        if (to_line > 0.0 && to_line < gap) { gap = to_line; leader_speed = 0.0; }
    }
    const double ratio = speed / std::max(0.1, desired_speed);
    double acceleration = v.config.max_acceleration * (1.0 - std::pow(ratio, 4.0));
    if (std::isfinite(gap)) {
        const double desired_gap = traffic_.minimum_gap + std::max(0.0,
            speed * traffic_.time_headway + speed * (speed - leader_speed) / (2.0 * std::sqrt(v.config.max_acceleration * 3.0)));
        acceleration -= v.config.max_acceleration * std::pow(desired_gap / std::max(0.1, gap), 2.0);
    }
    const Vec2 relative = target.position - v.state.position();
    const double distance = norm(relative);
    const double alpha = wrapAngle(std::atan2(relative.y, relative.x) - v.state.yaw);
    const double steering = distance > 1e-6 ? std::atan2(2.0 * v.config.wheelbase * std::sin(alpha), distance) : 0.0;
    return {std::clamp(acceleration, -v.config.max_deceleration, v.config.max_acceleration),
            std::clamp(steering, -v.config.max_steering, v.config.max_steering)};
}

VehicleControl World::automaticControl(std::size_t index,
    const std::vector<LaneProjection>& projections, double time) const {
    const auto& vehicle = vehicles_[index];
    const auto& projection = projections[index];
    const auto& lane = road_.lane(vehicle.lane);
    const double v = vehicle.state.velocity;
    double gap = std::numeric_limits<double>::infinity();
    double leader_speed = v;
    for (std::size_t j = 0; j < vehicles_.size(); ++j) {
        const auto& other = vehicles_[j];
        if (j == index || !other.active || other.lane != vehicle.lane) continue;
        const double ahead = projections[j].progress - projection.progress;
        const double candidate = ahead - (vehicle.config.length + other.config.length) / 2.0;
        if (ahead > 0.0 && candidate < gap) { gap = candidate; leader_speed = other.state.velocity; }
    }
    const double front = projection.progress + vehicle.config.length / 2.0;
    bool may_enter = greenFor(vehicle.lane, time, traffic_);
    if (traffic_.enabled && may_enter) {
        for (std::size_t j = 0; j < vehicles_.size(); ++j) {
            const auto& other = vehicles_[j];
            if (!other.active || (other.lane < 2) == (vehicle.lane < 2)) continue;
            if (projections[j].progress + other.config.length / 2.0 > road_.stopLine()
                && projections[j].progress - other.config.length / 2.0
                    < road_.arm_length + road_.lane_width + 1.0) {
                may_enter = false;
                break;
            }
        }
    }
    if (!may_enter && front < road_.stopLine()) {
        const double to_line = road_.stopLine() - front;
        if (to_line < gap) { gap = to_line; leader_speed = 0.0; }
    }
    const double ratio = v / vehicle.desired_speed;
    double acceleration = vehicle.config.max_acceleration * (1.0 - ratio * ratio * ratio * ratio);
    if (std::isfinite(gap)) {
        const double closing = v - leader_speed;
        const double desired_gap = traffic_.minimum_gap + std::max(0.0,
            v * traffic_.time_headway + v * closing / (2.0 * std::sqrt(vehicle.config.max_acceleration * 3.0)));
        const double gap_ratio = desired_gap / std::max(0.1, gap);
        acceleration -= vehicle.config.max_acceleration * gap_ratio * gap_ratio;
    }
    const auto target = lane.sample(projection.progress + 5.0 + 0.6 * v);
    const Vec2 relative = target.position - vehicle.state.position();
    const double distance = norm(relative);
    double steering = 0.0;
    if (distance > 1e-6) {
        const double angle = wrapAngle(std::atan2(relative.y, relative.x) - vehicle.state.yaw);
        steering = std::atan2(2.0 * vehicle.config.wheelbase * std::sin(angle), distance);
    }
    return {std::clamp(acceleration, -vehicle.config.max_deceleration, vehicle.config.max_acceleration),
        std::clamp(steering, -vehicle.config.max_steering, vehicle.config.max_steering)};
}

void World::step(double dt, double time, std::span<const RecordedControl> recorded, StepProfile* profile) {
    if (!std::isfinite(dt) || dt <= 0.0 || !std::isfinite(time) || time < 0.0)
        throw std::invalid_argument("Invalid world step or simulation time");
    using Clock = std::chrono::steady_clock;
    auto mark = profile ? Clock::now() : Clock::time_point{};
    const auto lap = [&](double StepProfile::*phase) {
        if (!profile) return;
        const auto now = Clock::now();
        profile->*phase += std::chrono::duration<double>(now - mark).count();
        mark = now;
    };
    if (profile) ++profile->steps;
    if (!recorded.empty()) {
        if (recorded.size() != vehicles_.size()) throw std::invalid_argument("Replay control count differs from fleet");
        for (std::size_t i=0; i<recorded.size(); ++i) {
            if (recorded[i].id != vehicles_[i].id) throw std::invalid_argument("Replay vehicle ids differ from fleet");
            validate(recorded[i].control);
        }
        for (std::size_t i=0; i<recorded.size(); ++i) {
            vehicles_[i].control=recorded[i].control; vehicles_[i].automatic=recorded[i].automatic;
        }
    }
    std::vector<LaneProjection> projections;
    projections.reserve(vehicles_.size());
    for (const auto& vehicle : vehicles_)
        projections.push_back(road_.lane(vehicle.lane).project(vehicle.state.position()));
    lap(&StepProfile::projection);
    if (road_.imported) {
        for (std::size_t i = 0; i < vehicles_.size(); ++i) {
            auto& v = vehicles_[i]; const auto& lane = road_.lane(v.lane);
            const double gap = length(lane) - 0.5 - projections[i].progress - v.config.length / 2.0;
            if (v.active && lane.stop_sign && gap < 3.0 && v.state.velocity < 0.15) v.stopped_time += dt;
            else if (gap > 4.0 || v.state.velocity > 0.5) v.stopped_time = 0.0;
        }
    }
    const auto admissions = road_.imported ? networkAdmissions(projections, time) : std::vector<bool>{};
    lap(&StepProfile::admission);
    for (std::size_t i = 0; i < vehicles_.size(); ++i)
        if (recorded.empty() && vehicles_[i].active && vehicles_[i].automatic)
            vehicles_[i].control = road_.imported ? networkControl(i, projections, admissions[i]) : automaticControl(i, projections, time);
    lap(&StepProfile::control);
    for (auto& vehicle : vehicles_) {
        if (!vehicle.active) continue;
        integrate(vehicle.state, vehicle.config, vehicle.control, dt);
    }
    lap(&StepProfile::integration);
    if (!road_.imported) { collisions_ = findCollisions(vehicles_); lap(&StepProfile::collision); }
    for (auto& vehicle : vehicles_) {
        if (!vehicle.active) continue;
        for (unsigned transition = 0; transition < 16; ++transition) {
            const auto& lane = road_.lane(vehicle.lane);
            const auto& points = lane.centerline;
            const Vec2 direction = points.back() - points[points.size() - 2];
            if (road_.imported && lane.project(vehicle.state.position()).progress < length(lane) - 2.0) break;
            if (dot(vehicle.state.position() - points.back(), direction) < 0.0) break;
            if (vehicle.route.empty() || vehicle.route_index + 1 >= vehicle.route.size()) { vehicle.active = false; break; }
            vehicle.lane = vehicle.route[++vehicle.route_index];
            vehicle.layer = road_.lane(vehicle.lane).layer;
            vehicle.stopped_time = 0.0;
            vehicle.junction_admitted = false;
        }
    }
    lap(&StepProfile::transition);
    if (road_.imported) { collisions_ = findCollisions(vehicles_); lap(&StepProfile::collision); }
}

void World::restore(const std::vector<Vehicle>& vehicles, const std::vector<CollisionPair>& collisions) {
    Scenario restored; restored.road=road_; restored.traffic=traffic_; restored.vehicles=vehicles;
    World checked(restored);
    if (checked.vehicles_.size()!=vehicles_.size()) throw std::invalid_argument("Snapshot fleet size differs");
    for (std::size_t i=0; i<vehicles_.size(); ++i)
        if (checked.vehicles_[i].id != vehicles_[i].id) throw std::invalid_argument("Snapshot vehicle ids differ");
    for (std::size_t i=0; i<collisions.size(); ++i) {
        const auto p=collisions[i];
        if (p.first>=p.second) throw std::invalid_argument("Invalid snapshot collision pair");
        checked.vehicle(p.first); checked.vehicle(p.second);
        if (i && std::pair{collisions[i-1].first,collisions[i-1].second} >= std::pair{p.first,p.second})
            throw std::invalid_argument("Snapshot collision pairs must be sorted and unique");
    }
    vehicles_=std::move(checked.vehicles_); collisions_=collisions;
}

std::size_t World::activeCount() const {
    return static_cast<std::size_t>(std::count_if(vehicles_.begin(), vehicles_.end(), [](const auto& v) { return v.active; }));
}
const Vehicle& World::vehicle(EntityId id) const {
    const auto found = std::find_if(vehicles_.begin(), vehicles_.end(), [id](const auto& v) { return v.id == id; });
    if (found == vehicles_.end()) throw std::invalid_argument("Unknown vehicle id");
    return *found;
}
}
