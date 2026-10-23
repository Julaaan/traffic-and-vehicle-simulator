#include "vsim/RoadNetwork.hpp"
#include <algorithm>
#include <limits>
#include <stdexcept>

namespace vsim {
double Lane::length() const {
    double result = 0.0;
    for (std::size_t i = 1; i < centerline.size(); ++i)
        result += norm(centerline[i] - centerline[i - 1]);
    return result;
}

LaneSample Lane::sample(double progress) const {
    if (!std::isfinite(progress) || centerline.size() < 2)
        throw std::invalid_argument("Cannot sample invalid lane or progress");
    double remaining = std::clamp(progress, 0.0, length());
    for (std::size_t i = 1; i < centerline.size(); ++i) {
        const Vec2 delta = centerline[i] - centerline[i - 1];
        const double segment = norm(delta);
        if (segment <= 0.0) continue;
        if (remaining <= segment || i == centerline.size() - 1)
            return {centerline[i - 1] + delta * (remaining / segment), std::atan2(delta.y, delta.x)};
        remaining -= segment;
    }
    throw std::invalid_argument("Lane has no nonzero segment");
}

LaneProjection Lane::project(Vec2 point) const {
    if (!finite(point) || centerline.size() < 2)
        throw std::invalid_argument("Cannot project on invalid lane");
    double best = std::numeric_limits<double>::infinity();
    double prefix = 0.0;
    LaneProjection result;
    for (std::size_t i = 1; i < centerline.size(); ++i) {
        const Vec2 delta = centerline[i] - centerline[i - 1];
        const double segment = norm(delta);
        if (segment <= 0.0) continue;
        const Vec2 unit = delta * (1.0 / segment);
        const double local = std::clamp(dot(point - centerline[i - 1], unit), 0.0, segment);
        const Vec2 difference = point - (centerline[i - 1] + unit * local);
        const double distance = dot(difference, difference);
        if (distance < best) {
            best = distance;
            result = {prefix + local, cross(unit, difference), std::atan2(unit.y, unit.x)};
        }
        prefix += segment;
    }
    if (!std::isfinite(best)) throw std::invalid_argument("Lane has no nonzero segment");
    return result;
}

RoadNetwork RoadNetwork::intersection(double arm, double width) {
    if (!std::isfinite(arm) || !std::isfinite(width) || width < 2.0 || arm < width * 4.0
        || arm > 10000.0 || width > 20.0)
        throw std::invalid_argument("Intersection requires arm_length in [4*lane_width, 10000] and lane_width in [2,20]");
    RoadNetwork road;
    road.arm_length = arm;
    road.lane_width = width;
    const double h = width / 2.0;
    const auto lane = [width](LaneId id, const char* name, Vec2 from, Vec2 to) {
        Lane result;
        result.id = id; result.name = name; result.width = width; result.centerline = {from, to};
        return result;
    };
    road.lanes = {lane(0, "eastbound", {-arm, -h}, {arm, -h}), lane(1, "westbound", {arm, h}, {-arm, h}),
        lane(2, "northbound", {h, -arm}, {h, arm}), lane(3, "southbound", {-h, arm}, {-h, -arm})};
    return road;
}

const Lane& RoadNetwork::lane(LaneId id) const {
    if (id < lanes.size() && lanes[id].id == id) return lanes[id];
    const auto found = std::find_if(lanes.begin(), lanes.end(), [id](const Lane& l) { return l.id == id; });
    if (found == lanes.end()) throw std::invalid_argument("Unknown lane id");
    return *found;
}

MapBounds RoadNetwork::bounds() const {
    MapBounds result{{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()},
                     {-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}};
    for (const auto& lane : lanes) for (Vec2 p : lane.centerline) {
        result.minimum.x = std::min(result.minimum.x, p.x - lane.width);
        result.minimum.y = std::min(result.minimum.y, p.y - lane.width);
        result.maximum.x = std::max(result.maximum.x, p.x + lane.width);
        result.maximum.y = std::max(result.maximum.y, p.y + lane.width);
    }
    if (!finite(result.minimum) || !finite(result.maximum)) return {{-10.0, -10.0}, {10.0, 10.0}};
    return result;
}
const Lane& RoadNetwork::lane(const std::string& name) const {
    const auto found = std::find_if(lanes.begin(), lanes.end(), [&name](const Lane& l) { return l.name == name; });
    if (found == lanes.end()) throw std::invalid_argument("Unknown lane name: " + name);
    return *found;
}

void validate(const TrafficConfig& c) {
    for (double value : {c.green_seconds, c.clearance_seconds, c.minimum_gap, c.time_headway})
        if (!std::isfinite(value) || value <= 0.0)
            throw std::invalid_argument("Traffic timing and following gaps must be finite and positive");
}
namespace {
enum class SignalPhase { NorthSouth, AllRed, EastWest };
SignalPhase signalPhase(double time, const TrafficConfig& c) {
    if (!std::isfinite(time) || time < 0.0) throw std::invalid_argument("Signal time must be finite and nonnegative");
    validate(c);
    const double half_cycle = c.green_seconds + c.clearance_seconds;
    const double phase = std::fmod(time, 2.0 * half_cycle);
    if (phase < c.green_seconds) return SignalPhase::NorthSouth;
    if (phase < half_cycle) return SignalPhase::AllRed;
    if (phase < half_cycle + c.green_seconds) return SignalPhase::EastWest;
    return SignalPhase::AllRed;
}
}
bool greenFor(LaneId lane, double time, const TrafficConfig& c) {
    if (!c.enabled) return true;
    const auto phase = signalPhase(time, c);
    return (lane < 2 && phase == SignalPhase::EastWest)
        || (lane >= 2 && phase == SignalPhase::NorthSouth);
}
bool greenFor(const RoadNetwork& road, LaneId id, double time, const TrafficConfig& c) {
    if (!road.imported) return greenFor(id, time, c);
    const auto& lane = road.lane(id);
    if (!c.enabled || !lane.junction || !road.junctions.at(*lane.junction).traffic_lights) return true;
    const auto phase = signalPhase(time, c);
    return (lane.signal_group == 0 && phase == SignalPhase::EastWest)
        || (lane.signal_group == 1 && phase == SignalPhase::NorthSouth);
}
}
