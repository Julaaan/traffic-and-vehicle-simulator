#include "vsim/TrafficEvents.hpp"
#include <cmath>
#include <stdexcept>

namespace vsim {
namespace {
constexpr const char* builtin_intersection = "center";

bool inJunction(const Lane& lane) { return lane.connector && lane.junction.has_value(); }
std::string junctionName(const RoadNetwork& road, const Lane& lane) {
    return "J" + std::to_string(road.junctions.at(*lane.junction).id);
}
double centerProgress(const RoadNetwork& road, const Vehicle& v) {
    return road.lane(v.lane).project(v.state.position()).progress;
}
// The built-in intersection's box, the same area World keeps crossing traffic out of.
bool insideBox(const RoadNetwork& road, const Vehicle& v) {
    const double center = centerProgress(road, v);
    return center + v.config.length / 2.0 > road.stopLine()
        && center - v.config.length / 2.0 < road.arm_length + road.lane_width + 1.0;
}
std::optional<std::string> approached(const RoadNetwork& road, const Vehicle& v) {
    if (!v.active) return std::nullopt;
    if (!road.imported) {
        if (centerProgress(road, v) + v.config.length / 2.0 <= road.stopLine()) return builtin_intersection;
        return std::nullopt;
    }
    if (inJunction(road.lane(v.lane)) || v.route.empty() || v.route_index + 1 >= v.route.size()) return std::nullopt;
    const auto& next = road.lane(v.route[v.route_index + 1]);
    if (!inJunction(next)) return std::nullopt;
    return junctionName(road, next);
}
}

std::string_view eventTypeName(TrafficEventType type) {
    switch (type) {
    case TrafficEventType::Entered: return "entered";
    case TrafficEventType::Exited: return "exited";
    case TrafficEventType::WaitStarted: return "wait_started";
    case TrafficEventType::WaitEnded: return "wait_ended";
    }
    throw std::invalid_argument("Unknown traffic event type");
}

void TrafficEventDetector::observe(const Simulation& sim, std::vector<TrafficEvent>& events) {
    const auto& road = sim.world().road();
    const auto& vehicles = sim.world().vehicles();
    if (!started_) {
        for (const auto& v : vehicles) {
            const bool inside = v.active && (road.imported ? inJunction(road.lane(v.lane)) : insideBox(road, v));
            tracks_.push_back({v.id, v.lane, v.route_index, v.active, inside, std::nullopt, std::nullopt});
        }
        started_ = true;
    }
    if (tracks_.size() != vehicles.size()) throw std::logic_error("Fleet changed between traffic event observations");
    for (std::size_t i = 0; i < vehicles.size(); ++i) {
        if (tracks_[i].id != vehicles[i].id) throw std::logic_error("Fleet changed between traffic event observations");
        update(road, vehicles[i], tracks_[i], sim.clock().tick(), events);
    }
}

void TrafficEventDetector::update(const RoadNetwork& road, const Vehicle& v, Track& track, std::uint64_t tick,
                                  std::vector<TrafficEvent>& events) {
    if (!track.active) return;
    const auto emit = [&](TrafficEventType type, const Open& open) {
        events.push_back({tick, type, v.id, open.intersection, open.interval});
    };
    const bool changed_lane = v.lane != track.lane || v.route_index != track.route_index;
    if (track.wait) {
        if (!v.active) track.wait.reset();
        else if (changed_lane || std::abs(v.state.velocity) > wait_end_speed || approached(road, v) != track.wait->intersection) {
            emit(TrafficEventType::WaitEnded, *track.wait);
            track.wait.reset();
        }
    }
    if (road.imported) {
        for (std::size_t r = track.route_index + 1; r <= v.route_index && r < v.route.size(); ++r) {
            const auto& from = road.lane(v.route[r - 1]);
            const auto& to = road.lane(v.route[r]);
            if (inJunction(from)) {
                if (track.passage) emit(TrafficEventType::Exited, *track.passage);
                track.passage.reset();
            }
            if (inJunction(to)) {
                track.passage = Open{newInterval(), junctionName(road, to)};
                emit(TrafficEventType::Entered, *track.passage);
            }
        }
    } else if (v.active) {
        const bool inside = insideBox(road, v);
        if (inside && !track.inside) {
            track.passage = Open{newInterval(), builtin_intersection};
            emit(TrafficEventType::Entered, *track.passage);
        } else if (!inside && track.inside) {
            if (track.passage) emit(TrafficEventType::Exited, *track.passage);
            track.passage.reset();
        }
        track.inside = inside;
    }
    if (v.active && !track.wait && std::abs(v.state.velocity) < wait_start_speed) {
        if (auto at = approached(road, v)) {
            track.wait = Open{newInterval(), std::move(*at)};
            emit(TrafficEventType::WaitStarted, *track.wait);
        }
    }
    track.lane = v.lane;
    track.route_index = v.route_index;
    track.active = v.active;
}
}
