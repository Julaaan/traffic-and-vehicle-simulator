#pragma once
#include "vsim/Simulation.hpp"
#include <optional>
#include <string>
#include <string_view>

namespace vsim {
enum class TrafficEventType { Entered, Exited, WaitStarted, WaitEnded };
std::string_view eventTypeName(TrafficEventType type);

// Something that became visible after the step that ended at `tick`.
struct TrafficEvent {
    std::uint64_t tick{};
    TrafficEventType type{};
    EntityId vehicle{};
    std::string intersection;
    std::uint64_t interval{};
};

// Derives intersection passages and waits from consecutive simulation states. It only reads the
// simulation, so results and state hashes are unchanged.
//   Passage: on imported maps, from entering a junction connector lane until leaving it; on the
//            built-in intersection, from the front crossing the stop line until the rear has left
//            the box World keeps crossing traffic out of.
//   Wait:    approaching an intersection slower than wait_start_speed, until faster than
//            wait_end_speed, leaving the approach lane, or entering the intersection.
// A passage already in progress at the first observation, or an interval that never finishes,
// produces no event for the missing side.
class TrafficEventDetector {
public:
    static constexpr double wait_start_speed = 0.1;
    static constexpr double wait_end_speed = 0.5;
    // Appends the events since the previous call in vehicle id order. Call once before the first
    // step (records the initial state and vehicles already waiting) and after every step.
    void observe(const Simulation& sim, std::vector<TrafficEvent>& events);
    // One id sequence for all intervals, so other interval kinds can draw from it too.
    std::uint64_t newInterval() { return next_interval_++; }
private:
    struct Open { std::uint64_t interval{}; std::string intersection; };
    struct Track {
        EntityId id{};
        LaneId lane{};
        std::size_t route_index{};
        bool active{};
        bool inside{};
        std::optional<Open> passage, wait;
    };
    void update(const RoadNetwork& road, const Vehicle& v, Track& track, std::uint64_t tick,
                std::vector<TrafficEvent>& events);
    std::vector<Track> tracks_;
    bool started_{};
    std::uint64_t next_interval_{};
};
}
