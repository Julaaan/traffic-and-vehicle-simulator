#pragma once
#include "Options.hpp"
#include "vsim/TrafficEvents.hpp"
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <random>
#include <string>
#include <utility>

namespace vsim::app {
// Simulated V2X link for --delivery network: every traffic event travels in its own message.
struct V2xNetwork {
    double latency_ms{20.0};  // fixed part of the delay
    double jitter_ms{30.0};   // mean of the exponential extra delay
    double loss{0.01};        // probability that a message never arrives
};
std::optional<V2xNetwork> v2xNetwork(const Options& options);
void validateRunId(const std::string& id);
// <scenario>-s<seed>-<hash of version, scenario, steps and delivery>, so re-exporting the same run
// yields the same id.
std::string defaultRunId(const Options& options, const Simulation& sim, std::uint64_t steps);

// Writes a run in DataGarage's dg-events/1 format (docs/EVENT_CONTRACT.md in data-garage):
// <dir>/<run_id>/run.json and events.jsonl. Events are flushed after every step, so DataGarage can
// follow a run while it is still being simulated.
class DataGarageExport {
public:
    // Creates the run directory and writes the events visible before the first step.
    DataGarageExport(const Options& options, const Simulation& sim, std::uint64_t steps);
    ~DataGarageExport();
    DataGarageExport(const DataGarageExport&) = delete;
    DataGarageExport& operator=(const DataGarageExport&) = delete;
    void capture(const Simulation& sim);
    // Delivers messages still in flight, then marks the run completed.
    void finish(const Simulation& sim);
    void failure(const std::string& message);
    const std::filesystem::path& directory() const { return directory_; }
private:
    struct Message {
        TrafficEvent event;
        std::uint64_t event_id{}, sent_id{}, received_id{}, interval{};
        std::int64_t sent_ms{}, received_ms{};
    };
    std::int64_t milliseconds(std::uint64_t tick) const;
    void accept(const TrafficEvent& event);
    void deliver(std::int64_t until_ms);
    void writeEvent(std::uint64_t id, std::int64_t ms, std::string_view type, EntityId vehicle,
                    const std::string& intersection, std::uint64_t interval);
    void writeManifest();
    double uniform();
    std::optional<V2xNetwork> network_;
    std::mt19937_64 random_;
    std::filesystem::path directory_;
    std::ofstream events_;
    nlohmann::ordered_json manifest_;
    TrafficEventDetector detector_;
    std::vector<TrafficEvent> pending_;
    std::map<std::pair<std::int64_t, std::uint64_t>, Message> in_flight_;
    std::uint64_t step_us_{}, next_event_{}, written_{}, sent_{}, lost_{};
    bool closed_{};
};
}
