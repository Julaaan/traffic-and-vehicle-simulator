#pragma once
#include "vsim/Serialization.hpp"
#include <deque>
#include <fstream>
#include <memory>
#include <set>

namespace vsim {
struct DiagnosticEvent {
    std::uint64_t tick{};
    std::string type, severity, message;
    std::vector<EntityId> vehicles;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(DiagnosticEvent, tick, type, severity, message, vehicles)
class RunRecorder {
public:
    RunRecorder(const Simulation& sim, const std::filesystem::path& file = {}, const std::filesystem::path& failure_dir = {});
    ~RunRecorder();
    void capture(const Simulation& sim);
    void finish(const Simulation& sim);
    void failure(const std::string& message);
private:
    struct Frame { nlohmann::json data; SimulationSnapshot snapshot; };
    void saveFailure(const std::vector<DiagnosticEvent>& events);
    nlohmann::json header(const SimulationSnapshot& start, const std::string& hash) const;
    void write(const nlohmann::json& data);
    Scenario scenario_;
    SimulationSnapshot base_, last_;
    std::string base_hash_, last_hash_;
    std::ofstream output_;
    std::filesystem::path failure_dir_;
    std::deque<Frame> ring_;
    std::set<std::string> previous_conditions_;
    std::size_t saved_failures_{};
    bool finished_{};
};
class ReplayPlayer {
public:
    explicit ReplayPlayer(const std::filesystem::path& path);
    Simulation& simulation() { return *simulation_; }
    const Simulation& simulation() const { return *simulation_; }
    std::uint64_t firstTick() const { return start_.tick; }
    std::uint64_t lastTick() const { return end_tick_; }
    bool advance();
    void seek(std::uint64_t tick);
    void verify();
private:
    struct Index { std::uint64_t tick; std::streampos offset; bool checkpoint; };
    nlohmann::json read(std::streampos offset);
    void check(const nlohmann::json& frame) const;
    std::ifstream input_;
    std::unique_ptr<Simulation> simulation_;
    SimulationSnapshot start_;
    std::string end_hash_;
    std::uint64_t end_tick_{};
    std::vector<Index> index_;
    std::size_t next_{};
};
}
