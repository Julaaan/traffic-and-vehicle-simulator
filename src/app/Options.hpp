#pragma once
#include "vsim/Scenario.hpp"
#include <filesystem>
#include <optional>

namespace vsim::app {
struct Options {
    std::filesystem::path scenario;
    std::filesystem::path map;
    ScenarioOverrides overrides;
    std::optional<double> duration;
    std::optional<std::uint64_t> steps;
    std::filesystem::path csv;
    std::filesystem::path report;
    std::filesystem::path screenshot;
    std::filesystem::path record, replay, failure_dir, dynamics_profile;
    std::optional<std::uint64_t> seek;
    std::uint64_t sample_every{10};
    bool help{};
    bool auto_exit{};
    bool map_only{};
    bool verify_replay{};
};
Options parseOptions(int argc, char** argv);
std::uint64_t runSteps(const Options& options, const Scenario& scenario);
void printHelp(bool viewer);
Scenario loadInput(const Options& options);
}
