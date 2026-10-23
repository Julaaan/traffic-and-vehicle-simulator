#pragma once
#include "vsim/RoadNetwork.hpp"
#include <filesystem>
#include <optional>

namespace vsim {
struct Scenario {
    std::string name{"Four-way intersection"};
    std::uint32_t seed{42};
    double time_step{0.01};
    double duration{60.0};
    RoadNetwork road{RoadNetwork::intersection()};
    TrafficConfig traffic;
    std::vector<Vehicle> vehicles;
};
struct ScenarioOverrides {
    std::optional<std::uint32_t> seed;
    std::optional<std::size_t> vehicle_count;
};
Scenario loadScenario(const std::filesystem::path& path, const ScenarioOverrides& overrides = {});
}
