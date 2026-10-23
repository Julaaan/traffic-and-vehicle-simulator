#include "vsim/Batch.hpp"
#include "vsim/Dynamics.hpp"
#include "vsim/OsmImporter.hpp"
#include "vsim/Serialization.hpp"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <charconv>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace {
struct BenchOptions {
    std::filesystem::path scenario, map, dynamics_profile, json;
    std::vector<std::uint64_t> envs{1, 8, 64, 256};
    std::vector<std::uint64_t> threads;
    std::optional<std::size_t> vehicles;
    std::uint64_t steps{500};
    std::uint32_t seed{1};
    bool help{};
};
struct Result { std::size_t envs{}; unsigned threads{}; std::size_t vehicles{}, active{}; double seconds{}; std::uint64_t hash{}; };

unsigned hardwareThreads() { return std::max(1U, std::thread::hardware_concurrency()); }
std::uint64_t integer(const std::string& value, const std::string& option, std::uint64_t low, std::uint64_t high) {
    std::uint64_t result{};
    const auto [end, error] = std::from_chars(value.data(), value.data() + value.size(), result);
    if (error != std::errc{} || end != value.data() + value.size() || result < low || result > high)
        throw std::invalid_argument(option + " requires integers in [" + std::to_string(low) + ", " + std::to_string(high) + "]");
    return result;
}
std::vector<std::uint64_t> list(const std::string& value, const std::string& option, std::uint64_t high) {
    std::vector<std::uint64_t> result;
    std::stringstream stream(value);
    for (std::string item; std::getline(stream, item, ',');)
        result.push_back(option == "--threads" && item == "all" ? hardwareThreads() : integer(item, option, 1, high));
    if (result.empty()) throw std::invalid_argument(option + " requires a comma-separated list");
    return result;
}
BenchOptions parse(int argc, char** argv) {
    BenchOptions o;
    o.scenario = std::filesystem::absolute(argv[0]).parent_path() / "scenarios" / "intersection.json";
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        if (argument == "--help" || argument == "-h") { o.help = true; continue; }
        if (i + 1 >= argc) throw std::invalid_argument("Missing value for " + argument);
        const std::string value = argv[++i];
        if (argument == "--scenario") o.scenario = value;
        else if (argument == "--map") o.map = value;
        else if (argument == "--dynamics-profile") o.dynamics_profile = value;
        else if (argument == "--json") o.json = value;
        else if (argument == "--envs") o.envs = list(value, argument, 100000);
        else if (argument == "--threads") o.threads = list(value, argument, 1024);
        else if (argument == "--vehicles") o.vehicles = static_cast<std::size_t>(integer(value, argument, 1, 1000));
        else if (argument == "--steps") o.steps = integer(value, argument, 1, 10000000);
        else if (argument == "--seed") o.seed = static_cast<std::uint32_t>(integer(value, argument, 0, UINT32_MAX));
        else throw std::invalid_argument("Unknown option: " + argument);
    }
    if (o.threads.empty()) o.threads = hardwareThreads() > 1 ? std::vector<std::uint64_t>{1, hardwareThreads()} : std::vector<std::uint64_t>{1};
    return o;
}
void printHelp() {
    std::cout << "Vehicle Simulator v" VSIM_VERSION " batch benchmark\nUsage: sim_bench [options]\n"
        << "  --scenario FILE          JSON scenario (default: bundled intersection)\n"
        << "  --map FILE               OSM map instead of a scenario (parsed once, shared by all environments)\n"
        << "  --vehicles N             Vehicles per environment\n"
        << "  --dynamics-profile FILE  Apply a vehicle dynamics/actuator profile\n"
        << "  --envs LIST              Environment counts to measure (default: 1,8,64,256)\n"
        << "  --threads LIST           Thread counts, or 'all' (default: 1,all)\n"
        << "  --steps N                Ticks per environment (default: 500)\n"
        << "  --seed N                 Seed of environment 0; environment i uses seed+i (default: 1)\n"
        << "  --json FILE              Write the results as JSON\n";
}
std::vector<vsim::Scenario> environments(const BenchOptions& o, std::size_t count,
    const std::optional<vsim::RoadNetwork>& road, const std::optional<vsim::VehicleConfig>& config) {
    std::vector<vsim::Scenario> result;
    result.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        const vsim::ScenarioOverrides overrides{static_cast<std::uint32_t>(o.seed + i), o.vehicles};
        auto scenario = road ? vsim::osmScenario(*road, overrides) : vsim::loadScenario(o.scenario, overrides);
        if (config) for (auto& vehicle : scenario.vehicles) {
            vehicle.config = *config; vehicle.desired_speed = std::min(vehicle.desired_speed, config->max_speed);
        }
        result.push_back(std::move(scenario));
    }
    return result;
}
std::string rate(double value) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(value >= 1e7 ? 0 : 1);
    if (value >= 1e6) stream << value / 1e6 << " M"; else if (value >= 1e3) stream << value / 1e3 << " k"; else stream << value << ' ';
    return stream.str();
}
}

int main(int argc, char** argv) {
    try {
        const auto o = parse(argc, argv);
        if (o.help) { printHelp(); return 0; }
        const std::optional<vsim::RoadNetwork> road = o.map.empty() ? std::nullopt : std::optional{vsim::loadOsmNetwork(o.map)};
        const std::optional<vsim::VehicleConfig> config = o.dynamics_profile.empty() ? std::nullopt
            : std::optional{vsim::loadVehicleProfile(o.dynamics_profile)};
        const auto sample = environments(o, 1, road, config).front();
        std::cout << "Vehicle Simulator v" VSIM_VERSION " batch benchmark\n"
            << (road ? "OSM " + o.map.filename().string() : sample.name) << " | " << sample.vehicles.size() << " vehicles/env | "
            << (config ? vsim::dynamicsModelName(config->dynamics_model) : "scenario dynamics") << " | " << o.steps << " steps of "
            << sample.time_step << " s | " << hardwareThreads() << " hardware threads\n\n"
            << std::setw(7) << "envs" << std::setw(9) << "threads" << std::setw(10) << "vehicles" << std::setw(10) << "wall s"
            << std::setw(18) << "vehicle-steps/s" << std::setw(14) << "env-steps/s" << std::setw(10) << "speedup"
            << "  state hash\n";
        std::vector<Result> results;
        bool deterministic = true;
        for (const auto count : o.envs) {
            const auto scenarios = environments(o, static_cast<std::size_t>(count), road, config);
            std::optional<double> single;
            std::vector<unsigned> measured;
            for (const auto requested : o.threads) {
                vsim::SimulationBatch batch(scenarios, static_cast<unsigned>(requested));
                if (std::find(measured.begin(), measured.end(), batch.threads()) != measured.end()) continue;
                measured.push_back(batch.threads());
                const auto start = std::chrono::steady_clock::now();
                batch.step(o.steps);
                Result r{batch.size(), batch.threads(), batch.vehicles(), 0,
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count(), batch.stateHash()};
                for (std::size_t i = 0; i < batch.size(); ++i) r.active += batch[i].world().activeCount();
                if (!results.empty() && results.back().envs == r.envs && results.back().hash != r.hash) deterministic = false;
                if (r.threads == 1) single = r.seconds;
                const double vehicle_steps = static_cast<double>(r.vehicles) * static_cast<double>(o.steps);
                std::cout << std::setw(7) << r.envs << std::setw(9) << r.threads << std::setw(10) << r.vehicles
                    << std::setw(10) << std::fixed << std::setprecision(3) << r.seconds
                    << std::setw(18) << rate(vehicle_steps / r.seconds) << std::setw(14) << rate(static_cast<double>(r.envs * o.steps) / r.seconds)
                    << std::setw(9) << std::setprecision(2) << (single ? *single / r.seconds : 1.0) << "x  " << vsim::hexString(r.hash) << '\n';
                results.push_back(r);
            }
        }
        vsim::SimulationBatch profiled(environments(o, static_cast<std::size_t>(o.envs.front()), road, config), 1);
        vsim::StepProfile profile;
        const auto start = std::chrono::steady_clock::now();
        profiled.step(o.steps, &profile);
        const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        const std::vector<std::pair<const char*, double>> phases{{"projection", profile.projection}, {"admission", profile.admission},
            {"control", profile.control}, {"integration", profile.integration}, {"transition", profile.transition},
            {"collision", profile.collision}, {"other", std::max(0.0, wall - profile.total())}};
        std::cout << "\nPhase breakdown (" << o.envs.front() << " env, 1 thread, " << std::setprecision(2)
            << wall / static_cast<double>(profile.steps) * 1e6 << " us per env-step):\n";
        for (const auto& [name, seconds] : phases)
            std::cout << "  " << std::left << std::setw(12) << name << std::right << std::setw(6) << std::setprecision(1)
                << 100.0 * seconds / wall << " %\n";
        std::cout << "\nDeterminism: " << (deterministic ? "identical batch state hash for every thread count" : "MISMATCH between thread counts") << '\n';
        if (!o.json.empty()) {
            nlohmann::json report{{"version", VSIM_VERSION}, {"scenario", road ? o.map.filename().string() : sample.name},
                {"vehicles_per_env", sample.vehicles.size()}, {"steps", o.steps}, {"time_step", sample.time_step},
                {"hardware_threads", hardwareThreads()}, {"deterministic", deterministic}, {"results", nlohmann::json::array()}};
            for (const auto& r : results)
                report["results"].push_back({{"envs", r.envs}, {"threads", r.threads}, {"vehicles", r.vehicles}, {"active", r.active},
                    {"seconds", r.seconds}, {"vehicle_steps_per_second", static_cast<double>(r.vehicles * o.steps) / r.seconds},
                    {"state_hash", vsim::hexString(r.hash)}});
            for (const auto& [name, seconds] : phases) report["profile_seconds"][name] = seconds;
            report["profile_seconds"]["wall"] = wall;
            if (!o.json.parent_path().empty()) std::filesystem::create_directories(o.json.parent_path());
            std::ofstream stream(o.json);
            if (!(stream << report.dump(2) << '\n')) throw std::runtime_error("Cannot write " + o.json.string());
        }
        return deterministic ? 0 : 2;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
}
