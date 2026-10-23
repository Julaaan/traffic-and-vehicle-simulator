#include "Options.hpp"
#include "vsim/OsmImporter.hpp"
#include "vsim/Serialization.hpp"
#include "vsim/Dynamics.hpp"
#include <algorithm>
#include <charconv>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace vsim::app {
namespace {
std::uint64_t integer(const std::string& value, const std::string& option) {
    std::uint64_t result{};
    const auto [end, error] = std::from_chars(value.data(), value.data() + value.size(), result);
    if (error != std::errc{} || end != value.data() + value.size())
        throw std::invalid_argument(option + " requires a nonnegative integer");
    return result;
}
double real(const std::string& value, const std::string& option) {
    std::size_t used{};
    const double result = std::stod(value, &used);
    if (used != value.size() || !std::isfinite(result) || result <= 0.0)
        throw std::invalid_argument(option + " requires a finite positive number");
    return result;
}
}
Options parseOptions(int argc, char** argv) {
    Options o;
    o.scenario = std::filesystem::absolute(argv[0]).parent_path() / "scenarios" / "intersection.json";
    bool explicit_scenario = false;
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        if (argument == "--help" || argument == "-h") { o.help = true; continue; }
        if (argument == "--auto-exit") { o.auto_exit = true; continue; }
        if (argument == "--map-only") { o.map_only = true; continue; }
        if (argument == "--verify-replay") { o.verify_replay = true; continue; }
        if (i + 1 >= argc) throw std::invalid_argument("Missing value for " + argument);
        const std::string value = argv[++i];
        if (argument == "--scenario") { o.scenario = value; explicit_scenario = true; }
        else if (argument == "--map") o.map = value;
        else if (argument == "--duration") o.duration = real(value, argument);
        else if (argument == "--steps") o.steps = integer(value, argument);
        else if (argument == "--seed") {
            const auto seed = integer(value, argument);
            if (seed > UINT32_MAX) throw std::invalid_argument("Seed exceeds 32 bits");
            o.overrides.seed = static_cast<std::uint32_t>(seed);
        } else if (argument == "--vehicles") {
            const auto count = integer(value, argument);
            if (count < 1 || count > 1000) throw std::invalid_argument("Vehicle count must be in [1,1000]");
            o.overrides.vehicle_count = static_cast<std::size_t>(count);
        } else if (argument == "--csv") o.csv = value;
        else if (argument == "--report") o.report = value;
        else if (argument == "--screenshot") o.screenshot = value;
        else if (argument == "--record") o.record = value;
        else if (argument == "--replay") o.replay = value;
        else if (argument == "--failure-dir") o.failure_dir = value;
        else if (argument == "--dynamics-profile") o.dynamics_profile = value;
        else if (argument == "--seek") o.seek = integer(value,argument);
        else if (argument == "--sample-every") {
            o.sample_every = integer(value, argument);
            if (o.sample_every == 0) throw std::invalid_argument("Sample interval must be positive");
        } else throw std::invalid_argument("Unknown option: " + argument);
    }
    if (o.duration && o.steps) throw std::invalid_argument("Use either --duration or --steps");
    if (!o.map.empty() && explicit_scenario) throw std::invalid_argument("Use either --map or --scenario");
    if (o.map_only && o.map.empty()) throw std::invalid_argument("--map-only requires --map");
    if (o.map_only && o.overrides.vehicle_count) throw std::invalid_argument("--map-only cannot be combined with --vehicles");
    if ((o.seek || o.verify_replay) && o.replay.empty()) throw std::invalid_argument("--seek/--verify-replay require --replay");
    if (!o.replay.empty() && (explicit_scenario || !o.map.empty() || o.map_only || o.overrides.seed || o.overrides.vehicle_count
        || !o.dynamics_profile.empty() || !o.record.empty() || !o.failure_dir.empty()))
        throw std::invalid_argument("Replay contains its own map, fleet and dynamics; do not combine it with input/recording overrides");
    if (o.verify_replay && (o.steps || o.duration || o.seek)) throw std::invalid_argument("--verify-replay verifies the complete recording");
    validateOutputPaths({o.csv,o.report,o.screenshot,o.record},
        {o.replay.empty() ? (o.map.empty() ? o.scenario : o.map) : o.replay,o.dynamics_profile});
    return o;
}
Scenario loadInput(const Options& o) {
    auto scenario = o.map.empty() ? loadScenario(o.scenario,o.overrides) : loadOsmScenario(o.map, o.overrides, o.map_only);
    if (!o.dynamics_profile.empty()) {
        const auto config=loadVehicleProfile(o.dynamics_profile);
        applyVehicleProfile(scenario,config);
        World checked(scenario);
        if (!checked.collisions().empty()) throw std::runtime_error("Vehicle profile creates overlapping initial footprints");
        std::cout<<"Dynamics: "<<dynamicsModelName(config.dynamics_model)<<" | friction "<<config.friction_coefficient<<'\n';
    }
    if (o.map.empty()) return scenario;
    const auto& info = scenario.road.import_info;
    std::cout << "OSM imported: " << info.road_segments << " road segments, " << info.road_lanes
        << " lanes, " << info.connectors << " connections\n";
    const std::size_t shown = std::min<std::size_t>(10, info.warnings.size());
    for (std::size_t i = 0; i < shown; ++i) std::cout << "Import note: " << info.warnings[i] << '\n';
    if (shown < info.warnings.size()) std::cout << info.warnings.size() - shown << " more import notes; use --report for the full list.\n";
    return scenario;
}
std::uint64_t runSteps(const Options& o, const Scenario& s) {
    if (o.steps) {
        if (*o.steps > 1000000000ULL) throw std::invalid_argument("At most 1 billion steps are supported per run");
        return *o.steps;
    }
    const double ticks = o.duration.value_or(s.duration) / s.time_step;
    if (!std::isfinite(ticks) || ticks > 1e9)
        throw std::invalid_argument("Duration exceeds 1 billion steps");
    const double rounded = std::round(ticks);
    if (std::abs(ticks - rounded) > 1e-6)
        throw std::invalid_argument("Duration must be a whole multiple of time_step; use --steps for an exact run");
    return static_cast<std::uint64_t>(rounded);
}
void printHelp(bool viewer) {
    std::cout << "Vehicle Simulator v" VSIM_VERSION "\nUsage: " << (viewer ? "sim_viewer" : "sim_cli") << " [options]\n"
        << "  --map FILE            Load an OSM XML file directly and generate the road network\n"
        << "  --map-only            Display/import the map without spawning vehicles\n"
        << "  --scenario FILE       JSON scenario (default: bundled intersection)\n"
        << "  --vehicles N          Override generated fleet count, 1..1000 (map must fit)\n"
        << "  --seed N              Override deterministic fleet seed\n"
        << "  --duration SECONDS    Simulated duration, multiple of time_step\n"
        << "  --steps N             Exact number of fixed simulation steps\n"
        << "  --csv FILE            Write sampled vehicle states\n"
        << "  --sample-every N      CSV sampling interval in steps (default: 10)\n"
        << "  --report FILE         Write JSON run summary and final states\n"
        << "  --dynamics-profile FILE  Apply a vehicle dynamics/actuator JSON profile\n"
        << "  --record FILE         Record a self-contained replay with checkpoints\n"
        << "  --failure-dir DIR     Save up to 500 preceding ticks and diagnose collisions/lane departures\n"
        << "  --replay FILE         Play recorded controls and check every state hash\n"
        << "  --seek TICK           Start playback at an absolute recorded tick\n"
        << "  --verify-replay       Verify the entire replay without graphics (CLI)\n"
        << "  --help                Show this help\n";
    if (viewer) std::cout
        << "  --auto-exit           Close after the requested simulation duration\n"
        << "  --screenshot FILE     Save last displayed frame as BMP on exit\n"
        << "Controls: Space pause, N single step, R reset, +/- speed, wheel zoom,\n"
        << "          right-drag pan, F fit map, C focus intersection, B boxes, T routes, H help, Esc quit.\n"
        << "Replay controls: Left/Right seek one simulated second, Home/End start/end.\n";
}
}
