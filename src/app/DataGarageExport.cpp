#include "DataGarageExport.hpp"
#include "vsim/Serialization.hpp"
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <format>
#include <limits>
#include <stdexcept>

namespace vsim::app {
namespace {
using Json = nlohmann::ordered_json;

std::string utcNow() {
    return std::format("{:%Y-%m-%dT%H:%M:%SZ}", std::chrono::floor<std::chrono::seconds>(std::chrono::system_clock::now()));
}
std::uint64_t hashText(std::uint64_t hash, std::string_view text) {
    for (const unsigned char c : text) { hash ^= c; hash *= 1099511628211ULL; }
    return hash;
}
std::string slug(const std::string& name) {
    std::string result;
    for (const unsigned char c : name) {
        if (result.size() == 40) break;
        if (std::isalnum(c)) result += static_cast<char>(std::tolower(c));
        else if (!result.empty() && result.back() != '-') result += '-';
    }
    while (!result.empty() && result.back() == '-') result.pop_back();
    return result.empty() ? "run" : result;
}
}

std::optional<V2xNetwork> v2xNetwork(const Options& o) {
    if (o.delivery != "network") return std::nullopt;
    V2xNetwork network;
    network.latency_ms = o.v2x_latency_ms.value_or(network.latency_ms);
    network.jitter_ms = o.v2x_jitter_ms.value_or(network.jitter_ms);
    network.loss = o.v2x_loss.value_or(network.loss);
    return network;
}

void validateRunId(const std::string& id) {
    const bool valid = !id.empty() && id.size() <= 64 && std::all_of(id.begin(), id.end(), [](unsigned char c) {
        return std::isalnum(c) || c == '_' || c == '-';
    });
    if (!valid) throw std::invalid_argument("Run id must be 1..64 characters from [A-Za-z0-9_-]: " + id);
}

std::string defaultRunId(const Options& o, const Simulation& sim, std::uint64_t steps) {
    std::uint64_t hash = hashText(hash_seed, VSIM_VERSION);
    hash = hashText(hash, nlohmann::json(sim.scenario()).dump());
    hash = hashAppend(hash, steps);
    hash = hashText(hash, o.delivery);
    if (const auto network = v2xNetwork(o))
        for (double value : {network->latency_ms, network->jitter_ms, network->loss}) hash = hashText(hash, std::to_string(value));
    return slug(sim.scenario().name) + "-s" + std::to_string(sim.scenario().seed) + "-" + hexString(hash).substr(0, 8);
}

DataGarageExport::DataGarageExport(const Options& options, const Simulation& sim, std::uint64_t steps)
    : network_(v2xNetwork(options)), random_(hashAppend(hash_seed, sim.scenario().seed)) {
    const double step_us = sim.clock().timeStep() * 1e6;
    if (std::abs(step_us - std::round(step_us)) > 1e-6)
        throw std::invalid_argument("--datagarage needs a time_step that is a whole number of microseconds");
    step_us_ = static_cast<std::uint64_t>(std::llround(step_us));
    const auto run_id = options.run_id.empty() ? defaultRunId(options, sim, steps) : options.run_id;
    validateRunId(run_id);
    std::filesystem::create_directories(options.datagarage);
    directory_ = options.datagarage / run_id;
    if (!std::filesystem::create_directory(directory_))
        throw std::invalid_argument("DataGarage run already exists: " + directory_.string() + " (delete it or use --run-id)");
    events_.open(directory_ / "events.jsonl", std::ios::binary);
    if (!events_) throw std::runtime_error("Cannot write DataGarage events in " + directory_.string());

    const auto& s = sim.scenario();
    Json params{{"steps", steps}, {"vehicles", s.vehicles.size()},
        {"road", s.road.imported ? "osm" : "four_way_intersection"},
        {"input", (options.map.empty() ? options.scenario : options.map).filename().string()},
        {"traffic", Json(nlohmann::json(s.traffic))},
        {"wait", {{"start_below_mps", TrafficEventDetector::wait_start_speed},
                  {"end_above_mps", TrafficEventDetector::wait_end_speed}}}};
    if (!options.dynamics_profile.empty()) params["dynamics_profile"] = options.dynamics_profile.filename().string();
    if (network_) params["v2x"] = {{"latency_ms", network_->latency_ms}, {"jitter_ms", network_->jitter_ms},
                                   {"loss", network_->loss}, {"messages", "one per traffic event"}};
    manifest_ = {{"contract", "dg-events/1"}, {"run_id", run_id}, {"simulator", "traffic-and-vehicle-simulator"},
        {"simulator_version", VSIM_VERSION}, {"scenario", s.name}, {"seed", s.seed},
        {"time_step_ms", static_cast<double>(step_us_) / 1000.0}, {"delivery", network_ ? "network" : "in_order"},
        {"params", params}, {"status", "running"}, {"started_at", utcNow()}};
    writeManifest();
    capture(sim);
}

DataGarageExport::~DataGarageExport() {
    if (!closed_) { try { failure("Simulation stopped before the export finished"); } catch (...) {} }
}

std::int64_t DataGarageExport::milliseconds(std::uint64_t tick) const {
    return static_cast<std::int64_t>(tick * step_us_ / 1000);
}

double DataGarageExport::uniform() { return static_cast<double>(random_() >> 11) * 0x1.0p-53; }

void DataGarageExport::capture(const Simulation& sim) {
    if (closed_) throw std::logic_error("DataGarage export is already closed");
    pending_.clear();
    detector_.observe(sim, pending_);
    for (const auto& event : pending_) accept(event);
    deliver(milliseconds(sim.clock().tick()));
    events_.flush();
    if (!events_) throw std::runtime_error("Failed writing DataGarage events");
}

void DataGarageExport::accept(const TrafficEvent& e) {
    const auto id = next_event_++;
    const auto ms = milliseconds(e.tick);
    if (!network_) { writeEvent(id, ms, eventTypeName(e.type), e.vehicle, e.intersection, e.interval); return; }
    ++sent_;
    if (uniform() < network_->loss) { ++lost_; return; }
    const double delay = network_->latency_ms - network_->jitter_ms * std::log1p(-uniform());
    Message message{e, id, next_event_++, next_event_++, detector_.newInterval(), ms, ms + std::llround(delay)};
    const auto key = std::pair{message.received_ms, message.sent_id};
    in_flight_.emplace(key, std::move(message));
}

void DataGarageExport::deliver(std::int64_t until_ms) {
    while (!in_flight_.empty() && in_flight_.begin()->first.first <= until_ms) {
        const auto m = std::move(in_flight_.begin()->second);
        in_flight_.erase(in_flight_.begin());
        writeEvent(m.sent_id, m.sent_ms, "msg_sent", m.event.vehicle, m.event.intersection, m.interval);
        writeEvent(m.received_id, m.received_ms, "msg_received", m.event.vehicle, m.event.intersection, m.interval);
        writeEvent(m.event_id, m.sent_ms, eventTypeName(m.event.type), m.event.vehicle, m.event.intersection, m.event.interval);
    }
}

void DataGarageExport::writeEvent(std::uint64_t id, std::int64_t ms, std::string_view type, EntityId vehicle,
                                  const std::string& intersection, std::uint64_t interval) {
    const Json line{{"event_id", id}, {"sim_time_ms", ms}, {"event_type", std::string(type)},
        {"vehicle_id", "v" + std::to_string(vehicle)}, {"intersection_id", intersection}, {"interval_id", interval}};
    events_ << line.dump() << '\n';
    ++written_;
}

void DataGarageExport::writeManifest() {
    const auto temporary = directory_ / "run.json.tmp";
    {
        std::ofstream output(temporary, std::ios::binary);
        output << manifest_.dump(2) << '\n';
        output.flush();
        if (!output) throw std::runtime_error("Failed writing DataGarage manifest in " + directory_.string());
    }
    std::filesystem::rename(temporary, directory_ / "run.json");
}

void DataGarageExport::finish(const Simulation& sim) {
    if (closed_) throw std::logic_error("DataGarage export is already closed");
    deliver(std::numeric_limits<std::int64_t>::max());
    events_.close();
    if (!events_) throw std::runtime_error("Failed writing DataGarage events");
    closed_ = true;
    manifest_["status"] = "completed";
    manifest_["ended_at"] = utcNow();
    manifest_["end_sim_time_ms"] = milliseconds(sim.clock().tick());
    manifest_["event_count"] = written_;
    manifest_["state_hash"] = hexString(sim.stateHash());
    if (network_) { manifest_["params"]["v2x"]["sent"] = sent_; manifest_["params"]["v2x"]["lost"] = lost_; }
    writeManifest();
}

void DataGarageExport::failure(const std::string& message) {
    if (closed_) return;
    closed_ = true;
    events_.close();
    manifest_["status"] = "failed";
    manifest_["ended_at"] = utcNow();
    manifest_["event_count"] = written_;
    manifest_["error"] = message;
    writeManifest();
}
}
