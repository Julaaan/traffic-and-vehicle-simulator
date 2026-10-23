#include "vsim/Replay.hpp"
#include "vsim/Dynamics.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <stdexcept>

namespace vsim {
namespace {
using Json=nlohmann::json;
std::string platform() {
#ifdef _MSC_VER
    return "msvc-" + std::to_string(_MSC_FULL_VER);
#elif defined(__GNUC__)
    return "gcc-" + std::string(__VERSION__);
#else
    return "unknown";
#endif
}
Json end(const SimulationSnapshot& snapshot, const std::string& hash) {
    return {{"type","end"},{"tick",snapshot.tick},{"hash",hash},{"snapshot",snapshot}};
}
Json commands(const Simulation& sim) {
    auto result=Json::array();
    for (const auto& v : sim.world().vehicles()) result.push_back({v.id,v.automatic,v.control.acceleration,v.control.steering_angle});
    return result;
}
std::vector<RecordedControl> decodeCommands(const Json& data) {
    if (!data.is_array() || data.size()>1000) throw std::runtime_error("Invalid replay command list");
    std::vector<RecordedControl> result;
    for (const auto& c : data) {
        if (!c.is_array() || c.size()!=4 || !c[0].is_number_unsigned() || c[0].get<std::uint64_t>()>UINT32_MAX || !c[1].is_boolean())
            throw std::runtime_error("Invalid replay control");
        RecordedControl command{c[0].get<EntityId>(),c[1].get<bool>(),{c[2].get<double>(),c[3].get<double>()}};
        validate(command.control); result.push_back(command);
    }
    return result;
}
Json readLine(std::ifstream& input) {
    std::string line;
    if (!std::getline(input,line)) throw std::runtime_error("Truncated replay");
    if (line.size()>64*1024*1024) throw std::runtime_error("Replay record exceeds 64 MiB");
    return Json::parse(line);
}
}
RunRecorder::RunRecorder(const Simulation& sim, const std::filesystem::path& file, const std::filesystem::path& failure_dir)
    : scenario_(sim.scenario()),base_(sim.snapshot()),last_(base_),base_hash_(hexString(sim.stateHash())),last_hash_(base_hash_),failure_dir_(failure_dir) {
    if (!file.empty()) {
        createParentDirectories(file); output_.open(file, std::ios::binary);
        if (!output_) throw std::runtime_error("Cannot write replay: " + file.string());
        write(header(base_,base_hash_));
    }
}
RunRecorder::~RunRecorder() {
    if (!finished_ && output_.is_open()) { try { write(end(last_,last_hash_)); output_.flush(); } catch (...) {} }
}
Json RunRecorder::header(const SimulationSnapshot& start, const std::string& hash) const {
    return {{"type","header"},{"format","vsim-replay"},{"format_version",1},{"engine_version",VSIM_VERSION},
        {"platform",platform()},{"scenario",scenario_},{"start_state",start},{"start_hash",hash}};
}
void RunRecorder::write(const Json& data) {
    if (!output_.is_open()) return;
    output_<<data.dump()<<'\n'; if (!output_) throw std::runtime_error("Failed writing replay");
}
void RunRecorder::capture(const Simulation& sim) {
    if (!output_.is_open() && failure_dir_.empty()) return;
    if (finished_ || sim.clock().tick()!=last_.tick+1) throw std::runtime_error("Recording requires consecutive simulation ticks");
    std::set<std::string> conditions; std::vector<DiagnosticEvent> events;
    const auto emit=[&](std::string key,std::string type,std::string severity,std::string message,std::vector<EntityId> ids) {
        conditions.insert(key);
        if (!previous_conditions_.contains(key)) events.push_back({sim.clock().tick(),std::move(type),std::move(severity),std::move(message),std::move(ids)});
    };
    for (const auto& p : sim.world().collisions())
        emit("collision/"+std::to_string(p.first)+"/"+std::to_string(p.second),"collision","error","Vehicle footprints intersect",{p.first,p.second});
    for (const auto& v : sim.world().vehicles()) {
        if (!v.active) continue;
        const auto& lane=sim.world().road().lane(v.lane);
        if (std::abs(lane.project(v.state.position()).lateral)>lane.width/2+0.25)
            emit("departure/"+std::to_string(v.id),"lane_departure","error","Reference point left the lane corridor",{v.id});
        if (v.state.grip_limited)
            emit("grip/"+std::to_string(v.id),"tire_limit","warning","Tire force or kinematic curvature reached the friction limit",{v.id});
        if (std::abs(v.control.steering_angle)>v.config.max_steering || v.control.acceleration>v.config.max_acceleration || v.control.acceleration < -v.config.max_deceleration)
            emit("saturation/"+std::to_string(v.id),"control_saturation","warning","Requested command exceeded actuator limits",{v.id});
    }
    previous_conditions_=std::move(conditions);
    Frame frame{{{"type","frame"},{"tick",sim.clock().tick()},{"hash",hexString(sim.stateHash())},
        {"commands",commands(sim)},{"events",events}},sim.snapshot()};
    if (sim.clock().tick()%100==0) frame.data["snapshot"]=frame.snapshot;
    write(frame.data); last_=frame.snapshot; last_hash_=frame.data.at("hash");
    if (!failure_dir_.empty()) {
        ring_.push_back(std::move(frame));
        if (ring_.size()>500) { base_=std::move(ring_.front().snapshot); base_hash_=ring_.front().data.at("hash"); ring_.pop_front(); }
        if (std::any_of(events.begin(),events.end(),[](const auto& e) { return e.severity=="error"; })) saveFailure(events);
    }
}
void RunRecorder::saveFailure(const std::vector<DiagnosticEvent>& events) {
    if (failure_dir_.empty() || saved_failures_>=16) return;
    std::filesystem::create_directories(failure_dir_);
    const auto nonce=std::chrono::steady_clock::now().time_since_epoch().count();
    const auto stem="failure-"+std::to_string(last_.tick)+"-"+std::to_string(nonce);
    const auto path=failure_dir_/(stem+".replay.jsonl");
    std::ofstream file(path, std::ios::binary); if (!file) throw std::runtime_error("Cannot write failure replay");
    file<<header(base_,base_hash_).dump()<<'\n';
    for (const auto& frame : ring_) file<<frame.data.dump()<<'\n';
    file<<end(last_,last_hash_).dump()<<'\n'; file.flush();
    if (!file) throw std::runtime_error("Failed writing failure replay");
    Json states=Json::array();
    for (const auto& event : events) for (auto id : event.vehicles) {
        const auto found=std::find_if(last_.vehicles.begin(),last_.vehicles.end(),[&](const auto& v) { return v.id==id; });
        if (found!=last_.vehicles.end()) states.push_back(*found);
    }
    std::ofstream report(failure_dir_/(stem+".diagnostic.json"));
    report<<Json{{"tick",last_.tick},{"replay",path.filename().string()},{"events",events},{"vehicles",states},
        {"state_hash",last_hash_},{"seed",scenario_.seed}}.dump(2)<<'\n'; report.flush();
    if (!report) throw std::runtime_error("Failed writing failure diagnosis");
    ++saved_failures_; std::cout<<"Failure replay saved: "<<path.string()<<'\n';
}
void RunRecorder::failure(const std::string& message) {
    saveFailure({{last_.tick,"execution_error","error",message,{}}});
}
void RunRecorder::finish(const Simulation& sim) {
    if (finished_) return;
    if (output_.is_open()) {
        if (sim.clock().tick()!=last_.tick) throw std::runtime_error("Unrecorded simulation steps");
        write(end(last_,last_hash_)); output_.flush();
        if (!output_) throw std::runtime_error("Failed flushing replay");
    }
    finished_=true;
}
ReplayPlayer::ReplayPlayer(const std::filesystem::path& path) : input_(path, std::ios::binary) {
    if (!input_) throw std::runtime_error("Cannot read replay: " + path.string());
    const auto h=readLine(input_);
    if (h.at("type")!="header" || h.at("format")!="vsim-replay" || h.at("format_version")!=1)
        throw std::runtime_error("Unsupported replay format");
    if (h.at("engine_version")!=VSIM_VERSION || h.at("platform")!=platform())
        throw std::runtime_error("Replay engine/platform differs; exact replay requires the recorded engine build");
    simulation_=std::make_unique<Simulation>(h.at("scenario").get<Scenario>());
    start_=h.at("start_state").get<SimulationSnapshot>();
    simulation_->restore(start_);
    if (hexString(simulation_->stateHash())!=h.at("start_hash").get<std::string>()) throw std::runtime_error("Replay start snapshot hash differs");
    std::uint64_t tick=start_.tick; bool ended=false;
    while (input_.peek()!=std::char_traits<char>::eof()) {
        const auto offset=input_.tellg(); const auto j=readLine(input_); const auto type=j.at("type").get<std::string>();
        if (type=="frame") {
            if (j.at("tick").get<std::uint64_t>()!=tick+1 || index_.size()>=2000000) throw std::runtime_error("Replay ticks are not consecutive or replay is too long");
            decodeCommands(j.at("commands")); j.at("events").get<std::vector<DiagnosticEvent>>();
            index_.push_back({++tick,offset,j.contains("snapshot")});
        } else if (type=="end") {
            end_tick_=j.at("tick"); end_hash_=j.at("hash");
            if (end_tick_!=tick || j.at("snapshot").at("tick")!=tick) throw std::runtime_error("Invalid replay end tick");
            Simulation end_check(simulation_->scenario()); end_check.restore(j.at("snapshot").get<SimulationSnapshot>());
            if (hexString(end_check.stateHash())!=end_hash_) throw std::runtime_error("Replay end snapshot hash differs");
            ended=true;
            if (input_.peek()!=std::char_traits<char>::eof()) throw std::runtime_error("Unexpected data after replay footer");
            break;
        } else throw std::runtime_error("Unknown replay record type");
    }
    if (!ended) throw std::runtime_error("Truncated replay: missing footer");
}
Json ReplayPlayer::read(std::streampos offset) { input_.clear(); input_.seekg(offset); return readLine(input_); }
void ReplayPlayer::check(const Json& j) const {
    if (hexString(simulation_->stateHash())!=j.at("hash").get<std::string>())
        throw std::runtime_error("Replay diverged at tick " + std::to_string(simulation_->clock().tick()) + "; expected "
            + j.at("hash").get<std::string>() + ", actual " + hexString(simulation_->stateHash()));
}
bool ReplayPlayer::advance() {
    if (next_==index_.size()) return false;
    const auto frame=read(index_[next_].offset);
    const auto controls=decodeCommands(frame.at("commands")); simulation_->step(controls); check(frame);
    if (frame.contains("snapshot") && Json(frame.at("snapshot").get<SimulationSnapshot>())!=Json(simulation_->snapshot()))
        throw std::runtime_error("Replay checkpoint differs from computed state at tick " + std::to_string(simulation_->clock().tick()));
    ++next_;
    if (next_==index_.size() && hexString(simulation_->stateHash())!=end_hash_) throw std::runtime_error("Replay final hash differs");
    return true;
}
void ReplayPlayer::seek(std::uint64_t tick) {
    if (tick<firstTick() || tick>lastTick()) throw std::out_of_range("Seek tick is outside replay");
    next_=0; simulation_->restore(start_);
    for (std::size_t i=0; i<index_.size() && index_[i].tick<=tick; ++i) if (index_[i].checkpoint) next_=i+1;
    if (next_) {
        const auto frame=read(index_[next_-1].offset);
        simulation_->restore(frame.at("snapshot").get<SimulationSnapshot>()); check(frame);
    }
    while (simulation_->clock().tick()<tick) advance();
}
void ReplayPlayer::verify() {
    simulation_->restore(start_); next_=0; while (advance()) {}
    if (hexString(simulation_->stateHash())!=end_hash_) throw std::runtime_error("Replay final hash differs");
}
}
