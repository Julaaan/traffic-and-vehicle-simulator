#include "vsim/DynamicsValidation.hpp"
#include "vsim/Dynamics.hpp"
#include <algorithm>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>

namespace vsim {
namespace {
std::vector<std::string> split(const std::string& line) {
    std::vector<std::string> fields; std::stringstream input(line); std::string field;
    while (std::getline(input,field,',')) { if (!field.empty() && field.back()=='\r') field.pop_back(); fields.push_back(field); }
    return fields;
}
struct Error { double squared{}, maximum{}; std::size_t count{}; void add(double e) { squared+=e*e; maximum=std::max(maximum,std::abs(e)); ++count; } };
}
DynamicsValidationResult validateDynamicsReference(const std::filesystem::path& manifest) {
    std::ifstream file(manifest); if (!file) throw std::runtime_error("Cannot open validation manifest: " + manifest.string());
    const auto j=nlohmann::json::parse(file);
    if (j.at("units")!="SI" || j.at("pose_reference")!="rear_axle") throw std::runtime_error("Reference requires SI units and rear_axle pose");
    auto config=j.at("vehicle_config").get<VehicleConfig>(); validate(config);
    const double step=j.value("time_step",0.01); SimulationClock valid_step(step);
    DynamicsValidationResult result; result.name=j.at("name"); result.reference_kind=j.at("reference_kind"); result.reference_source=j.at("reference_source");
    if (result.reference_source.empty() || (result.reference_kind!="analytical" && result.reference_kind!="measured" && result.reference_kind!="external_simulator"))
        throw std::runtime_error("Reference must identify its provenance");
    result.model=dynamicsModelName(config.dynamics_model); result.vehicle_config=config; result.time_step=step;
    std::ifstream trace(manifest.parent_path()/j.at("trace").get<std::string>());
    if (!trace) throw std::runtime_error("Cannot open reference trace");
    std::string line; if (!std::getline(trace,line)) throw std::runtime_error("Empty reference trace");
    const auto names=split(line); std::map<std::string,std::size_t> columns;
    for (std::size_t i=0; i<names.size(); ++i) if (!columns.emplace(names[i],i).second) throw std::runtime_error("Duplicate reference column");
    for (const char* key : {"time","acceleration_command","steering_command","x","y","yaw","velocity"})
        if (!columns.contains(key)) throw std::runtime_error(std::string("Missing reference column: ")+key);
    std::vector<std::vector<double>> rows;
    while (std::getline(trace,line)) {
        if (line.empty() || line=="\r") continue;
        const auto fields=split(line); if (fields.size()!=names.size()) throw std::runtime_error("Reference column count differs");
        std::vector<double> row;
        for (const auto& text : fields) { std::size_t used{}; const double v=std::stod(text,&used); if (used!=text.size() || !std::isfinite(v)) throw std::runtime_error("Reference values must be finite numbers"); row.push_back(v); }
        const double time=row[columns.at("time")];
        if ((rows.empty() && time!=0) || (!rows.empty() && time<=rows.back()[columns.at("time")]) || time>1800 || rows.size()>=1000000)
            throw std::runtime_error("Reference timestamps must increase from zero and span at most 1800 s");
        rows.push_back(std::move(row));
    }
    if (rows.size()<2) throw std::runtime_error("Reference requires at least two samples");
    const auto value=[&](std::size_t row,const std::string& key) { return rows[row][columns.at(key)]; };
    VehicleState state;
    state.x=value(0,"x"); state.y=value(0,"y"); state.yaw=value(0,"yaw"); state.velocity=value(0,"velocity");
    for (const auto& [key,pointer] : std::vector<std::pair<std::string,double*>>{{"yaw_rate",&state.yaw_rate},{"lateral_velocity",&state.lateral_velocity},{"steering_angle",&state.steering_angle}})
        if (columns.contains(key)) *pointer=value(0,key);
    if (j.contains("initial_state")) state=j.at("initial_state").get<VehicleState>();
    validate(state,config);
    std::map<std::string,Error> errors;
    for (std::size_t i=0; i<rows.size(); ++i) {
        if (i) {
            double time=value(i-1,"time"); const double target=value(i,"time");
            const VehicleControl control{value(i-1,"acceleration_command"),value(i-1,"steering_command")};
            while (time<target-1e-12) { const double h=std::min(step,target-time); integrate(state,config,control,h); time+=h; }
        }
        errors["position"].add(std::hypot(state.x-value(i,"x"),state.y-value(i,"y")));
        errors["yaw"].add(wrapAngle(state.yaw-value(i,"yaw"))); errors["velocity"].add(state.velocity-value(i,"velocity"));
        for (const auto& [key,actual] : std::vector<std::pair<std::string,double>>{{"yaw_rate",state.yaw_rate},{"lateral_velocity",state.lateral_velocity},{"steering_angle",state.steering_angle}})
            if (columns.contains(key)) errors[key].add(actual-value(i,key));
    }
    const std::map<std::string,std::string> units{{"position","m"},{"yaw","rad"},{"velocity","m/s"},{"yaw_rate","rad/s"},{"lateral_velocity","m/s"},{"steering_angle","rad"}};
    const auto tolerances=j.at("tolerances"); result.passed=true;
    for (const auto& [name,error] : errors) {
        const auto limits=tolerances.at(name); const double rmse_limit=limits.at("rmse"), maximum_limit=limits.at("max");
        if (!std::isfinite(rmse_limit) || !std::isfinite(maximum_limit) || rmse_limit<0 || maximum_limit<0) throw std::runtime_error("Invalid reference tolerance");
        ValidationMetric metric{name,units.at(name),std::sqrt(error.squared/static_cast<double>(error.count)),error.maximum,rmse_limit,maximum_limit};
        metric.passed=metric.rmse<=rmse_limit && metric.maximum<=maximum_limit; result.passed=result.passed && metric.passed;
        result.metrics.push_back(metric);
    }
    result.samples=rows.size(); result.seconds=value(rows.size()-1,"time"); return result;
}
void writeDynamicsValidation(const std::filesystem::path& path, const DynamicsValidationResult& result) {
    if (path.empty()) return;
    createParentDirectories(path);
    std::ofstream out(path); out<<nlohmann::json(result).dump(2)<<'\n'; out.flush();
    if (!out) throw std::runtime_error("Cannot write dynamics validation report");
}
}
