#include "vsim/Serialization.hpp"
#include "vsim/Dynamics.hpp"
#include <algorithm>
#include <cwctype>
#include <fstream>
#include <stdexcept>

namespace vsim {
void to_json(nlohmann::json& j, const DynamicsModel& model) { j=dynamicsModelName(model); }
void from_json(const nlohmann::json& j, DynamicsModel& model) {
    const auto value=j.get<std::string>();
    if (value=="kinematic") model=DynamicsModel::Kinematic;
    else if (value=="dynamic_bicycle") model=DynamicsModel::DynamicBicycle;
    else throw std::invalid_argument("Unknown dynamics_model: " + value);
}
void to_json(nlohmann::json& j, const Lane& l) {
    j={{"id",l.id},{"name",l.name},{"width",l.width},{"centerline",l.centerline},{"speed_limit",l.speed_limit},
        {"source_way",l.source_way},{"from_node",l.from_node},{"to_node",l.to_node},{"layer",l.layer},
        {"connector",l.connector},{"signal_group",l.signal_group},{"stop_sign",l.stop_sign},{"give_way",l.give_way},
        {"priority",l.priority},{"turn",l.turn},{"successors",l.successors},{"lateral_index",l.lateral_index},
        {"directional_count",l.directional_count},{"junction",l.junction ? nlohmann::json(*l.junction) : nlohmann::json(nullptr)}};
}
void from_json(const nlohmann::json& j, Lane& l) {
    l.id=j.at("id"); l.name=j.at("name"); l.width=j.at("width"); l.centerline=j.at("centerline").get<std::vector<Vec2>>();
    l.speed_limit=j.value("speed_limit",l.speed_limit); l.source_way=j.value("source_way",OsmId{});
    l.from_node=j.value("from_node",OsmId{}); l.to_node=j.value("to_node",OsmId{}); l.layer=j.value("layer",0);
    l.connector=j.value("connector",false); l.signal_group=j.value("signal_group",0);
    l.stop_sign=j.value("stop_sign",false); l.give_way=j.value("give_way",false); l.priority=j.value("priority",0);
    l.turn=j.value("turn",std::string{}); l.successors=j.value("successors",std::vector<LaneId>{});
    l.lateral_index=j.value("lateral_index",0U); l.directional_count=j.value("directional_count",1U);
    if (j.contains("junction") && !j.at("junction").is_null()) l.junction=j.at("junction").get<std::size_t>();
    else l.junction.reset();
}
VehicleConfig loadVehicleProfile(const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input) throw std::runtime_error("Cannot read vehicle profile: " + path.string());
    const auto j=nlohmann::json::parse(input);
    auto c=j.value("vehicle_config",j).get<VehicleConfig>(); validate(c); return c;
}
void createParentDirectories(const std::filesystem::path& path) {
    if (!path.parent_path().empty()) std::filesystem::create_directories(path.parent_path());
}
void validateOutputPaths(const std::vector<std::filesystem::path>& outputs, const std::vector<std::filesystem::path>& inputs) {
    const auto normalized=[](const auto& path) {
        auto result=std::filesystem::weakly_canonical(std::filesystem::absolute(path)).wstring();
#ifdef _WIN32
        std::transform(result.begin(),result.end(),result.begin(),[](wchar_t c) { return static_cast<wchar_t>(std::towlower(c)); });
#endif
        return result;
    };
    const auto same=[&](const auto& a,const auto& b) {
        std::error_code error;
        return normalized(a)==normalized(b) || std::filesystem::equivalent(a,b,error);
    };
    for (std::size_t i=0; i<outputs.size(); ++i) {
        if (outputs[i].empty()) continue;
        for (const auto& input : inputs) if (!input.empty() && same(outputs[i],input))
            throw std::invalid_argument("Output would overwrite an input file: " + outputs[i].string());
        for (std::size_t j=0; j<i; ++j) if (!outputs[j].empty() && same(outputs[i],outputs[j]))
            throw std::invalid_argument("Output files must use different paths: " + outputs[i].string());
    }
}
}
