#pragma once
#include "vsim/Scenario.hpp"
#include <map>

namespace vsim {
using OsmTags = std::map<std::string, std::string>;
struct OsmNode { OsmId id{}; double latitude{}, longitude{}; OsmTags tags; };
struct OsmWay { OsmId id{}; std::vector<OsmId> nodes; OsmTags tags; };
struct OsmMember { std::string type, role; OsmId reference{}; };
struct OsmRelation { OsmId id{}; std::vector<OsmMember> members; OsmTags tags; };
struct OsmData {
    std::map<OsmId, OsmNode> nodes;
    std::map<OsmId, OsmWay> ways;
    std::map<OsmId, OsmRelation> relations;
};
class MapProjection {
public:
    MapProjection(double latitude, double longitude);
    Vec2 project(double latitude, double longitude) const;
private:
    double latitude_, longitude_;
    double origin_x_, origin_y_, origin_z_;
};
OsmData readOsm(const std::filesystem::path& path);
RoadNetwork buildOsmNetwork(const OsmData& data);
RoadNetwork loadOsmNetwork(const std::filesystem::path& path);
Scenario loadOsmScenario(const std::filesystem::path& path,
    const ScenarioOverrides& overrides = {}, bool map_only = false);
Scenario osmScenario(RoadNetwork road, const ScenarioOverrides& overrides = {}, bool map_only = false);
}
