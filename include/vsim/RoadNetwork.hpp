#pragma once
#include "vsim/Types.hpp"
#include <string>
#include <vector>
#include <optional>

namespace vsim {
struct LaneSample { Vec2 position; double yaw{}; };
struct LaneProjection { double progress{}; double lateral{}; double yaw{}; };

struct Lane {
    LaneId id{};
    std::string name;
    double width{3.5};
    std::vector<Vec2> centerline;
    double speed_limit{13.8888888889};
    OsmId source_way{};
    OsmId from_node{};
    OsmId to_node{};
    int layer{};
    bool connector{};
    std::optional<std::size_t> junction;
    int signal_group{};
    bool stop_sign{};
    bool give_way{};
    int priority{};
    std::string turn;
    std::vector<LaneId> successors;
    unsigned lateral_index{};
    unsigned directional_count{1};
    double length() const;
    LaneSample sample(double progress) const;
    LaneProjection project(Vec2 point) const;
};

struct Junction {
    OsmId id{};
    Vec2 position;
    double radius{5.0};
    bool traffic_lights{};
    std::vector<LaneId> incoming;
    std::vector<LaneId> outgoing;
    std::vector<LaneId> connectors;
};
struct MapBounds { Vec2 minimum; Vec2 maximum; };
struct ImportInfo {
    std::size_t nodes{}, ways{}, road_segments{}, road_lanes{}, connectors{}, skipped_ways{};
    double origin_latitude{}, origin_longitude{};
    std::vector<std::string> warnings;
};

struct RoadNetwork {
    double arm_length{240.0};
    double lane_width{3.5};
    std::vector<Lane> lanes;
    bool imported{};
    std::vector<Junction> junctions;
    ImportInfo import_info;
    static RoadNetwork intersection(double arm_length = 240.0, double lane_width = 3.5);
    const Lane& lane(LaneId id) const;
    const Lane& lane(const std::string& name) const;
    double stopLine() const { return arm_length - lane_width - 1.0; }
    MapBounds bounds() const;
};

struct TrafficConfig {
    bool enabled{true};
    double green_seconds{12.0};
    double clearance_seconds{2.0};
    double minimum_gap{2.0};
    double time_headway{1.3};
};
void validate(const TrafficConfig& config);
bool greenFor(LaneId lane, double time, const TrafficConfig& config);
bool greenFor(const RoadNetwork& road, LaneId lane, double time, const TrafficConfig& config);
}
