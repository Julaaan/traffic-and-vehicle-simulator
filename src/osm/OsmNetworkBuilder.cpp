#include "vsim/OsmImporter.hpp"
#include "vsim/RoutePlanner.hpp"
#include "vsim/World.hpp"
#include <algorithm>
#include <charconv>
#include <random>
#include <set>
#include <stdexcept>

namespace vsim {
namespace {
std::string tag(const OsmTags& tags, const std::string& key, const std::string& fallback = {}) {
    const auto found = tags.find(key); return found == tags.end() ? fallback : found->second;
}
bool yes(const std::string& value) { return value == "yes" || value == "true" || value == "1"; }
std::vector<std::string> split(const std::string& text, char separator) {
    std::vector<std::string> result;
    std::size_t begin = 0;
    do {
        const auto end = text.find(separator, begin);
        result.push_back(text.substr(begin, end == std::string::npos ? end : end - begin));
        if (end == std::string::npos) break;
        begin = end + 1;
    } while (true);
    return result;
}
void warn(RoadNetwork& road, OsmId id, const std::string& message) {
    road.import_info.warnings.push_back("OSM " + std::to_string(id) + ": " + message);
}
std::optional<double> numeric(const std::string& text) {
    if (text.empty()) return {};
    std::size_t used{};
    try {
        const double value = std::stod(text, &used);
        const auto tail = text.substr(used);
        if (!std::isfinite(value) || value <= 0.0 || (!tail.empty() && tail != " m" && tail != "m" && tail != " km/h" && tail != " mph")) return {};
        return tail == " mph" ? value * 1.609344 : value;
    } catch (...) { return {}; }
}
unsigned countTag(const OsmTags& tags, const std::string& key, unsigned fallback, RoadNetwork& road, OsmId id) {
    const auto value = tag(tags, key);
    if (value.empty()) return fallback;
    unsigned parsed{};
    const auto [end, error] = std::from_chars(value.data(), value.data() + value.size(), parsed);
    if (error == std::errc{} && end == value.data() + value.size() && parsed <= 12) return parsed;
    warn(road, id, "unsupported " + key + "='" + value + "'; using " + std::to_string(fallback));
    return fallback;
}
bool roadAccess(const OsmWay& way) {
    static const std::set<std::string> classes{"motorway", "motorway_link", "trunk", "trunk_link", "primary", "primary_link",
        "secondary", "secondary_link", "tertiary", "tertiary_link", "unclassified", "residential", "living_street", "service", "road"};
    if (!classes.contains(tag(way.tags, "highway")) || yes(tag(way.tags, "area"))) return false;
    for (const char* key : {"motorcar", "motor_vehicle", "vehicle", "access"}) {
        const auto access = tag(way.tags, key);
        if (!access.empty()) return access != "no" && access != "private" && access != "agricultural" && access != "forestry";
    }
    return true;
}
int priority(const std::string& highway) {
    if (highway.find("motorway") == 0) return 6;
    if (highway.find("trunk") == 0) return 5;
    if (highway.find("primary") == 0) return 4;
    if (highway.find("secondary") == 0) return 3;
    if (highway.find("tertiary") == 0) return 2;
    return 1;
}
double defaultSpeed(const std::string& highway) {
    if (highway == "motorway") return 100.0 / 3.6;
    if (highway == "trunk") return 80.0 / 3.6;
    if (highway == "living_street") return 7.0 / 3.6;
    if (highway == "service") return 20.0 / 3.6;
    if (highway == "residential") return 30.0 / 3.6;
    return 50.0 / 3.6;
}
double speed(const OsmWay& way, bool forward, RoadNetwork& road) {
    auto value = tag(way.tags, forward ? "maxspeed:forward" : "maxspeed:backward", tag(way.tags, "maxspeed"));
    if (value == "walk") return 1.4;
    if (value == "DE:urban") return 50.0 / 3.6;
    if (value == "DE:rural") return 100.0 / 3.6;
    if (auto parsed = numeric(value); parsed && *parsed <= 300.0) return *parsed / 3.6;
    const double fallback = defaultSpeed(tag(way.tags, "highway"));
    if (!forward && !tag(way.tags, "maxspeed").empty() && tag(way.tags, "maxspeed:backward").empty()) return fallback;
    warn(road, way.id, "speed inferred as " + std::to_string(static_cast<int>(std::round(fallback * 3.6))) + " km/h" + (value.empty() ? " (missing maxspeed)" : " (unsupported maxspeed='" + value + "')"));
    return fallback;
}
std::vector<Vec2> offset(const std::vector<Vec2>& points, double distance) {
    std::vector<Vec2> result;
    for (std::size_t i = 0; i < points.size(); ++i) {
        const auto normal = [&](std::size_t a, std::size_t b) {
            const Vec2 d = points[b] - points[a]; const double length = norm(d);
            return Vec2{d.y / length, -d.x / length};
        };
        Vec2 n = i == 0 ? normal(0, 1) : normal(i - 1, i);
        if (i > 0 && i + 1 < points.size()) {
            const Vec2 after = normal(i, i + 1);
            const Vec2 sum = n + after;
            if (norm(sum) > 1e-6) {
                const Vec2 bisector = sum * (1.0 / norm(sum));
                n = bisector * (1.0 / std::max(0.5, dot(bisector, after)));
            }
        }
        result.push_back(points[i] + n * distance);
    }
    return result;
}
std::vector<Vec2> trim(const std::vector<Vec2>& points, double begin, double end) {
    Lane geometry; geometry.centerline = points;
    const double length = geometry.length();
    begin = std::min(begin, length * 0.3); end = std::min(end, length * 0.3);
    std::vector<Vec2> result{geometry.sample(begin).position};
    double progress = 0.0;
    for (std::size_t i = 1; i + 1 < points.size(); ++i) {
        progress += norm(points[i] - points[i - 1]);
        if (progress > begin + 1e-6 && progress < length - end - 1e-6) result.push_back(points[i]);
    }
    result.push_back(geometry.sample(length - end).position);
    return result;
}
struct Segment { const OsmWay* way{}; OsmId from{}, to{}; std::vector<Vec2> geometry; };
struct Restriction { OsmId from{}, to{}, via{}; std::string value; };
bool turnPermitted(const std::string& markings, const std::string& turn) {
    if (markings.empty() || markings == "none") return true;
    for (const auto& value : split(markings, ';')) {
        if (value == turn || (turn == "left" && (value == "slight_left" || value == "sharp_left" || value == "merge_to_left"))
            || (turn == "right" && (value == "slight_right" || value == "sharp_right" || value == "merge_to_right"))) return true;
    }
    return false;
}
std::vector<Vec2> curve(const Lane& incoming, const Lane& outgoing) {
    const Vec2 p0 = incoming.centerline.back(), p3 = outgoing.centerline.front();
    const auto in = incoming.sample(incoming.length()), out = outgoing.sample(0.0);
    const double reach = std::max(1.0, norm(p3 - p0) * 0.45);
    const Vec2 p1 = p0 + Vec2{std::cos(in.yaw), std::sin(in.yaw)} * reach;
    const Vec2 p2 = p3 - Vec2{std::cos(out.yaw), std::sin(out.yaw)} * reach;
    const int count = std::max(8, static_cast<int>(std::ceil(norm(p3 - p0))));
    std::vector<Vec2> result;
    for (int i = 0; i <= count; ++i) {
        const double t = static_cast<double>(i) / count, u = 1.0 - t;
        const Vec2 p = p0 * (u*u*u) + p1 * (3*u*u*t) + p2 * (3*u*t*t) + p3 * (t*t*t);
        if (result.empty() || norm(p - result.back()) > 1e-6) result.push_back(p);
    }
    return result;
}
}

RoadNetwork buildOsmNetwork(const OsmData& data) {
    RoadNetwork road; road.imported = true;
    road.import_info.nodes = data.nodes.size(); road.import_info.ways = data.ways.size();
    std::vector<const OsmWay*> ways;
    std::map<OsmId, unsigned> references;
    std::set<OsmId> used;
    double min_lat = 90.0, max_lat = -90.0, min_lon = 180.0, max_lon = -180.0;
    for (const auto& [id, way] : data.ways) {
        if (!roadAccess(way) || way.nodes.size() < 2) { ++road.import_info.skipped_ways; continue; }
        ways.push_back(&way);
        std::set<OsmId> unique;
        for (OsmId ref : way.nodes) {
            const auto found = data.nodes.find(ref);
            if (found == data.nodes.end()) throw std::runtime_error("Way " + std::to_string(id) + " references missing node " + std::to_string(ref) + "; export complete ways with their nodes");
            const auto& n = found->second;
            min_lat = std::min(min_lat, n.latitude); max_lat = std::max(max_lat, n.latitude);
            min_lon = std::min(min_lon, n.longitude); max_lon = std::max(max_lon, n.longitude);
            used.insert(ref);
            if (unique.insert(ref).second) ++references[ref];
        }
        if (tag(way.tags, "access") == "destination") warn(road, id, "destination-only access is approximated as ordinary access");
        if (way.tags.contains("placement")) warn(road, id, "placement tag is not modeled; OSM way is treated as road centerline");
        for (const auto& entry : way.tags)
            if (entry.first.find("conditional") != std::string::npos) warn(road, id, "conditional tag '" + entry.first + "' is not modeled");
    }
    if (ways.empty()) throw std::runtime_error("OSM contains no supported, car-accessible roads");
    road.import_info.origin_latitude = (min_lat + max_lat) / 2.0;
    road.import_info.origin_longitude = (min_lon + max_lon) / 2.0;
    const MapProjection projection(road.import_info.origin_latitude, road.import_info.origin_longitude);
    std::map<OsmId, Vec2> positions;
    for (OsmId id : used) {
        const auto& node = data.nodes.at(id);
        const Vec2 position = projection.project(node.latitude, node.longitude);
        if (norm(position) > 100000.0) throw std::runtime_error("Map exceeds the 100 km local projection radius; select a city-sized extract");
        positions[id] = position;
    }
    std::vector<Segment> segments;
    std::map<OsmId, std::set<OsmId>> neighbors;
    for (const auto* way : ways) {
        std::size_t begin = 0;
        const bool closed = way->nodes.size() >= 4 && way->nodes.front() == way->nodes.back();
        for (std::size_t i = 1; i < way->nodes.size(); ++i) {
            const auto& node = data.nodes.at(way->nodes[i]);
            const auto control = tag(node.tags, "highway");
            if (i + 1 != way->nodes.size() && !(closed && i == (way->nodes.size() - 1) / 2)
                && references.at(node.id) < 2 && control != "traffic_signals" && control != "stop" && control != "give_way") continue;
            Segment segment{way, way->nodes[begin], way->nodes[i], {}};
            for (std::size_t p = begin; p <= i; ++p) {
                const Vec2 position = positions.at(way->nodes[p]);
                if (segment.geometry.empty() || norm(position - segment.geometry.back()) > 0.01) segment.geometry.push_back(position);
            }
            begin = i;
            if (segment.geometry.size() < 2 || polylineLength(segment.geometry) < 0.5) { warn(road, way->id, "zero-length/very short road segment skipped"); continue; }
            neighbors[segment.from].insert(segment.to); neighbors[segment.to].insert(segment.from);
            segments.push_back(std::move(segment));
        }
    }
    road.import_info.road_segments = segments.size();
    std::map<OsmId, std::size_t> junction_ids;
    for (const auto& [id, adjacent] : neighbors) {
        Junction junction; junction.id = id; junction.position = positions.at(id);
        junction.radius = adjacent.size() >= 2 ? 7.0 : 0.0;
        junction.traffic_lights = tag(data.nodes.at(id).tags, "highway") == "traffic_signals";
        if (junction.traffic_lights) warn(road, id, "traffic signal timing synthesized from simulation settings");
        junction_ids[id] = road.junctions.size(); road.junctions.push_back(junction);
    }
    for (const auto& segment : segments) {
        const auto& way = *segment.way;
        const std::string highway = tag(way.tags, "highway");
        const std::string one = tag(way.tags, "oneway", (tag(way.tags, "junction") == "roundabout" || highway == "motorway" || highway == "motorway_link") ? "yes" : "no");
        const bool reverse = one == "-1", single = reverse || yes(one);
        if (!single && one != "no" && one != "false" && one != "0") warn(road, way.id, "unsupported oneway value; assuming two-way traffic");
        const unsigned shared = countTag(way.tags, "lanes:both_ways", 0, road, way.id);
        const unsigned tagged_forward = countTag(way.tags, "lanes:forward", 0, road, way.id);
        const unsigned tagged_backward = countTag(way.tags, "lanes:backward", 0, road, way.id);
        const unsigned inferred_total = single ? std::max(1U, reverse ? tagged_backward : tagged_forward)
            : std::max(2U, tagged_forward + tagged_backward + shared);
        const unsigned total = countTag(way.tags, "lanes", inferred_total, road, way.id);
        if (shared) warn(road, way.id, "shared center turn lanes are omitted");
        const unsigned regular = total > shared ? total - shared : 2U;
        unsigned forward = reverse ? 0U : single ? regular : way.tags.contains("lanes:forward") ? tagged_forward
            : way.tags.contains("lanes:backward") ? (regular > tagged_backward ? regular - tagged_backward : 0U) : regular / 2U;
        unsigned backward = reverse ? regular : single ? 0U : way.tags.contains("lanes:backward") ? tagged_backward
            : regular > forward ? regular - forward : 0U;
        if (!single && (!forward || !backward)) { forward = std::max(1U, forward); backward = std::max(1U, backward); warn(road, way.id, "shared or ambiguous lane layout approximated with separate directional lanes"); }
        if (forward + backward != regular) warn(road, way.id, "directional lane counts differ from lanes; using directional counts");
        if (!way.tags.contains("lanes") && !way.tags.contains("lanes:forward") && !way.tags.contains("lanes:backward"))
            warn(road, way.id, "lane count inferred as " + std::to_string(forward + backward));
        if (!single && regular % 2U && !way.tags.contains("lanes:forward") && !way.tags.contains("lanes:backward") && regular > 1U)
            warn(road, way.id, "odd lane count split by default; directional lane counts are missing");
        if (forward + backward == 0) { warn(road, way.id, "road with no usable lanes skipped"); continue; }
        double width = 3.5;
        if (auto parsed = numeric(tag(way.tags, "width")); parsed) width = *parsed / static_cast<double>(forward + backward + shared);
        if (width < 2.4 || width > 5.0) { warn(road, way.id, "unsupported lane width; assuming 3.5 m"); width = 3.5; }
        int layer = yes(tag(way.tags, "bridge")) ? 1 : yes(tag(way.tags, "tunnel")) ? -1 : 0;
        if (!tag(way.tags, "layer").empty()) {
            const auto value = tag(way.tags, "layer");
            const auto [end, error] = std::from_chars(value.data(), value.data() + value.size(), layer);
            if (error != std::errc{} || end != value.data() + value.size() || std::abs(layer) > 100) throw std::runtime_error("Invalid layer for way " + std::to_string(way.id));
        }
        const double separation = (static_cast<double>(backward) - forward) * width / 2.0;
        const auto addDirection = [&](bool fwd, unsigned count) {
            const auto direction_key = fwd ? "forward" : "backward";
            std::string markings = tag(way.tags, std::string("turn:lanes:") + direction_key);
            if (markings.empty() && single) markings = tag(way.tags, "turn:lanes");
            const auto turns = split(markings, '|');
            if (!markings.empty() && turns.size() != count) warn(road, way.id, "turn:lanes count does not match directional lane count");
            std::string access;
            for (const char* mode : {"motorcar", "motor_vehicle", "vehicle"}) {
                access = tag(way.tags, std::string(mode) + ":lanes:" + direction_key);
                if (access.empty() && single) access = tag(way.tags, std::string(mode) + ":lanes");
                if (!access.empty()) break;
            }
            const auto permissions = split(access, '|');
            const double limit = speed(way, fwd, road);
            for (unsigned i = 0; i < count; ++i) {
                if (!access.empty() && i < permissions.size() && (permissions[i] == "no" || permissions[i] == "private")) continue;
                double lateral = single ? (static_cast<double>(i) + 0.5 - count / 2.0) * width
                    : fwd ? separation + (i + 0.5) * width : separation - (i + 0.5) * width;
                if (single && !fwd) lateral = -lateral;
                auto geometry = offset(segment.geometry, lateral);
                if (!fwd) std::reverse(geometry.begin(), geometry.end());
                const OsmId from = fwd ? segment.from : segment.to, to = fwd ? segment.to : segment.from;
                const auto from_j = junction_ids.at(from), to_j = junction_ids.at(to);
                const double radius = std::max(7.0, width * static_cast<double>(std::max(forward, backward)) + 3.0);
                if (road.junctions[from_j].radius > 0.0) road.junctions[from_j].radius = std::max(road.junctions[from_j].radius, radius);
                if (road.junctions[to_j].radius > 0.0) road.junctions[to_j].radius = std::max(road.junctions[to_j].radius, radius);
                Lane lane;
                lane.id = static_cast<LaneId>(road.lanes.size());
                lane.name = std::to_string(way.id) + "/" + std::to_string(from) + "/" + direction_key + "/" + std::to_string(i);
                lane.width = width; lane.centerline = std::move(geometry); lane.speed_limit = limit;
                lane.source_way = way.id; lane.from_node = from; lane.to_node = to; lane.layer = layer;
                lane.junction = to_j; lane.priority = priority(highway);
                const auto control = tag(data.nodes.at(to).tags, "highway");
                lane.stop_sign = control == "stop"; lane.give_way = control == "give_way";
                lane.turn = !markings.empty() && i < turns.size() ? turns[i] : "";
                lane.lateral_index = i; lane.directional_count = count;
                road.junctions[from_j].outgoing.push_back(lane.id); road.junctions[to_j].incoming.push_back(lane.id);
                road.lanes.push_back(std::move(lane));
            }
        };
        if (forward) addDirection(true, forward);
        if (backward) addDirection(false, backward);
    }
    if (road.lanes.empty()) throw std::runtime_error("No usable car lanes could be generated");
    road.import_info.road_lanes = road.lanes.size();
    for (auto& lane : road.lanes) {
        lane.centerline = trim(lane.centerline, road.junctions[junction_ids.at(lane.from_node)].radius,
            road.junctions[junction_ids.at(lane.to_node)].radius);
        const double heading = lane.sample(lane.length()).yaw;
        lane.signal_group = std::abs(std::cos(heading)) >= std::abs(std::sin(heading)) ? 0 : 1;
    }
    std::vector<Restriction> restrictions;
    for (const auto& [id, relation] : data.relations) {
        if (tag(relation.tags, "type") != "restriction") continue;
        if (tag(relation.tags, "except").find("motorcar") != std::string::npos || tag(relation.tags, "except").find("motor_vehicle") != std::string::npos) continue;
        Restriction restriction;
        restriction.value = tag(relation.tags, "restriction:motorcar", tag(relation.tags, "restriction:motor_vehicle", tag(relation.tags, "restriction")));
        unsigned from_count = 0, to_count = 0, via_count = 0, total_via = 0;
        for (const auto& member : relation.members) {
            if (member.role == "from" && member.type == "way") { restriction.from = member.reference; ++from_count; }
            if (member.role == "to" && member.type == "way") { restriction.to = member.reference; ++to_count; }
            if (member.role == "via" && member.type == "node") { restriction.via = member.reference; ++via_count; }
            if (member.role == "via") ++total_via;
        }
        const bool conditional = std::any_of(relation.tags.begin(), relation.tags.end(),
            [](const auto& entry) { return entry.first.find("conditional") != std::string::npos; });
        static const std::set<std::string> supported{"no_left_turn", "no_right_turn", "no_straight_on", "no_u_turn", "no_entry", "no_exit",
            "only_left_turn", "only_right_turn", "only_straight_on", "only_u_turn"};
        if (conditional || from_count != 1 || to_count != 1 || via_count != 1 || total_via != 1 || !supported.contains(restriction.value)) {
            warn(road, id, "unsupported conditional/via-way/complex restriction; not applied"); continue;
        }
        if (restriction.value == "only_u_turn") warn(road, id, "only_u_turn blocks other movements; U-turn geometry is not generated");
        restrictions.push_back(restriction);
    }
    for (std::size_t j = 0; j < road.junctions.size(); ++j) {
        auto& junction = road.junctions[j];
        for (LaneId from_id : junction.incoming) for (LaneId to_id : junction.outgoing) {
            const Lane incoming = road.lane(from_id), outgoing = road.lane(to_id);
            const double delta = wrapAngle(outgoing.sample(0.0).yaw - incoming.sample(incoming.length()).yaw);
            if (std::abs(delta) > 2.6) continue;
            const std::string turn = delta > 0.4 ? "left" : delta < -0.4 ? "right" : "through";
            if (!turnPermitted(incoming.turn, turn)) continue;
            const unsigned target_index = incoming.directional_count == 1 && outgoing.directional_count > 1
                ? (turn == "left" ? 0U : outgoing.directional_count - 1U)
                : std::min(incoming.lateral_index, outgoing.directional_count - 1U);
            if (outgoing.lateral_index != target_index) continue;
            bool allowed = true;
            for (const auto& r : restrictions) {
                if (r.from != incoming.source_way || r.via != junction.id) continue;
                if (r.value == "no_u_turn") continue;
                if (r.value == "only_u_turn") { allowed = false; continue; }
                if (r.value.find("no_") == 0 && r.to == outgoing.source_way) allowed = false;
                if (r.value.find("only_") == 0 && r.to != outgoing.source_way) allowed = false;
            }
            if (!allowed) continue;
            Lane connector;
            connector.id = static_cast<LaneId>(road.lanes.size()); connector.connector = true;
            connector.name = "junction/" + std::to_string(junction.id) + "/" + std::to_string(from_id) + "/" + std::to_string(to_id);
            connector.width = std::min(incoming.width, outgoing.width); connector.centerline = curve(incoming, outgoing);
            if (connector.centerline.size() < 2 || polylineLength(connector.centerline) < 0.05) continue;
            connector.speed_limit = std::min(incoming.speed_limit, outgoing.speed_limit);
            if (turn != "through") connector.speed_limit = std::min(connector.speed_limit, std::sqrt(2.5 * std::max(3.0, junction.radius * 0.5)));
            connector.junction = j; connector.layer = incoming.layer; connector.signal_group = incoming.signal_group;
            connector.priority = incoming.priority; connector.turn = turn; connector.successors = {to_id};
            road.lanes.push_back(std::move(connector));
            road.lanes[from_id].successors.push_back(road.lanes.back().id);
            junction.connectors.push_back(road.lanes.back().id);
        }
    }
    road.import_info.connectors = road.lanes.size() - road.import_info.road_lanes;
    const auto bounds = road.bounds();
    road.arm_length = std::max(bounds.maximum.x - bounds.minimum.x, bounds.maximum.y - bounds.minimum.y) / 2.0;
    return road;
}
RoadNetwork loadOsmNetwork(const std::filesystem::path& path) {
    try { return buildOsmNetwork(readOsm(path)); }
    catch (const std::exception& error) { throw std::runtime_error("Cannot import '" + path.string() + "': " + error.what()); }
}
Scenario loadOsmScenario(const std::filesystem::path& path, const ScenarioOverrides& overrides, bool map_only) {
    auto scenario = osmScenario(loadOsmNetwork(path), overrides, map_only);
    scenario.name = "OSM / " + path.filename().string();
    return scenario;
}
Scenario osmScenario(RoadNetwork road, const ScenarioOverrides& overrides, bool map_only) {
    Scenario scenario;
    scenario.name = "OSM"; scenario.seed = overrides.seed.value_or(42);
    scenario.road = std::move(road);
    if (map_only) return scenario;
    struct Slot { LaneId lane; double progress; };
    std::vector<std::vector<Slot>> lanes;
    for (const auto& lane : scenario.road.lanes) {
        if (lane.connector || lane.width < 2.0) continue;
        std::vector<Slot> slots;
        for (double p = 3.0; p < lane.length() - 5.0; p += 9.0) slots.push_back({lane.id, p});
        if (!slots.empty()) lanes.push_back(std::move(slots));
    }
    std::vector<Slot> slots;
    for (std::size_t row = 0; ; ++row) {
        bool any = false;
        for (const auto& lane : lanes) if (row < lane.size()) { slots.push_back(lane[row]); any = true; }
        if (!any) break;
    }
    if (slots.empty()) throw std::runtime_error("Map has no lanes long enough to spawn vehicles; use --map-only");
    const std::size_t count = overrides.vehicle_count.value_or(std::min<std::size_t>(100, slots.size()));
    if (count == 0 || count > 1000 || count > slots.size()) throw std::runtime_error("Requested fleet does not fit; map spawn capacity is " + std::to_string(slots.size()));
    std::mt19937 random(scenario.seed);
    std::map<LaneId, std::vector<LaneId>> routes;
    for (std::size_t i = 0; i < count; ++i) {
        const auto& lane = scenario.road.lane(slots[i].lane);
        if (!routes.contains(lane.id)) routes[lane.id] = planRoute(scenario.road, lane.id);
        const auto sample = lane.sample(slots[i].progress);
        Vehicle v; v.id = static_cast<EntityId>(i + 1); v.lane = lane.id; v.layer = lane.layer;
        v.state.x = sample.position.x; v.state.y = sample.position.y; v.state.yaw = sample.yaw;
        v.route = routes.at(v.lane);
        v.desired_speed = std::min(v.config.max_speed, lane.speed_limit) * (0.9 + 0.1 * static_cast<double>(random()) / 4294967295.0);
        scenario.vehicles.push_back(std::move(v));
    }
    World check(scenario);
    if (!check.collisions().empty()) throw std::runtime_error("Generated spawn positions overlap; reduce --vehicles or use --map-only to inspect the geometry");
    return scenario;
}
}
