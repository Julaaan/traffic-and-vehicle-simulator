#include "vsim/RoutePlanner.hpp"
#include <algorithm>
#include <map>
#include <queue>

namespace vsim {
std::vector<LaneId> planRoute(const RoadNetwork& road, LaneId start) {
    road.lane(start);
    std::map<LaneId, double> distances;
    std::map<LaneId, LaneId> previous;
    using Item = std::pair<double, LaneId>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue;
    distances[start] = 0.0; queue.push({0.0, start});
    while (!queue.empty()) {
        const auto [cost, id] = queue.top(); queue.pop();
        if (cost > distances.at(id)) continue;
        for (LaneId next : road.lane(id).successors) {
            const auto& lane = road.lane(next);
            const double candidate = cost + lane.length() / lane.speed_limit;
            if (!distances.contains(next) || candidate < distances.at(next)) {
                distances[next] = candidate; previous[next] = id;
                queue.push({candidate, next});
            }
        }
    }
    LaneId goal = start;
    bool terminal_found = false;
    double furthest = -1.0;
    for (const auto& [id, distance] : distances) {
        const auto& lane = road.lane(id);
        if (lane.connector) continue;
        const bool terminal = lane.successors.empty();
        if ((terminal && !terminal_found) || (terminal == terminal_found && distance > furthest)) {
            goal = id; furthest = distance; terminal_found = terminal;
        }
    }
    std::vector<LaneId> result{goal};
    while (result.back() != start) result.push_back(previous.at(result.back()));
    std::reverse(result.begin(), result.end());
    return result;
}
}
