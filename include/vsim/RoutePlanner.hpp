#pragma once
#include "vsim/RoadNetwork.hpp"

namespace vsim {
std::vector<LaneId> planRoute(const RoadNetwork& road, LaneId start);
}
