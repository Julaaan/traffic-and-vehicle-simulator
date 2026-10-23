#pragma once
#include "vsim/Collision.hpp"
#include "vsim/Scenario.hpp"

namespace vsim {
struct StepProfile {
    double projection{}, admission{}, control{}, integration{}, transition{}, collision{};
    std::uint64_t steps{};
    double total() const { return projection + admission + control + integration + transition + collision; }
    StepProfile& operator+=(const StepProfile& other);
};

class World {
public:
    explicit World(const Scenario& scenario);
    void step(double dt, double time, std::span<const RecordedControl> recorded = {}, StepProfile* profile = nullptr);
    void restore(const std::vector<Vehicle>& vehicles, const std::vector<CollisionPair>& collisions);
    const RoadNetwork& road() const { return road_; }
    const TrafficConfig& traffic() const { return traffic_; }
    const std::vector<Vehicle>& vehicles() const { return vehicles_; }
    const std::vector<CollisionPair>& collisions() const { return collisions_; }
    std::size_t activeCount() const;
    const Vehicle& vehicle(EntityId id) const;
private:
    VehicleControl automaticControl(std::size_t index,
        const std::vector<LaneProjection>& projections, double time) const;
    VehicleControl networkControl(std::size_t index, const std::vector<LaneProjection>& projections,
                                  bool may_enter) const;
    std::vector<bool> networkAdmissions(const std::vector<LaneProjection>& projections, double time);
    double length(const Lane& lane) const;
    RoadNetwork road_;
    std::vector<double> lane_lengths_;
    TrafficConfig traffic_;
    std::vector<Vehicle> vehicles_;
    std::vector<CollisionPair> collisions_;
};
}
