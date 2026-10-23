#pragma once
#include "vsim/World.hpp"

namespace vsim {
std::string hexString(std::uint64_t value);
inline constexpr std::uint64_t hash_seed = 14695981039346656037ULL;
// FNV-1a over the eight little-endian bytes of value, continuing from hash.
inline std::uint64_t hashAppend(std::uint64_t hash, std::uint64_t value) {
    for (unsigned byte = 0; byte < 8; ++byte) {
        hash ^= (value >> (byte * 8U)) & 0xffU;
        hash *= 1099511628211ULL;
    }
    return hash;
}

class SimulationClock {
public:
    explicit SimulationClock(double dt);
    double time() const { return static_cast<double>(tick_) * dt_; }
    double timeStep() const { return dt_; }
    std::uint64_t tick() const { return tick_; }
    void advance();
    void restore(std::uint64_t tick) { tick_ = tick; }
private:
    double dt_;
    std::uint64_t tick_{};
};

struct SimulationSnapshot {
    std::uint64_t tick{}, collision_events{};
    std::size_t peak_collision_pairs{};
    std::vector<Vehicle> vehicles;
    std::vector<CollisionPair> collisions;
};

class Simulation {
public:
    explicit Simulation(Scenario scenario);
    void step(std::span<const RecordedControl> recorded = {}, StepProfile* profile = nullptr);
    void reset();
    const World& world() const { return world_; }
    const SimulationClock& clock() const { return clock_; }
    const Scenario& scenario() const { return initial_; }
    std::uint64_t collisionEvents() const { return collision_events_; }
    std::size_t peakCollisionPairs() const { return peak_collision_pairs_; }
    std::uint64_t stateHash() const;
    SimulationSnapshot snapshot() const;
    void restore(const SimulationSnapshot& snapshot);
private:
    Scenario initial_;
    World world_;
    SimulationClock clock_;
    std::uint64_t collision_events_{};
    std::size_t peak_collision_pairs_{};
    std::vector<CollisionPair> previous_collisions_;
};
}
