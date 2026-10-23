#pragma once
#include "vsim/Simulation.hpp"
#include <memory>

namespace vsim {
class SimulationBatch {
public:
    explicit SimulationBatch(std::vector<Scenario> scenarios, unsigned threads = 0);
    ~SimulationBatch();
    SimulationBatch(const SimulationBatch&) = delete;
    SimulationBatch& operator=(const SimulationBatch&) = delete;
    void step(std::uint64_t steps = 1, StepProfile* profile = nullptr);
    std::size_t size() const { return simulations_.size(); }
    unsigned threads() const;
    std::size_t vehicles() const;
    const Simulation& operator[](std::size_t index) const { return simulations_.at(index); }
    std::uint64_t stateHash() const;
private:
    class Pool;
    std::vector<Simulation> simulations_;
    std::unique_ptr<Pool> pool_;
};
}
