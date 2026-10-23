#pragma once
#include "vsim/Simulation.hpp"
#include <filesystem>
#include <fstream>

namespace vsim::app {
class CsvOutput {
public:
    explicit CsvOutput(const std::filesystem::path& path);
    void sample(const Simulation& simulation);
    void flush();
private:
    std::ofstream stream_;
};
void writeReport(const std::filesystem::path& path, const Simulation& simulation, double wall_seconds);
void printSummary(const Simulation& simulation, double wall_seconds);
}
