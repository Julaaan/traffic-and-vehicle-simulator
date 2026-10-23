#pragma once
#include "vsim/Serialization.hpp"

namespace vsim {
struct ValidationMetric { std::string name, unit; double rmse{}, maximum{}, rmse_limit{}, maximum_limit{}; bool passed{}; };
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ValidationMetric, name, unit, rmse, maximum, rmse_limit, maximum_limit, passed)
struct DynamicsValidationResult {
    std::string name, reference_kind, reference_source, model;
    std::size_t samples{};
    double seconds{};
    double time_step{};
    VehicleConfig vehicle_config;
    bool passed{};
    std::vector<ValidationMetric> metrics;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(DynamicsValidationResult, name, reference_kind, reference_source, model, samples, seconds, time_step, vehicle_config, passed, metrics)
DynamicsValidationResult validateDynamicsReference(const std::filesystem::path& manifest);
void writeDynamicsValidation(const std::filesystem::path& path, const DynamicsValidationResult& result);
}
