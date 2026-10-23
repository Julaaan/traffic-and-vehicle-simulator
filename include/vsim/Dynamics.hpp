#pragma once
#include "vsim/Types.hpp"

namespace vsim {
void validate(const VehicleConfig& config);
void validate(const VehicleState& state, const VehicleConfig& config);
void validate(const VehicleControl& control);
void integrate(VehicleState& state, const VehicleConfig& config,
               VehicleControl control, double dt);
const char* dynamicsModelName(DynamicsModel model);
}
