#pragma once
#include "vsim/Types.hpp"
#include <array>
#include <span>
#include <vector>

namespace vsim {
struct CollisionPair {
    EntityId first{};
    EntityId second{};
    bool operator==(const CollisionPair&) const = default;
};
std::array<Vec2, 4> corners(const Vehicle& vehicle);
std::vector<CollisionPair> findCollisions(std::span<const Vehicle> vehicles);
}
