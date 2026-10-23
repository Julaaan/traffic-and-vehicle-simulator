#include "vsim/Collision.hpp"
#include <algorithm>
#include <limits>

namespace vsim {
std::array<Vec2, 4> corners(const Vehicle& v) {
    const Vec2 forward{std::cos(v.state.yaw), std::sin(v.state.yaw)};
    const Vec2 lateral{-forward.y, forward.x};
    const Vec2 x = forward * (v.config.length / 2.0);
    const Vec2 y = lateral * (v.config.width / 2.0);
    const Vec2 p = v.state.position();
    return {p + x + y, p + x - y, p - x - y, p - x + y};
}

namespace {
double diagonal(const Vehicle& v) { return std::hypot(v.config.length, v.config.width); }
bool overlaps(const Vehicle& a, const Vehicle& b, double diagonal_a, double diagonal_b) {
    const double radius = (diagonal_a + diagonal_b) / 2.0;
    const Vec2 delta = a.state.position() - b.state.position();
    if (dot(delta, delta) > radius * radius) return false;
    const auto ca = corners(a);
    const auto cb = corners(b);
    const std::array<Vec2, 4> axes{{
        {std::cos(a.state.yaw), std::sin(a.state.yaw)},
        {-std::sin(a.state.yaw), std::cos(a.state.yaw)},
        {std::cos(b.state.yaw), std::sin(b.state.yaw)},
        {-std::sin(b.state.yaw), std::cos(b.state.yaw)}
    }};
    for (Vec2 axis : axes) {
        double min_a = std::numeric_limits<double>::infinity(), max_a = -min_a;
        double min_b = min_a, max_b = -min_a;
        for (Vec2 p : ca) { const double value = dot(p, axis); min_a = std::min(min_a, value); max_a = std::max(max_a, value); }
        for (Vec2 p : cb) { const double value = dot(p, axis); min_b = std::min(min_b, value); max_b = std::max(max_b, value); }
        if (max_a < min_b - 1e-9 || max_b < min_a - 1e-9) return false;
    }
    return true;
}
}

std::vector<CollisionPair> findCollisions(std::span<const Vehicle> vehicles) {
    std::vector<CollisionPair> result;
    std::vector<double> diagonals;
    diagonals.reserve(vehicles.size());
    for (const auto& vehicle : vehicles) diagonals.push_back(diagonal(vehicle));
    for (std::size_t i = 0; i < vehicles.size(); ++i) {
        if (!vehicles[i].active) continue;
        for (std::size_t j = i + 1; j < vehicles.size(); ++j) {
            if (vehicles[j].active && vehicles[i].layer == vehicles[j].layer
                && overlaps(vehicles[i], vehicles[j], diagonals[i], diagonals[j]))
                result.push_back({std::min(vehicles[i].id, vehicles[j].id), std::max(vehicles[i].id, vehicles[j].id)});
        }
    }
    std::sort(result.begin(), result.end(), [](const auto& a, const auto& b) {
        return a.first < b.first || (a.first == b.first && a.second < b.second);
    });
    return result;
}
}
