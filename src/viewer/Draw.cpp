#include "Draw.hpp"
#include <algorithm>
#include <array>
#include <cstdio>
#include <filesystem>
#include <stdexcept>
#include <utility>

namespace vsim::viewer {
namespace {
using Color = SDL_Color;
using Size = TextSize;
constexpr Color background{17, 25, 35, 255}, overlay{26, 37, 50, 245}, hover{42, 57, 72, 255};
constexpr Color ink{223, 232, 242, 255}, muted{124, 145, 164, 255}, divider{124, 145, 164, 60};
constexpr Color accent{93, 225, 174, 255}, red{255, 94, 115, 255};
constexpr std::array<Color, 4> fleet{{{78, 192, 235, 255}, {246, 182, 96, 255},
    {170, 147, 244, 255}, {93, 225, 174, 255}}};

void color(SDL_Renderer* r, Color c) { SDL_SetRenderDrawColor(r, c.r, c.g, c.b, c.a); }
void rect(SDL_Renderer* r, SDL_Rect area, Color c) { color(r, c); SDL_RenderFillRect(r, &area); }
std::string fixed(double value, int places = 1) {
    if (std::abs(value) < 0.5 * std::pow(10.0, -places)) value = 0.0;
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.*f", places, value);
    return buffer;
}
SDL_FPoint pixel(Vec2 p, SDL_Rect area, Camera c) {
    return {static_cast<float>(area.x + area.w / 2.0 + (p.x - c.center.x) * c.scale),
        static_cast<float>(area.y + area.h / 2.0 - (p.y - c.center.y) * c.scale)};
}
void line(SDL_Renderer* r, Vec2 a, Vec2 b, SDL_Rect area, Camera c, Color tint) {
    const auto p = pixel(a, area, c), q = pixel(b, area, c);
    color(r, tint); SDL_RenderDrawLineF(r, p.x, p.y, q.x, q.y);
}
void worldRect(SDL_Renderer* r, Vec2 low, Vec2 high, SDL_Rect area, Camera c, Color tint) {
    const auto a = pixel({low.x, high.y}, area, c), b = pixel({high.x, low.y}, area, c);
    const SDL_FRect rectangle{a.x, a.y, b.x - a.x, b.y - a.y};
    color(r, tint); SDL_RenderFillRectF(r, &rectangle);
}
void circle(SDL_Renderer* r, float x, float y, int radius, Color tint) {
    color(r, tint);
    for (int dy = -radius; dy <= radius; ++dy) {
        const int dx = static_cast<int>(std::sqrt(static_cast<double>(radius * radius - dy * dy)));
        SDL_RenderDrawLineF(r, x - static_cast<float>(dx), y + static_cast<float>(dy),
            x + static_cast<float>(dx), y + static_cast<float>(dy));
    }
}
void triangle(SDL_Renderer* r, SDL_FPoint a, SDL_FPoint b, SDL_FPoint c, Color tint) {
    const std::array<SDL_Vertex, 3> vertices{{{a, tint, {}}, {b, tint, {}}, {c, tint, {}}}};
    SDL_RenderGeometry(r, nullptr, vertices.data(), 3, nullptr, 0);
}
void polygon(SDL_Renderer* r, const std::array<Vec2, 4>& corners, SDL_Rect area, Camera c, Color tint) {
    std::array<SDL_Vertex, 4> vertices{};
    for (std::size_t i = 0; i < vertices.size(); ++i) {
        vertices[i].position = pixel(corners[i], area, c); vertices[i].color = tint;
    }
    constexpr int indices[]{0, 1, 2, 0, 2, 3};
    SDL_RenderGeometry(r, nullptr, vertices.data(), 4, indices, 6);
}
void outline(SDL_Renderer* r, const std::array<Vec2, 4>& corners, SDL_Rect area, Camera c, Color tint) {
    for (std::size_t i = 0; i < corners.size(); ++i)
        line(r, corners[i], corners[(i + 1) % 4], area, c, tint);
}
void networkGeometry(SDL_Renderer* r, SDL_Rect area, Camera camera, const Simulation& sim, const ViewState& view) {
    const auto& road = sim.world().road();
    std::vector<const Lane*> lanes;
    for (const auto& lane : road.lanes) lanes.push_back(&lane);
    std::stable_sort(lanes.begin(), lanes.end(), [](const auto* a, const auto* b) { return a->layer < b->layer; });
    for (const auto* lane : lanes) {
        for (std::size_t i = 1; i < lane->centerline.size(); ++i) {
            const Vec2 a = lane->centerline[i - 1], b = lane->centerline[i];
            const Vec2 delta = b - a; const double length = norm(delta);
            if (length < 1e-6) continue;
            const Vec2 side{-delta.y / length * lane->width / 2.0, delta.x / length * lane->width / 2.0};
            polygon(r, {a + side, b + side, b - side, a - side}, area, camera,
                lane->connector ? Color{39, 53, 66, 255} : Color{36, 47, 60, 255});
            if (!lane->connector) {
                line(r, a + side, b + side, area, camera, {95, 108, 117, 180});
                line(r, a - side, b - side, area, camera, {95, 108, 117, 180});
            }
            if (view.routes) {
                const auto tint = fleet[lane->id % 4];
                line(r, a, b, area, camera, {tint.r, tint.g, tint.b, 90});
            }
        }
        if (!lane->connector && !lane->successors.empty()) {
            const auto sample = lane->sample(std::max(0.0, lane->length() - 0.5));
            const Vec2 side{-std::sin(sample.yaw), std::cos(sample.yaw)};
            line(r, sample.position - side * (lane->width * 0.45), sample.position + side * (lane->width * 0.45), area, camera, ink);
            if (lane->junction && road.junctions[*lane->junction].traffic_lights) {
                const auto p = pixel(sample.position + side * (lane->width / 2.0 + 0.5), area, camera);
                circle(r, p.x, p.y, 4, greenFor(road, lane->id, sim.clock().time(), sim.world().traffic()) ? accent : red);
            }
        }
    }
    if (view.selected) {
        const auto& vehicle = sim.world().vehicle(*view.selected);
        for (LaneId id : vehicle.route) {
            const auto& lane = road.lane(id);
            for (std::size_t i = 1; i < lane.centerline.size(); ++i)
                line(r, lane.centerline[i - 1], lane.centerline[i], area, camera, accent);
        }
    }
}
void world(SDL_Renderer* r, SDL_Rect area, Camera camera, const Simulation& sim, const ViewState& view) {
    rect(r, area, background);
    SDL_RenderSetClipRect(r, &area);
    const double a = sim.world().road().arm_length, w = sim.world().road().lane_width;
    const double grid = std::pow(10.0, std::ceil(std::log10(35.0 / camera.scale)));
    const Vec2 low = screenToWorld(area.x, area.y + area.h, area, camera);
    const Vec2 high = screenToWorld(area.x + area.w, area.y, area, camera);
    for (double x = std::floor(low.x / grid) * grid; x <= high.x; x += grid)
        line(r, {x, low.y}, {x, high.y}, area, camera, {25, 36, 49, 255});
    for (double y = std::floor(low.y / grid) * grid; y <= high.y; y += grid)
        line(r, {low.x, y}, {high.x, y}, area, camera, {25, 36, 49, 255});
    if (sim.world().road().imported) networkGeometry(r, area, camera, sim, view);
    else {
        worldRect(r, {-a, -w - 0.5}, {a, w + 0.5}, area, camera, {63, 75, 87, 255});
        worldRect(r, {-w - 0.5, -a}, {w + 0.5, a}, area, camera, {63, 75, 87, 255});
        worldRect(r, {-a, -w}, {a, w}, area, camera, {36, 47, 60, 255});
        worldRect(r, {-w, -a}, {w, a}, area, camera, {36, 47, 60, 255});
        for (double s = -a; s < a; s += 6.0) {
            if (s > -w - 6.0 && s < w) continue;
            line(r, {s, 0}, {std::min(s + 3.0, a), 0}, area, camera, {168, 151, 107, 255});
            line(r, {0, s}, {0, std::min(s + 3.0, a)}, area, camera, {168, 151, 107, 255});
        }
        if (view.routes) for (const auto& lane : sim.world().road().lanes) {
            const auto c = fleet[lane.id % 4];
            line(r, lane.centerline.front(), lane.centerline.back(), area, camera, {c.r, c.g, c.b, 65});
            for (double s = a - 60.0; s < a + 60.0; s += 30.0) {
                const auto sample = lane.sample(s);
                const Vec2 forward{std::cos(sample.yaw), std::sin(sample.yaw)};
                const Vec2 lateral{-forward.y, forward.x};
                line(r, sample.position - forward * 1.0 + lateral * 0.6, sample.position + forward * 1.0, area, camera, c);
                line(r, sample.position - forward * 1.0 - lateral * 0.6, sample.position + forward * 1.0, area, camera, c);
            }
        }
        for (const auto& lane : sim.world().road().lanes) {
            const auto stop = lane.sample(sim.world().road().stopLine());
            const Vec2 side{-std::sin(stop.yaw), std::cos(stop.yaw)};
            line(r, stop.position - side * (w / 2.0), stop.position + side * (w / 2.0), area, camera, {214, 223, 226, 255});
            const auto location = pixel(stop.position + side * (w / 2.0 + 1.0), area, camera);
            circle(r, location.x, location.y, 6, greenFor(lane.id, sim.clock().time(), sim.world().traffic()) ? accent : red);
        }
    }
    for (const auto& vehicle : sim.world().vehicles()) {
        if (!vehicle.active) continue;
        const auto p = pixel(vehicle.state.position(), area, camera);
        if (p.x < area.x - 30 || p.y < area.y - 30 || p.x > area.x + area.w + 30 || p.y > area.y + area.h + 30) continue;
        bool collision = false;
        for (const auto& pair : sim.world().collisions())
            if (pair.first == vehicle.id || pair.second == vehicle.id) collision = true;
        polygon(r, corners(vehicle), area, camera, collision ? red : fleet[vehicle.lane % 4]);
        const Vec2 forward{std::cos(vehicle.state.yaw), std::sin(vehicle.state.yaw)};
        const Vec2 side{-forward.y, forward.x};
        const Vec2 front = vehicle.state.position() + forward * (vehicle.config.length * 0.25);
        line(r, front - side * (vehicle.config.width * 0.3), front + side * (vehicle.config.width * 0.3),
            area, camera, {28, 44, 60, 255});
        if (view.boxes || view.selected == vehicle.id) {
            auto inflated = vehicle; inflated.config.length += 0.6; inflated.config.width += 0.6;
            outline(r, corners(inflated), area, camera, view.selected == vehicle.id ? ink : muted);
        }
    }
    SDL_RenderSetClipRect(r, nullptr);
}
void inspector(SDL_Renderer* r, const Fonts& f, int width, const Vehicle& v) {
    const std::string speed = fixed(v.state.velocity, 1) + " m/s";
    const int pad = 16, gap = 16, large = f.capHeight(Size::Large), small = f.capHeight();
    const SDL_Rect box{width - 216, 16, 200, 2 * pad + large + gap + small};
    rect(r, box, overlay);
    rect(r, {box.x + pad, box.y + pad + large / 2 - 4, 8, 8}, fleet[v.lane % 4]);
    f.draw(r, box.x + pad + 16, box.y + pad, "Vehicle " + std::to_string(v.id), ink, Size::Large);
    f.draw(r, box.x + pad, box.y + pad + large + gap, "Speed", muted);
    f.draw(r, box.x + box.w - pad - f.width(speed), box.y + pad + large + gap, speed, ink);
}
enum class Icon { Play, Pause, Step, Reset, Slower, Faster, Fit, Help };
Icon iconFor(int action, const ViewState& view) {
    switch (action) {
    case 0: return view.paused ? Icon::Play : Icon::Pause;
    case 1: return Icon::Step;
    case 2: return Icon::Reset;
    case 3: return Icon::Fit;
    case 5: return Icon::Slower;
    case 6: return Icon::Faster;
    default: return Icon::Help;
    }
}
void icon(SDL_Renderer* r, const Fonts& f, Icon kind, SDL_Rect button, Color tint) {
    const int cx = button.x + button.w / 2, cy = button.y + button.h / 2;
    const float x = static_cast<float>(cx), y = static_cast<float>(cy);
    switch (kind) {
    case Icon::Play: triangle(r, {x - 5, y - 7}, {x + 7, y}, {x - 5, y + 7}, tint); break;
    case Icon::Pause: rect(r, {cx - 6, cy - 7, 4, 14}, tint); rect(r, {cx + 2, cy - 7, 4, 14}, tint); break;
    case Icon::Step: triangle(r, {x - 7, y - 7}, {x + 3, y}, {x - 7, y + 7}, tint); rect(r, {cx + 4, cy - 7, 3, 14}, tint); break;
    case Icon::Reset: {
        constexpr double start = 1.75, radius = 6.5;
        color(r, tint);
        for (double angle = start; angle < start + 5.3; angle += 0.04) {
            const SDL_FRect dot{x + static_cast<float>(radius * std::cos(angle)) - 1.0f,
                y - static_cast<float>(radius * std::sin(angle)) - 1.0f, 2.0f, 2.0f};
            SDL_RenderFillRectF(r, &dot);
        }
        const SDL_FPoint p{x + static_cast<float>(radius * std::cos(start)), y - static_cast<float>(radius * std::sin(start))};
        const float tx = static_cast<float>(std::sin(start)), ty = static_cast<float>(std::cos(start)), nx = ty, ny = -tx;
        triangle(r, {p.x + tx * 4.5f, p.y + ty * 4.5f}, {p.x - tx * 1.5f + nx * 4.0f, p.y - ty * 1.5f + ny * 4.0f},
            {p.x - tx * 1.5f - nx * 4.0f, p.y - ty * 1.5f - ny * 4.0f}, tint);
        break;
    }
    case Icon::Slower: rect(r, {cx - 6, cy - 1, 12, 2}, tint); break;
    case Icon::Faster: rect(r, {cx - 6, cy - 1, 12, 2}, tint); rect(r, {cx - 1, cy - 6, 2, 12}, tint); break;
    case Icon::Fit:
        for (int sx : {-1, 1}) for (int sy : {-1, 1}) {
            const int px = cx + sx * 7, py = cy + sy * 7;
            rect(r, {sx < 0 ? px : px - 5, sy < 0 ? py : py - 2, 5, 2}, tint);
            rect(r, {sx < 0 ? px : px - 2, sy < 0 ? py : py - 5, 2, 5}, tint);
        }
        break;
    case Icon::Help:
        f.draw(r, cx - f.width("?", Size::Large) / 2, cy - f.capHeight(Size::Large) / 2, "?", tint, Size::Large);
        break;
    }
}
SDL_Rect controlBar(int width, int height) { return {(width - 368) / 2, height - 72, 368, 48}; }
void controls(SDL_Renderer* r, const Fonts& f, int width, int height, const ViewState& view) {
    const auto bar = controlBar(width, height);
    rect(r, bar, overlay);
    for (int offset : {130, 278}) rect(r, {bar.x + offset, bar.y + 12, 1, 24}, divider);
    for (const auto& button : buttons(width, height)) {
        const bool primary = button.action == 0, hovered = SDL_PointInRect(&view.mouse, &button.rect);
        if (primary) rect(r, button.rect, hovered ? Color{45, 96, 82, 255} : Color{37, 78, 67, 255});
        else if (hovered || (button.action == 7 && view.help)) rect(r, button.rect, hover);
        icon(r, f, iconFor(button.action, view), button.rect, primary ? accent : ink);
    }
    char speed[16];
    std::snprintf(speed, sizeof(speed), "%g×", view.speed);
    f.draw(r, bar.x + 204 - f.width(speed, Size::Large) / 2, bar.y + (bar.h - f.capHeight(Size::Large)) / 2, speed, ink, Size::Large);
}
void help(SDL_Renderer* r, const Fonts& f, int width, int height, bool replay) {
    std::vector<std::pair<std::string, std::string>> rows{{"Space", "Play / pause"}, {"N", "Step"}, {"R", "Reset"},
        {"+ / -", "Speed"}, {"F", "Fit map"}, {"C", "Focus junction"}, {"B", "Bounding boxes"}, {"T", "Routes"},
        {"Click", "Select vehicle"}, {"Wheel", "Zoom"}, {"Right drag", "Pan"}};
    if (replay) rows.insert(rows.end(), {{"Left / Right", "Seek 1 s"}, {"Home / End", "Start / end"}});
    rows.insert(rows.end(), {{"H", "Help"}, {"Esc", "Close / quit"}});
    rect(r, {0, 0, width, height}, {0, 0, 0, 90});
    const int pad = 24, large = f.capHeight(Size::Large), row = f.capHeight() + 14;
    const int w = 320, h = 2 * pad + large + 20 + static_cast<int>(rows.size()) * row - 14;
    const SDL_Rect box{(width - w) / 2, (height - h) / 2, w, h};
    rect(r, box, overlay);
    f.draw(r, box.x + pad, box.y + pad, "Controls", ink, Size::Large);
    for (std::size_t i = 0; i < rows.size(); ++i) {
        const int y = box.y + pad + large + 20 + static_cast<int>(i) * row;
        f.draw(r, box.x + pad, y, rows[i].first, ink);
        f.draw(r, box.x + pad + 116, y, rows[i].second, muted);
    }
}
}

SDL_Rect viewport(int width, int height) { return {0, 0, width, height}; }
std::vector<Button> buttons(int width, int height) {
    const auto bar = controlBar(width, height);
    std::vector<Button> result;
    for (const auto& [offset, action] : std::array<std::pair<int, int>, 7>{{{6, 0}, {46, 1}, {86, 2}, {138, 5}, {234, 6}, {286, 3}, {326, 7}}})
        result.push_back({{bar.x + offset, bar.y + 6, 36, 36}, action});
    return result;
}
Vec2 screenToWorld(int x, int y, SDL_Rect area, Camera camera) {
    return {camera.center.x + (x - area.x - area.w / 2.0) / camera.scale,
        camera.center.y - (y - area.y - area.h / 2.0) / camera.scale};
}
void fit(Camera& camera, SDL_Rect area, MapBounds bounds) {
    camera.center = (bounds.minimum + bounds.maximum) * 0.5;
    camera.scale = std::min(area.w / (bounds.maximum.x - bounds.minimum.x + 20.0),
        area.h / (bounds.maximum.y - bounds.minimum.y + 20.0)) * 0.95;
}
void draw(SDL_Renderer* r, const Fonts& fonts, int width, int height, const Simulation& sim, const ViewState& view) {
    color(r, background); SDL_RenderClear(r);
    const auto area = viewport(width, height);
    world(r, area, view.camera, sim, view);
    if (view.selected) inspector(r, fonts, width, sim.world().vehicle(*view.selected));
    controls(r, fonts, width, height, view);
    if (view.help) help(r, fonts, width, height, view.replay);
}
void saveScreenshot(SDL_Renderer* renderer, const std::string& path) {
    int width{}, height{};
    if (SDL_GetRendererOutputSize(renderer, &width, &height) != 0)
        throw std::runtime_error(SDL_GetError());
    SDL_Surface* surface = SDL_CreateRGBSurfaceWithFormat(0, width, height, 32, SDL_PIXELFORMAT_ARGB8888);
    if (!surface) throw std::runtime_error(SDL_GetError());
    const std::filesystem::path destination(path);
    if (!destination.parent_path().empty()) std::filesystem::create_directories(destination.parent_path());
    const bool success = SDL_RenderReadPixels(renderer, nullptr, surface->format->format, surface->pixels, surface->pitch) == 0
        && SDL_SaveBMP(surface, path.c_str()) == 0;
    const std::string error = SDL_GetError();
    SDL_FreeSurface(surface);
    if (!success) throw std::runtime_error("Screenshot failed: " + error);
}
}
