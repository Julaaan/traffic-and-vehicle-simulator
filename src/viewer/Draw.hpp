#pragma once
#include "Font.hpp"
#include "vsim/Simulation.hpp"
#include <SDL.h>
#include <optional>
#include <string>
#include <vector>

namespace vsim::viewer {
struct Camera { Vec2 center{}; double scale{5.0}; };
enum class Action { PlayPause, Step, Reset, Fit, FocusJunction, Slower, Faster, Help };
struct Button { SDL_Rect rect; Action action{}; };
struct ViewState {
    bool paused{};
    bool boxes{};
    bool routes{};
    bool help{};
    SDL_Point mouse{-1, -1};
    double speed{1.0};
    bool finished{};
    bool replay{};
    std::optional<EntityId> selected;
    Camera camera;
};
SDL_Rect viewport(int width, int height);
std::vector<Button> buttons(int width, int height);
Vec2 screenToWorld(int x, int y, SDL_Rect area, Camera camera);
void fit(Camera& camera, SDL_Rect area, MapBounds bounds);
void draw(SDL_Renderer* renderer, const Fonts& fonts, int width, int height, const Simulation& sim, const ViewState& view);
void saveScreenshot(SDL_Renderer* renderer, const std::string& path);
}
