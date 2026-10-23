#include "Draw.hpp"
#include "Options.hpp"
#include "Output.hpp"
#include "vsim/Replay.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <limits>
#include <stdexcept>

namespace {
struct SdlSession {
    SdlSession() {
        SDL_SetMainReady();
        if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) throw std::runtime_error(SDL_GetError());
    }
    ~SdlSession() { SDL_Quit(); }
};
bool contains(SDL_Rect r, int x, int y) { return x >= r.x && y >= r.y && x < r.x + r.w && y < r.y + r.h; }
}

int main(int argc, char** argv) {
    try {
        const auto options = vsim::app::parseOptions(argc, argv);
        if (options.help) { vsim::app::printHelp(true); return 0; }
        if (options.verify_replay) throw std::invalid_argument("Use sim_cli for --verify-replay");
        std::unique_ptr<vsim::ReplayPlayer> playback;
        std::unique_ptr<vsim::Simulation> live;
        if (!options.replay.empty()) playback=std::make_unique<vsim::ReplayPlayer>(options.replay);
        else live=std::make_unique<vsim::Simulation>(vsim::app::loadInput(options));
        auto& sim=playback ? playback->simulation() : *live;
        if (playback) playback->seek(options.seek.value_or(playback->firstTick()));
        const auto start_tick=sim.clock().tick();
        const auto limit=playback ? (options.steps || options.duration
            ? std::min(playback->lastTick(),start_tick+vsim::app::runSteps(options,sim.scenario())) : playback->lastTick())
            : vsim::app::runSteps(options,sim.scenario());
        auto recorder=std::make_unique<vsim::RunRecorder>(sim,options.record,options.failure_dir);
        vsim::app::CsvOutput csv(options.csv);
        csv.sample(sim);
        SdlSession session;
        std::unique_ptr<SDL_Window, decltype(&SDL_DestroyWindow)> window(SDL_CreateWindow(
            "Vehicle Simulator", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1440, 900,
            SDL_WINDOW_RESIZABLE), SDL_DestroyWindow);
        if (!window) throw std::runtime_error(SDL_GetError());
        SDL_SetWindowMinimumSize(window.get(), 800, 520);
        SDL_Renderer* raw = SDL_CreateRenderer(window.get(), -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
        if (!raw) raw = SDL_CreateRenderer(window.get(), -1, SDL_RENDERER_SOFTWARE);
        std::unique_ptr<SDL_Renderer, decltype(&SDL_DestroyRenderer)> renderer(raw, SDL_DestroyRenderer);
        if (!renderer) throw std::runtime_error(SDL_GetError());
        SDL_SetRenderDrawBlendMode(renderer.get(), SDL_BLENDMODE_BLEND);
        const vsim::viewer::Fonts fonts(renderer.get());
        vsim::viewer::ViewState view;
        view.replay=static_cast<bool>(playback);
        int width = 1440, height = 900;
        if (sim.world().road().imported) vsim::viewer::fit(view.camera, vsim::viewer::viewport(width, height), sim.world().road().bounds());
        bool running = true;
        double accumulator = 0.0;
        auto previous = std::chrono::steady_clock::now();
        const auto started = previous;
        auto reset = [&] {
            recorder->finish(sim);
            if (playback) playback->seek(start_tick); else sim.reset();
            recorder=std::make_unique<vsim::RunRecorder>(sim,options.record,options.failure_dir);
            view.finished = false; accumulator = 0.0;
            csv = vsim::app::CsvOutput({});
            csv = vsim::app::CsvOutput(options.csv); csv.sample(sim);
        };
        auto step = [&] {
            if (sim.clock().tick() >= limit) { view.finished = true; view.paused = true; return; }
            try {
                if (playback) playback->advance();
                else { sim.step(); recorder->capture(sim); }
            } catch (const std::exception& error) { recorder->failure(error.what()); throw; }
            if (sim.clock().tick() % options.sample_every == 0 || sim.clock().tick() == limit) csv.sample(sim);
            if (sim.clock().tick() >= limit) { view.finished = true; view.paused = true; }
        };
        auto seek = [&](std::uint64_t tick) {
            if (!playback) return;
            playback->seek(std::clamp(tick,playback->firstTick(),limit));
            view.paused=true; view.finished=sim.clock().tick()>=limit; accumulator=0;
            csv=vsim::app::CsvOutput({});
            csv=vsim::app::CsvOutput(options.csv); csv.sample(sim);
        };
        auto action = [&](int id) {
            if (id == 0) { if (view.finished) reset(); view.paused = !view.paused; accumulator = 0.0; }
            else if (id == 1) { view.paused = true; accumulator = 0.0; step(); }
            else if (id == 2) reset();
            else if (id == 3) vsim::viewer::fit(view.camera, vsim::viewer::viewport(width, height), sim.world().road().bounds());
            else if (id == 4) {
                const auto bounds = sim.world().road().bounds();
                view.camera.center = (bounds.minimum + bounds.maximum) * 0.5;
                double nearest = std::numeric_limits<double>::infinity();
                for (const auto& junction : sim.world().road().junctions) {
                    const double distance = vsim::norm(junction.position - (bounds.minimum + bounds.maximum) * 0.5);
                    if (junction.connectors.size() >= 3 && distance < nearest) { nearest = distance; view.camera.center = junction.position; }
                }
                view.camera.scale = 5.0;
            }
            else if (id == 5) view.speed = std::max(0.25, view.speed / 2.0);
            else if (id == 6) view.speed = std::min(16.0, view.speed * 2.0);
            else if (id == 7) view.help = !view.help;
        };
        while (running) {
            const auto now = std::chrono::steady_clock::now();
            const double elapsed = std::chrono::duration<double>(now - previous).count();
            previous = now;
            SDL_Event event;
            while (SDL_PollEvent(&event)) {
                if (event.type == SDL_QUIT) running = false;
                else if (event.type == SDL_KEYDOWN && event.key.repeat == 0) {
                    switch (event.key.keysym.sym) {
                    case SDLK_ESCAPE: if (view.help) view.help = false; else running = false; break;
                    case SDLK_h: action(7); break;
                    case SDLK_SPACE: action(0); break;
                    case SDLK_n: action(1); break;
                    case SDLK_r: action(2); break;
                    case SDLK_f: action(3); break;
                    case SDLK_c: action(4); break;
                    case SDLK_MINUS: case SDLK_KP_MINUS: action(5); break;
                    case SDLK_PLUS: case SDLK_EQUALS: case SDLK_KP_PLUS: action(6); break;
                    case SDLK_b: view.boxes = !view.boxes; break;
                    case SDLK_t: view.routes = !view.routes; break;
                    case SDLK_LEFT: {
                        const auto amount=static_cast<std::uint64_t>(std::round(1.0/sim.clock().timeStep()));
                        seek(sim.clock().tick()>amount ? sim.clock().tick()-amount : 0); break;
                    }
                    case SDLK_RIGHT: seek(sim.clock().tick()+static_cast<std::uint64_t>(std::round(1.0/sim.clock().timeStep()))); break;
                    case SDLK_HOME: if (playback) seek(playback->firstTick()); break;
                    case SDLK_END: seek(limit); break;
                    default: break;
                    }
                } else if (event.type == SDL_MOUSEWHEEL) {
                    int mx{}, my{}; SDL_GetMouseState(&mx, &my);
                    const auto area = vsim::viewer::viewport(width, height);
                    if (contains(area, mx, my)) {
                        const auto before = vsim::viewer::screenToWorld(mx, my, area, view.camera);
                        view.camera.scale = std::clamp(view.camera.scale * std::pow(1.15, event.wheel.y), 0.001, 40.0);
                        const auto after = vsim::viewer::screenToWorld(mx, my, area, view.camera);
                        view.camera.center = view.camera.center + before - after;
                    }
                } else if (event.type == SDL_MOUSEMOTION) {
                    view.mouse = {event.motion.x, event.motion.y};
                    if (event.motion.state & SDL_BUTTON_RMASK) {
                        view.camera.center.x -= event.motion.xrel / view.camera.scale;
                        view.camera.center.y += event.motion.yrel / view.camera.scale;
                    }
                } else if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_LEAVE) {
                    view.mouse = {-1, -1};
                } else if (event.type == SDL_MOUSEBUTTONDOWN && event.button.button == SDL_BUTTON_LEFT) {
                    bool pressed = false;
                    for (const auto& button : vsim::viewer::buttons(width, height))
                        if (contains(button.rect, event.button.x, event.button.y)) { action(button.action); pressed = true; }
                    const auto area = vsim::viewer::viewport(width, height);
                    if (!pressed && contains(area, event.button.x, event.button.y)) {
                        const auto p = vsim::viewer::screenToWorld(event.button.x, event.button.y, area, view.camera);
                        view.selected.reset();
                        double nearest = std::max(3.0, 12.0 / view.camera.scale);
                        for (const auto& vehicle : sim.world().vehicles()) {
                            const double distance = vsim::norm(vehicle.state.position() - p);
                            if (vehicle.active && distance < nearest) { nearest = distance; view.selected = vehicle.id; }
                        }
                    }
                }
            }
            SDL_GetWindowSize(window.get(), &width, &height);
            if (!view.paused) {
                accumulator += std::min(elapsed, 0.25) * view.speed;
                int budget = 400;
                while (accumulator >= sim.clock().timeStep() && budget-- > 0 && !view.finished) {
                    step(); accumulator -= sim.clock().timeStep();
                }
            }
            if (limit == 0) { view.finished = true; view.paused = true; }
            vsim::viewer::draw(renderer.get(), fonts, width, height, sim, view);
            if (!running || (view.finished && options.auto_exit)) {
                if (!options.screenshot.empty()) vsim::viewer::saveScreenshot(renderer.get(), options.screenshot.string());
                running = false;
            }
            SDL_RenderPresent(renderer.get());
            SDL_Delay(1);
        }
        if (sim.clock().tick() % options.sample_every != 0 && sim.clock().tick() != limit) csv.sample(sim);
        csv.flush();
        recorder->finish(sim);
        const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
        vsim::app::writeReport(options.report, sim, wall);
        vsim::app::printSummary(sim, wall);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
}
