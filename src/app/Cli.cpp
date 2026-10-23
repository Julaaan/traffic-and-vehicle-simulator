#include "Options.hpp"
#include "Output.hpp"
#include "vsim/Replay.hpp"
#include <chrono>
#include <iostream>
#include <stdexcept>

int main(int argc, char** argv) {
    try {
        const auto options = vsim::app::parseOptions(argc, argv);
        if (options.help) { vsim::app::printHelp(false); return 0; }
        if (options.auto_exit || !options.screenshot.empty())
            throw std::invalid_argument("--auto-exit and --screenshot are viewer options");
        if (!options.replay.empty()) {
            vsim::ReplayPlayer player(options.replay);
            const auto start=std::chrono::steady_clock::now();
            if (options.verify_replay) player.verify();
            else {
                player.seek(options.seek.value_or(player.firstTick()));
                const auto count=options.steps || options.duration ? vsim::app::runSteps(options,player.simulation().scenario()) : player.lastTick()-player.simulation().clock().tick();
                vsim::app::CsvOutput csv(options.csv); csv.sample(player.simulation());
                auto sampled_tick=player.simulation().clock().tick();
                for (std::uint64_t i=0; i<count && player.advance(); ++i) {
                    if (player.simulation().clock().tick()%options.sample_every==0 || i+1==count) {
                        csv.sample(player.simulation()); sampled_tick=player.simulation().clock().tick();
                    }
                }
                if (sampled_tick!=player.simulation().clock().tick()) csv.sample(player.simulation());
                csv.flush();
            }
            const double wall=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
            vsim::app::writeReport(options.report,player.simulation(),wall); vsim::app::printSummary(player.simulation(),wall);
            std::cout<<"Replay "<<(options.verify_replay ? "verified" : "played")<<" through tick "<<player.simulation().clock().tick()<<'\n'; return 0;
        }
        vsim::Simulation sim(vsim::app::loadInput(options));
        vsim::RunRecorder recorder(sim,options.record,options.failure_dir);
        const auto steps = vsim::app::runSteps(options, sim.scenario());
        vsim::app::CsvOutput csv(options.csv);
        csv.sample(sim);
        const auto start = std::chrono::steady_clock::now();
        try {
            for (std::uint64_t i = 0; i < steps; ++i) {
                sim.step(); recorder.capture(sim);
                if (sim.clock().tick() % options.sample_every == 0 || i + 1 == steps) csv.sample(sim);
            }
        } catch (const std::exception& error) { recorder.failure(error.what()); throw; }
        recorder.finish(sim);
        csv.flush();
        const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        vsim::app::writeReport(options.report, sim, wall);
        vsim::app::printSummary(sim, wall);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
}
