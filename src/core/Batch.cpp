#include "vsim/Batch.hpp"
#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <exception>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

namespace vsim {
class SimulationBatch::Pool {
public:
    explicit Pool(unsigned workers) {
        for (unsigned worker = 1; worker < workers; ++worker) threads_.emplace_back([this, worker] { loop(worker); });
    }
    ~Pool() {
        { std::lock_guard lock(mutex_); stop_ = true; }
        wake_.notify_all();
        for (auto& thread : threads_) thread.join();
    }
    unsigned size() const { return static_cast<unsigned>(threads_.size()) + 1; }
    void run(const std::function<void(unsigned)>& task) {
        {
            std::lock_guard lock(mutex_);
            task_ = &task; pending_ = static_cast<unsigned>(threads_.size()); error_ = nullptr; ++generation_;
        }
        wake_.notify_all();
        execute(0);
        std::unique_lock lock(mutex_);
        done_.wait(lock, [this] { return pending_ == 0; });
        task_ = nullptr;
        if (error_) std::rethrow_exception(std::exchange(error_, nullptr));
    }
private:
    void execute(unsigned worker) {
        try { (*task_)(worker); }
        catch (...) { std::lock_guard lock(mutex_); if (!error_) error_ = std::current_exception(); }
    }
    void loop(unsigned worker) {
        std::uint64_t seen = 0;
        for (;;) {
            {
                std::unique_lock lock(mutex_);
                wake_.wait(lock, [&] { return stop_ || generation_ != seen; });
                if (stop_) return;
                seen = generation_;
            }
            execute(worker);
            std::lock_guard lock(mutex_);
            if (--pending_ == 0) done_.notify_one();
        }
    }
    std::vector<std::thread> threads_;
    std::mutex mutex_;
    std::condition_variable wake_, done_;
    const std::function<void(unsigned)>* task_{};
    std::uint64_t generation_{};
    unsigned pending_{};
    bool stop_{};
    std::exception_ptr error_;
};

SimulationBatch::SimulationBatch(std::vector<Scenario> scenarios, unsigned threads) {
    if (scenarios.empty()) throw std::invalid_argument("A batch needs at least one scenario");
    simulations_.reserve(scenarios.size());
    for (auto& scenario : scenarios) simulations_.emplace_back(std::move(scenario));
    if (threads == 0) threads = std::max(1U, std::thread::hardware_concurrency());
    pool_ = std::make_unique<Pool>(static_cast<unsigned>(std::min<std::size_t>(threads, simulations_.size())));
}
SimulationBatch::~SimulationBatch() = default;
unsigned SimulationBatch::threads() const { return pool_->size(); }
std::size_t SimulationBatch::vehicles() const {
    std::size_t total = 0;
    for (const auto& simulation : simulations_) total += simulation.world().vehicles().size();
    return total;
}
void SimulationBatch::step(std::uint64_t steps, StepProfile* profile) {
    std::atomic<std::size_t> next{0};
    std::mutex merge;
    pool_->run([&](unsigned) {
        StepProfile local;
        for (std::size_t i; (i = next.fetch_add(1, std::memory_order_relaxed)) < simulations_.size();)
            for (std::uint64_t s = 0; s < steps; ++s) simulations_[i].step({}, profile ? &local : nullptr);
        if (profile) { std::lock_guard lock(merge); *profile += local; }
    });
}
std::uint64_t SimulationBatch::stateHash() const {
    std::uint64_t hash = 14695981039346656037ULL;
    for (const auto& simulation : simulations_) {
        const std::uint64_t value = simulation.stateHash();
        for (unsigned byte = 0; byte < 8; ++byte) {
            hash ^= (value >> (byte * 8U)) & 0xffU;
            hash *= 1099511628211ULL;
        }
    }
    return hash;
}
}
