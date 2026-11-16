#pragma once

#include "logger.hpp"
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

// Real-B2-only replacement for the archived detached LoopFunc. The callback
// must return; shutdown joins it before its referenced robot state is freed.
// No exception swallowing: retain the existing callback failure semantics.
class VA15JoinableLoop
{
public:
    VA15JoinableLoop(std::string name, float period, std::function<void()> callback)
        : name_(std::move(name)), period_(period), callback_(std::move(callback))
    {
        if (!std::isfinite(period_) || period_ <= 0.f || !callback_)
            throw std::invalid_argument("Invalid real loop period/callback");
    }
    ~VA15JoinableLoop() { shutdown(); }
    VA15JoinableLoop(const VA15JoinableLoop&) = delete;
    VA15JoinableLoop& operator=(const VA15JoinableLoop&) = delete;

    void start()
    {
        std::lock_guard<std::mutex> lifecycle(lifecycle_);
        if (thread_.joinable()) throw std::logic_error("Loop is already started");
        running_.store(true);
        try { thread_ = std::thread([this] { run(); }); }
        catch (...) { running_.store(false); throw; }
        std::cout << LOGGER::INFO << "[Loop] Loop start - name: " << name_
                  << ", period: " << period_ * 1000.f << "ms (joinable)" << std::endl;
    }
    void shutdown()
    {
        std::lock_guard<std::mutex> lifecycle(lifecycle_);
        if (!thread_.joinable()) return;
        if (thread_.get_id() == std::this_thread::get_id())
            throw std::logic_error("Real loop must be stopped by its owner thread");
        {
            std::lock_guard<std::mutex> wait(wait_mutex_);
            running_.store(false);
        }
        wake_.notify_all();
        thread_.join();
        std::cout << LOGGER::INFO << "[Loop] Loop end - name: " << name_ << std::endl;
    }
private:
    void run()
    {
        while (running_.load())
        {
            const auto started = std::chrono::steady_clock::now();
            callback_();
            const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - started);
            // Preserve the archived loop's period calculation; change only
            // thread ownership/teardown, not the controller's time base.
            const auto sleep = std::chrono::milliseconds(static_cast<int>(period_ * 1000.f - elapsed.count()));
            if (sleep.count() > 0)
            {
                std::unique_lock<std::mutex> wait(wait_mutex_);
                wake_.wait_for(wait, sleep, [this] { return !running_.load(); });
            }
        }
    }
    std::string name_;
    float period_;
    std::function<void()> callback_;
    std::atomic<bool> running_{false};
    std::mutex lifecycle_, wait_mutex_;
    std::condition_variable wake_;
    std::thread thread_;
};
