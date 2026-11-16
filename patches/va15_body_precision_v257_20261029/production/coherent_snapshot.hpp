#pragma once
#include <cstdint>
#include <mutex>

// A receive callback may advance its timestamp while control waits for this
// mutex. Sampling the evaluation clock BEFORE acquiring the mutex makes a
// newer, perfectly fresh sample appear to come from the future. Never fix
// this by accepting future timestamps: read the clock AFTER locking instead.
template<class Clock,class Reader>
std::uint64_t VA15ReadCoherentSnapshot(std::mutex& mutex,Clock clock,Reader reader)
{
    std::lock_guard<std::mutex> lock(mutex);
    const std::uint64_t now=clock();
    reader(now);
    return now;
}
