#include "production/coherent_snapshot.hpp"
#include <atomic>
#include <cassert>
#include <iostream>
#include <thread>

int main()
{
    std::mutex mutex;
    std::atomic<std::uint64_t> clock{1000000000ULL};
    std::atomic<bool> waiting{false};
    std::uint64_t source=clock.load(),evaluated=0;
    const auto control_begin=clock.load();
    bool fresh=false;
    mutex.lock(); // receive callback owns the source lock
    std::thread control([&]{
        waiting=true;
        VA15ReadCoherentSnapshot(mutex,[&]{return clock.load();},[&](std::uint64_t now){
            evaluated=now;fresh=now>=source&&now-source<=100000000ULL;
        });
    });
    while(!waiting.load())std::this_thread::yield();
    source+=5000000ULL;clock=source+1000; // callback arrives after control began
    mutex.unlock();control.join();
    assert(source>control_begin); // archived control-begin check rejects this
    assert(fresh&&evaluated>=source);
    clock=source+100000001ULL;
    VA15ReadCoherentSnapshot(mutex,[&]{return clock.load();},[&](std::uint64_t now){
        fresh=now>=source&&now-source<=100000000ULL;
    });
    assert(!fresh); // genuine stale data is NOT relaxed
    source=clock.load()+1;
    VA15ReadCoherentSnapshot(mutex,[&]{return clock.load();},[&](std::uint64_t now){
        fresh=now>=source&&now-source<=100000000ULL;
    });
    assert(!fresh); // genuinely future/reversed source is still rejected
    std::cout<<"PASS callback/control timestamp interleaving; clock sampled after lock; actual stale/future still rejected\n";
}
