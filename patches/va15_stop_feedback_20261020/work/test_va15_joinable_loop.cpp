#include "va15_joinable_loop.hpp"
#include <cassert>
#include <future>
#include <sstream>

int main()
{
    std::ostringstream quiet;
    auto* old = std::cout.rdbuf(quiet.rdbuf());
    std::atomic<int> finished{0};
    std::promise<void> entered, release;
    auto ready = release.get_future().share();
    VA15JoinableLoop loop("owner_join_test", .001f, [&] {
        entered.set_value(); ready.wait(); finished.fetch_add(1);
    });
    loop.start(); entered.get_future().wait();
    auto stopped = std::async(std::launch::async, [&] { loop.shutdown(); });
    assert(stopped.wait_for(std::chrono::milliseconds(20)) == std::future_status::timeout);
    release.set_value(); stopped.get();
    assert(finished.load() == 1);
    loop.shutdown(); // repeated shutdown is harmless
    std::atomic<int> calls{0};
    for (int trial=0; trial<100; ++trial)
    {
        VA15JoinableLoop restart("restart",.001f,[&]{calls.fetch_add(1);});
        restart.start(); restart.shutdown(); restart.start(); restart.shutdown();
    }
    const int captured=calls.load();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    assert(calls.load()==captured); // no detached callback after owner death
    {
        VA15JoinableLoop automatic("automatic",.001f,[&]{calls.fetch_add(1);});
        automatic.start();
    }
    const int after=calls.load();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    assert(calls.load()==after);
    bool refused=false;
    try {VA15JoinableLoop invalid("bad",0,[]{});} catch (const std::invalid_argument&) {refused=true;}
    assert(refused);
    std::cout.rdbuf(old);
    std::cout<<"PASS: owner waits for in-flight callback, idempotent stop, restart/destructor, no detached callbacks\n";
}
