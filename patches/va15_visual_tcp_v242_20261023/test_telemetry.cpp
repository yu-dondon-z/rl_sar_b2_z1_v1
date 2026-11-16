#include "production/trial_telemetry.hpp"
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <vector>

static std::size_t Columns(const std::string& s)
{std::size_t n=1;for(char c:s)n+=c==',';return n;}
int main(int argc,char** argv)
{
    using namespace va15_telemetry;
    Queue<unsigned,4> q;
    assert(q.Push(1)&&q.Push(2)&&q.Push(3)&&!q.Push(4)&&q.dropped.load()==1);
    unsigned v=0;assert(q.Pop(v)&&v==1);assert(q.Push(5));
    for(unsigned wanted:{2u,3u,5u})assert(q.Pop(v)&&v==wanted);
    assert(!q.Pop(v));
    std::string directory;
    if(argc==2)directory=argv[1];
    else {char path[]="/tmp/va15_telemetry_test_XXXXXX";const auto p=mkdtemp(path);assert(p);directory=p;}
    Recorder recorder(directory);
    bool rejected=false;try {Recorder collision(directory);}catch(const std::exception&){rejected=true;}assert(rejected);
    std::thread commands([&]{for(unsigned i=1;i<=200;++i)
    {
        Command r;r.begin_ns=Now();r.previous_begin_ns=r.begin_ns-2000000;r.end_ns=r.begin_ns+1000;
        r.max_interval_ns=2000000;r.sequence=i;r.latest_actor_completion=i;r.write_ok=1;
        r.source={r.begin_ns-500,r.begin_ns-1000,i*20};
        r.context.control_sequence=i;r.context.control_start_ns=r.begin_ns-400;
        r.context.command_generated_ns=r.begin_ns-200;r.context.fsm=2;r.context.trial=3;r.context.policy_running=1;
        r.context.raw_axes={0.f,.2f,0.f};r.context.requested={.07f,0,0};r.context.limited={.07f,0,0};
        r.context.policy_command={.07f,0,0};r.context.remote_valid=1;r.context.front_handoff=300-i;
        r.context.request_seconds=.005*i;
        for(int j=0;j<12;++j){r.q[j]=-.5f;r.cmd_q[j]=-.4f;r.kp[j]=160.f;r.kd[j]=5.f;r.tau[j]=16.f;r.mode[j]=10;}
        recorder.Push(r);std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }});
    std::thread controls([&]{for(unsigned i=1;i<=200;++i)
    {Control r;r.end_ns=Now();r.context.control_start_ns=r.end_ns-100000;r.context.control_sequence=i;
     r.previous_start_ns=r.context.control_start_ns-5000000;r.max_interval_ns=5000000;
     r.context.fsm=2;r.context.trial=3;r.context.front_handoff=300-i;r.context.arm_blend=400-i;
     r.context.raw_axes={0.f,.2f,0.f};r.context.limited={.07f,0,0};r.context.policy_command={.07f,0,0};
     recorder.Push(r);std::this_thread::sleep_for(std::chrono::milliseconds(1));}});
    std::thread actors([&]{for(unsigned i=1;i<=200;++i)
    {Actor r;r.begin_ns=Now();r.end_ns=r.begin_ns+50000;r.previous_begin_ns=r.begin_ns-20000000;
     r.model_completion=r.queued_frame=i;r.model_executed=1;r.command={.07f,0,0};
     recorder.Push(r);std::this_thread::sleep_for(std::chrono::milliseconds(1));}});
    commands.join();controls.join();actors.join();
    assert(recorder.Close());
    for(const char* name:{"commands.csv","control.csv","actor.csv"})
    {
        std::ifstream f(std::filesystem::path(directory)/name);std::string line;assert(std::getline(f,line));
        const auto columns=Columns(line);std::size_t rows=0;
        while(std::getline(f,line)){assert(Columns(line)==columns);++rows;}
        assert(rows==200);
    }
    std::ifstream summary(std::filesystem::path(directory)/"telemetry_summary.json");
    const std::string text((std::istreambuf_iterator<char>(summary)),std::istreambuf_iterator<char>());
    assert(text.find("\"io_failed\":false")!=std::string::npos);
    assert(text.find("\"commands_dropped\":0")!=std::string::npos);
    assert(text.find("\"dds_write_is_motor_ack\":false")!=std::string::npos);
    std::cout<<"PASS bounded queue/drop/ordering, concurrent streams, complete CSV, overwrite refusal, bounded close; "<<directory<<'\n';
}
