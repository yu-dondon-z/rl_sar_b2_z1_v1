#pragma once
// Diagnostics only: no Unitree SDK, policy, motor or control interfaces.
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <thread>
#include <type_traits>

namespace va15_telemetry
{
inline std::uint64_t Now()
{return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
struct Source
{
    std::uint64_t copy_ns=0,received_ns=0;
    std::uint32_t tick=0;
};
struct Context
{
    std::uint64_t control_sequence=0,control_start_ns=0,command_generated_ns=0;
    int fsm=0,trial=0,stop=0,policy_running=0,inhibit=0,safety_fault=0;
    unsigned raw_keys=0,remote_valid=0;
    std::array<float,3> raw_axes{},requested{},limited{},policy_command{};
    double request_seconds=0.;
    float h_lower=0.f,v_upper=0.f,stance_offset=0.f;
    int estimated_valid=0,front_handoff=0,rear_handoff=0,arm_blend=0;
    int startup_stage=0;
    double startup_wait_s=0.,startup_elapsed_s=0.;
    float startup_pd_join_error=0.f;
};
struct Command
{
    Source source;
    Context context;
    std::uint64_t begin_ns=0,end_ns=0,previous_begin_ns=0,max_interval_ns=0,sequence=0,latest_actor_completion=0;
    unsigned write_ok=0;
    unsigned bit_flag=0,contract_ready=0,contract_latched=0,contract_reason=0;
    int contract_bad_joint=-1;
    unsigned contract_baseline_frozen=0,contract_startup_changes=0;
    double contract_stable_seconds=0.;
    std::array<unsigned,12> contract_baseline{};
    std::array<float,12> q{},dq{},tau{},cmd_q{},cmd_dq{},kp{},kd{},ff{};
    std::array<unsigned,12> mode{},lost{};
    std::array<unsigned,12> motor_fault{},motor_frequency{};
};
struct Control
{
    Source source;
    Context context;
    std::uint64_t end_ns=0,previous_start_ns=0,max_interval_ns=0,latest_actor_completion=0;
    std::array<float,18> q{},dq{},tau{},fsm_q{};
};
struct Actor
{
    Source source;
    std::uint64_t begin_ns=0,end_ns=0,previous_begin_ns=0,queued_frame=0,model_completion=0;
    unsigned model_executed=0;
    std::array<float,3> command{};
    std::array<float,70> observation{};
    std::array<float,18> raw_action{},filtered_action{},target_q{};
};

// One producer and one consumer PER queue. No allocation, mutex, filesystem
// I/O, wait, notify or spin in Push. Full queues drop telemetry, not commands.
template<class T,std::size_t N=256> class Queue
{
    static_assert(std::is_trivially_copyable<T>::value);
    std::array<T,N> data_{};
    alignas(64) std::atomic<std::size_t> head_{0};
    alignas(64) std::atomic<std::size_t> tail_{0};
public:
    std::atomic<std::uint64_t> dropped{0};
    bool Push(const T& value)
    {
        const auto head=head_.load(std::memory_order_relaxed),next=(head+1)%N;
        if(next==tail_.load(std::memory_order_acquire)){dropped.fetch_add(1,std::memory_order_relaxed);return false;}
        data_[head]=value;head_.store(next,std::memory_order_release);return true;
    }
    bool Pop(T& value)
    {
        const auto tail=tail_.load(std::memory_order_relaxed);
        if(tail==head_.load(std::memory_order_acquire))return false;
        value=data_[tail];tail_.store((tail+1)%N,std::memory_order_release);return true;
    }
};

template<class T,std::size_t N> inline void Values(std::ostream& o,const std::array<T,N>& a)
{for(const auto v:a)o<<','<<v;}
inline void Headers(std::ostream& o,const char* prefix,int count)
{for(int i=0;i<count;++i)o<<','<<prefix<<i;}
inline void ContextHeader(std::ostream& o)
{
    o<<",control_sequence,control_start_ns,command_generated_ns,fsm,trial_stage,stop_stage,policy_running,inhibit,safety_fault,raw_keys,remote_valid";
    for(const char* name:{"raw_axis_","requested_","limited_","policy_command_"})Headers(o,name,3);
    o<<",request_seconds,h_lower_m,v_upper_m_s,stance_offset,estimate_valid,front_handoff_cycles,rear_handoff_cycles,arm_blend_cycles";
    o<<",startup_stage,startup_wait_s,startup_elapsed_s,startup_pd_join_error";
}
inline void ContextValues(std::ostream& o,const Context& c)
{
    o<<','<<c.control_sequence<<','<<c.control_start_ns<<','<<c.command_generated_ns<<','<<c.fsm<<','<<c.trial<<','<<c.stop
     <<','<<c.policy_running<<','<<c.inhibit<<','<<c.safety_fault<<','<<c.raw_keys<<','<<c.remote_valid;
    Values(o,c.raw_axes);Values(o,c.requested);Values(o,c.limited);Values(o,c.policy_command);
    o<<','<<c.request_seconds<<','<<c.h_lower<<','<<c.v_upper<<','<<c.stance_offset<<','<<c.estimated_valid
     <<','<<c.front_handoff<<','<<c.rear_handoff<<','<<c.arm_blend;
    o<<','<<c.startup_stage<<','<<c.startup_wait_s<<','<<c.startup_elapsed_s<<','<<c.startup_pd_join_error;
}
inline void SourceHeader(std::ostream& o){o<<",state_copy_ns,low_callback_received_ns,low_tick";}
inline void SourceValues(std::ostream& o,const Source& s){o<<','<<s.copy_ns<<','<<s.received_ns<<','<<s.tick;}

class Recorder
{
    struct State
    {
        Queue<Command> commands;
        Queue<Control> controls;
        Queue<Actor> actors;
        std::atomic<bool> stop{false},done{false},io_failed{false};
        std::uint64_t command_rows=0,control_rows=0,actor_rows=0;
        std::filesystem::path directory;
        std::ofstream command_file,control_file,actor_file;
        explicit State(const std::filesystem::path& p):directory(p)
        {
            std::filesystem::create_directories(p);
            for(const char* name:{"commands.csv","control.csv","actor.csv","telemetry_summary.json"})
                if(std::filesystem::exists(p/name))throw std::runtime_error("Refusing telemetry overwrite: "+(p/name).string());
            command_file.open(p/"commands.csv");control_file.open(p/"control.csv");actor_file.open(p/"actor.csv");
            if(!command_file||!control_file||!actor_file)throw std::runtime_error("Cannot open diagnostic output before takeover");
            command_file<<std::setprecision(9);control_file<<std::setprecision(9);actor_file<<std::setprecision(9);
            command_file<<"publish_begin_ns,publish_end_ns,previous_publish_begin_ns,max_publish_interval_ns,command_sequence,latest_actor_completion,dds_write_return";
            SourceHeader(command_file);ContextHeader(command_file);
            for(const char* name:{"actual_q_dds_","actual_dq_dds_","actual_tau_est_dds_","sent_q_dds_","sent_dq_dds_","sent_kp_dds_","sent_kd_dds_","sent_ff_tau_dds_","mode_dds_","lost_dds_"})Headers(command_file,name,12);
            command_file<<",bit_flag,contract_ready,contract_latched,contract_reason,contract_bad_joint";
            Headers(command_file,"motor_fault_dds_",12);Headers(command_file,"motor_frequency_dds_",12);
            command_file<<",contract_baseline_frozen,contract_stable_seconds,contract_startup_changes";
            Headers(command_file,"contract_baseline_dds_",12);
            command_file<<'\n';
            control_file<<"control_end_ns,previous_control_start_ns,max_control_interval_ns,latest_actor_completion";
            SourceHeader(control_file);ContextHeader(control_file);
            for(const char* name:{"actual_q_policy_","actual_dq_policy_","actual_tau_policy_","fsm_target_q_policy_"})Headers(control_file,name,18);
            control_file<<'\n';
            actor_file<<"actor_begin_ns,actor_end_ns,previous_actor_begin_ns,queued_frame,model_completion,model_executed";
            SourceHeader(actor_file);Headers(actor_file,"policy_command_",3);Headers(actor_file,"observation_",70);
            Headers(actor_file,"raw_action_",18);Headers(actor_file,"filtered_action_",18);Headers(actor_file,"target_q_policy_",18);actor_file<<'\n';
        }
        bool Drain()
        {
            bool work=false;Command c;Control r;Actor a;
            // Bounded batches ensure an active producer cannot starve other
            // streams or shutdown. Only this background thread writes files.
            for(int n=0;n<256&&commands.Pop(c);++n)
            {
                work=true;++command_rows;
                if(io_failed.load())continue;
                command_file<<c.begin_ns<<','<<c.end_ns<<','<<c.previous_begin_ns<<','<<c.max_interval_ns<<','<<c.sequence<<','<<c.latest_actor_completion<<','<<c.write_ok;
                SourceValues(command_file,c.source);ContextValues(command_file,c.context);
                Values(command_file,c.q);Values(command_file,c.dq);Values(command_file,c.tau);Values(command_file,c.cmd_q);
                Values(command_file,c.cmd_dq);Values(command_file,c.kp);Values(command_file,c.kd);Values(command_file,c.ff);
                Values(command_file,c.mode);Values(command_file,c.lost);
                command_file<<','<<c.bit_flag<<','<<c.contract_ready<<','<<c.contract_latched<<','<<c.contract_reason<<','<<c.contract_bad_joint;
                Values(command_file,c.motor_fault);Values(command_file,c.motor_frequency);
                command_file<<','<<c.contract_baseline_frozen<<','<<c.contract_stable_seconds<<','<<c.contract_startup_changes;
                Values(command_file,c.contract_baseline);command_file<<'\n';
            }
            for(int n=0;n<256&&controls.Pop(r);++n)
            {
                work=true;++control_rows;
                if(io_failed.load())continue;
                control_file<<r.end_ns<<','<<r.previous_start_ns<<','<<r.max_interval_ns<<','<<r.latest_actor_completion;
                SourceValues(control_file,r.source);ContextValues(control_file,r.context);
                Values(control_file,r.q);Values(control_file,r.dq);Values(control_file,r.tau);Values(control_file,r.fsm_q);control_file<<'\n';
            }
            for(int n=0;n<256&&actors.Pop(a);++n)
            {
                work=true;++actor_rows;
                if(io_failed.load())continue;
                actor_file<<a.begin_ns<<','<<a.end_ns<<','<<a.previous_begin_ns<<','<<a.queued_frame<<','<<a.model_completion<<','<<a.model_executed;
                SourceValues(actor_file,a.source);Values(actor_file,a.command);Values(actor_file,a.observation);
                Values(actor_file,a.raw_action);Values(actor_file,a.filtered_action);Values(actor_file,a.target_q);actor_file<<'\n';
            }
            if(!command_file||!control_file||!actor_file)io_failed.store(true);
            return work;
        }
        void Run() noexcept
        {
            try
            {
                auto last=Now();
                while(!stop.load(std::memory_order_acquire))
                {
                    const bool work=Drain();
                    if(Now()-last>1000000000ull){command_file.flush();control_file.flush();actor_file.flush();last=Now();}
                    if(!work)std::this_thread::sleep_for(std::chrono::milliseconds(2));
                }
                // Caller stops all three producer loops before Close().
                while(Drain()){}
                command_file.flush();control_file.flush();actor_file.flush();
                if(!command_file||!control_file||!actor_file)io_failed.store(true);
                std::ofstream summary(directory/"telemetry_summary.json");
                summary<<"{\"schema\":\"va15_diagnostic_v1\",\"command_rows\":"<<command_rows
                    <<",\"control_rows\":"<<control_rows<<",\"actor_rows\":"<<actor_rows
                    <<",\"commands_dropped\":"<<commands.dropped.load()<<",\"control_dropped\":"<<controls.dropped.load()
                    <<",\"actor_dropped\":"<<actors.dropped.load()<<",\"io_failed\":"<<(io_failed.load()?"true":"false")
                    <<",\"hardware_qualified\":false,\"dds_write_is_motor_ack\":false}\n";
                if(!summary)io_failed.store(true);
            }
            catch(...){io_failed.store(true);}
            done.store(true,std::memory_order_release);
        }
    };
    std::shared_ptr<State> state_;
    std::thread worker_;
public:
    explicit Recorder(const std::filesystem::path& p):state_(std::make_shared<State>(p)),worker_([s=state_]{s->Run();}){}
    Recorder(const Recorder&)=delete;
    Recorder& operator=(const Recorder&)=delete;
    ~Recorder(){Close();}
    bool Push(const Command& c){return state_->commands.Push(c);}
    bool Push(const Control& c){return state_->controls.Push(c);}
    bool Push(const Actor& c){return state_->actors.Push(c);}
    bool Close()
    {
        state_->stop.store(true,std::memory_order_release);
        if(worker_.joinable())
        {
            const auto deadline=Now()+250000000ull;
            while(!state_->done.load(std::memory_order_acquire)&&Now()<deadline)
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            if(state_->done.load(std::memory_order_acquire))worker_.join();
            else worker_.detach(); // shared State owns I/O; never keeps robot/this alive
        }
        return state_->done.load(std::memory_order_acquire)&&!state_->io_failed.load();
    }
};
}
