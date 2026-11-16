// Standalone READ-ONLY observer. It has NO publisher, motion-switch client,
// policy, Z1 SDK, lowcmd or control-configuration write path.
#include "independent_model_io.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <mutex>
#include <thread>
#include <memory>
#include <csignal>
#ifdef VA15_READ_ONLY_DDS
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#endif

namespace fs=std::filesystem;
using namespace va15_independent;
static double Now(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
static volatile std::sig_atomic_t interrupted=0;
static void RequestInterrupt(int signal){interrupted=signal;}
struct InterruptGuard
{
    using Handler=void(*)(int);
    Handler previous_int=std::signal(SIGINT,RequestInterrupt),previous_term=std::signal(SIGTERM,RequestInterrupt);
    ~InterruptGuard(){std::signal(SIGINT,previous_int);std::signal(SIGTERM,previous_term);}
};

#ifdef VA15_READ_ONLY_DDS
// The callback owns its inbox, never references a stack object. Use direct DDS
// delivery: copying a single sample does not need the SDK's detached queue
// worker (whose Future::Wait is not a pthread join). All observer work and I/O
// remain on the polling thread. No publishers or control clients exist here.
class ReadOnlyFeed
{
    struct Inbox
    {
        std::mutex mutex;
        Sample latest;
        std::uint32_t tick=0;
        double received=0;
        bool ready=false,closing=false;
    };
    std::shared_ptr<Inbox> inbox=std::make_shared<Inbox>();
    bool factory_initialized=false;
    unitree::robot::ChannelPtr<unitree_go::msg::dds_::LowState_> channel;
public:
    ReadOnlyFeed()=default;
    ReadOnlyFeed(const ReadOnlyFeed&)=delete;
    ReadOnlyFeed& operator=(const ReadOnlyFeed&)=delete;
    ~ReadOnlyFeed()
    {
        try{Close();}catch(const std::exception& e){std::cerr<<"Read-only DDS cleanup failed: "<<e.what()<<'\n';}
    }
    void Start(const char* interface)
    {
        unitree::robot::ChannelFactory::Instance()->Init(0,interface);
        factory_initialized=true;
        channel=unitree::robot::ChannelFactory::Instance()->CreateRecvChannel<unitree_go::msg::dds_::LowState_>("rt/lowstate",[state_inbox=inbox](const void* p)
        {
            if(!p)return;
            const auto& state=*static_cast<const unitree_go::msg::dds_::LowState_*>(p);
            std::lock_guard<std::mutex> lock(state_inbox->mutex);
            if(state_inbox->closing||(state_inbox->ready&&state.tick()==state_inbox->tick))return;
            auto& s=state_inbox->latest;
            for(int i=0;i<12;++i){const auto& m=state.motor_state()[i];s.q[i]=m.q();s.dq[i]=m.dq();s.tau[i]=m.tau_est();s.mode[i]=m.mode();s.lost[i]=m.lost();}
            const auto& imu=state.imu_state();s.quaternion=Eigen::Quaterniond(imu.quaternion()[0],imu.quaternion()[1],imu.quaternion()[2],imu.quaternion()[3]);
            for(int i=0;i<3;++i){s.gyro[i]=imu.gyroscope()[i];s.specific_force[i]=imu.accelerometer()[i];}
            state_inbox->tick=state.tick();state_inbox->received=Now();state_inbox->ready=true;
        },0);
    }
    bool Latest(Sample& s,std::uint32_t& tick,double& received)
    {
        std::lock_guard<std::mutex> lock(inbox->mutex);
        s=inbox->latest;tick=inbox->tick;received=inbox->received;
        return inbox->ready&&!inbox->closing;
    }
    void Close()
    {
        // Mark closed under the same mutex as callbacks, then close the DDS
        // reader before releasing its factory. Shared captures also protect
        // exception unwinding; no arbitrary sleep or _Exit hides destruction.
        {std::lock_guard<std::mutex> lock(inbox->mutex);inbox->closing=true;}
        if(channel)
        {
            // SDK CloseChannel/reset alone drops its wrapper, whose embedded
            // listener can die while the native reader retains a callback ref.
            // Native close() first calls prevent_callbacks(), then unregisters
            // the listener. Keep the SDK owner alive until it has completed.
            auto native=channel->GetReader()->GetNative();
            native.close();
            native=dds::core::null;
            channel.reset();
        }
        if(factory_initialized){unitree::robot::ChannelFactory::Instance()->Release();factory_initialized=false;}
    }
};
#endif
static std::vector<std::string> Split(const std::string& text)
{std::istringstream s(text);std::vector<std::string> v;std::string p;while(std::getline(s,p,',')){if(!p.empty()&&p.back()=='\r')p.pop_back();v.push_back(p);}return v;}
static double Quantile(std::vector<double> a,double q)
{if(a.empty())return NaN;std::sort(a.begin(),a.end());return a[std::size_t((a.size()-1)*q)];}
static void Number(std::ostream& o,double v){if(std::isfinite(v))o<<v;else o<<"null";}

struct Recorder
{
    std::ofstream csv;std::string prefix;std::size_t samples=0,review_valid=0,stops=0,initialized=0;
    std::vector<double> heights,speeds,timings;std::array<unsigned,4> loads{};
    Result last;
    explicit Recorder(const std::string& p):prefix(p)
    {
        if(fs::exists(p+".csv")||fs::exists(p+".json"))throw std::runtime_error("Refusing to overwrite capture");
        if(!fs::path(p).parent_path().empty())fs::create_directories(fs::path(p).parent_path());
        csv.open(p+".csv");if(!csv)throw std::runtime_error("Cannot create output CSV");
        csv<<"elapsed,tick,input_valid,sequence_valid,fresh,initialized,review_valid,stop_review,support_count,accepted_feet,height,vx,vy,vz,force_FR,force_FL,force_RR,force_RL,load_DDS,reason\n";
    }
    void Add(double t,std::uint32_t tick,const Result& r,double us)
    {
        ++samples;review_valid+=r.estimate_valid_for_review;stops+=r.stop_candidate_for_review;initialized+=r.initialized;last=r;timings.push_back(us);
        if(r.estimate_valid_for_review){heights.push_back(r.base_height);speeds.push_back(r.planar_speed);}
        for(int i=0;i<4;++i)loads[i]+=r.load_candidate[i];
        csv<<std::setprecision(12)<<t<<','<<tick<<','<<r.input_valid<<','<<r.sequence_valid<<','<<r.fresh<<','<<r.initialized<<','
           <<r.estimate_valid_for_review<<','<<r.stop_candidate_for_review<<','<<r.support_count<<','<<r.accepted_feet<<','<<r.base_height<<','
           <<r.velocity.x()<<','<<r.velocity.y()<<','<<r.velocity.z();
        for(double f:r.normal_equivalent)csv<<','<<f;
        csv<<',';for(bool c:r.load_candidate)csv<<(c?'1':'0');csv<<','<<r.reason<<'\n';
    }
    void Finish(const Model& model,const std::string& mode)
    {
        std::ofstream out(prefix+".json");out<<std::setprecision(12)<<"{\n  \"mode\": \""<<mode<<"\",\n  \"source_sha256\": \""<<model.source_sha256
           <<"\",\n  \"samples\": "<<samples<<",\n  \"review_valid_rows\": "<<review_valid<<",\n  \"initialized_rows\": "<<initialized
           <<",\n  \"stop_review_rows\": "<<stops<<",\n  \"control_authorized\": false,\n  \"hardware_qualified\": false,\n  \"sport_required\": false,\n  \"reported_ddq_used\": false,\n  \"common_mode_slip_observable\": false,\n  \"flat_ground_assumed\": true,\n  \"height_median\": ";
        Number(out,Quantile(heights,.5));out<<",\n  \"speed_p95\": ";Number(out,Quantile(speeds,.95));
        out<<",\n  \"observer_us_p95\": ";Number(out,Quantile(timings,.95));out<<",\n  \"observer_us_max\": ";Number(out,Quantile(timings,1.));
        out<<",\n  \"load_candidate_rows_dds\": [";for(int i=0;i<4;++i){if(i)out<<',';out<<loads[i];}
        out<<"],\n  \"gyro_bias\": ["<<last.gyro_bias.x()<<','<<last.gyro_bias.y()<<','<<last.gyro_bias.z()<<"],\n  \"accel_bias\": ["
           <<last.accel_bias.x()<<','<<last.accel_bias.y()<<','<<last.accel_bias.z()<<"],\n  \"note\": \"Conditional model estimates, not certified contacts or a safe-stop acceptance. No robot command interface.\"\n}\n";
        if(!out||!csv)throw std::runtime_error("Output I/O error");
        std::cout<<"READ-ONLY "<<mode<<": samples="<<samples<<" review_valid="<<review_valid<<" stop_review="<<stops
                 <<" height_median="<<Quantile(heights,.5)<<" speed_p95="<<Quantile(speeds,.95)<<"; authority=0; "<<prefix<<".json\n";
    }
};

int main(int argc,char** argv)
{
    try
    {
        if(argc!=5&&argc!=7)throw std::runtime_error("Usage: --replay model.json samples.csv NEW_prefix OR --subscribe model.json interface NEW_prefix --seconds N (READ ONLY)");
        const std::string mode=argv[1];if(mode!="--replay"&&mode!="--subscribe")throw std::runtime_error("Only read-only replay/subscribe supported");
        InterruptGuard interrupt_guard;
        const Model model=LoadIndependentModel(argv[2]);Observer observer(model);Recorder record(argv[4]);
        if(mode=="--replay")
        {
            if(argc!=5)throw std::runtime_error("Unexpected replay options");
            std::ifstream input(argv[3]);std::string line;if(!std::getline(input,line))throw std::runtime_error("Missing CSV");
            const auto names=Split(line);std::map<std::string,std::size_t> columns;for(std::size_t i=0;i<names.size();++i)columns[names[i]]=i;
            while(!interrupted&&std::getline(input,line))
            {
                const auto values=Split(line);if(values.size()!=names.size())throw std::runtime_error("Malformed CSV");
                auto value=[&](const std::string& k){return std::stod(values.at(columns.at(k)));};Sample s;
                for(int i=0;i<12;++i)
                {const auto k=std::to_string(i);s.q[i]=value("q_"+k);s.dq[i]=value("dq_"+k);s.tau[i]=value("tau_"+k);
                 const double m=value("mode_"+k),l=value("lost_"+k);if(!std::isfinite(m)||!std::isfinite(l)||m<0||l<0||m!=std::floor(m)||l!=std::floor(l))throw std::runtime_error("Invalid mode/lost");
                 s.mode[i]=static_cast<unsigned>(m);s.lost[i]=static_cast<unsigned>(l);}
                s.quaternion=Eigen::Quaterniond(value("quat_0"),value("quat_1"),value("quat_2"),value("quat_3"));
                for(int i=0;i<3;++i){s.gyro[i]=value("gyro_"+std::to_string(i));s.specific_force[i]=value("accel_"+std::to_string(i));}
                const double t=value("elapsed"),age=value("low_age");const auto tick=static_cast<std::uint32_t>(value("low_tick"));
                const double start=Now();if(!std::isfinite(age)||age<0||age>.1)observer.Reset();else observer.Update(s,tick,t);
                record.Add(t,tick,observer.Snapshot(t),1e6*(Now()-start));
            }
        }
        else
        {
#ifdef VA15_READ_ONLY_DDS
            if(argc!=7||std::string(argv[5])!="--seconds")throw std::runtime_error("--seconds required");
            const double seconds=std::stod(argv[6]);if(!std::isfinite(seconds)||seconds<.5||seconds>60.)throw std::runtime_error("Duration must be .5 to 60 seconds");
            ReadOnlyFeed feed;feed.Start(argv[3]);
            std::cout<<"[Read-only transport] direct DDS callback; shared inbox; explicit close; authority=0"<<std::endl;
            const double start=Now();std::uint32_t last_tick=0;bool have=false;
            while(!interrupted&&Now()-start<seconds)
            {
                Sample s;std::uint32_t source_tick;double receive_time;bool available;
                available=feed.Latest(s,source_tick,receive_time);
                if(available&&(!have||source_tick!=last_tick))
                {
                    const double begin=Now();observer.Update(s,source_tick,receive_time);const auto r=observer.Snapshot(Now());
                    record.Add(receive_time-start,source_tick,r,1e6*(Now()-begin));last_tick=source_tick;have=true;
                    if(record.samples%100==0)
                        std::cout<<"[Independent review] fresh="<<r.fresh<<" initialized="<<r.initialized<<" review_valid="<<r.estimate_valid_for_review
                            <<" load_DDS="<<r.load_candidate[0]<<r.load_candidate[1]<<r.load_candidate[2]<<r.load_candidate[3]
                            <<" h="<<r.base_height<<" speed="<<r.planar_speed<<" authority=0; "<<r.reason<<std::endl;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            feed.Close();
#else
            throw std::runtime_error("This build is replay-only: no DDS was compiled");
#endif
        }
        record.Finish(model,mode);if(record.samples==0)throw std::runtime_error("No samples; no acceptance");return interrupted?128+interrupted:0;
    }
    catch(const std::exception& e){std::cerr<<"ERROR: "<<e.what()<<'\n';return 1;}
}
