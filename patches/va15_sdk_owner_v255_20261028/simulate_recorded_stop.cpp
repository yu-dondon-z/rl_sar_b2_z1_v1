// OFFLINE ONLY. Compare the exact V251 and V252 stop controllers on one
// logged/generated planted stance in MuJoCo. The pose is set ONCE, before
// trial t=0; no pose, velocity, contacts or forces are rewritten during the
// trial. Physics truth is explicitly separate from unchanged model evidence.
#ifdef VA15_STOP_BASELINE
#include "../va15_curve_entry_v251_20261026/production/fsm_b2_z1_no_gun.hpp"
#include "../va15_curve_entry_v251_20261026/production/independent_model_io.hpp"
#else
#include "production/fsm_b2_z1_no_gun.hpp"
#include "production/independent_model_io.hpp"
#endif
#define main archived_simulation_main_not_called
#include "../va15_controlled_getdown_20261020/simulate_controlled_getdown.cpp"
#undef main
#include <sstream>
#include <map>
#include <numeric>

namespace {
struct RecordedPose {
    std::vector<float> q,command;
    std::array<double,4> quaternion{{1.,0.,0.,0.}};
    double height=.52;
    unsigned source_row=0;
};
std::vector<std::string> Split(const std::string& text) {
    std::stringstream input(text);std::string item;std::vector<std::string> out;
    while(std::getline(input,item,','))out.push_back(item);
    return out;
}
RecordedPose LoadPose(const std::string& file,const std::string& scenario) {
    std::ifstream input(file);if(!input)throw std::runtime_error("Cannot open recorded control.csv");
    std::string line;std::getline(input,line);const auto names=Split(line);
    std::map<std::string,unsigned> index;
    for(unsigned i=0;i<names.size();++i)index[names[i]]=i;
    unsigned row=1;
    while(std::getline(input,line)) {
        ++row;
        if(line.find('\0')!=std::string::npos)continue;
        const auto values=Split(line);if(values.size()!=names.size())continue;
        auto value=[&](const std::string& key){return std::stod(values.at(index.at(key)));};
        if(value("stop_stage")!=(scenario=="recorded_capture"?2.:3.))continue;
        if(value("stop_source_valid")!=1.||value("safety_fault")!=0.)continue;
        RecordedPose result;result.source_row=row;result.height=value("h_lower_m");
        for(unsigned i=0;i<18;++i) {
            result.q.push_back(value("actual_q_policy_"+std::to_string(i)));
            result.command.push_back(value("fsm_target_q_policy_"+std::to_string(i)));
        }
        for(unsigned i=0;i<4;++i)result.quaternion[i]=value("body_quaternion_wxyz_"+std::to_string(i));
        const double norm=std::inner_product(result.quaternion.begin(),result.quaternion.end(),result.quaternion.begin(),0.);
        if(!std::isfinite(result.height)||result.height<.45||result.height>.65||std::fabs(norm-1.)>.05)
            throw std::runtime_error("Invalid recorded initial pose");
        for(float q:result.q)if(!std::isfinite(q))throw std::runtime_error("Nonfinite recorded joint pose");
        for(float q:result.command)if(!std::isfinite(q))throw std::runtime_error("Nonfinite recorded command");
        return result;
    }
    throw std::runtime_error("No complete source-valid recorded stop row");
}
unsigned Bits(const std::array<bool,4>& contacts) {
    unsigned value=0;for(unsigned i=0;i<4;++i)if(contacts[i])value|=1u<<i;return value;
}
std::string Number(double value) {
    if(!std::isfinite(value))return "null";
    std::ostringstream out;out<<std::setprecision(10)<<value;return out.str();
}
struct ObserverInput {
    va15_independent::Observer observer;
    std::array<int,12> qids{},vids{};
    int imu=-1;
    std::array<double,12> qdd{};
    va15_independent::Sample previous;
    va15_independent::V3 alpha=va15_independent::V3::Zero();
    bool have=false;double previous_time=0.;
    std::array<double,4> lateral{{NAN,NAN,NAN,NAN}},lateral_limit{{NAN,NAN,NAN,NAN}};
    std::array<bool,4> instantaneous_credible{};
    ObserverInput(SimRL& rl,const std::string& model):observer(LoadIndependentModel(model),true) {
        const char* legs[]={"FR","FL","RR","RL"};const char* joints[]={"hip","thigh","calf"};
        for(int i=0;i<4;++i)for(int j=0;j<3;++j) {
            const std::string name=std::string(legs[i])+"_"+joints[j]+"_joint";
            const int id=mj_name2id(rl.m,mjOBJ_JOINT,name.c_str());if(id<0)throw std::runtime_error("Missing joint");
            qids[3*i+j]=rl.m->jnt_qposadr[id];vids[3*i+j]=rl.m->jnt_dofadr[id];
        }
        imu=mj_name2id(rl.m,mjOBJ_SENSOR,"imu_acc");if(imu<0)throw std::runtime_error("Missing IMU sensor");
    }
    void Update(SimRL& rl) {
        const double now=rl.d->time;
        va15_independent::Sample sample;sample.mode.fill(10);sample.documented_motor_health=true;
        sample.quaternion=Eigen::Quaterniond(rl.d->qpos[3],rl.d->qpos[4],rl.d->qpos[5],rl.d->qpos[6]).normalized();
        for(unsigned i=0;i<12;++i) {
            sample.q[i]=rl.d->qpos[qids[i]];sample.dq[i]=rl.d->qvel[vids[i]];
            sample.tau[i]=rl.d->qfrc_actuator[vids[i]];
        }
        for(unsigned i=0;i<3;++i) {
            sample.gyro[i]=rl.robot_state.imu.gyroscope[i];
            sample.specific_force[i]=rl.d->sensordata[rl.m->sensor_adr[imu]+i];
        }
        observer.Update(sample,std::lround(now*1000.),now);
        // Diagnostic-only reconstruction of the observer's instantaneous
        // force-credibility check. It NEVER substitutes for model contacts.
        if(have&&now>previous_time&&now-previous_time<=.05) {
            const double dt=now-previous_time,a=1.-std::exp(-dt/.025);
            for(unsigned i=0;i<12;++i)qdd[i]+=a*((sample.dq[i]-previous.dq[i])/dt-qdd[i]);
            alpha+=a*((sample.gyro-previous.gyro)/dt-alpha);
            const auto result=observer.Snapshot(now);
            for(unsigned i=0;i<4;++i)try {
                const auto f=observer.dynamics.Evaluate(i,sample,qdd,alpha,result.gyro_bias,result.accel_bias).force_world;
                lateral[i]=f.head<2>().norm();lateral_limit[i]=std::max(30.,.8*std::max(0.,f.z()));
                instantaneous_credible[i]=std::fabs(f.z())<1500.&&lateral[i]<lateral_limit[i];
            }catch(const std::exception&) {
                lateral[i]=lateral_limit[i]=NAN;instantaneous_credible[i]=false;
            }
        }
        previous=sample;previous_time=now;have=true;
    }
};

int Run(const std::map<std::string,std::string>& options) {
    const auto get=[&](const std::string& key,const std::string& fallback){auto it=options.find(key);return it==options.end()?fallback:it->second;};
#ifdef VA15_STOP_BASELINE
    const std::string controller="baseline";
#else
    const std::string controller="candidate";
#endif
    if(get("--controller",controller)!=controller)throw std::runtime_error("--controller must match the compiled target");
    const std::string scenario=get("--scenario","recorded_capture");
    const std::string feedback=get("--feedback","truth"),output=get("--output","");
    const std::string recording=get("--recording","/home/dong/rl_sar_b2_z1/outputs/diagnosis_v251_yumjt3ry_20261027/control.csv");
    const double mu=std::stod(get("--friction",".8"));
    if(output.empty()||fs::exists(output)||fs::exists(output+".json")||!std::isfinite(mu)||mu<.3||mu>1.5)
        throw std::runtime_error("New --output and friction [.3,1.5] required");
    const bool step_wide=scenario=="supported_stepwide";
    const bool supported_stand=scenario=="supported_stand";
    const bool continuous=supported_stand||scenario=="supported_wide"||scenario=="supported_wide_large"||step_wide;
    if(scenario!="recorded_capture"&&scenario!="recorded_settle"&&scenario!="wide_generated"&&!continuous)throw std::runtime_error("Unknown scenario");
    if(feedback!="truth"&&feedback!="estimated")throw std::runtime_error("Unknown feedback");
    const auto model=(fs::path(__FILE__).parent_path()/"production/b2_independent_model.json").string();
    const std::string scene="/home/dong/rl_sar_b2_z1_old/policy/b2_z1_no_gun/VA15/scene_flat.xml";
    SimRL rl(scene,true,mu,0.f);
    // Separate pre-trial initialization: obtain the known supported standing
    // effort from actual stable MuJoCo dynamics, not from a moving gait.
    const auto stand=DeploymentStandPosition(rl);
    const auto kp=DeploymentStandGain(rl,"deployment_stand_kp","fixed_kp");
    const auto kd=DeploymentStandGain(rl,"deployment_stand_kd","fixed_kd");
    VA15ControlledStop stop;double reference_quiet=0.;
    for(unsigned n=0;n<1200&&!stop.stand_reference_valid;++n) {
        rl.ReadState();const auto& f=rl.robot_state.real_stop_feedback;
        float speed=0.f;for(int i:VA15LegIndices)speed=std::max(speed,std::fabs(rl.robot_state.motor_state.dq[i]));
        float gyro=0.f;for(float v:rl.robot_state.imu.gyroscope)gyro+=v*v;
        reference_quiet=VA15RealStopSettled(f,std::sin(rl.Tilt()),std::sqrt(gyro),speed,.48f,.15f)?reference_quiet+.005:0.;
        if(reference_quiet>=.5)stop.ObserveStanding(rl.robot_state,rl.robot_command);
        rl.Integrate();
    }
    if(!stop.stand_reference_valid)throw std::runtime_error("No physical four-foot quiet prewalk reference");
    RecordedPose pose;double preparation_tilt=0.;
    ObserverInput observed(rl,model);
    if(continuous) {
        // Stronger experiment: no teleport, including before the test.
        // The supported_stand case holds the prewalk stand unchanged for two
        // seconds; wider-foot cases retain their separate physical preparation.
        // The unchanged observer initializes on the prewalk stand and runs
        // throughout each preparation.
        const auto initial_command=rl.robot_command.motor_command.q;
        const double offset=supported_stand?0.:(step_wide?.12:(scenario=="supported_wide_large"?.14:.10));
        const auto smooth=[](double p){p=std::clamp(p,0.,1.);return p*p*(3.-2.*p);};
        const unsigned preparation_steps=supported_stand?400u:(step_wide?3400u:2200u);
        for(unsigned n=0;n<preparation_steps;++n) {
            rl.ReadState();if(n%2==0)observed.Update(rl);
            preparation_tilt=std::max(preparation_tilt,double(rl.Tilt()));
            rl.robot_command.motor_command.q=initial_command;
            if(step_wide) {
                // Actual physical foot placement, not a pose rewrite. Lift
                // one front foot slowly, move it out while unloaded, put it
                // back, then do the other. No foot is declared in contact
                // while lifted; all contact/estimate updates remain physical.
                for(unsigned leg=0;leg<2;++leg) {
                    const double local=double(n)*.005-1.-7.*leg;
                    const double lift=local<2.?smooth(local/2.):local<4.?1.:1.-smooth((local-4.)/2.);
                    const double outward=smooth((local-2.)/2.);
                    rl.robot_command.motor_command.q[leg]+=(leg==0?1.:-1.)*offset*outward;
                    rl.robot_command.motor_command.q[4+leg]+=.30*lift;
                    rl.robot_command.motor_command.q[9+leg]-=.60*lift;
                }
            }else if(!supported_stand) {
                const double blend=smooth((double(n)*.005-1.)/8.);
                rl.robot_command.motor_command.q[0]+=offset*blend;
                rl.robot_command.motor_command.q[1]-=offset*blend;
            }
            rl.Integrate();
        }
        rl.ReadState();pose.q=rl.robot_state.motor_state.q;
        pose.command=rl.robot_command.motor_command.q;
        pose.height=rl.robot_state.real_stop_feedback.base_height;
        for(unsigned i=0;i<4;++i)pose.quaternion[i]=rl.d->qpos[3+i];
    }else if(scenario=="wide_generated") {
        pose.q=pose.command=stand;
        for(unsigned i=0;i<4;++i)pose.q[i]=(i%2==0?1.f:-1.f)*(i<2?.16f:.055f);
    }else pose=LoadPose(recording,scenario);
    if(!continuous) {
        // INITIALIZATION ONLY, before monitored trial. Logged translation/
        // velocity are not replayed as sensors. These cases can be physically
        // unrepresentative; a failure before support/capture is inconclusive.
        for(unsigned i=0;i<18;++i)rl.d->qpos[rl.qids[i]]=pose.q[i];
        rl.d->qpos[0]=rl.d->qpos[1]=0.;rl.d->qpos[2]=pose.height;
        for(unsigned i=0;i<4;++i)rl.d->qpos[3+i]=pose.quaternion[i];
        for(int i=0;i<rl.m->nv;++i)rl.d->qvel[i]=0.;
        rl.robot_command.motor_command.q=pose.command;
        rl.robot_command.motor_command.kp=kp;rl.robot_command.motor_command.kd=kd;
        rl.robot_command.motor_command.dq.assign(18,0.f);rl.robot_command.motor_command.tau.assign(18,0.f);
        mj_forward(rl.m,rl.d);
    }
    rl.ReadState();
    const bool initial_physical_four=Bits(rl.robot_state.real_stop_feedback.contacts)==15;
    const bool initial_supported=initial_physical_four&&rl.robot_state.real_stop_feedback.base_height>=.48f&&
        rl.Tilt()<=.15f&&rl.robot_state.real_stop_feedback.planar_speed<.04f;
    const bool initial_model_four=Bits(observed.observer.Snapshot(rl.d->time).load_candidate)==15;
    fs::create_directories(fs::path(output).parent_path());
    std::ofstream csv(output);csv<<std::setprecision(10);
    csv<<"time,stop_stage,state,height,tilt,truth_speed,physical_contacts_FL_FR_RL_RR,estimated_contacts_DDS_FR_FL_RR_RL,estimate_valid,stop_captured,stop_ready,quiet,progress";
    for(unsigned i=0;i<4;++i)csv<<",model_fz_"<<i<<",model_horizontal_"<<i<<",model_horizontal_limit_"<<i<<",model_instantaneous_credible_"<<i;
    for(unsigned i=0;i<4;++i)csv<<",hip_actual_"<<i<<",hip_command_"<<i<<",hip_reference_"<<i;
    csv<<'\n';
    const double initial=rl.d->time;
    bool fell=false,prone=false,lowered=false,physical_capture=false,model_capture=false,lowering_without_physical_four=false;
    double capture_time=-1.,lower_time=-1.,prone_time=-1.,max_tilt=0.,model_four_seconds=0.,physical_four_seconds=0.;
    double physical_four_model_partial_seconds=0.,physical_quiet=0.;
    unsigned model_partial_while_physical_four=0,physical_four_samples=0;
    stop.Request();
    for(unsigned step=0;step<8000;++step) {
        rl.ReadState();const auto truth=rl.robot_state.real_stop_feedback;const double t=rl.d->time-initial;
        if(step%2==0)observed.Update(rl);
        const auto estimate=observed.observer.Snapshot(rl.d->time);
        const auto estimated=va15_trial::Feedback(estimate);
        const bool physical_four=Bits(truth.contacts)==15&&truth.planar_speed<.04f&&rl.Tilt()<.15f;
        // Recovery-only statistics: descent/prone contact loss is expected
        // and must not be reported as reproducing the upright FR rejection.
        if(!lowered&&physical_four) {
            physical_four_seconds+=.005;++physical_four_samples;
            if(Bits(estimate.load_candidate)!=15){physical_four_model_partial_seconds+=.005;++model_partial_while_physical_four;}
        }
        if(!lowered&&Bits(estimate.load_candidate)==15)model_four_seconds+=.005;
        rl.robot_state.real_stop_feedback=feedback=="estimated"?estimated:truth;
        if(!lowered) {
            stop.Step(rl.robot_state,rl.robot_command,stand,kp,kd,.005,0.f);
            if(stop.captured&&capture_time<0.) {
                capture_time=t;physical_capture=Bits(truth.contacts)==15;
                model_capture=Bits(estimate.load_candidate)==15&&va15_trial::FeedbackValid(estimated);
            }
            if(stop.Ready()) {
                lower_time=t;lowered=true;
                lowering_without_physical_four=!physical_four;
                rl.fsm.RequestStateChange("RLFSMStateGetDown");
            }
        }
        if(lowered) {
            rl.control.current_keyboard=Input::Keyboard::None;rl.control.current_gamepad=Input::Gamepad::None;
            rl.control.x=rl.control.y=rl.control.yaw=0.f;
            rl.StateController(&rl.robot_state,&rl.robot_command);
        }
        const auto state=rl.fsm.current_state_->GetStateName();
        const double tilt=rl.Tilt();max_tilt=std::max(max_tilt,tilt);
        if(!std::isfinite(tilt)||tilt>.5||!std::isfinite(truth.base_height)){fell=true;break;}
        if(step%4==0) {
            csv<<t<<','<<int(stop.stage)<<','<<state<<','<<truth.base_height<<','<<tilt<<','<<truth.planar_speed
               <<','<<Bits(truth.contacts)<<','<<Bits(estimate.load_candidate)<<','<<estimate.estimate_valid_for_review
               <<','<<stop.captured<<','<<stop.Ready()<<','<<stop.quiet<<','<<stop.progress;
            for(unsigned i=0;i<4;++i)csv<<','<<estimate.normal_equivalent[i]<<','<<observed.lateral[i]
                <<','<<observed.lateral_limit[i]<<','<<observed.instantaneous_credible[i];
            for(unsigned i=0;i<4;++i) {
                double ref=stand[i];
#ifndef VA15_STOP_BASELINE
                if(stop.recovery_reference_q.size()==18)ref=stop.recovery_reference_q[i];
#endif
                csv<<','<<rl.robot_state.motor_state.q[i]<<','<<rl.robot_command.motor_command.q[i]<<','<<ref;
            }
            csv<<'\n';
        }
        float dq=0.f;for(int i:VA15LegIndices)dq=std::max(dq,std::fabs(rl.robot_state.motor_state.dq[i]));
        if(lowered&&state=="RLFSMStatePassive"&&truth.base_height<.22&&tilt<.3&&truth.planar_speed<.04f&&dq<.55f)
            physical_quiet+=.005;else physical_quiet=0.;
        if(physical_quiet>=1.){prone=true;prone_time=t;break;}
        rl.Integrate();
    }
    const bool pass=!fell&&prone&&lowered&&physical_capture&&!lowering_without_physical_four;
    const bool inconclusive=!pass&&!stop.captured&&(!continuous||!initial_supported||preparation_tilt>.15);
    std::ofstream report(output+".json");
    report<<std::boolalpha<<"{\n\"pass\":"<<pass<<",\"controller\":\""<<controller<<"\",\"scenario\":\""<<scenario
        <<"\",\"feedback\":\""<<feedback<<"\",\"friction\":"<<mu<<",\"recorded_initial_row\":"<<pose.source_row
        <<",\"continuous_physical_preparation\":"<<continuous<<",\"initial_supported\":"<<initial_supported
        <<",\"support_preparation\":\""<<(supported_stand?"quiet stand hold; no foot or hip repositioning":"scenario-specific physical preparation")<<"\""
        <<",\"initial_physical_four\":"<<initial_physical_four<<",\"initial_model_four\":"<<initial_model_four
        <<",\"preparation_max_tilt_rad\":"<<Number(preparation_tilt)<<",\"inconclusive_initial_condition\":"<<inconclusive
        <<",\"initial_pose_only\":"<<!continuous<<",\"pose_resets_during_trial\":0,\"fake_contacts\":false,\"actor_used_during_recovery\":false"
        <<",\"physical_capture\":"<<physical_capture<<",\"model_four_at_capture\":"<<model_capture
        <<",\"capture_s\":"<<Number(capture_time)<<",\"lower_s\":"<<Number(lower_time)<<",\"prone_s\":"<<Number(prone_time)
        <<",\"prone\":"<<prone<<",\"fell\":"<<fell<<",\"max_tilt_rad\":"<<Number(max_tilt)
        <<",\"lowering_without_physical_four\":"<<lowering_without_physical_four
        <<",\"physical_four_quiet_s\":"<<Number(physical_four_seconds)<<",\"model_four_s\":"<<Number(model_four_seconds)
        <<",\"physical_four_model_partial_s\":"<<Number(physical_four_model_partial_seconds)
        <<",\"physical_four_samples\":"<<physical_four_samples<<",\"physical_four_model_partial_samples\":"<<model_partial_while_physical_four
        <<",\"final_stop_stage\":"<<int(stop.stage)<<",\"final_reason\":\""<<stop.reason
        <<"\",\"hardware_qualified\":false,\"no_gun_no_hose\":true,\"z1_servo\":\"MuJoCo implicit servo proxy, not real Z1 qualification\"}\n";
    std::cout<<"RECORDED STOP "<<(pass?"PASS":"FAIL")<<" controller="<<controller<<" feedback="<<feedback
        <<" scenario="<<scenario<<" mu="<<mu<<" capture="<<capture_time<<" lower="<<lower_time<<" prone="<<prone
        <<" model_partial_while_actual_four_s="<<physical_four_model_partial_seconds<<'\n';
    return pass?0:2;
}
}

int main(int argc,char** argv) {
    try {
        std::map<std::string,std::string> options;
        for(int i=1;i<argc;i+=2) {
            if(i+1>=argc)throw std::runtime_error("Each option requires a value");
            const std::string key=argv[i];
            if(key!="--controller"&&key!="--scenario"&&key!="--feedback"&&key!="--friction"&&key!="--output"&&key!="--recording")
                throw std::runtime_error("Unknown option: "+key);
            if(!options.emplace(key,argv[i+1]).second)throw std::runtime_error("Repeated option");
        }
        return Run(options);
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
