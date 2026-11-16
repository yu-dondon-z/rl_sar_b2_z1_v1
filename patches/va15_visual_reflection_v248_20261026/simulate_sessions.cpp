// SDK-free closed-loop physics: actual VA15/FSM, independent estimator and
// current-health guard, synthetic camera packets. NOT hardware acceptance.
#include "production/fsm_b2_z1_no_gun.hpp"
#include "production/independent_model_io.hpp"
#include "production/trial_motor_contract.hpp"
#include "production/near_visual_guide.hpp"
#define main archived_simulation_main_not_called
#include "../va15_controlled_getdown_20261020/simulate_controlled_getdown.cpp"
#undef main

int main(int argc,char** argv) {
    try {
        if(argc!=4||fs::exists(argv[2]))throw std::runtime_error("scenario new-output.csv friction required");
        const std::string scenario=argv[1];const double mu=std::stod(argv[3]);
        const bool health_fault=scenario=="motor_fault"||scenario=="stale";
        const bool final_b=scenario=="b"||scenario=="b_start_same_frame";
        if(scenario!="repeat"&&scenario!="x"&&scenario!="b"&&scenario!="loss"&&
           scenario!="motor_fault"&&scenario!="stale"&&scenario!="stick"&&scenario!="b_start_same_frame")
            throw std::runtime_error("unknown session scenario");
        SimRL rl("/home/dong/rl_sar_b2_z1_old/policy/b2_z1_no_gun/VA15/scene_flat.xml",true,mu,0.);
        rl.params.config_node["isolated_forward_trial"]=true;
        rl.params.config_node["isolated_forward_trial_motion"]=true;
        rl.params.config_node["near_visual_trial"]=true;
        rl.params.config_node["repeatable_visual_session"]=true;
        rl.params.config_node["continuous_visual_session"]=true;
        auto loco=std::dynamic_pointer_cast<RLFSMStateRLLocomotion>(rl.fsm.states_.at("RLFSMStateRLLocomotion"));
        loco->trial_configured=false;loco->Enter();
        va15_independent::Observer observer(LoadIndependentModel((fs::path(__FILE__).parent_path()/"production/b2_independent_model.json").string()),true);
        va15_motor_contract::Guard health{va15_motor_contract::LostPolicy::DiagnosticOnly};
        va15_visual::Guide guide(true,true);
        std::array<int,12> qids,vids;const char* legs[]={"FR","FL","RR","RL"};const char* joints[]={"hip","thigh","calf"};
        for(int i=0;i<4;++i)for(int j=0;j<3;++j){const auto name=std::string(legs[i])+"_"+joints[j]+"_joint";
            const int id=mj_name2id(rl.m,mjOBJ_JOINT,name.c_str());qids[3*i+j]=rl.m->jnt_qposadr[id];vids[3*i+j]=rl.m->jnt_dofadr[id];}
        const int imu=mj_name2id(rl.m,mjOBJ_SENSOR,"imu_acc");
        const double initial=rl.d->time;
        bool fell=false,prone=false,auto_motion=false,rearm_early=false,start_done=false,second_done=false,stop_done=false;
        unsigned starts=0,quiet_catches=0;double max_tilt=0.,requested=0.,moving_seconds=0.,longest_session=0.,first_motion=-1.;
        double first_x=0.,last_x=0.;std::uint64_t seq=0;
        std::ofstream csv(argv[2]);csv<<"time,starts,guide,trial,stop,state,x,policy_x,height,tilt,estimate_valid,contract_ready\n";
        for(int step=0;step<10200;++step) {
            rl.ReadState();const auto truth=rl.robot_state.real_stop_feedback;
            const double now=rl.d->time,t=now-initial;
            const auto ns=static_cast<std::uint64_t>((now+100.)*1.e9);
            if(step%2==0&&!(scenario=="stale"&&t>=12.&&t<12.20)) {
                va15_motor_contract::Raw r;r.tick=std::lround(now*1000.);r.bit_flag=4;r.mode.fill(10);r.frequency.fill(535);
                r.lost[4]=r.lost[7]=static_cast<unsigned>(t*20.)%7;
                if(scenario=="motor_fault"&&t>=12.)r.fault[4]=1;
                health.Update(r,now);
                va15_independent::Sample sample;sample.mode=r.mode;sample.lost=r.lost;
                sample.documented_motor_health=health.Snapshot(now).ready;
                for(int i=0;i<12;++i){sample.q[i]=rl.d->qpos[qids[i]];sample.dq[i]=rl.d->qvel[vids[i]];sample.tau[i]=rl.d->qfrc_actuator[vids[i]];}
                sample.quaternion=Eigen::Quaterniond(rl.robot_state.imu.quaternion[0],rl.robot_state.imu.quaternion[1],rl.robot_state.imu.quaternion[2],rl.robot_state.imu.quaternion[3]);
                for(int i=0;i<3;++i){sample.gyro[i]=rl.robot_state.imu.gyroscope[i];sample.specific_force[i]=rl.d->sensordata[rl.m->sensor_adr[imu]+i];}
                observer.Update(sample,r.tick,now);
            }
            auto estimate=observer.Snapshot(now);if(!health.Snapshot(now).ready)estimate.input_valid=false;
            rl.robot_state.real_stop_feedback=va15_trial::Feedback(estimate);
            const bool healthy=health.Snapshot(now).ready&&va15_trial::HealthyFeedback(rl.robot_state.real_stop_feedback);
            if(step%36==0) {
                va15_visual::Packet p;p.valid=!(scenario=="loss"&&t>=12.&&t<13.);p.session_matched=true;
                p.sequence=++seq;p.capture_ns=ns-120000000;p.publish_ns=ns;
                p.u=p.look_u=935.;p.v=972.;p.look_v=.76*1080.;p.confidence=p.look_confidence=.99;
                p.width=30.;p.contrast=.8;p.rms=1.;p.path_count=50;p.path_span=300.;guide.Observe(p,ns);
            }
            rl.control.current_keyboard=Input::Keyboard::None;rl.control.current_gamepad=Input::Gamepad::None;
            bool stop_key=false,centered=true;
            if((scenario=="stick"&&t>=16.&&t<16.1))centered=false;
            if(t>=6.&&!start_done) {
                start_done=true;
                if(guide.Start(ns,loco->RealStopSettled()&&loco->real_start_support_cycles>=100&&
                    loco->trial.stage==va15_trial::Supervisor::Stage::Armed,centered,healthy))++starts;
            }
            if(t>=16.&&!stop_done) {
                stop_done=true;
                if(scenario=="x")stop_key=true;
                else if(scenario=="b") {stop_key=true;rl.control.current_gamepad=Input::Gamepad::B;}
                else if(scenario!="stick")guide.Stop("explicit R1+Y stop");
            }
            if(t>=17.&&!second_done&&loco->trial.Stopping()&&!loco->controlled_stop.Ready()) {
                // The explicit start API must refuse while still braking.
                if(loco->RearmVisualSession(centered,healthy))rearm_early=true;
            }
            if(t>=25.&&!second_done&&!health_fault&&scenario!="b") {
                second_done=true;
                const bool simultaneous_b=scenario=="b_start_same_frame";
                if(simultaneous_b){stop_key=true;rl.control.current_gamepad=Input::Gamepad::B;}
                if(!stop_key&&guide.Ready(ns)&&loco->RearmVisualSession(centered,healthy)&&
                   guide.Start(ns,loco->RealStopSettled(),centered,healthy))++starts;
            }
            if(t>=35.&&guide.stage==va15_visual::Guide::Stage::Running)guide.Stop("second explicit R1+Y stop");
            auto output=guide.Tick(ns,.005,healthy,stop_key,centered);
            if(output.stop)loco->trial.Stop(guide.reason);
            rl.control.x=output.x;rl.control.y=0.f;rl.control.yaw=output.yaw;
            if(t>=43.&&!health_fault)rl.control.current_gamepad=Input::Gamepad::B;
            rl.StateController(&rl.robot_state,&rl.robot_command);
            const bool forward=loco->trial.Forward()&&guide.stage==va15_visual::Guide::Stage::Running;
            if(!forward)rl.control.x=rl.control.y=rl.control.yaw=0.f;
            if(step%4==0)rl.Infer();
            if(forward) {
                if(first_motion<0){first_motion=t;first_x=rl.d->qpos[0];}
                last_x=rl.d->qpos[0];moving_seconds+=.005;
                if(starts==0||(t>16.&&t<25.)||(health_fault&&t>=12.1))auto_motion=true;
            }
            longest_session=std::max(longest_session,loco->trial.requested);
            if(loco->controlled_stop.Ready())++quiet_catches;
            max_tilt=std::max(max_tilt,double(rl.Tilt()));
            const auto state=rl.fsm.current_state_->GetStateName();
            if(step%4==0)csv<<t<<','<<starts<<','<<int(guide.stage)<<','<<int(loco->trial.stage)<<','<<int(loco->controlled_stop.stage)
                <<','<<state<<','<<rl.control.x<<','<<rl.obs.commands[0]<<','<<truth.base_height<<','<<rl.Tilt()<<','<<estimate.estimate_valid_for_review<<','<<healthy<<'\n';
            if(!std::isfinite(rl.Tilt())||rl.Tilt()>.5){fell=true;break;}
            if(state=="RLFSMStatePassive"&&truth.base_height<.2){prone=true;break;}
            if(health_fault&&t>=24.)break;
            rl.Integrate();
        }
        const unsigned expected_starts=health_fault||final_b?1:2;
        const bool pass=!fell&&!auto_motion&&!rearm_early&&starts==expected_starts&&first_motion>=6.&&first_motion<6.1&&
            (health_fault?(!health.Snapshot(rl.d->time).ready&&loco->trial.Stopping()):prone&&quiet_catches>0)&&
            longest_session>5.;
        std::ofstream report(std::string(argv[2])+".json");report<<std::boolalpha
            <<"{\"scenario\":\""<<scenario<<"\",\"pass\":"<<pass<<",\"starts\":"<<starts
            <<",\"expected_starts\":"<<expected_starts<<",\"first_motion_s\":"<<first_motion
            <<",\"longest_session_request_s\":"<<longest_session<<",\"total_motion_request_s\":"<<moving_seconds
            <<",\"forward_m\":"<<last_x-first_x<<",\"auto_motion\":"<<auto_motion<<",\"premature_rearm\":"<<rearm_early
            <<",\"prone\":"<<prone<<",\"fell\":"<<fell<<",\"max_tilt_rad\":"<<max_tilt
            <<",\"friction\":"<<mu<<",\"hardware_qualified\":false,\"synthetic_camera\":true}\n";
        return pass?0:2;
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
