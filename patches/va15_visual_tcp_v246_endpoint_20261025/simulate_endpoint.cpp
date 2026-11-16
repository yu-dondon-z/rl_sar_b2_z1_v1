// SDK-free actual VA15 actor/FSM dynamics with finite observed pixel curves.
// Synthetic calibrated camera rays + MuJoCo truth pose ONLY in this harness.
// No camera network, DDS, UDP, SDK clients or real commands are initialized.
#include "production/fsm_b2_z1_no_gun.hpp"
#include "production/endpoint_follow.hpp"
#include "production/tcp_fk.hpp"
#include "production/independent_model_io.hpp"
#define main archived_simulation_main_not_called
#include "../va15_controlled_getdown_20261020/simulate_controlled_getdown.cpp"
#undef main
#include <deque>

static double amplitude=0.;
#ifdef VA15_ENDPOINT_ESTIMATED_POSE
static constexpr bool estimated_pose=true;
#else
static constexpr bool estimated_pose=false;
#endif
static double terminal_x=5.;
static double Path(double x){return amplitude*std::sin(.8*x);}
static double Pixel(const va15_tcp::Pose& pose,double v) {
    double low=384.,high=1536.;
    for(int n=0;n<45;++n) {
        const double u=.5*(low+high);va15_tcp::V3 world;
        if(!va15_tcp::Project(pose,{u,v},world))return NAN;
        if(world.y()-Path(world.x())>0.)low=u;else high=u;
    }
    return .5*(low+high);
}
static va15_visual::Packet Frame(const va15_tcp::Pose& pose,std::uint64_t sequence) {
    va15_visual::Packet p;p.session_matched=p.image_received=true;
    p.sequence=sequence;p.capture_ns=pose.ns;p.publish_ns=pose.ns+100000000ULL;
    p.confidence=p.look_confidence=.995;p.width=30.;p.contrast=.8;p.rms=1.;
    // Every path point is a directly observed synthetic pixel on the finite
    // ground curve. No point is created beyond the physical terminal.
    bool cap_inside=false;
    for(double v=1044.;v>=618.;v-=6.) {
        const double u=Pixel(pose,v);va15_tcp::V3 ground;
        if(!std::isfinite(u)||!va15_tcp::Project(pose,{u,v},ground))break;
        if(ground.x()>terminal_x) {
            if(!p.path.empty()) {
                double low=v,high=p.path.back()[1];
                for(int i=0;i<45;++i) {
                    const double row=.5*(low+high),col=Pixel(pose,row);va15_tcp::V3 at;
                    if(!std::isfinite(col)||!va15_tcp::Project(pose,{col,row},at))break;
                    if(at.x()>terminal_x)low=row;else high=row;
                }
                const double row=.5*(low+high),col=Pixel(pose,row);
                if(std::isfinite(col)) {p.path.push_back({col,row});cap_inside=true;}
            }
            break;
        }
        p.path.push_back({u,v});
    }
    if(p.path.size()<2)return p;
    const double last=p.path.back()[1];
    p.v=972.;p.look_v=std::max(va15_visual::NominalLookV,last+12.);
    p.u=va15_visual::CurveU(p,p.v);p.look_u=va15_visual::CurveU(p,p.look_v);
    p.path_count=p.path.size();p.path_span=p.path.front()[1]-last;
    p.valid=p.path_count>=25&&p.path_span>=200.&&p.look_v<=va15_visual::MaximumLookV&&
        std::isfinite(p.u)&&std::isfinite(p.look_u);
    p.endpoint_candidate=p.valid&&cap_inside&&last>=690.&&last<=900.;
    p.endpoint_u=p.path.back()[0];p.endpoint_v=last;
    p.endpoint_confidence=.995;p.endpoint_clear_pixels=132.;
    return p;
}
static double Finite(double x){return std::isfinite(x)?x:-1.;}
int main(int argc,char** argv) {
    try {
        if((argc!=4&&argc!=5)||fs::exists(argv[2])||fs::exists(std::string(argv[2])+".json"))
            throw std::runtime_error("scenario NEW.csv friction [endpoint_x=5] required; no overwrites");
        const std::string scenario=argv[1];const double mu=std::stod(argv[3]);
        if(argc==5)terminal_x=std::stod(argv[4]);
        const bool fault=scenario!="straight"&&scenario!="curve";
        if(scenario!="straight"&&scenario!="curve"&&scenario!="unknown_loss"&&scenario!="camera_failure"&&
           scenario!="pose_loss"&&scenario!="operator_x")throw std::runtime_error("Unknown endpoint scenario");
        if(!std::isfinite(mu)||mu<.5||mu>1.)throw std::runtime_error("Undeclared friction");
        if(!std::isfinite(terminal_x)||terminal_x<5.||terminal_x>7.)throw std::runtime_error("Undeclared endpoint distance");
        amplitude=scenario=="curve"?.035:0.;
        SimRL rl("/home/dong/rl_sar_b2_z1_old/policy/b2_z1_no_gun/VA15/scene_flat.xml",true,mu,0.);
        rl.params.config_node["isolated_forward_trial"]=true;
        rl.params.config_node["isolated_forward_trial_motion"]=true;
        rl.params.config_node["near_visual_trial"]=true;
        rl.params.config_node["repeatable_visual_session"]=true;
        rl.params.config_node["continuous_visual_session"]=true;
        auto loco=std::dynamic_pointer_cast<RLFSMStateRLLocomotion>(rl.fsm.states_.at("RLFSMStateRLLocomotion"));
        loco->trial_configured=false;loco->Enter();
        va15_visual::Guide guide(true,true,true);va15_tcp::Tracker tracker;va15_tcp::History poses;
        va15_endpoint::Follower endpoint;endpoint.Reset(0,1);
        va15_independent::Observer observer(LoadIndependentModel((fs::path(__FILE__).parent_path()/
            "production/b2_independent_model.json").string()),true);
        std::array<int,12> dds_q,dds_v;
        const char* legs[]={"FR","FL","RR","RL"};const char* joints[]={"hip","thigh","calf"};
        for(int l=0;l<4;++l)for(int j=0;j<3;++j) {
            const auto name=std::string(legs[l])+"_"+joints[j]+"_joint";
            const int id=mj_name2id(rl.m,mjOBJ_JOINT,name.c_str());
            if(id<0)throw std::runtime_error("Missing independent-estimator leg joint");
            dds_q[3*l+j]=rl.m->jnt_qposadr[id];dds_v[3*l+j]=rl.m->jnt_dofadr[id];
        }
        const int imu=mj_name2id(rl.m,mjOBJ_SENSOR,"imu_acc");
        if(imu<0)throw std::runtime_error("Missing IMU specific force");
        std::uint64_t last_supported=0;
        std::deque<va15_visual::Packet> delayed;
        const double initial=rl.d->time;std::uint64_t seq=0,camera_stamp=0;
        const int tcp_site=mj_name2id(rl.m,mjOBJ_SITE,"ee_site");
        if(tcp_site<0)throw std::runtime_error("Missing measured TCP reference site");
        bool started=false,fell=false,ready=false,completed=false,injected=false,tail_attempted=false;
        bool virtual_fk_valid=true,stable_completion=false,auto_lowered=false;
        double tail_time=-1.,confirmed_time=-1.,stop_time=-1.,completion_time=-1.,ready_since=-1.;
        double max_tilt=0.,max_fk_error=0.,max_target_step=0.,max_body_lateral=0.;
        double terminal_error=-1.,terminal_height=-1.,final_speed=-1.,stopped_x=-1.;
        double control_speed_upper=NAN;
        va15_tcp::V3 gauge=va15_tcp::V3::Zero(),final_pose_drift=va15_tcp::V3::Zero();
        bool gauge_valid=false;
        va15_tcp::V3 previous_position=va15_tcp::V3::Zero();bool have_previous=false;
        va15_endpoint::Output eo;
        std::ofstream csv(argv[2]);csv<<std::setprecision(10);
        csv<<"time,x,y,height,tilt,guide,endpoint,stop_stage,confirmed_frames,bank_points,tail_s,remaining_arc,endpoint_error,command_x,command_yaw,tcp_x,tcp_y,tcp_z,actual_speed,cap_valid,target_x,target_y,target_z,reason,control_speed_upper,control_tcp_x,control_tcp_y,control_tcp_z,endpoint_world_x,endpoint_world_y,endpoint_world_z,aligned_pose_drift_x,aligned_pose_drift_y\n";
        for(int step=0;step<17000;++step) {
            rl.ReadState();const double t=rl.d->time-initial;
            // Label truth correctly. This tests physics/FSM, NOT the real
            // estimator nor hardware foot-load calibration.
            auto& feedback=rl.robot_state.real_stop_feedback;
            feedback.sport_fresh=false;feedback.unavailable_reason=va15_trial::Source;
            const auto ns=std::uint64_t((100.+t)*1.e9);
            va15_tcp::Pose pose{ns,{rl.d->qpos[0],rl.d->qpos[1],rl.d->qpos[2]},
                Eigen::Quaterniond(rl.d->qpos[3],rl.d->qpos[4],rl.d->qpos[5],rl.d->qpos[6]).normalized(),true};
            const auto physical_pose=pose;
            control_speed_upper=feedback.planar_speed;
            if(estimated_pose) {
                if(step%2==0) {
                    va15_independent::Sample sample;sample.mode.fill(10);sample.documented_motor_health=true;
                    sample.quaternion=physical_pose.q;
                    for(int i=0;i<12;++i) {
                        sample.q[i]=rl.d->qpos[dds_q[i]];sample.dq[i]=rl.d->qvel[dds_v[i]];
                        sample.tau[i]=rl.d->qfrc_actuator[dds_v[i]];
                    }
                    for(int i=0;i<3;++i) {
                        sample.gyro[i]=rl.robot_state.imu.gyroscope[i];
                        sample.specific_force[i]=rl.d->sensordata[rl.m->sensor_adr[imu]+i];
                    }
                    observer.Update(sample,std::lround(rl.d->time*1000.),rl.d->time);
                }
                const auto estimate=observer.Snapshot(rl.d->time);
                feedback=va15_trial::Feedback(estimate);
                // Match the real control snapshot: health-only prediction
                // still supplies bounded velocity, but NEVER contact/stop
                // permission. Feedback's NaN is not the real tail input.
                control_speed_upper=estimate.planar_speed+2.*estimate.velocity_sigma;
                if(va15_trial::Healthy(estimate)&&estimate.estimate_valid_for_review)last_supported=ns;
                pose.p=estimate.position;pose.valid=va15_trial::Healthy(estimate)&&last_supported&&
                    ns-last_supported<=250000000ULL;
                if(pose.valid&&step%2==0)poses.Add(pose);
                if(pose.valid) {
                    const auto velocity=(pose.q.conjugate()*estimate.velocity).eval();
                    guide.ObserveBodyVelocity(velocity.x(),velocity.y(),.005);
                }
            } else poses.Add(pose);
            if(pose.valid&&!gauge_valid) {gauge=physical_pose.p-pose.p;gauge_valid=true;}
            if(pose.valid&&gauge_valid)final_pose_drift=pose.p+gauge-physical_pose.p;
            va15_tcp::V3 actual;
            if(!va15_tcp::MeasuredTCPWorld(physical_pose,rl.robot_state.motor_state.q,actual)) {
                virtual_fk_valid=false;throw std::runtime_error("Measured virtual TCP FK invalid");
            }
            const va15_tcp::V3 site(rl.d->site_xpos[3*tcp_site],rl.d->site_xpos[3*tcp_site+1],rl.d->site_xpos[3*tcp_site+2]);
            max_fk_error=std::max(max_fk_error,(actual-site).norm());
            if(have_previous&&!estimated_pose) {
                const auto velocity=(pose.q.conjugate()*((pose.p-previous_position)/.005)).eval();
                guide.ObserveBodyVelocity(velocity.x(),velocity.y(),.005);
            }
            previous_position=pose.p;have_previous=true;
            if((scenario=="unknown_loss"&&t>=18.)||(tail_time>=0.&&t-tail_time>=2.&&
               (scenario=="camera_failure"||scenario=="pose_loss"||scenario=="operator_x")))injected=true;
            if(step%30==0&&!(scenario=="camera_failure"&&injected)) {
                auto packet=Frame(physical_pose,++seq);
                if(scenario=="unknown_loss"&&injected) {packet.valid=false;packet.endpoint_candidate=false;}
                delayed.push_back(packet);
            }
            while(!delayed.empty()&&delayed.front().publish_ns<=ns) {
                const auto packet=delayed.front();delayed.pop_front();camera_stamp=packet.capture_ns;
                guide.Observe(packet,ns);va15_tcp::Pose capture;
                if(poses.At(packet.capture_ns,capture)) {
                    if(va15_visual::Valid(packet,ns))tracker.Observe(packet,capture);
                    endpoint.Observe(packet,capture,ns,0,1,{packet.endpoint_candidate,
                        packet.endpoint_u,packet.endpoint_v,packet.endpoint_confidence,packet.endpoint_clear_pixels});
                }
            }
            if(endpoint.Confirmed()&&confirmed_time<0.)confirmed_time=t;
            if(scenario=="pose_loss"&&injected)pose.valid=false;
            guide.metric_bearing_valid=tracker.Guidance(pose,guide.metric_bearing);
            rl.control.current_keyboard=Input::Keyboard::None;rl.control.current_gamepad=Input::Gamepad::None;
            if(t>=6.&&!started) {
                started=true;
                if(!guide.Start(ns,loco->RealStopSettled(),true,true))throw std::runtime_error(guide.reason);
                tracker.Begin(pose);
            }
            va15_endpoint::Input input;
            input.current=pose;input.now_ns=ns;input.camera_ns=camera_stamp;
            // Real controller uses the measured joints in its LOCAL estimate;
            // physical truth above remains only for rendering/error scoring.
            const bool control_fk_valid=va15_tcp::MeasuredTCPWorld(pose,rl.robot_state.motor_state.q,input.tcp_world);
            input.healthy=pose.valid&&control_fk_valid;input.camera_healthy=camera_stamp&&ns>=camera_stamp&&
                ns-camera_stamp<=va15_visual::MaximumAgeNs;
            input.actual_speed_upper=control_speed_upper;input.dt=.005;input.generation=0;input.session=1;
            input.terminal_metric_verified=!estimated_pose; // truth harness ONLY; never from model health
            input.target_seed=tracker.target;input.clearance=tracker.clearance;input.speed_seed=.15;input.yaw_seed=guide.yaw;
            input.operator_stop=scenario=="operator_x"&&injected;
            if(started&&endpoint.GetStage()==va15_endpoint::Stage::Confirmed&&tracker.ever_tracked) {
                tail_attempted=true;
                // Confirmed-but-not-covered remains a harmless pending state:
                // the actual tool can lag the nominal -0.775m target. Keep
                // collecting normal observed coverage and retry, never invent
                // a backward extension to force a finite tail.
                if(endpoint.BeginTail(input))tail_time=t;
            }
            const auto previous_target=tracker.target;
            bool stopping=false;
            if(endpoint.Active()||endpoint.Completed()||endpoint.GetStage()==va15_endpoint::Stage::Fault||
                    endpoint.GetStage()==va15_endpoint::Stage::Stopped) {
                eo=endpoint.Step(input);
                rl.control.x=eo.x;rl.control.y=0.;rl.control.yaw=eo.yaw;
                loco->endpoint_speed_cap=eo.x;
                if(eo.target_valid)tracker.target=eo.target;
                if(eo.stop) {stopping=true;guide.Stop(eo.reason);loco->trial.Stop(eo.reason);}
            } else {
                if(input.operator_stop||!pose.valid)guide.Stop(input.operator_stop?"operator X":"local pose lost");
                const auto out=guide.Tick(ns,.005,pose.valid,false,true);
                rl.control.x=out.x;rl.control.y=0.;rl.control.yaw=out.yaw;
                loco->endpoint_speed_cap=-1.;
                tracker.Step(pose,.005,guide.stage==va15_visual::Guide::Stage::Running);
                if(out.stop){stopping=true;loco->trial.Stop(guide.reason);}
            }
            if(stopping&&stop_time<0.)stop_time=t;
            rl.SetRuntimeTargetCommands({float(tracker.target.x()),float(tracker.target.y()),float(tracker.target.z()),0.,-.707107,0.,.707107});
            max_target_step=std::max(max_target_step,(tracker.target-previous_target).cwiseAbs().maxCoeff());
            rl.StateController(&rl.robot_state,&rl.robot_command);
            if(step%4==0)rl.Infer();
            const auto state=rl.fsm.current_state_->GetStateName();
            auto_lowered|=state=="RLFSMStateGetDown"||state=="RLFSMStatePassive";
            ready=loco->controlled_stop.Ready();
            if(endpoint.AwaitingStand()&&endpoint.NotifyStableStand(input,ready)) {
                completed=true;stable_completion=ready&&input.actual_speed_upper<=.04;
                completion_time=t;
            }
            max_tilt=std::max(max_tilt,double(rl.Tilt()));
            max_body_lateral=std::max(max_body_lateral,std::fabs(pose.p.y()-Path(pose.p.x())));
            if(step%10==0)csv<<t<<','<<physical_pose.p.x()<<','<<physical_pose.p.y()<<','<<physical_pose.p.z()<<','<<rl.Tilt()
                <<','<<int(guide.stage)<<','<<int(endpoint.GetStage())<<','<<int(loco->controlled_stop.stage)
                <<','<<endpoint.ConfirmingFrames()<<','<<endpoint.BankPoints()<<','<<(tail_time<0.?-1.:t-tail_time)
                <<','<<Finite(endpoint.RemainingArc())<<','<<Finite(eo.endpoint_error)
                <<','<<rl.control.x<<','<<rl.control.yaw<<','<<actual.x()<<','<<actual.y()<<','<<actual.z()
                <<','<<feedback.planar_speed<<','<<guide.latest.endpoint_candidate
                <<','<<tracker.target.x()<<','<<tracker.target.y()<<','<<tracker.target.z()<<','<<endpoint.Reason()
                <<','<<input.actual_speed_upper<<','<<input.tcp_world.x()<<','<<input.tcp_world.y()<<','<<input.tcp_world.z()
                <<','<<endpoint.Endpoint().x()<<','<<endpoint.Endpoint().y()<<','<<endpoint.Endpoint().z()
                <<','<<final_pose_drift.x()<<','<<final_pose_drift.y()<<'\n';
            if(!std::isfinite(rl.Tilt())||rl.Tilt()>.5){fell=true;break;}
            if(ready&&stop_time>=0.) {
                if(ready_since<0.)ready_since=t;
                if(t-ready_since>=2.) {
                    terminal_error=(actual-va15_tcp::V3(terminal_x,Path(terminal_x),0.)).head<2>().norm();
                    terminal_height=actual.z();final_speed=feedback.planar_speed;stopped_x=physical_pose.p.x();break;
                }
            } else ready_since=-1.;
            rl.Integrate();
        }
        const bool endpoint_success=completed&&stable_completion&&terminal_error>=0.&&
            terminal_error<=va15_endpoint::Follower::EndpointTolerance&&
            std::fabs(terminal_height-.09)<=.05;
        const bool pass=started&&!fell&&!auto_lowered&&max_fk_error<1.e-5&&max_target_step<=.000201&&
            ready&&final_speed<=.04&&(fault?(injected&&!completed&&stop_time>=0.):endpoint_success);
        std::ofstream report(std::string(argv[2])+".json");report<<std::boolalpha<<std::setprecision(10)
            <<"{\"pass\":"<<pass<<",\"scenario\":\""<<scenario<<"\",\"friction\":"<<mu
            <<",\"physical_endpoint_x_m\":"<<terminal_x
            <<",\"started\":"<<started<<",\"fell\":"<<fell<<",\"auto_lowered\":"<<auto_lowered
            <<",\"tail_attempted\":"<<tail_attempted<<",\"virtual_fk_valid\":"<<virtual_fk_valid
            <<",\"endpoint_confirmed_s\":"<<confirmed_time<<",\"tail_started_s\":"<<tail_time
            <<",\"stop_requested_s\":"<<stop_time<<",\"completion_s\":"<<completion_time
            <<",\"completed\":"<<completed<<",\"original_stop_ready\":"<<ready
            <<",\"actual_terminal_planar_error_m\":"<<terminal_error<<",\"actual_tcp_height_m\":"<<terminal_height
            <<",\"final_actual_planar_speed_mps\":"<<final_speed<<",\"stopped_base_x_m\":"<<stopped_x
            <<",\"max_tilt_rad\":"<<max_tilt<<",\"max_fk_vs_mujoco_error_m\":"<<max_fk_error
            <<",\"max_target_step_m\":"<<max_target_step<<",\"max_body_lateral_m\":"<<max_body_lateral
            <<",\"fault_injected\":"<<injected<<",\"endpoint_stage\":"<<int(endpoint.GetStage())
            <<",\"reason\":\""<<endpoint.Reason()<<"\",\"synthetic_rgb\":true,\"truth_pose_harness_only\":"<<!estimated_pose
            <<",\"independent_estimated_pose\":"<<estimated_pose
            <<",\"final_aligned_odometry_drift_xy_m\":"<<final_pose_drift.head<2>().norm()
            <<",\"final_endpoint_projection_error_xy_m\":"<<(confirmed_time>=0.?
                (endpoint.Endpoint()+gauge-va15_tcp::V3(terminal_x,Path(terminal_x),0.)).head<2>().norm():-1.)
            <<",\"real_camera_or_segmentation_qualification\":false,\"hardware_qualified\":false}\n";
        return pass?0:2;
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
