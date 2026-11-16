// V2.4.7 SDK-free VA15/FSM dynamics. Camera pixels are synthetic, while control uses
// the independent IMU/joint observer, NOT MuJoCo world position. No hardware I/O.
#include "production/fsm_b2_z1_no_gun.hpp"
#include "production/visual_tcp.hpp"
#include "production/visual_rear_completion.hpp"
#include "production/tcp_fk.hpp"
#include "production/independent_model_io.hpp"
#define main archived_simulation_main_not_called
#include "../va15_controlled_getdown_20261020/simulate_controlled_getdown.cpp"
#undef main
#include <deque>

static double amplitude=0.,terminal_x=5.;
static double Path(double x){return amplitude*std::sin(.8*x);}
static double Pixel(const va15_tcp::Pose& pose,double v) {
    double low=384.,high=1536.;
    for(int i=0;i<45;++i) {
        const double u=.5*(low+high);va15_tcp::V3 p;
        if(!va15_tcp::Project(pose,{u,v},p))return NAN;
        if(p.y()-Path(p.x())>0.)low=u;else high=u;
    }
    return .5*(low+high);
}
static va15_visual::Packet Frame(const va15_tcp::Pose& pose,std::uint64_t sequence) {
    va15_visual::Packet p;p.session_matched=p.image_received=true;
    p.sequence=sequence;p.capture_ns=pose.ns;p.publish_ns=pose.ns+100000000ULL;
    p.confidence=p.look_confidence=.995;p.width=30.;p.contrast=.8;p.rms=1.;
    bool cap=false;
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
                if(std::isfinite(col)){p.path.push_back({col,row});cap=true;}
            }
            break;
        }
        p.path.push_back({u,v});
    }
    if(p.path.size()<2)return p;
    const double last=p.path.back()[1];p.v=972.;p.look_v=std::max(va15_visual::NominalLookV,last+12.);
    p.u=va15_visual::CurveU(p,p.v);p.look_u=va15_visual::CurveU(p,p.look_v);
    p.path_count=p.path.size();p.path_span=p.path.front()[1]-last;
    p.valid=p.path_count>=25&&p.path_span>=200.&&p.look_v<=va15_visual::MaximumLookV&&
        std::isfinite(p.u)&&std::isfinite(p.look_u);
    p.endpoint_candidate=p.valid&&cap&&last>=690.&&last<=885.6;
    p.endpoint_u=p.path.back()[0];p.endpoint_v=last;p.endpoint_confidence=.995;p.endpoint_clear_pixels=132.;
    return p;
}
int main(int argc,char** argv) {
    try {
        if(argc<4||argc>6||fs::exists(argv[2])||fs::exists(std::string(argv[2])+".json"))
            throw std::runtime_error("scenario NEW.csv friction [end_m] [pixel_noise] required; no overwrites");
        const std::string scenario=argv[1];const double mu=std::stod(argv[3]);
        const bool normal=scenario=="straight"||scenario=="curve";
        const bool terminal_b=scenario=="terminal_b",terminal_pose_loss=scenario=="terminal_pose_loss";
        const bool operator_b=scenario=="operator_b"||terminal_b;
        if(!normal&&scenario!="no_end"&&scenario!="unknown_loss"&&scenario!="camera_failure"&&
           scenario!="one_false_cap"&&!operator_b&&!terminal_pose_loss)throw std::runtime_error("Unknown scenario");
        if(!std::isfinite(mu)||mu<.5||mu>1.)throw std::runtime_error("Undeclared friction");
        const double requested_end=argc>=5?std::stod(argv[4]):5.;
        const double pixel_noise=argc>=6?std::stod(argv[5]):0.;
        if(!std::isfinite(requested_end)||requested_end<2.||requested_end>12.||!std::isfinite(pixel_noise)||pixel_noise<0.||pixel_noise>4.)
            throw std::runtime_error("Undeclared path/noise range");
        amplitude=scenario=="curve"?.035:0.;terminal_x=(normal||terminal_b||terminal_pose_loss)?requested_end:100.;
        SimRL rl("/home/dong/rl_sar_b2_z1_old/policy/b2_z1_no_gun/VA15/scene_flat.xml",true,mu,0.);
        for(const auto* name:{"isolated_forward_trial","isolated_forward_trial_motion","near_visual_trial",
                             "repeatable_visual_session","continuous_visual_session"})rl.params.config_node[name]=true;
        auto loco=std::dynamic_pointer_cast<RLFSMStateRLLocomotion>(rl.fsm.states_.at("RLFSMStateRLLocomotion"));
        loco->trial_configured=false;loco->Enter();
        va15_visual::Guide guide(true,true,true);va15_tcp::Tracker tracker;va15_tcp::History poses;
        va15_visual_end::RearCompletion end;
        tracker.training_workspace_recovery=true;
        va15_independent::Observer observer(LoadIndependentModel((fs::path(__FILE__).parent_path()/
            "production/b2_independent_model.json").string()),true);
        std::array<int,12> dds_q,dds_v;const char* legs[]={"FR","FL","RR","RL"};
        const char* joints[]={"hip","thigh","calf"};
        for(int l=0;l<4;++l)for(int j=0;j<3;++j) {
            const auto name=std::string(legs[l])+"_"+joints[j]+"_joint";
            const int id=mj_name2id(rl.m,mjOBJ_JOINT,name.c_str());
            if(id<0)throw std::runtime_error("Missing leg joint");
            dds_q[3*l+j]=rl.m->jnt_qposadr[id];dds_v[3*l+j]=rl.m->jnt_dofadr[id];
        }
        const int imu=mj_name2id(rl.m,mjOBJ_SENSOR,"imu_acc");if(imu<0)throw std::runtime_error("Missing IMU");
        const double initial=rl.d->time;std::uint64_t seq=0,last_supported=0;
        std::deque<va15_visual::Packet> delayed;va15_visual::Packet latest;
        bool started=false,fell=false,prone=false,ready=false,stable=false,unexpected=false,manual=false;
        double max_completion_error=0.,final_tcp_error=100.,cap_confirmed_at=-1.,first_tcp_active=-1.,max_target_step=0.;
        unsigned tcp_active_samples=0;
        double first_cap=-1.,confirmed=-1.,stop_time=-1.,quiet_since=-1.,max_tilt=0.;
        unsigned cap_frames=0;double stopped_x=-1.;
        std::ofstream csv(argv[2]);csv<<"time,x,y,height,tilt,guide,end_stage,end_frames,stop_stage,command_x,command_yaw,pose_valid,reason,remaining,lateral,z_error,xy_error,endpoint_x,endpoint_y,tcp_est_x,tcp_est_y,tcp_truth_x,tcp_truth_y,base_yaw,target_x,target_y,target_z,tcp_active,estimate_x,estimate_y,workspace_limited,allocation_error,visual_scale_valid,visual_scale,scale_baseline,scale_rms,endpoint_updates,stand_reference_valid,capture_rejection,capture_joint,capture_anchor,stop_gate_mask,stop_contacts\n";
        for(int step=0;step<26000;++step) {
            rl.ReadState();const double t=rl.d->time-initial;const auto ns=std::uint64_t((100.+t)*1.e9);
            const va15_tcp::Pose physical{ns,{rl.d->qpos[0],rl.d->qpos[1],rl.d->qpos[2]},
                Eigen::Quaterniond(rl.d->qpos[3],rl.d->qpos[4],rl.d->qpos[5],rl.d->qpos[6]).normalized(),true};
            if(step%2==0) {
                va15_independent::Sample sample;sample.mode.fill(10);sample.documented_motor_health=true;
                sample.quaternion=physical.q;
                for(int i=0;i<12;++i){sample.q[i]=rl.d->qpos[dds_q[i]];sample.dq[i]=rl.d->qvel[dds_v[i]];sample.tau[i]=rl.d->qfrc_actuator[dds_v[i]];}
                for(int i=0;i<3;++i){sample.gyro[i]=rl.robot_state.imu.gyroscope[i];sample.specific_force[i]=rl.d->sensordata[rl.m->sensor_adr[imu]+i];}
                observer.Update(sample,std::lround(rl.d->time*1000.),rl.d->time);
            }
            const auto estimate=observer.Snapshot(rl.d->time);
            rl.robot_state.real_stop_feedback=va15_trial::Feedback(estimate);
            if(va15_trial::Healthy(estimate)&&estimate.estimate_valid_for_review)last_supported=ns;
            va15_tcp::Pose pose=physical;pose.p=estimate.position;
            pose.valid=va15_trial::Healthy(estimate)&&last_supported&&ns-last_supported<=250000000ULL;
            if(terminal_pose_loss&&t>=28.)pose.valid=false;
            if(pose.valid) {
                if(step%2==0)poses.Add(pose);
                const auto velocity=(pose.q.conjugate()*estimate.velocity).eval();
                guide.ObserveBodyVelocity(velocity.x(),velocity.y(),.005);
            }
            if(step%30==0&&!(scenario=="camera_failure"&&t>=18.)) {
                // One plausible interior cap frame followed by continuing
                // tape must not satisfy temporal end confirmation.
                if(scenario=="one_false_cap"&&t>=14.&&t<14.15) {
                    va15_tcp::V3 at;
                    if(va15_tcp::Project(physical,{Pixel(physical,780.),780.},at))terminal_x=at.x();
                }
                auto packet=Frame(physical,++seq);
                if(pixel_noise>0.&&!packet.path.empty()) {
                    for(auto& uv:packet.path) {
                        const double row=uv[1];uv[0]+=pixel_noise*std::sin(.73*seq+.03*row);
                        uv[1]+=.5*pixel_noise*std::cos(.61*seq+.02*row);
                    }
                    packet.u=va15_visual::CurveU(packet,packet.v);packet.look_u=va15_visual::CurveU(packet,packet.look_v);
                    packet.path_span=packet.path.front()[1]-packet.path.back()[1];
                    packet.endpoint_u=packet.path.back()[0];packet.endpoint_v=packet.path.back()[1];
                }
                if(scenario=="one_false_cap")terminal_x=100.;
                if(scenario=="unknown_loss"&&t>=18.){packet.valid=false;packet.endpoint_candidate=false;}
                delayed.push_back(packet);
            }
            while(!delayed.empty()&&delayed.front().publish_ns<=ns) {
                latest=delayed.front();delayed.pop_front();guide.Observe(latest,ns);va15_tcp::Pose capture;
                if(va15_visual::Valid(guide.latest,ns)&&poses.At(guide.latest.capture_ns,capture))tracker.Observe(guide.latest,capture);
            }
            guide.metric_bearing_valid=tracker.Guidance(pose,guide.metric_bearing);
            guide.body_reference_valid=tracker.BodyGuidance(pose,guide.body_lateral,guide.body_heading,guide.body_history_used);
            rl.control.current_keyboard=Input::Keyboard::None;rl.control.current_gamepad=Input::Gamepad::None;
            if(t>=6.&&!started) {
                if(!guide.Start(ns,loco->RealStopSettled(),true,true))throw std::runtime_error(guide.reason);
                started=true;tracker.Begin(pose);end.Begin();
            }
            const bool operator_stop=(!normal&&t>=35.)||(operator_b&&t>=(terminal_b?28.:18.));
            if(operator_stop) {
                manual=true;guide.Stop(operator_b?"operator B getdown":"explicit X stop");end.Interrupt("operator stop");
                if(operator_b)rl.control.current_gamepad=Input::Gamepad::B;
            }
            if(started&&!pose.valid){guide.Stop("local estimate invalid");end.Interrupt("local estimate invalid");}
            if(guide.stage==va15_visual::Guide::Stage::Running) {
                va15_tcp::Pose capture;
                const bool exact=poses.At(latest.capture_ns,capture);
                end.Observe(latest,capture,ns,exact&&guide.qualified>=6&&guide.latest.sequence==latest.sequence&&va15_visual::Valid(guide.latest,ns),0);
                if(end.Confirmed()&&cap_confirmed_at<0.)cap_confirmed_at=t;
                if(end.frames&&first_cap<0.)first_cap=t;
                if(end.Requested()) {
                    if(confirmed<0.){confirmed=t;cap_frames=end.frames;}
                    guide.Stop(end.reason);
                }
            }
            va15_tcp::V3 measured_tcp=va15_tcp::V3::Zero(),truth_tcp=va15_tcp::V3::Zero();
            va15_tcp::Pose terminal_pose;
            const bool terminal_pose_valid=end.TerminalPose(pose,terminal_pose);
            const bool tcp_valid=pose.valid&&terminal_pose_valid&&va15_tcp::MeasuredTCPWorld(terminal_pose,rl.robot_state.motor_state.q,measured_tcp);
            va15_tcp::MeasuredTCPWorld(physical,rl.robot_state.motor_state.q,truth_tcp);
            final_tcp_error=std::hypot(truth_tcp.x()-terminal_x,truth_tcp.y()-Path(terminal_x));
            va15_visual::Output output;
            if(guide.stage==va15_visual::Guide::Stage::Running&&end.Confirmed()) {
                output=end.Tick(ns,.005,terminal_pose,measured_tcp,tcp_valid,0,va15_trial::Healthy(estimate),operator_stop,true,guide.yaw,guide.visual_integral);
                guide.yaw=output.yaw;
                guide.visual_integral=end.steering_integral;
                if(output.stop)guide.Stop(end.reason);
            } else output=guide.Tick(ns,.005,va15_trial::Healthy(estimate),operator_stop,true);
            if(end.Requested()&&confirmed<0.){confirmed=t;cap_frames=end.frames;max_completion_error=final_tcp_error;}
            if(output.stop) {
                loco->trial.Stop(guide.reason);
                if(stop_time<0.&&started)stop_time=t;
                if(!end.Requested())end.Interrupt("protective/manual stop, not visual completion");
                if(normal&&!end.Requested())unexpected=true;
            }
            end.Park(.005,va15_trial::Healthy(estimate)&&pose.valid,loco->controlled_stop.Ready()&&loco->RealStopSettled(),measured_tcp,tcp_valid);
            rl.control.x=output.x;rl.control.y=0.;rl.control.yaw=output.yaw;
            const auto prior_target=tracker.target;
            if(!end.Confirmed()||!tracker.StepEndpoint(terminal_pose,.005,end.endpoint))
                tracker.Step(end.Confirmed()?terminal_pose:pose,.005,guide.stage==va15_visual::Guide::Stage::Running);
            max_target_step=std::max(max_target_step,(tracker.target-prior_target).cwiseAbs().maxCoeff());
            if(tracker.active){++tcp_active_samples;if(first_tcp_active<0.)first_tcp_active=t;}
            rl.SetRuntimeTargetCommands({float(tracker.target.x()),float(tracker.target.y()),float(tracker.target.z()),0.,-.707107,0.,.707107});
            rl.StateController(&rl.robot_state,&rl.robot_command);
            if(!loco->trial.Forward())rl.control.x=rl.control.y=rl.control.yaw=0.;
            if(step%4==0)rl.Infer();
            const double tilt=rl.Tilt();max_tilt=std::max(max_tilt,tilt);
            const bool current_ready=loco->controlled_stop.Ready()&&loco->RealStopSettled();
            if(started&&stop_time>=0.&&current_ready) {
                ready=true;stopped_x=physical.p.x();
                if(quiet_since<0.)quiet_since=t;
                if(t-quiet_since>=2.&&(!normal||end.Stood()))stable=true;
            }else quiet_since=-1.;
            if(step%10==0)csv<<t<<','<<physical.p.x()<<','<<physical.p.y()<<','<<physical.p.z()<<','<<tilt
                <<','<<int(guide.stage)<<','<<int(end.stage)<<','<<end.frames<<','<<int(loco->controlled_stop.stage)
                <<','<<rl.control.x<<','<<rl.control.yaw<<','<<pose.valid<<','<<end.reason
                <<','<<end.remaining<<','<<end.lateral<<','<<end.height_error<<','<<end.distance
                <<','<<end.endpoint.x()<<','<<end.endpoint.y()<<','<<measured_tcp.x()<<','<<measured_tcp.y()
                <<','<<truth_tcp.x()<<','<<truth_tcp.y()<<','<<std::atan2(physical.q.toRotationMatrix()(1,0),physical.q.toRotationMatrix()(0,0))
                <<','<<tracker.target.x()<<','<<tracker.target.y()<<','<<tracker.target.z()<<','<<tracker.active
                <<','<<pose.p.x()<<','<<pose.p.y()<<','<<tracker.workspace_limited<<','<<tracker.allocation_error
                <<','<<end.visual_scale.valid<<','<<end.visual_scale.scale<<','<<end.visual_scale.baseline<<','<<end.visual_scale.rms
                <<','<<end.endpoint_updates<<','<<loco->controlled_stop.stand_reference_valid
                <<','<<loco->controlled_stop.capture_rejection<<','<<loco->controlled_stop.capture_rejected_joint
                <<','<<loco->controlled_stop.capture_rejected_anchor<<','<<loco->controlled_stop.diagnostic.mask
                <<','<<loco->controlled_stop.diagnostic.contacts<<'\n';
            if(!std::isfinite(tilt)||tilt>.5){fell=true;break;}
            if(rl.fsm.current_state_->GetStateName()=="RLFSMStatePassive"&&physical.p.z()<.2){prone=true;break;}
            if(stable&&!operator_b)break;
            rl.Integrate();
        }
        const bool proof=normal?end.Stood()&&cap_frames>=6&&cap_confirmed_at-first_cap>=.5&&confirmed>cap_confirmed_at+3.&&max_completion_error<=.10&&final_tcp_error<=.12:!end.Requested();
        const bool timing=normal?confirmed>10.:
            (terminal_pose_loss?stop_time>=28.&&stop_time<29.:scenario=="unknown_loss"||scenario=="camera_failure"?stop_time>=18.&&stop_time<20.:stop_time>=(operator_b?(terminal_b?28.:18.):35.));
        const bool pass=started&&!fell&&!unexpected&&proof&&timing&&max_target_step<=.000201&&
            (!normal||tcp_active_samples>100)&&(operator_b?prone:stable&&!prone);
        std::ofstream report(std::string(argv[2])+".json");report<<std::boolalpha
            <<"{\"pass\":"<<pass<<",\"scenario\":\""<<scenario<<"\",\"friction\":"<<mu
            <<",\"fell\":"<<fell<<",\"prone\":"<<prone<<",\"original_stop_ready\":"<<ready<<",\"stable_stop\":"<<stable
            <<",\"normal_visual_end_stood\":"<<end.Stood()<<",\"cap_frames\":"<<cap_frames<<",\"first_cap_s\":"<<first_cap
            <<",\"confirmed_end_s\":"<<confirmed<<",\"stop_requested_s\":"<<stop_time<<",\"stop_x_m\":"<<stopped_x
            <<",\"cap_confirmed_s\":"<<cap_confirmed_at<<",\"truth_tcp_error_at_brake_m\":"<<max_completion_error<<",\"final_truth_tcp_xy_error_m\":"<<final_tcp_error
            <<",\"physical_endpoint_x_m\":"<<terminal_x<<",\"max_tilt_rad\":"<<max_tilt<<",\"manual_stop\":"<<manual
            <<",\"tcp_active_samples\":"<<tcp_active_samples<<",\"first_tcp_active_s\":"<<first_tcp_active<<",\"max_target_step_m\":"<<max_target_step
            <<",\"visual_scale_valid\":"<<end.visual_scale.valid<<",\"visual_scale\":"<<end.visual_scale.scale<<",\"scale_rms_m\":"<<end.visual_scale.rms
            <<",\"endpoint_observation_updates\":"<<end.endpoint_updates<<",\"pixel_noise_px\":"<<pixel_noise
            <<",\"synthetic_rgb\":true,\"independent_estimated_pose\":true,\"rear_tcp_arrival_required\":true,\"hardware_qualified\":false}\n";
        return pass?0:2;
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}

