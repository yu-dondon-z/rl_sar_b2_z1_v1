// SDK-free physics with the actual VA15 actor/FSM. Synthetic calibrated RGB
// coordinates and ground-truth poses ONLY in this harness; not real odometry
// or a segmentation/hardware qualification. Legacy suites test the observer.
#include "production/fsm_b2_z1_no_gun.hpp"
#include "production/visual_tcp.hpp"
#include "production/independent_model_io.hpp"
#define main archived_simulation_main_not_called
#include "../va15_controlled_getdown_20261020/simulate_controlled_getdown.cpp"
#undef main
#include <deque>

static double amplitude=.035;
static double Path(double x){return amplitude*std::sin(.8*x);}
static double Pixel(const va15_tcp::Pose& pose,double v) {
    double low=384.,high=1536.;
    for(int i=0;i<45;++i) {
        const double u=.5*(low+high);va15_tcp::V3 world;
        if(!va15_tcp::Project(pose,{u,v},world))return NAN;
        if(world.y()-Path(world.x())>0.)low=u;else high=u;
    }
    return .5*(low+high);
}
int main(int argc,char** argv) {
    try {
        if(argc!=4||fs::exists(argv[2]))throw std::runtime_error("scenario new.csv friction");
        const std::string scenario=argv[1];const double mu=std::stod(argv[3]);
        const bool estimated=scenario.rfind("estimated",0)==0;
        if(scenario=="estimated_straight"||scenario.rfind("estimated_probe",0)==0)amplitude=0.;
        SimRL rl("/home/dong/rl_sar_b2_z1_old/policy/b2_z1_no_gun/VA15/scene_flat.xml",true,mu,0.);
        rl.params.config_node["isolated_forward_trial"]=true;
        rl.params.config_node["isolated_forward_trial_motion"]=true;
        rl.params.config_node["near_visual_trial"]=true;
        rl.params.config_node["repeatable_visual_session"]=true;
        rl.params.config_node["continuous_visual_session"]=true;
        auto loco=std::dynamic_pointer_cast<RLFSMStateRLLocomotion>(rl.fsm.states_.at("RLFSMStateRLLocomotion"));
        loco->trial_configured=false;loco->Enter();
        va15_visual::Guide guide(true,true,true);va15_tcp::Tracker tracker;va15_tcp::History poses;
        va15_independent::Observer observer(LoadIndependentModel((fs::path(__FILE__).parent_path()/"production/b2_independent_model.json").string()),true);
        std::array<int,12> dds_q,dds_v;const char* leg_names[]={"FR","FL","RR","RL"};
        const char* joint_names[]={"hip","thigh","calf"};
        for(int l=0;l<4;++l)for(int j=0;j<3;++j) {
            const auto name=std::string(leg_names[l])+"_"+joint_names[j]+"_joint";
            const int id=mj_name2id(rl.m,mjOBJ_JOINT,name.c_str());
            dds_q[3*l+j]=rl.m->jnt_qposadr[id];dds_v[3*l+j]=rl.m->jnt_dofadr[id];
        }
        const int imu=mj_name2id(rl.m,mjOBJ_SENSOR,"imu_acc");
        std::deque<va15_visual::Packet> delayed;
        const double initial=rl.d->time;std::uint64_t seq=0,last_supported_ns=0;
        bool started=false,fell=false,prone=false,did_track=false,unexpected_stop=false;
        double max_tilt=0.,max_target_step=0.,target_y_max=0.,distance=0.,tcp_error_sq=0.,start_x=0.;
        double first_tracking=-1.,steady_error_sq=0.,max_body_lateral=0.;unsigned steady_samples=0;
        double late_body_error_sq=0.;unsigned late_body_samples=0;
        double xyz_error_sq=0.,vertical_error_sq=0.,longitudinal_error_sq=0.,steady_xyz_error_sq=0.;
        unsigned tracked=0;const int tcp=mj_name2id(rl.m,mjOBJ_SITE,"ee_site");
        va15_tcp::V3 previous_position=va15_tcp::V3::Zero();bool have_previous=false;
        std::ofstream csv(argv[2]);csv<<"time,x,y,height,tilt,guide,stop,cache,active,target_x,target_y,target_z,tcp_z,tcp_lateral,course_slip,yaw_command,body_e,body_heading,body_history,rear_crossings,workspace_rejections,rear_gap,estimate_x,estimate_y,yaw\n";
        for(int step=0;step<24000;++step) {
            rl.ReadState();const double t=rl.d->time-initial;
            // The private FSM explicitly consumes tagged estimate feedback.
            // Only this synthetic-pose physics harness adapts MuJoCo truth;
            // do NOT leave it branded as production SportMode feedback.
            rl.robot_state.real_stop_feedback.sport_fresh=false;
            rl.robot_state.real_stop_feedback.unavailable_reason=va15_trial::Source;
            const auto ns=std::uint64_t((100.+t)*1.e9);
            va15_tcp::Pose pose{ns,va15_tcp::V3(rl.d->qpos[0],rl.d->qpos[1],rl.d->qpos[2]),
                Eigen::Quaterniond(rl.d->qpos[3],rl.d->qpos[4],rl.d->qpos[5],rl.d->qpos[6]).normalized(),true};
            const auto render_pose=pose;
            if(estimated) {
                if(step%2==0) {
                    va15_independent::Sample sample;sample.mode.fill(10);sample.documented_motor_health=true;
                    sample.quaternion=pose.q;
                    for(int i=0;i<12;++i){sample.q[i]=rl.d->qpos[dds_q[i]];sample.dq[i]=rl.d->qvel[dds_v[i]];sample.tau[i]=rl.d->qfrc_actuator[dds_v[i]];}
                    for(int j=0;j<3;++j){sample.gyro[j]=rl.robot_state.imu.gyroscope[j];sample.specific_force[j]=rl.d->sensordata[rl.m->sensor_adr[imu]+j];}
                    observer.Update(sample,std::lround(rl.d->time*1000.),rl.d->time);
                }
                const auto estimate=observer.Snapshot(rl.d->time);
                rl.robot_state.real_stop_feedback=va15_trial::Feedback(estimate);
                if(va15_trial::Healthy(estimate)&&estimate.estimate_valid_for_review)last_supported_ns=ns;
                pose.p=estimate.position;pose.valid=va15_trial::Healthy(estimate)&&last_supported_ns&&ns-last_supported_ns<=250000000ULL;
                if(pose.valid) {
                    if(step%2==0)poses.Add(pose);
                    const auto velocity=(pose.q.conjugate()*estimate.velocity).eval();
                    guide.ObserveBodyVelocity(velocity.x(),velocity.y(),.005);
                }
            } else poses.Add(pose);
            if(!estimated&&have_previous) {
                const auto velocity=(pose.q.conjugate()*((pose.p-previous_position)/.005)).eval();
                guide.ObserveBodyVelocity(velocity.x(),velocity.y(),.005);
            }
            previous_position=pose.p;have_previous=true;
            if(step%30==0) {
                va15_visual::Packet packet;packet.valid=packet.session_matched=true;packet.sequence=++seq;
                packet.capture_ns=ns;packet.publish_ns=ns+100000000;
                const bool adaptive=scenario=="estimated_adaptive"&&int(t*2.)%2;
                packet.v=972.;packet.look_v=adaptive?852.:820.8;
                packet.u=Pixel(render_pose,packet.v);packet.look_u=Pixel(render_pose,packet.look_v);
                packet.confidence=packet.look_confidence=.99;packet.width=30;packet.contrast=.8;packet.rms=1.;
                for(int v=1044;v>=(adaptive?840:744);v-=6)packet.path.push_back({Pixel(render_pose,v),double(v)});
                packet.path_count=packet.path.size();packet.path_span=adaptive?204.:300.;
                if(scenario.find("dropout")!=std::string::npos&&t>=18.&&t<18.15)packet.valid=false;
                // Inject loss after the long front-to-rear warmup, so the
                // regression exercises an actually active TCP, not a forced
                // early target. No runtime timeout is added to production.
                if(scenario=="estimated_loss"&&t>=55.&&t<57.)packet.valid=false;
                delayed.push_back(packet);
            }
            while(!delayed.empty()&&delayed.front().publish_ns<=ns) {
                guide.Observe(delayed.front(),ns);va15_tcp::Pose capture;
                if(va15_visual::Valid(guide.latest,ns)&&poses.At(guide.latest.capture_ns,capture))tracker.Observe(guide.latest,capture);
                delayed.pop_front();
            }
            guide.metric_bearing_valid=tracker.Guidance(pose,guide.metric_bearing);
            guide.body_reference_valid=tracker.BodyGuidance(pose,guide.body_lateral,guide.body_heading,guide.body_history_used);
            rl.control.current_keyboard=Input::Keyboard::None;rl.control.current_gamepad=Input::Gamepad::None;
            if(t>=6.&&!started) {
                started=true;start_x=rl.d->qpos[0];
                if(!guide.Start(ns,loco->RealStopSettled(),true,true))throw std::runtime_error(guide.reason);
                tracker.Begin(pose);
            }
            if(t>=106.)guide.Stop("explicit X stop");
            if(scenario=="estimated_pose_loss"&&t>=55.&&t<55.4)pose.valid=false;
            if(started&&!pose.valid)guide.Stop("local estimate invalid");
            auto output=guide.Tick(ns,.005,true,false,true);
            if(output.stop)loco->trial.Stop(guide.reason);
            if(started&&t<106.&&output.stop)unexpected_stop=true;
            rl.control.x=output.x;rl.control.y=0.;rl.control.yaw=output.yaw;
            const auto before=tracker.target;
            tracker.Step(pose,.005,guide.stage==va15_visual::Guide::Stage::Running);
            rl.SetRuntimeTargetCommands({float(tracker.target.x()),float(tracker.target.y()),float(tracker.target.z()),0.,-.707107,0.,.707107});
            max_target_step=std::max(max_target_step,(tracker.target-before).cwiseAbs().maxCoeff());
            target_y_max=std::max(target_y_max,std::fabs(tracker.target.y()));
            if(tracker.active) {
                did_track=true;++tracked;
                const double error=rl.d->site_xpos[3*tcp+1]-Path(rl.d->site_xpos[3*tcp]);tcp_error_sq+=error*error;
                const va15_tcp::V3 actual(rl.d->site_xpos[3*tcp],rl.d->site_xpos[3*tcp+1],rl.d->site_xpos[3*tcp+2]);
                const va15_tcp::V3 desired=render_pose.p+render_pose.q*tracker.target;
                const va15_tcp::V3 error_body=render_pose.q.conjugate()*(actual-desired);
                xyz_error_sq+=(actual-desired).squaredNorm();vertical_error_sq+=std::pow(actual.z()-desired.z(),2);
                longitudinal_error_sq+=error_body.x()*error_body.x();
                if(first_tracking<0.)first_tracking=t;
                if(t-first_tracking>=8.){steady_error_sq+=error*error;steady_xyz_error_sq+=(actual-desired).squaredNorm();++steady_samples;}
            }
            if(t>=112.)rl.control.current_gamepad=Input::Gamepad::B;
            rl.StateController(&rl.robot_state,&rl.robot_command);
            if(!loco->trial.Forward())rl.control.x=rl.control.y=rl.control.yaw=0.;
            // OFFLINE ONLY: characterize the actor's lateral response while
            // it is already walking. No real launcher enables these probes.
            if(loco->trial.Forward()) {
                if(scenario=="lateral_02")rl.control.y=.02;
                if(scenario=="lateral_04")rl.control.y=.04;
                if(scenario=="lateral_06")rl.control.y=.06;
                if(scenario=="estimated_lateral_02")rl.control.y=.02;
                if(scenario=="estimated_lateral_04")rl.control.y=.04;
                if(scenario=="estimated_lateral_06")rl.control.y=.06;
                if(scenario=="estimated_probe_p15"){rl.control.y=.15;rl.control.yaw=0.;}
                if(scenario=="estimated_probe_n15"){rl.control.y=-.15;rl.control.yaw=0.;}
                if(scenario=="estimated_probe_n06"){rl.control.y=-.06;rl.control.yaw=0.;}
            }
            if(step%4==0)rl.Infer();
            max_tilt=std::max(max_tilt,double(rl.Tilt()));
            if(t<106.){distance=rl.d->qpos[0]-start_x;max_body_lateral=std::max(max_body_lateral,std::fabs(rl.d->qpos[1]-Path(rl.d->qpos[0])));}
            if(t>=70.&&t<106.&&!output.stop){late_body_error_sq+=std::pow(rl.d->qpos[1]-Path(rl.d->qpos[0]),2);++late_body_samples;}
            if(step%10==0)csv<<t<<','<<rl.d->qpos[0]<<','<<rl.d->qpos[1]<<','<<rl.d->qpos[2]<<','<<rl.Tilt()
                <<','<<int(guide.stage)<<','<<int(loco->controlled_stop.stage)<<','<<tracker.curves.size()<<','<<tracker.active
                <<','<<tracker.target.x()<<','<<tracker.target.y()<<','<<tracker.target.z()<<','<<rl.d->site_xpos[3*tcp+2]
                <<','<<rl.d->site_xpos[3*tcp+1]-Path(rl.d->site_xpos[3*tcp])
                <<','<<guide.course_slip<<','<<output.yaw<<','<<guide.body_lateral<<','<<guide.body_heading<<','<<guide.body_history_used
                <<','<<tracker.rear_crossings<<','<<tracker.workspace_rejections<<','<<tracker.nearest_rear_gap
                <<','<<pose.p.x()<<','<<pose.p.y()<<','<<std::atan2(2.*(pose.q.w()*pose.q.z()+pose.q.x()*pose.q.y()),1.-2.*(pose.q.y()*pose.q.y()+pose.q.z()*pose.q.z()))<<'\n';
            if(!std::isfinite(rl.Tilt())||rl.Tilt()>.5){fell=true;break;}
            if(rl.fsm.current_state_->GetStateName()=="RLFSMStatePassive"&&rl.d->qpos[2]<.2){prone=true;break;}
            rl.Integrate();
        }
        const bool intended_loss=scenario=="estimated_loss"||scenario=="estimated_pose_loss";
        const double late_body_rms=late_body_samples?std::sqrt(late_body_error_sq/late_body_samples):-1.;
        const double steady_tcp_rms=steady_samples?std::sqrt(steady_error_sq/steady_samples):-1.;
        const bool pass=started&&tracked>=1000&&!fell&&(unexpected_stop==intended_loss)&&prone&&distance>2.&&max_target_step<.000201&&
            (intended_loss||(late_body_samples>1000&&late_body_rms<.06&&steady_tcp_rms>=0.&&steady_tcp_rms<.08));
        std::ofstream report(std::string(argv[2])+".json");report<<std::boolalpha
            <<"{\"pass\":"<<pass<<",\"scenario\":\""<<scenario<<"\",\"friction\":"<<mu
            <<",\"tracked\":"<<did_track<<",\"fell\":"<<fell<<",\"prone\":"<<prone
            <<",\"unexpected_stop\":"<<unexpected_stop<<",\"forward_m\":"<<distance
            <<",\"max_tilt_rad\":"<<max_tilt<<",\"max_target_step_m\":"<<max_target_step
            <<",\"max_target_y_m\":"<<target_y_max<<",\"tcp_lateral_rms_m\":"<<std::sqrt(tcp_error_sq/std::max(1u,tracked))
            <<",\"tcp_first_active_s\":"<<first_tracking<<",\"tcp_active_seconds\":"<<tracked*.005
            <<",\"steady_lateral_rms_m\":"<<(steady_samples?std::sqrt(steady_error_sq/steady_samples):-1.)
            <<",\"max_body_lateral_m\":"<<max_body_lateral
            <<",\"late_body_lateral_rms_m\":"<<late_body_rms
            <<",\"tcp_xyz_command_rms_m\":"<<std::sqrt(xyz_error_sq/std::max(1u,tracked))
            <<",\"tcp_vertical_command_rms_m\":"<<std::sqrt(vertical_error_sq/std::max(1u,tracked))
            <<",\"tcp_body_x_command_rms_m\":"<<std::sqrt(longitudinal_error_sq/std::max(1u,tracked))
            <<",\"steady_xyz_command_rms_m\":"<<(steady_samples?std::sqrt(steady_xyz_error_sq/steady_samples):-1.)
            <<",\"synthetic_rgb\":true,\"truth_pose_harness_only\":"<<!estimated<<",\"independent_estimated_pose\":"<<estimated
            <<",\"hardware_qualified\":false}\n";
        return pass?0:2;
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
