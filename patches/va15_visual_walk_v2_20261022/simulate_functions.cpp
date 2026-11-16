// SDK-free whole-body functional characterisation. Ground truth is used ONLY
// in this simulation, never as a real sensor, contact qualification or odometry.
#include "production/fsm_b2_z1_no_gun.hpp"
#include "production/near_visual_guide.hpp"
#include "production/visual_review.hpp"
#define main archived_simulation_main_not_called
#include "../va15_controlled_getdown_20261020/simulate_controlled_getdown.cpp"
#undef main
#include <deque>

double Pixel(double bearing,double v=972.) {
    double lo=600.,hi=1250.;
    for(int k=0;k<45;++k) {const double u=.5*(lo+hi);
        if(va15_visual::Bearing(u,v)>bearing)lo=u;else hi=u;}
    return .5*(lo+hi);
}

int main(int argc,char** argv) {
    try {
        if((argc<3||argc>5)||fs::exists(argv[2]))throw std::runtime_error("scenario new-output.csv [OFFLINE feedback-gain multiplier [friction]] required");
        const std::string scenario=argv[1];
        const bool visual=scenario.rfind("visual_",0)==0;
        const double offline_gain=argc>=4?std::stod(argv[3]):1.;
        const double friction=argc>=5?std::stod(argv[4]):.8;
        if(!std::isfinite(offline_gain)||offline_gain<.25||offline_gain>4.||(!visual&&argc>=4)||
           !std::isfinite(friction)||friction<.5||friction>1.)
            throw std::runtime_error("Invalid SDK-free gain experiment");
        const bool recognised=visual||scenario=="forward"||scenario=="backward"||scenario=="left"||
            scenario=="right"||scenario=="yaw_left"||scenario=="yaw_right"||scenario=="tcp_sweep"||scenario=="standby";
        if(!recognised)throw std::runtime_error("Unknown scenario");
        SimRL rl("/home/dong/rl_sar_b2_z1_old/policy/b2_z1_no_gun/VA15/scene_flat.xml",true,friction,0.);
        rl.params.config_node["isolated_forward_trial"]=visual;
        rl.params.config_node["isolated_forward_trial_motion"]=visual;
        rl.params.config_node["near_visual_trial"]=visual;
        auto loco=std::dynamic_pointer_cast<RLFSMStateRLLocomotion>(rl.fsm.states_.at("RLFSMStateRLLocomotion"));
        loco->trial_configured=false;loco->Enter();
        const int tcp=mj_name2id(rl.m,mjOBJ_SITE,"ee_site");
        if(tcp<0)throw std::runtime_error("TCP site missing");
        const auto nominal=rl.params.Get<std::vector<float>>("target_commands");
        auto target=nominal;
        va15_visual::Guide guide;va15_visual::Output output;
        va15_visual::Review review;
        std::deque<va15_visual::Packet> delay;
        std::uint64_t sequence=0;
        const double initial=rl.d->time;
        bool fell=false,prone=false,ever=false,start_attempted=false,stopped=false;
        bool second_request=false,review_passed=false,review_leak=false;
        double max_tilt=0.,min_height=1.,max_yaw=0.,positive=0.,tcp_sq=0.,leg_dq_sq=0.,tcp_vertical_sq=0.;
        double first_x=0.,first_y=0.,first_heading=0.,last_x=0.,last_y=0.,last_heading=0.,initial_tcp_z=0.;
        double max_command_jump=0.,last_yaw_command=0.;unsigned samples=0;
        std::array<double,4> air{};
        std::ofstream csv(argv[2]);
        csv<<"time,state,x,y,height,tilt,yaw,command_x,command_y,command_yaw,policy_x,policy_y,policy_yaw,tcp_error_m,tcp_z,visual_stage\n";
        for(int step=0;step<6800;++step) {
            rl.ReadState();const double t=rl.d->time-initial;
            const auto truth=rl.robot_state.real_stop_feedback;
            // SDK-FREE HARNESS ONLY: give the trial FSM a synthetic truth
            // adapter. This does NOT test/qualify a real state estimator.
            // The separate 34-case suite exercises the independent observer.
            if(visual) {
                rl.robot_state.real_stop_feedback.sport_fresh=false;
                rl.robot_state.real_stop_feedback.unavailable_reason=va15_trial::Source;
            }
            const auto q=QuaternionYawOnly(rl.robot_state.imu.quaternion);
            const double heading=2.*std::atan2(q[3],q[0]);
            const auto ns=static_cast<std::uint64_t>((t+100.)*1.e9);
            rl.control.current_keyboard=Input::Keyboard::None;
            rl.control.current_gamepad=Input::Gamepad::None;
            rl.control.x=rl.control.y=rl.control.yaw=0.f;
            if(visual) {
                if(step%20==0) {
                    va15_visual::Packet p;p.valid=p.session_matched=true;
                    p.sequence=++sequence;p.capture_ns=ns;p.publish_ns=ns+120000000;
                    const double curve=scenario=="visual_left"?.015:scenario=="visual_right"?-.015:0.;
                    const double offset=scenario=="visual_offset_left"?.08:scenario=="visual_offset_right"?-.08:0.;
                    const double x=rl.d->qpos[0]+.9;
                    const double y=offset+curve*x*x;
                    const double bearing=std::atan2(y-rl.d->qpos[1],.9)-heading;
                    // OFFLINE gain sensitivity experiment only. Default 1
                    // is the actual geometric bearing used for qualification.
                    p.u=Pixel(std::clamp(bearing*offline_gain,-.3,.3));p.v=972.;
                    p.confidence=.99;p.contrast=.8;p.width=30.;p.rms=1.;
                    const double far_x=rl.d->qpos[0]+1.5;
                    const double far_bearing=std::atan2(offset+curve*far_x*far_x-rl.d->qpos[1],1.5)-heading;
                    p.look_v=.76*1080.;p.look_u=Pixel(std::clamp(far_bearing*offline_gain,-.3,.3),p.look_v);
                    p.look_confidence=.99;p.path_count=50;p.path_span=300.;
                    if(scenario=="visual_invalid"&&t>=13.)p.valid=false;
                    if(scenario!="visual_stale"||t<13.)delay.push_back(p);
                }
                while(!delay.empty()&&delay.front().publish_ns<=ns) {
                    guide.Observe(delay.front(),ns);review.Observe(delay.front(),ns);delay.pop_front();
                }
                if(t>=6.&&!start_attempted) {
                    start_attempted=true;
                    review.Start(guide,ns,loco->RealStopSettled()&&loco->trial.stage==va15_trial::Supervisor::Stage::Armed,true,true);
                }
                const bool quiet=loco->RealStopSettled()&&loco->trial.stage==va15_trial::Supervisor::Stage::Armed;
                review.audit.Notify(ns,scenario=="visual_review_writer"&&t>=8.?30000000:2000000,ns-1000000,true);
                if(guide.stage!=va15_visual::Guide::Stage::Running)
                    review.Tick(guide,ns,.005,quiet,true,false,true);
                review_passed=review_passed||review.approved;
                // Two explicit operator events, never an automatic transition
                // at review completion. The actual guide stays idle until 11s.
                if(t>=11.&&!second_request) {
                    second_request=true;
                    if(review.approved)guide.Start(ns,quiet,true,true);
                }
                output=guide.Tick(ns,.005,true,scenario=="visual_x"&&t>=13.,true);
                if(t<11.&&(output.x!=0.||output.yaw!=0.))review_leak=true;
                if(output.stop) {loco->trial.Stop(guide.reason);stopped=true;}
                rl.control.x=output.x;rl.control.yaw=output.yaw;
            } else if(t>=6.&&t<14.) {
                if(scenario=="forward"||scenario=="tcp_sweep")rl.control.x=.15f;
                if(scenario=="backward")rl.control.x=-.15f;
                if(scenario=="left")rl.control.y=.15f;
                if(scenario=="right")rl.control.y=-.15f;
                if(scenario=="yaw_left")rl.control.yaw=.06f;
                if(scenario=="yaw_right")rl.control.yaw=-.06f;
            }
            if(scenario=="tcp_sweep") {
                target=nominal;
                target[1]=t>=6.? .03f*std::sin((t-6.)*.5):0.f;
                rl.SetRuntimeTargetCommands(target);
            }
            if(t>=(visual?23.:18.))rl.control.current_gamepad=Input::Gamepad::B;
            rl.StateController(&rl.robot_state,&rl.robot_command);
            if(visual&&!loco->trial.Forward())rl.control.x=rl.control.y=rl.control.yaw=0.f;
            if(step%4==0)rl.Infer();
            max_tilt=std::max(max_tilt,double(rl.Tilt()));
            const bool moving=std::hypot(rl.control.x,rl.control.y)>.03||std::fabs(rl.control.yaw)>.02;
            if(moving) {
                if(!ever) {first_x=rl.d->qpos[0];first_y=rl.d->qpos[1];first_heading=heading;initial_tcp_z=rl.d->site_xpos[3*tcp+2];}
                ever=true;positive+=.005;last_x=rl.d->qpos[0];last_y=rl.d->qpos[1];last_heading=heading;
                min_height=std::min(min_height,double(truth.base_height));
                max_yaw=std::max(max_yaw,std::fabs(double(rl.control.yaw)));
                for(int i=0;i<4;++i)if(!truth.contacts[i])air[i]+=.005;
            }
            const double dx=rl.d->site_xpos[3*tcp]-rl.d->qpos[0],dy=rl.d->site_xpos[3*tcp+1]-rl.d->qpos[1];
            const double tx=std::cos(heading)*dx+std::sin(heading)*dy,ty=-std::sin(heading)*dx+std::cos(heading)*dy;
            const double tz=rl.d->site_xpos[3*tcp+2]-rl.d->qpos[2];
            const double error=std::sqrt(std::pow(tx-target[0],2)+std::pow(ty-target[1],2)+std::pow(tz-target[2],2));
            if(moving&&t>7.) {
                ++samples;tcp_sq+=error*error;tcp_vertical_sq+=std::pow(rl.d->site_xpos[3*tcp+2]-initial_tcp_z,2);
                for(int j:VA15LegIndices)leg_dq_sq+=std::pow(rl.robot_state.motor_state.dq[j],2);
            }
            if(visual&&output.x>0.)max_command_jump=std::max(max_command_jump,std::fabs(output.yaw-last_yaw_command));
            last_yaw_command=output.yaw;
            const auto state=rl.fsm.current_state_->GetStateName();
            if(step%4==0)csv<<t<<','<<state<<','<<rl.d->qpos[0]<<','<<rl.d->qpos[1]<<','<<truth.base_height<<','<<rl.Tilt()<<','<<heading
                <<','<<rl.control.x<<','<<rl.control.y<<','<<rl.control.yaw<<','<<rl.obs.commands[0]<<','<<rl.obs.commands[1]<<','<<rl.obs.commands[2]
                <<','<<error<<','<<rl.d->site_xpos[3*tcp+2]<<','<<int(guide.stage)<<'\n';
            if(!std::isfinite(rl.Tilt())||rl.Tilt()>.5){fell=true;break;}
            if(state=="RLFSMStatePassive"&&truth.base_height<.2){prone=true;break;}
            rl.Integrate();
        }
        const double dx=last_x-first_x,dy=last_y-first_y;
        const double forward=std::cos(first_heading)*dx+std::sin(first_heading)*dy;
        const double lateral=-std::sin(first_heading)*dx+std::cos(first_heading)*dy;
        double yaw=std::atan2(std::sin(last_heading-first_heading),std::cos(last_heading-first_heading));
        // X is deliberately injected earlier than the full request; require
        // observable movement, not the full-distance threshold of that run.
        const double required_forward=scenario=="visual_x"?.03:.1;
        const bool refused=scenario=="visual_review_writer";
        const bool invariants=!fell&&prone&&(!visual||(!review_leak&&target==nominal&&
            (refused?(!ever&&!review_passed):(review_passed&&ever&&stopped&&forward>required_forward&&max_yaw<=.060001&&
            positive<=4.005&&max_command_jump<=.000201))));
        const bool direction=scenario=="backward"?forward<-.1:scenario=="left"?lateral>.1:scenario=="right"?lateral<-.1:
            scenario=="yaw_left"?yaw>.04:scenario=="yaw_right"?yaw<-.04:scenario=="standby"?!ever:forward>required_forward;
        std::ofstream report(std::string(argv[2])+".json");report<<std::boolalpha
            <<"{\"scenario\":\""<<scenario<<"\",\"simulation_invariants_pass\":"<<invariants
            <<",\"direction_pass\":"<<direction<<",\"fell\":"<<fell<<",\"prone\":"<<prone<<",\"ever_requested\":"<<ever
            <<",\"max_tilt_rad\":"<<max_tilt<<",\"min_moving_height_m\":"<<min_height<<",\"request_s\":"<<positive
            <<",\"forward_m\":"<<forward<<",\"lateral_m\":"<<lateral<<",\"yaw_change_rad\":"<<yaw
            <<",\"tcp_rms_m\":"<<(samples?std::sqrt(tcp_sq/samples):-1.)
            <<",\"tcp_vertical_rms_from_start_m\":"<<(samples?std::sqrt(tcp_vertical_sq/samples):-1.)
            <<",\"leg_dq_rms\":"<<(samples?std::sqrt(leg_dq_sq/(12.*samples)):-1.)
            <<",\"max_yaw_command_rad_s\":"<<max_yaw<<",\"max_visual_yaw_step\":"<<max_command_jump
            <<",\"visual_stopped\":"<<stopped<<",\"offline_feedback_gain_multiplier\":"<<offline_gain
            <<",\"inline_review_passed\":"<<review_passed<<",\"motion_before_second_request\":"<<review_leak
            <<",\"friction\":"<<friction<<",\"hardware_qualified\":false,\"synthetic_camera_not_recognition_test\":true}\n";
        return invariants?0:2;
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
