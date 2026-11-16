// Headless physics, no Unitree SDK/DDS/UDP/hardware clients.
#include "production/fsm_b2_z1_no_gun.hpp"
#include "production/independent_model_io.hpp"
#define main archived_simulation_main_not_called
#include "../va15_controlled_getdown_20261020/simulate_controlled_getdown.cpp"
#undef main

int main(int argc,char** argv)
{
    try
    {
        if(argc!=6&&argc!=8)throw std::runtime_error("model output scenario friction tau_scale [offline_speed offline_seconds]");
        const std::string scenario=argv[3];const double friction=std::stod(argv[4]),tau_scale=std::stod(argv[5]);
        const bool steady=scenario=="steady";
        const double offline_speed=steady&&argc==8?std::stod(argv[6]):.1;
        const double offline_seconds=steady&&argc==8?std::stod(argv[7]):2.;
        if((steady&&argc!=8)||(!steady&&argc!=6)||!std::isfinite(offline_speed)||offline_speed<.03||offline_speed>.3||
           !std::isfinite(offline_seconds)||offline_seconds<2.||offline_seconds>12.)throw std::runtime_error("Invalid offline-only experiment");
        if(scenario!="deadline"&&scenario!="release"&&scenario!="x"&&scenario!="b"&&
           scenario!="lost"&&scenario!="stale"&&scenario!="preview"&&scenario!="held"&&!steady)throw std::runtime_error("Unknown scenario");
        if(fs::exists(argv[2]))throw std::runtime_error("Output already exists");
        SimRL rl("/home/dong/rl_sar_b2_z1_old/policy/b2_z1_no_gun/VA15/scene_flat.xml",true,friction,0.);
        // Only THIS SDK-free headless executable can exercise longer inputs.
        // The real binary's supervisor, 4 s wall deadline, gains and limits
        // are unchanged and are NOT linked/called by this experiment.
        rl.params.config_node["isolated_forward_trial"]=!steady;
        rl.params.config_node["isolated_forward_trial_motion"]=scenario!="preview";
        auto loco=std::dynamic_pointer_cast<RLFSMStateRLLocomotion>(rl.fsm.states_.at("RLFSMStateRLLocomotion"));
        loco->trial_configured=false;loco->Enter();
        va15_independent::Observer observer(LoadIndependentModel(argv[1]));
        const int imu=mj_name2id(rl.m,mjOBJ_SENSOR,"imu_acc");
        std::array<int,12> qids,vids;const char* legs[]={"FR","FL","RR","RL"};const char* joints[]={"hip","thigh","calf"};
        for(int i=0;i<4;++i)for(int j=0;j<3;++j)
        {const auto name=std::string(legs[i])+"_"+joints[j]+"_joint";const int id=mj_name2id(rl.m,mjOBJ_JOINT,name.c_str());
         if(id<0)throw std::runtime_error("joint missing");qids[3*i+j]=rl.m->jnt_qposadr[id];vids[3*i+j]=rl.m->jnt_dofadr[id];}
        if(imu<0)throw std::runtime_error("imu missing");
        const double initial=rl.d->time;double max_tilt=0.,min_walk_h=1.,last_received=initial,max_command=0.;
        unsigned false_catch=0,sport_leak=0;bool fell=false,ever_forward=false,held_stand=false,prone=false,stop_seen=false;
        double first_stop=-1.;std::uint32_t tick=0;
        double positive_request_seconds=0.;
        bool request_origin_set=false;
        double start_x=0.,start_y=0.,start_yaw=0.,forward_at_stop=0.,lateral_at_stop=0.,max_leg_error=0.;
        std::array<double,4> air_time{};
        std::ofstream csv(argv[2]);csv<<"time,state,trial_stage,stop_stage,request_s,command,policy_x,truth_h,truth_v,truth_tilt,estimate_valid,source_sport,fault,truth_x,truth_y,forward_displacement,lateral_displacement,max_leg_tracking_error\n";
        for(int step=0;step<5800;++step)
        {
            rl.ReadState();const auto truth=rl.robot_state.real_stop_feedback;
            const double now=rl.d->time,t=now-initial;
            const bool inject=(scenario=="lost"||scenario=="stale")&&t>=5.5;
            if(step%2==0)
            {
                va15_independent::Sample s;s.mode.fill(10);
                for(int i=0;i<12;++i){s.q[i]=rl.d->qpos[qids[i]];s.dq[i]=rl.d->qvel[vids[i]];s.tau[i]=tau_scale*rl.d->qfrc_actuator[vids[i]];}
                s.quaternion=Eigen::Quaterniond(rl.robot_state.imu.quaternion[0],rl.robot_state.imu.quaternion[1],rl.robot_state.imu.quaternion[2],rl.robot_state.imu.quaternion[3]);
                for(int i=0;i<3;++i){s.gyro[i]=rl.robot_state.imu.gyroscope[i];s.specific_force[i]=rl.d->sensordata[rl.m->sensor_adr[imu]+i];}
                if(inject&&scenario=="lost")s.lost[1]=1;
                if(!(inject&&scenario=="stale")){tick=std::lround(now*1000.);last_received=now;observer.Update(s,tick,last_received);}
            }
            auto estimate=observer.Snapshot(now);
            rl.robot_state.real_stop_feedback=va15_trial::Feedback(estimate);
            if(rl.robot_state.real_stop_feedback.sport_fresh)++sport_leak;
            rl.control.current_keyboard=Input::Keyboard::None;rl.control.current_gamepad=Input::Gamepad::None;
            // A held request throughout preparation MUST NOT start walking.
            rl.control.x=(scenario=="held"||t>=3.)?.30f:0.f;
            if(steady)rl.control.x=t>=3.&&t<3.+offline_seconds?offline_speed:0.f;
            rl.control.y=rl.control.yaw=0.f;
            if(scenario=="release"&&t>=5.5)rl.control.x=0.f;
            if(scenario=="x"&&t>=5.5&&t<5.505)rl.control.current_keyboard=Input::Keyboard::Space;
            if((scenario=="b"&&t>=5.5)||t>=15.||(steady&&t>=3.+offline_seconds))rl.control.current_gamepad=Input::Gamepad::B;
            rl.StateController(&rl.robot_state,&rl.robot_command);
            const bool forward=(steady?(rl.control.x>.03f&&t<3.+offline_seconds):loco->trial.Forward())&&rl.fsm.current_state_->GetStateName()=="RLFSMStateRLLocomotion";
            if(forward)positive_request_seconds+=.005;
            if(forward&&!request_origin_set)
            {
                request_origin_set=true;start_x=rl.d->qpos[0];start_y=rl.d->qpos[1];
                const auto yaw=QuaternionYawOnly(rl.robot_state.imu.quaternion);
                start_yaw=2.*std::atan2(yaw[3],yaw[0]);
            }
            const double dx=request_origin_set?rl.d->qpos[0]-start_x:0.,dy=request_origin_set?rl.d->qpos[1]-start_y:0.;
            const double true_forward=std::cos(start_yaw)*dx+std::sin(start_yaw)*dy;
            const double true_lateral=-std::sin(start_yaw)*dx+std::cos(start_yaw)*dy;
            rl.control.x=forward?std::clamp(rl.control.x,0.f,float(steady?offline_speed:va15_trial::Supervisor::SpeedLimit)):0.f;rl.control.y=rl.control.yaw=0.f;
            if(step%4==0)rl.Infer();
            max_command=std::max(max_command,double(rl.control.x));ever_forward=ever_forward||forward;
            const double tilt=rl.Tilt();max_tilt=std::max(max_tilt,tilt);
            if(forward)min_walk_h=std::min(min_walk_h,truth.base_height*1.);
            if(forward)
            {
                for(int j:VA15LegIndices)max_leg_error=std::max(max_leg_error,double(std::fabs(rl.robot_command.motor_command.q[j]-rl.robot_state.motor_state.q[j])));
                for(int j=0;j<4;++j)if(!truth.contacts[j])air_time[j]+=.005;
            }
            const auto& stop=loco->controlled_stop;
            if((steady?t>=3.+offline_seconds:loco->trial.Stopping())&&!stop_seen){stop_seen=true;first_stop=t;forward_at_stop=true_forward;lateral_at_stop=true_lateral;}
            const bool four=std::all_of(truth.contacts.begin(),truth.contacts.end(),[](bool v){return v;});
            if(stop.OwnsCommand()&&stop.brake_time==0.&&(!four||truth.base_height<.48||tilt>.15))++false_catch;
            if(stop.Ready())held_stand=true;
            const auto state=rl.fsm.current_state_->GetStateName();
            if(state=="RLFSMStatePassive"&&truth.base_height<.2)prone=true;
            if(step%4==0)csv<<t<<','<<state<<','<<int(loco->trial.stage)<<','<<int(stop.stage)<<','<<loco->trial.requested<<','<<rl.control.x<<','<<rl.obs.commands[0]<<','<<truth.base_height<<','<<truth.planar_speed<<','<<tilt<<','<<estimate.estimate_valid_for_review<<','<<rl.robot_state.real_stop_feedback.sport_fresh<<','<<inject<<','<<rl.d->qpos[0]<<','<<rl.d->qpos[1]<<','<<true_forward<<','<<true_lateral<<','<<max_leg_error<<'\n';
            if(!std::isfinite(tilt)||tilt>.5){fell=true;break;}
            rl.Integrate();
            if(prone&&t>17.)break;
        }
        const bool negative=scenario=="preview"||scenario=="held";
        const bool fault=scenario=="lost"||scenario=="stale";
        const bool gait_formed=forward_at_stop>.10&&std::all_of(air_time.begin(),air_time.end(),[](double t){return t>.01;});
        const bool pass=!fell&&sport_leak==0&&false_catch==0&&max_command<=(steady?offline_speed:va15_trial::Supervisor::SpeedLimit)+.000001&&loco->trial.requested<=va15_trial::Supervisor::RequestSeconds+.000001&&
            (negative?(!ever_forward&&prone):fault?(stop_seen&&!held_stand):ever_forward&&held_stand&&prone)&&
            (scenario!="deadline"||gait_formed);
        std::ofstream report(std::string(argv[2])+".json");report<<std::boolalpha<<"{\"scenario\":\""<<scenario<<"\",\"pass\":"<<pass
            <<",\"ever_forward\":"<<ever_forward<<",\"positive_request_s\":"<<loco->trial.requested<<",\"max_command\":"<<max_command
            <<",\"offline_only_extended_input\":"<<steady<<",\"offline_speed\":"<<offline_speed<<",\"actual_request_seconds\":"<<positive_request_seconds
            <<",\"first_stop_s\":"<<first_stop<<",\"stand_catch_qualified\":"<<held_stand<<",\"prone\":"<<prone
            <<",\"fell\":"<<fell<<",\"max_tilt_rad\":"<<max_tilt<<",\"min_walk_h\":"<<min_walk_h
            <<",\"false_catch\":"<<false_catch<<",\"sport_leak\":"<<sport_leak<<",\"forward_and_each_foot_liftoff\":"<<gait_formed
            <<",\"forward_at_request_end_m\":"<<forward_at_stop<<",\"lateral_at_request_end_m\":"<<lateral_at_stop
            <<",\"max_leg_tracking_error_rad\":"<<max_leg_error<<",\"foot_air_time_s_FL_FR_RL_RR\":["
            <<air_time[0]<<','<<air_time[1]<<','<<air_time[2]<<','<<air_time[3]<<"],\"hardware_qualified\":false}\n";
        std::cout<<"trial result="<<pass<<" tilt="<<max_tilt<<" request="<<loco->trial.requested<<" stand="<<held_stand<<" prone="<<prone<<std::endl;
        return pass?0:2;
    }
    catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}
}
