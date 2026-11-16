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
        if(argc!=6)throw std::runtime_error("model output scenario friction tau_scale");
        const std::string scenario=argv[3];const double friction=std::stod(argv[4]),tau_scale=std::stod(argv[5]);
        if(scenario!="deadline"&&scenario!="release"&&scenario!="x"&&scenario!="b"&&
           scenario!="lost"&&scenario!="stale"&&scenario!="preview"&&scenario!="held")throw std::runtime_error("Unknown scenario");
        if(fs::exists(argv[2]))throw std::runtime_error("Output already exists");
        SimRL rl("/home/dong/rl_sar_b2_z1_old/policy/b2_z1_no_gun/VA15/scene_flat.xml",true,friction,0.);
        rl.params.config_node["isolated_forward_trial"]=true;
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
        std::ofstream csv(argv[2]);csv<<"time,state,trial_stage,stop_stage,request_s,command,policy_x,truth_h,truth_v,truth_tilt,estimate_valid,source_sport,fault\n";
        for(int step=0;step<5800;++step)
        {
            rl.ReadState();const auto truth=rl.robot_state.real_stop_feedback;
            const double now=rl.d->time,t=now-initial;
            const bool inject=(scenario=="lost"||scenario=="stale")&&t>=3.6;
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
            rl.control.y=rl.control.yaw=0.f;
            if(scenario=="release"&&t>=3.6)rl.control.x=0.f;
            if(scenario=="x"&&t>=3.6&&t<3.605)rl.control.current_keyboard=Input::Keyboard::Space;
            if((scenario=="b"&&t>=3.6)||t>=15.)rl.control.current_gamepad=Input::Gamepad::B;
            rl.StateController(&rl.robot_state,&rl.robot_command);
            const bool forward=loco->trial.Forward()&&rl.fsm.current_state_->GetStateName()=="RLFSMStateRLLocomotion";
            rl.control.x=forward?std::clamp(rl.control.x,0.f,.10f):0.f;rl.control.y=rl.control.yaw=0.f;
            if(step%4==0)rl.Infer();
            max_command=std::max(max_command,double(rl.control.x));ever_forward=ever_forward||forward;
            const double tilt=rl.Tilt();max_tilt=std::max(max_tilt,tilt);
            if(forward)min_walk_h=std::min(min_walk_h,truth.base_height*1.);
            const auto& stop=loco->controlled_stop;
            if(loco->trial.Stopping()&&!stop_seen){stop_seen=true;first_stop=t;}
            const bool four=std::all_of(truth.contacts.begin(),truth.contacts.end(),[](bool v){return v;});
            if(stop.OwnsCommand()&&stop.brake_time==0.&&(!four||truth.base_height<.48||tilt>.15))++false_catch;
            if(stop.Ready())held_stand=true;
            const auto state=rl.fsm.current_state_->GetStateName();
            if(state=="RLFSMStatePassive"&&truth.base_height<.2)prone=true;
            if(step%4==0)csv<<t<<','<<state<<','<<int(loco->trial.stage)<<','<<int(stop.stage)<<','<<loco->trial.requested<<','<<rl.control.x<<','<<rl.obs.commands[0]<<','<<truth.base_height<<','<<truth.planar_speed<<','<<tilt<<','<<estimate.estimate_valid_for_review<<','<<rl.robot_state.real_stop_feedback.sport_fresh<<','<<inject<<'\n';
            if(!std::isfinite(tilt)||tilt>.5){fell=true;break;}
            rl.Integrate();
            if(prone&&t>17.)break;
        }
        const bool negative=scenario=="preview"||scenario=="held";
        const bool fault=scenario=="lost"||scenario=="stale";
        const bool pass=!fell&&sport_leak==0&&false_catch==0&&max_command<=.100001&&loco->trial.requested<=2.000001&&
            (negative?(!ever_forward&&prone):fault?(stop_seen&&!held_stand):ever_forward&&held_stand&&prone);
        std::ofstream report(std::string(argv[2])+".json");report<<std::boolalpha<<"{\"scenario\":\""<<scenario<<"\",\"pass\":"<<pass
            <<",\"ever_forward\":"<<ever_forward<<",\"positive_request_s\":"<<loco->trial.requested<<",\"max_command\":"<<max_command
            <<",\"first_stop_s\":"<<first_stop<<",\"stand_catch_qualified\":"<<held_stand<<",\"prone\":"<<prone
            <<",\"fell\":"<<fell<<",\"max_tilt_rad\":"<<max_tilt<<",\"min_walk_h\":"<<min_walk_h
            <<",\"false_catch\":"<<false_catch<<",\"sport_leak\":"<<sport_leak<<",\"hardware_qualified\":false}\n";
        std::cout<<"trial result="<<pass<<" tilt="<<max_tilt<<" request="<<loco->trial.requested<<" stand="<<held_stand<<" prone="<<prone<<std::endl;
        return pass?0:2;
    }
    catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}
}
