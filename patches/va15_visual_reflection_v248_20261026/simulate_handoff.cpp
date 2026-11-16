// No DDS, UDP, camera, keyboard, or robot client. Same actor/FSM in physics.
#include "production/fsm_b2_z1_no_gun.hpp"
#define main archived_simulation_main_not_called
#include "../va15_controlled_getdown_20261020/simulate_controlled_getdown.cpp"
#undef main

int main(int argc,char** argv)
{
    try {
        if(argc!=6)throw std::runtime_error("candidate|legacy speed friction walk|b_early|x_early new.csv");
        const std::string mode=argv[1];
        const bool candidate=mode!="legacy";
        const float speed=std::stof(argv[2]),mu=std::stof(argv[3]);const std::string scenario=argv[4],output=argv[5];
        if((mode!="candidate"&&mode!="legacy"&&mode!="v242")||speed<.1f||speed>.2f||mu<.5f||mu>1.f||
           (scenario!="walk"&&scenario!="b_early"&&scenario!="x_early")||fs::exists(output))throw std::runtime_error("Invalid/new output required");
        SimRL rl("/home/dong/rl_sar_b2_z1_old/policy/b2_z1_no_gun/VA15/scene_flat.xml",true,mu,0.f);
        rl.params.config_node["torque_continuous_policy_handoff"]=candidate;
        rl.params.config_node["va15_prefit_arm_entry"]=mode=="candidate";
        auto loco=std::dynamic_pointer_cast<RLFSMStateRLLocomotion>(rl.fsm.states_.at("RLFSMStateRLLocomotion"));
        loco->Enter();
        const double initial=rl.d->time,walk_start=3.,walk_end=scenario=="walk"?23.:3.20;
        bool requested=false,stop=false,fell=false,prone=false,join_failed=false;double max_tilt=0.,min_start_h=10.,jump=0.,gain_jump=0.;
        double first_x=0.,last_x=0.,max_start_h=0.;bool first=true;
        std::vector<float> arm_min(18,1.e9f),arm_max(18,-1.e9f);
        std::ofstream csv(output);csv<<std::setprecision(10)<<"time,height,tilt,gyro_z,policy_x,handoff_active,handoff_elapsed,state";
        for(int i:VA15LegIndices)csv<<",actual_"<<i<<",target_"<<i<<",kp_"<<i;
        for(int i:{8,13,14,15,16,17})csv<<",arm_actual_"<<i<<",arm_target_"<<i;csv<<'\n';
        for(int step=0;step<9200;++step)
        {
            rl.ReadState();const double t=rl.d->time-initial;
            rl.control.current_keyboard=Input::Keyboard::None;rl.control.current_gamepad=Input::Gamepad::None;
            if(t>=walk_start&&t<walk_end){rl.control.x=speed;requested=true;}
            if(t>=walk_end){rl.control.x=0.f;stop=true;if(scenario!="x_early")rl.control.current_gamepad=Input::Gamepad::B;}
            if(scenario=="x_early"&&t>=7.)rl.control.current_gamepad=Input::Gamepad::B;
            const auto previous=rl.robot_command;
            rl.StateController(&rl.robot_state,&rl.robot_command);
            for(int i:VA15LegIndices)if(previous.motor_command.kp[i]>400.f&&rl.robot_command.motor_command.kp[i]<200.f)
            {
                const auto effort=[&](const RobotCommand<float>& c){const auto& v=c.motor_command;return v.kp[i]*(v.q[i]-rl.robot_state.motor_state.q[i])+v.kd[i]*(v.dq[i]-rl.robot_state.motor_state.dq[i])+v.tau[i];};
                gain_jump=std::max(gain_jump,double(std::fabs(effort(rl.robot_command)-effort(previous))));
            }
            if(loco->policy_running&&first)
            {
                first=false;first_x=rl.d->qpos[0];
                for(int i:VA15LegIndices)
                {
                    const auto torque=[&](const RobotCommand<float>& c){const auto& v=c.motor_command;return v.kp[i]*(v.q[i]-rl.robot_state.motor_state.q[i])+v.kd[i]*(v.dq[i]-rl.robot_state.motor_state.dq[i])+v.tau[i];};
                    jump=std::max(jump,double(std::fabs(torque(rl.robot_command)-torque(previous))));
                }
            }
            // Deliberately delay the first actor by 35 ms to cover the
            // asynchronous startup gap, then use the original 50 Hz cadence.
            if(step%4==0&&t>=walk_start+.035)rl.Infer();
            const auto state=rl.fsm.current_state_->GetStateName();
            const auto& e=rl.robot_state.real_stop_feedback;
            join_failed=join_failed||loco->policy_handoff.failed;
            max_tilt=std::max(max_tilt,double(rl.Tilt()));
            if(t>=walk_start&&t<walk_start+2.)min_start_h=std::min(min_start_h,double(e.base_height));
            if(t>=walk_start&&t<walk_start+3.) {
                max_start_h=std::max(max_start_h,double(e.base_height));
                for(int i:{8,13,14,15,16,17}) {
                    arm_min[i]=std::min(arm_min[i],rl.robot_state.motor_state.q[i]);
                    arm_max[i]=std::max(arm_max[i],rl.robot_state.motor_state.q[i]);
                }
            }
            if(t>=walk_start&&t<walk_end)last_x=rl.d->qpos[0];
            if(step%4==0){csv<<t<<','<<e.base_height<<','<<rl.Tilt()<<','<<rl.robot_state.imu.gyroscope[2]<<','<<rl.obs.commands[0]<<','<<loco->policy_handoff.Active()<<','<<loco->policy_handoff.elapsed<<','<<state;for(int i:VA15LegIndices)csv<<','<<rl.robot_state.motor_state.q[i]<<','<<rl.robot_command.motor_command.q[i]<<','<<rl.robot_command.motor_command.kp[i];for(int i:{8,13,14,15,16,17})csv<<','<<rl.robot_state.motor_state.q[i]<<','<<rl.robot_command.motor_command.q[i];csv<<'\n';}
            if(!std::isfinite(rl.Tilt())||rl.Tilt()>.5){fell=true;break;}
            if(stop&&state=="RLFSMStatePassive"&&e.base_height<.20){prone=true;break;}
            rl.Integrate();
        }
        const double travel=last_x-first_x;
        float arm_start_range=0.f;for(int i:{8,13,14,15,16,17})if(arm_max[i]>=arm_min[i])arm_start_range=std::max(arm_start_range,arm_max[i]-arm_min[i]);
        const bool pass=requested&&!fell&&prone&&!join_failed&&(scenario!="walk"||travel>2.)&&(!candidate||(jump<.001&&gain_jump<.001));
        std::ofstream report(output+".json");report<<std::boolalpha<<std::setprecision(10)
            <<"{\"pass\":"<<pass<<",\"candidate\":"<<candidate<<",\"mode\":\""<<mode<<"\",\"speed\":"<<speed<<",\"friction\":"<<mu<<",\"scenario\":\""<<scenario<<"\",\"fell\":"<<fell<<",\"prone\":"<<prone<<",\"handoff_failed\":"<<join_failed<<",\"initial_effort_jump_Nm\":"<<jump<<",\"gain_switch_effort_jump_Nm\":"<<gain_jump<<",\"min_start_height_m\":"<<min_start_h<<",\"max_start_height_m\":"<<max_start_h<<",\"arm_start_range_rad\":"<<arm_start_range<<",\"max_tilt_deg\":"<<max_tilt*57.2957795<<",\"forward_m\":"<<travel<<",\"fixed_tcp\":true,\"camera_initialized\":false,\"hardware_qualified\":false}\n";
        return pass?0:2;
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
