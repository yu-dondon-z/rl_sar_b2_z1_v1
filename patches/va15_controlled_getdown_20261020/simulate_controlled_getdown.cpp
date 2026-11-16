// Headless dynamics test: SAME native candidate FSM + VA15 actor. No Unitree
// SDK headers, DDS, UDP, camera, keyboard or hardware clients are initialized.
#include "fsm_b2_z1_no_gun.hpp"
#include "va15_action_state.hpp"
#include <mujoco/mujoco.h>
#include <fstream>
#include <filesystem>
#include <iomanip>

using namespace b2_z1_no_gun_fsm;
namespace fs=std::filesystem;

class SimRL final:public RL
{
public:
    mjModel* m=nullptr; mjData* d=nullptr;
    std::vector<int> qids,vids,jids;
    std::array<int,4> feet;
    int base=-1;
    VA15ActionState actions;
    std::string fault="none";
    bool fault_active=false, candidate=true;
    bool RequiresRealStopFeedback()const override{return true;}
    ~SimRL(){if(d)mj_deleteData(d);if(m)mj_deleteModel(m);}

    SimRL(const std::string& scene,bool enabled,float friction,float tool_mass):candidate(enabled)
    {
        robot_name="b2_z1_no_gun";ang_vel_axis="body";
        ReadYaml(robot_name,"base.yaml");InitRL(robot_name+"/VA15");
        char error[2048]{};m=mj_loadXML(scene.c_str(),nullptr,error,sizeof(error));
        if(!m)throw std::runtime_error(error);
        d=mj_makeData(m);base=mj_name2id(m,mjOBJ_BODY,"base_link");
        if(base<0||m->nu!=18)throw std::runtime_error("Unexpected B2/Z1 scene");
        const auto names=params.Get<std::vector<std::string>>("joint_names");
        for(int i=0;i<18;++i)
        {
            const int joint=mj_name2id(m,mjOBJ_JOINT,names.at(i).c_str());
            if(joint<0 || m->actuator_trnid[2*i]!=joint)throw std::runtime_error("Actuator routing mismatch");
            jids.push_back(joint);qids.push_back(m->jnt_qposadr[joint]);vids.push_back(m->jnt_dofadr[joint]);
            // Keep the existing VA15 MuJoCo Z1 implicit servo. It is a SIM
            // proxy, not a qualification of the real Z1 SDK's inner loops.
            // B2 uses explicit PD refreshed at the native 200 Hz below.
            if(i!=8&&i<13)
            {
                m->actuator_biastype[i]=mjBIAS_NONE;m->actuator_gaintype[i]=mjGAIN_FIXED;
                for(int k=0;k<mjNGAIN;++k)m->actuator_gainprm[mjNGAIN*i+k]=k==0?1.:0.;
                for(int k=0;k<mjNBIAS;++k)m->actuator_biasprm[mjNBIAS*i+k]=0.;
                const auto limits=params.Get<std::vector<float>>("torque_limits");
                m->actuator_ctrlrange[2*i]=-limits[i];m->actuator_ctrlrange[2*i+1]=limits[i];
                m->actuator_forcerange[2*i]=-limits[i];m->actuator_forcerange[2*i+1]=limits[i];
            }
        }
        const char* foot_names[]={"FL_foot_collision","FR_foot_collision","RL_foot_collision","RR_foot_collision"};
        for(int i=0;i<4;++i){feet[i]=mj_name2id(m,mjOBJ_GEOM,foot_names[i]);if(feet[i]<0)throw std::runtime_error("Missing foot");}
        for(int i=0;i<m->ngeom;++i)m->geom_friction[3*i]=friction;
        int gun=mj_name2id(m,mjOBJ_GEOM,"welding_gun");
        if(gun>=0)m->geom_contype[gun]=m->geom_conaffinity[gun]=0; // detached before lowering
        int payload=mj_name2id(m,mjOBJ_BODY,"dummy_target");
        if(payload>=0)
        {
            const double ratio=std::max(.001f,tool_mass)/m->body_mass[payload];
            m->body_mass[payload]*=ratio;
            for(int i=0;i<3;++i)m->body_inertia[3*payload+i]*=ratio;
            mj_setConst(m,d); // initialization only, before setting qpos
        }
        auto stand=DeploymentStandPosition(*this);
        for(int i=0;i<18;++i)d->qpos[qids[i]]=stand[i];
        d->qpos[2]=.58;mj_forward(m,d);
        auto lying=stand;
        for(int i=0;i<4;++i){lying[i]=(i%2==0?.30f:-.30f);lying[4+i]=1.21f;lying[9+i]=-2.80f;}
        SetArmPosition(lying,DeploymentLyingArmPosition(*this));start_state.motor_state.q=lying;
        robot_command.motor_command.q=stand;
        robot_command.motor_command.kp=DeploymentStandGain(*this,"deployment_stand_kp","fixed_kp");
        robot_command.motor_command.kd=DeploymentStandGain(*this,"deployment_stand_kd","fixed_kd");
        const auto limits=params.Get<std::vector<float>>("torque_limits");
        for(int i=0;i<600;++i){ReadState();Integrate();} // settle physical standing, not a reset during walk
        ReadState();
        fsm.AddState(std::make_shared<RLFSMStateRLLocomotion>(this));
        fsm.AddState(std::make_shared<RLFSMStateGetDown>(this));
        fsm.AddState(std::make_shared<RLFSMStateGetUp>(this));
        fsm.AddState(std::make_shared<RLFSMStatePassive>(this));
        StatePointers();fsm.SetInitialState("RLFSMStateRLLocomotion");
        params.config_node["independent_B_stop_candidate"]=candidate;
    }
    void StatePointers()
    {
        for(auto& kv:fsm.states_)
            if(auto s=std::dynamic_pointer_cast<RLFSMState>(kv.second))
            {s->fsm_state=&robot_state;s->fsm_command=&robot_command;}
    }
    void GetState(RobotState<float>* value) override{*value=robot_state;}
    void SetCommand(const RobotCommand<float>* value) override{robot_command=*value;}
    std::vector<float> Forward()override
    {
        const auto raw=model->forward({ComputeObservation()});
        auto applied=actions.Update(raw,obs.actions,.2f,5.f,reset_policy_action_state||episode_length_buf==1);
        reset_policy_action_state=false;
        return clamp(applied,params.Get<std::vector<float>>("clip_actions_lower"),params.Get<std::vector<float>>("clip_actions_upper"));
    }
    void Infer()
    {
        if(!rl_init_done)return;
        ++episode_length_buf;
        obs.ang_vel=robot_state.imu.gyroscope;obs.base_quat=robot_state.imu.quaternion;
        obs.dof_pos=robot_state.motor_state.q;obs.dof_vel=robot_state.motor_state.dq;
        const float requests[]={control.x,control.y,control.yaw};
        const auto accel=params.Get<std::vector<float>>("command_accel_limits");
        const auto decel=params.Get<std::vector<float>>("command_decel_limits");
        for(int i=0;i<3;++i)
        {
            const bool accelerating=obs.commands[i]*requests[i]>=0.f&&std::fabs(requests[i])>std::fabs(obs.commands[i]);
            const float step=(accelerating?accel[i]:decel[i])*.02f;
            obs.commands[i]+=std::clamp(requests[i]-obs.commands[i],-step,step);
        }
        ComputeOutput(Forward(),output_dof_pos,output_dof_vel,output_dof_tau);
        output_dof_pos_queue.push(output_dof_pos);output_dof_vel_queue.push(output_dof_vel);output_dof_tau_queue.push(output_dof_tau);
    }
    void ReadState()
    {
        mj_forward(m,d);
        for(int i=0;i<18;++i)
        {robot_state.motor_state.q[i]=d->qpos[qids[i]];robot_state.motor_state.dq[i]=d->qvel[vids[i]];
         robot_state.motor_state.tau_est[i]=d->qfrc_actuator[vids[i]];}
        for(int i=0;i<4;++i)robot_state.imu.quaternion[i]=d->qpos[3+i];
        double world[6]{};
        mj_objectVelocity(m,d,mjOBJ_BODY,base,world,0);
        // mjOBJ_BODY local axes can be the principal-inertia frame (ximat),
        // NOT base_link. Explicitly rotate WORLD angular velocity into xmat.
        // The real DDS IMU contract is base/body gyro, as in VA15 playback.
        for(int i=0;i<3;++i)
        {robot_state.imu.gyroscope[i]=0.f;for(int j=0;j<3;++j)
            robot_state.imu.gyroscope[i]+=d->xmat[9*base+3*j+i]*world[j];}
        auto& e=robot_state.real_stop_feedback;
        e.lowstate_fresh=e.sport_fresh=e.contacts_valid=e.height_valid=e.velocity_valid=true;
        e.base_height=d->xpos[3*base+2];e.planar_speed=std::hypot(world[3],world[4]);e.contacts.fill(false);
        for(int i=0;i<d->ncon;++i)
        {
            const auto& contact=d->contact[i];
            const bool ground1=m->geom_bodyid[contact.geom1]==0;
            const bool ground2=m->geom_bodyid[contact.geom2]==0;
            if(!ground1&&!ground2)continue;
            double force[6]{};mj_contactForce(m,d,i,force);
            for(int foot=0;foot<4;++foot)
                if((contact.geom1==feet[foot]||contact.geom2==feet[foot])&&force[0]>5.)e.contacts[foot]=true;
        }
        e.unavailable_reason="synthetic MuJoCo truth; NOT B2 calibration";
        if(fault_active)
        {
            if(fault=="contacts")e.contacts_valid=false;
            if(fault=="partial")e.contacts[2]=false;
            if(fault=="velocity")e.velocity_valid=false;
            if(fault=="stale"||fault=="stale_after_catch")e.lowstate_fresh=false;
        }
    }
    void Integrate()
    {
        const auto limits=params.Get<std::vector<float>>("torque_limits");
        const auto& c=robot_command.motor_command;
        // Real command is held for 5 ms; do not sneak in a 1 kHz PD update.
        for(int i=0;i<18;++i)
            d->ctrl[i]=(i==8||i>=13) ? c.q[i] : std::clamp(c.kp[i]*(c.q[i]-d->qpos[qids[i]])+
                c.kd[i]*(c.dq[i]-d->qvel[vids[i]])+c.tau[i],-double(limits[i]),double(limits[i]));
        const int steps=static_cast<int>(std::lround(.005/m->opt.timestep));
        if(std::fabs(steps*m->opt.timestep-.005)>1.e-7)throw std::runtime_error("Nonintegral control/physics timestep");
        for(int i=0;i<steps;++i)mj_step(m,d);
    }
    float Tilt()const
    {const auto g=QuatRotateInverse(robot_state.imu.quaternion,{0.f,0.f,-1.f});return std::asin(std::clamp(std::hypot(g[0],g[1]),0.f,1.f));}
};

int main(int argc,char** argv)
{
    try
    {
        std::string scene="/home/dong/rl_sar_b2_z1_old/policy/b2_z1_no_gun/VA15/scene_flat.xml",output,fault="none";
        float speed=.2f,walk=8.f,friction=.8f,mass=0.f;bool enabled=true;
        for(int i=1;i<argc;++i)
        {
            const std::string arg=argv[i];
            if(arg=="--legacy")enabled=false;
            else if(i+1>=argc)throw std::runtime_error("Missing option value");
            else if(arg=="--output")output=argv[++i];
            else if(arg=="--scene")scene=argv[++i];
            else if(arg=="--speed")speed=std::stof(argv[++i]);
            else if(arg=="--walk-seconds")walk=std::stof(argv[++i]);
            else if(arg=="--friction")friction=std::stof(argv[++i]);
            else if(arg=="--tool-mass")mass=std::stof(argv[++i]);
            else if(arg=="--fault")fault=argv[++i];
            else throw std::runtime_error("Unknown option: "+arg);
        }
        if(output.empty()||fs::exists(output)||fs::exists(output+".json"))throw std::runtime_error("New --output CSV required; refusing overwrite");
        if(!std::isfinite(speed)||!std::isfinite(walk)||!std::isfinite(friction)||!std::isfinite(mass)||
            speed<0||speed>.3f||walk<0||walk>60||friction<.3||friction>1.5||mass<0||mass>1)throw std::runtime_error("Test outside declared range");
        if(fault!="none"&&fault!="contacts"&&fault!="partial"&&fault!="velocity"&&fault!="stale"&&fault!="stale_after_catch")throw std::runtime_error("Unknown fault");
        fs::create_directories(fs::path(output).parent_path());
        SimRL rl(scene,enabled,friction,mass);rl.fault=fault;
        std::ofstream csv(output);csv<<std::setprecision(10);
        csv<<"time,state,stop_stage,height,tilt,planar_speed,policy_vx,feet,stand_progress,quiet_s,max_leg_dq";
        for(int i=0;i<rl.m->nq;++i)csv<<",qpos_"<<i;csv<<'\n';
        const double initial=rl.d->time,start_walk=initial+3.,press_b=start_walk+walk;
        bool b_sent=false,lowered=false,fell=false,captured=false;
        double capture_time=-1.,lower_time=-1.,prone_time=-1.,passive_since=-1.,max_tilt=0.,distance_at_b=0.;
        bool blocked=false;
        std::string final_reason="not completed";
        int step=0;
        for(;step<static_cast<int>((3.+walk+22.)/.005);++step)
        {
            rl.ReadState();
            const double now=rl.d->time;
            rl.control.current_keyboard=Input::Keyboard::None;rl.control.current_gamepad=Input::Gamepad::None;
            if(now>=start_walk&&!b_sent)rl.control.x=speed;
            if(!b_sent&&now>=press_b)
            {b_sent=true;rl.control.current_gamepad=Input::Gamepad::B;rl.fault_active=fault!="none"&&fault!="stale_after_catch";distance_at_b=rl.d->qpos[0];rl.ReadState();}
            rl.StateController(&rl.robot_state,&rl.robot_command);
            if(step%4==0)rl.Infer();
            auto loc=std::dynamic_pointer_cast<RLFSMStateRLLocomotion>(rl.fsm.states_.at("RLFSMStateRLLocomotion"));
            const auto& stop=loc->controlled_stop;
            if(stop.captured&&!captured){captured=true;capture_time=now-press_b;if(fault=="stale_after_catch")rl.fault_active=true;}
            const auto state=rl.fsm.current_state_->GetStateName();
            if(state=="RLFSMStateGetDown"&&!lowered){lowered=true;lower_time=now-press_b;}
            const double tilt=rl.Tilt();max_tilt=std::max(max_tilt,tilt);
            if(tilt>.50 || !std::isfinite(rl.d->qpos[2])){fell=true;final_reason="tilt/nonfinite failure";break;}
            const auto& e=rl.robot_state.real_stop_feedback;
            float dq=0.f;for(int i:VA15LegIndices)dq=std::max(dq,std::fabs(rl.robot_state.motor_state.dq[i]));
            if(step%4==0)
            {
                csv<<now<<','<<state<<','<<static_cast<int>(stop.stage)<<','<<e.base_height<<','<<tilt<<','<<e.planar_speed<<','
                    <<(rl.obs.commands.empty()?0.f:rl.obs.commands[0])<<',';
                for(bool v:e.contacts)csv<<(v?'1':'0');
                csv<<','<<stop.progress<<','<<stop.quiet<<','<<dq;
                for(int i=0;i<rl.m->nq;++i)csv<<','<<rl.d->qpos[i];csv<<'\n';
            }
            if(state=="RLFSMStatePassive"&&lowered&&rl.d->qpos[2]<.22)
            {
                if(passive_since<0.)passive_since=now;
                if(now-passive_since>=2. && tilt<.3 && e.planar_speed<.04f && dq<.55f)
                {prone_time=now-press_b;final_reason="prone/passive completed, 2 s settled";break;}
            }
            if(b_sent&&stop.stage==VA15ControlledStop::Stage::Blocked)
            {blocked=true;final_reason=stop.reason;break;}
            rl.Integrate();
        }
        const bool success=fault=="none" ? (!fell&&lowered&&prone_time>=0) : (!fell&&!lowered&&b_sent&&blocked);
        std::ofstream report(output+".json");
        report<<std::boolalpha<<std::setprecision(10)<<"{\n  \"success\": "<<success<<",\n  \"candidate\": "<<enabled
            <<",\n  \"hardware_qualified\": false,\n  \"z1_servo\": \"existing MuJoCo implicit position proxy\",\n  \"simulated_ground_truth_only\": true,\n  \"speed\": "<<speed
            <<",\n  \"walk_s\": "<<walk<<",\n  \"friction\": "<<friction<<",\n  \"tool_mass_kg\": "<<mass
            <<",\n  \"fault\": \""<<fault<<"\",\n  \"fell\": "<<fell<<",\n  \"capture_after_B_s\": "<<capture_time
            <<",\n  \"lower_after_B_s\": "<<lower_time<<",\n  \"prone_after_B_s\": "<<prone_time
            <<",\n  \"max_tilt_deg\": "<<max_tilt*57.2957795<<",\n  \"x_at_B\": "<<distance_at_b
            <<",\n  \"reason\": \""<<final_reason<<"\"\n}\n";
        std::cout<<"SIMULATION "<<(success?"PASS":"FAIL")<<": "<<final_reason<<"; capture="<<capture_time
            <<" lower="<<lower_time<<" prone="<<prone_time<<" max_tilt_deg="<<max_tilt*57.2957795<<std::endl;
        return success?0:2;
    }
    catch(const std::exception& e){std::cerr<<"ERROR: "<<e.what()<<std::endl;return 1;}
}
