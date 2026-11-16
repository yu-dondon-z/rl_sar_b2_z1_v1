#pragma once
#ifndef VA15_PARALLEL_GETDOWN_SIMULATION_ONLY
#error "Parallel getdown is an unqualified simulation-only candidate"
#endif

// Included AFTER the native FSM, with its GetDown class renamed. No native
// control program, standing/stop gate, actor, gain table or sensor is replaced.
namespace b2_z1_no_gun_fsm
{
class RLFSMStateGetDown : public RLFSMState
{
public:
    explicit RLFSMStateGetDown(RL* value):RLFSMState(*value,"RLFSMStateGetDown"){}
    RobotCommand<float> entry,held;
    std::vector<float> target,fixed_kp,fixed_kd;
    double elapsed=0.,arm_progress=0.,body_progress=0.,quiet=0.;
    bool first=true,blocked=false,complete=false,body_paused=false;
    const char* reason="not entered";
    static constexpr double Duration=3.; // smoothstep peak slope equals old 2 s linear slope
    static bool Arm(int i){return i==8||i>=13;}
    static float Blend(double p){return static_cast<float>(p*p*(3.-2.*p));}
    static bool Vector(const std::vector<float>& v)
    {return v.size()==18&&std::all_of(v.begin(),v.end(),[](float x){return std::isfinite(x);});}
    static bool Command(const RobotCommand<float>& c)
    {return Vector(c.motor_command.q)&&Vector(c.motor_command.dq)&&Vector(c.motor_command.kp)&&
        Vector(c.motor_command.kd)&&Vector(c.motor_command.tau);}
    void Block(const char* message)
    {
        blocked=true;reason=message;
        std::cout<<std::endl<<LOGGER::WARNING<<"[GetDown] "<<reason
                 <<"; holding last command, operator intervention required"<<std::endl;
    }
    void Enter() override
    {
        elapsed=arm_progress=body_progress=quiet=0.;first=true;blocked=complete=body_paused=false;
        entry=held=*fsm_command;target=rl.start_state.motor_state.q;
        SetArmPosition(target,DeploymentLyingArmPosition(rl));
        fixed_kp=rl.params.Get<std::vector<float>>("fixed_kp");
        fixed_kd=rl.params.Get<std::vector<float>>("fixed_kd");
        if(!Command(entry)||!Vector(target)||!Vector(fixed_kp)||!Vector(fixed_kd))
        {Block("invalid entry/config; no automatic lowering");return;}
        for(int i=0;i<18;++i)
            if(fixed_kp[i]<=0||fixed_kd[i]<0||entry.motor_command.kp[i]<=0||entry.motor_command.kd[i]<0)
            {Block("invalid entry gains; no automatic lowering");return;}
        reason="parallel arm stow + B2 lowering; tool must already be removed";
        std::cout<<std::endl<<LOGGER::NOTE<<reason<<std::endl;
    }
    void Run() override
    {
        if(blocked||complete){*fsm_command=held;return;}
        const double dt=rl.params.Get<float>("dt");
        if(!std::isfinite(dt)||dt<=0||dt>.02||!Vector(fsm_state->motor_state.q)||!Vector(fsm_state->motor_state.dq))
        {Block("invalid measured pose/time");*fsm_command=held;return;}
        // Exact command continuity, including gains, dq and feedforward.
        if(first){first=false;*fsm_command=held;return;}
        elapsed+=dt;
        if(elapsed>10.)
        {Block("parallel lowering did not finish; no timed passive release");*fsm_command=held;return;}
        float arm_lag=0.f;
        for(int i=0;i<18;++i)if(Arm(i))
            arm_lag=std::max(arm_lag,std::fabs(fsm_state->motor_state.q[i]-held.motor_command.q[i]));
        arm_progress=std::min(1.,arm_progress+dt/Duration);
        const bool previous_pause=body_paused;
        body_paused=arm_lag>.20f;
        if(body_paused!=previous_pause)
            std::cout<<std::endl<<LOGGER::WARNING<<"[GetDown] "
                     <<(body_paused?"B2 descent paused: arm tracking lag":"Arm tracking recovered; B2 descent resumed")
                     <<" (rad="<<arm_lag<<")"<<std::endl;
        // Arm keeps folding if it lags; B2 stops descending rather than
        // waiting for 100% stow before it may begin its first lowering step.
        if(!body_paused)body_progress=std::min(arm_progress,body_progress+dt/Duration);
        for(int i=0;i<18;++i)
        {
            const float s=Blend(Arm(i)?arm_progress:body_progress);
            auto& c=held.motor_command;const auto& initial=entry.motor_command;
            c.q[i]=(1.f-s)*initial.q[i]+s*target[i];
            c.kp[i]=(1.f-s)*initial.kp[i]+s*fixed_kp[i];
            c.kd[i]=(1.f-s)*initial.kd[i]+s*fixed_kd[i];
            c.dq[i]=(1.f-s)*initial.dq[i];c.tau[i]=(1.f-s)*initial.tau[i];
        }
        float arm_error=0.f,leg_error=0.f,max_dq=0.f;
        for(int i=0;i<18;++i)
        {
            const float error=std::fabs(fsm_state->motor_state.q[i]-target[i]);
            if(Arm(i))arm_error=std::max(arm_error,error);else leg_error=std::max(leg_error,error);
            max_dq=std::max(max_dq,std::fabs(fsm_state->motor_state.dq[i]));
        }
        const bool finished=arm_progress>=1.&&body_progress>=1.&&arm_error<.12f&&leg_error<.18f&&max_dq<.5f;
        quiet=finished?quiet+dt:0.;complete=quiet+1.e-9>=.20;
        if(complete)std::cout<<std::endl<<LOGGER::INFO<<"[GetDown] Parallel arm stow and B2 lowering completed"<<std::endl;
        reason=complete?"parallel lowering finished with quiet measured joints":
            (body_paused?"B2 lowering paused: arm lag exceeds 0.20 rad":"parallel arm stow + B2 lowering");
        *fsm_command=held;
    }
    void Exit() override{}
    std::string CheckChange() override
    {
        if(rl.control.current_keyboard==Input::Keyboard::P||rl.control.current_gamepad==Input::Gamepad::LB_X)
            return "RLFSMStatePassive";
        if(complete)return "RLFSMStatePassive";
        if(!blocked&&(rl.control.current_keyboard==Input::Keyboard::Num0||rl.control.current_gamepad==Input::Gamepad::A))
            return "RLFSMStateGetUp";
        return state_name_;
    }
};
}
