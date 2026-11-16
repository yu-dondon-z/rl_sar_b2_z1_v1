#pragma once
#ifndef VA15_ESTIMATED_STOP_SIMULATION_ONLY
#error "Local stop-transition shape experiment is simulation-only"
#endif

#include "rl_sdk.hpp"
#include "va15_stand_gains.hpp"
#include <array>

// Candidate, OFF on PC4. Four-foot capture is distinct from final quiet-stand
// qualification: a learned zero-speed gait need not already be motionless.
// This controller never manufactures contact, height, velocity or freshness.
class VA15ControlledStop
{
public:
    enum class Stage { Idle, Decelerate, CatchStand, Settle, ReadyToLower, Blocked };
    Stage stage = Stage::Idle;
    const char* reason = "not requested";
    bool captured = false;
    bool stand_reference_valid = false;
    double elapsed = 0., progress = 0., quiet = 0., capture_window = 0.;
    double brake_time = 0.;
    double pose_blend_duration = 2.; // trajectory duration, NOT a support/quiet threshold
    bool hold_supported_pose = false;
    RobotCommand<float> held;

    void Reset() { *this = VA15ControlledStop(); }
    bool Active() const { return stage != Stage::Idle; }
    bool Ready() const { return stage == Stage::ReadyToLower; }
    bool OwnsCommand() const { return captured; }

    // Record the already verified quiet standing effort BEFORE walking. It
    // supplies a load-bearing reference while the last dynamic gait effort
    // is faded out. Never learn it from a swing phase or missing feedback.
    void ObserveStanding(const RobotState<float>& state,const RobotCommand<float>& command)
    {
        if(Active() || !ValidPose(state) || !ValidCommand(command))return;
        const auto gravity=QuatRotateInverse(state.imu.quaternion,{0.f,0.f,-1.f});
        float gyro=0.f,speed=0.f;
        for(float v:state.imu.gyroscope)gyro+=v*v;
        for(int i:VA15LegIndices)speed=std::max(speed,std::fabs(state.motor_state.dq[i]));
        if(!VA15RealStopSettled(state.real_stop_feedback,std::hypot(gravity[0],gravity[1]),
            std::sqrt(gyro),speed,.48f,.15f))return;
        std::vector<float> effort(18,0.f);
        for(int i:VA15LegIndices)
        {
            const auto& c=command.motor_command;
            if(c.kp[i]<=0.f||c.kd[i]<0.f)return;
            effort[i]=c.kp[i]*(c.q[i]-state.motor_state.q[i])+
                c.kd[i]*(c.dq[i]-state.motor_state.dq[i])+c.tau[i];
            const float limit=i>=9?320.f:200.f;
            if(!std::isfinite(effort[i])||std::fabs(effort[i])>.8f*limit)return;
        }
        stand_tau=std::move(effort);stand_reference_valid=true;
    }

    void Request()
    {
        if (Active()) return;
        stage = Stage::Decelerate;
        reason = "decelerating; waiting for validated four-foot catch";
    }

    // Output is untouched while waiting for a supported catch. Once captured,
    // the actor is no longer used. No timeout ever grants lowering permission.
    void Step(const RobotState<float>& state, RobotCommand<float>& command,
              const std::vector<float>& stand_q, const std::vector<float>& stand_kp,
              const std::vector<float>& stand_kd, double dt, float command_speed,
              float minimum_height = .48f, float maximum_tilt = .15f)
    {
        if (!Active()) return;
        if (stage == Stage::Blocked || Ready())
        { if (captured) command = held; return; }
        if (!std::isfinite(dt) || dt <= 0. || dt > .02 || !ValidPose(state) ||
            !std::isfinite(pose_blend_duration) || pose_blend_duration<.5 || pose_blend_duration>2. ||
            !ValidVector(stand_q) || !ValidVector(stand_kp) || !ValidVector(stand_kd) || !ValidCommand(command) ||
            !std::isfinite(command_speed) || !std::isfinite(minimum_height) || minimum_height <= 0.f ||
            !std::isfinite(maximum_tilt) || maximum_tilt <= 0.f)
        { Block("invalid state/config/time; no lowering"); if (captured) command=held; return; }
        for(int i:VA15LegIndices)
            if(stand_kp[i]<=0.f || stand_kd[i]<0.f || command.motor_command.kp[i]<0.f || command.motor_command.kd[i]<0.f)
            {Block("invalid leg gains; no lowering");if(captured)command=held;return;}
        elapsed += dt;
        const auto gravity=QuatRotateInverse(state.imu.quaternion,{0.f,0.f,-1.f});
        const float tilt=std::hypot(gravity[0],gravity[1]);
        float gyro=0.f, joint_speed=0.f;
        for (float v:state.imu.gyroscope) gyro+=v*v;
        gyro=std::sqrt(gyro);
        for (int i:VA15LegIndices) joint_speed=std::max(joint_speed,std::fabs(state.motor_state.dq[i]));
        const auto& evidence=state.real_stop_feedback;
        const bool sources=VA15StopFeedbackValid(evidence);
        const bool four=sources && std::all_of(evidence.contacts.begin(),evidence.contacts.end(),[](bool v){return v;});
        const bool upright=sources && evidence.base_height>=minimum_height && tilt<=maximum_tilt;

        if (stage==Stage::Decelerate)
        {
            // Keep all four REAL contacts and the same height/tilt gate. Relax
            // neither contact calibration nor final stillness requirements.
            // Only the *catch* may have residual joint motion; final settling
            // below still requires <=0.55 rad/s and 0.5 s stationary support.
            const bool can_catch=stand_reference_valid && four && upright && command_speed<=.03f &&
                evidence.planar_speed<.08f && gyro<.35f && joint_speed<3.f;
            capture_window=can_catch ? capture_window+dt : 0.;
            if (capture_window+1.e-9>=.010)
            {
                if (!Capture(state,command))
                {
                    // A high-error gait frame is not a reason to erase B or
                    // raise the envelope. Wait for another supported frame.
                    capture_window=0.;
                    reason="waiting for a well-conditioned supported catch";
                    if(elapsed>=8.)Block("no bounded supported catch in 8 s; no forced lowering");
                    return;
                }
                stage=Stage::CatchStand;
                reason="actor stopped; torque-continuous fixed-stand catch";
                command=held;
                return; // first frame is exactly torque-equivalent
            }
            if (elapsed>=8.) Block("no supported catch in 8 s; no forced lowering");
            return;
        }
        // A stale/missing source cannot be turned into a supporting foot by
        // the FK shadow, a timer, a button press or a repeated request.
        if (!sources || !upright || gyro>.6f)
        { Block("support feedback/attitude lost after catch; no lowering"); command=held; return; }
        if (elapsed>=14.)
        { Block("fixed stand did not qualify in time; no lowering"); command=held; return; }
        brake_time+=dt;
        if (stage==Stage::CatchStand)
        {
            if(four)progress=std::min(1.,progress+dt/pose_blend_duration);
            else {quiet=0.;reason="four-foot pose blend paused; independent damping remains active";}
            const float blend=static_cast<float>(progress*progress*(3.-2.*progress));
            const float effort_progress=std::min(1.,brake_time/.30);
            const float effort_blend=effort_progress*effort_progress*(3.f-2.f*effort_progress);
            for (int i:VA15LegIndices)
            {
                const float kp=(1.f-blend)*entry.motor_command.kp[i]+blend*stand_kp[i];
                const float kd=(1.f-blend)*entry.motor_command.kd[i]+blend*stand_kd[i];
                if (kp<=0.f || kd<0.f || !std::isfinite(kp) || !std::isfinite(kd))
                { Block("invalid stand gains; no lowering"); command=held; return; }
                // Equivalent static target for the LAST REQUESTED total PD+FF
                // torque, not an arbitrary measured swing pose. Arm stays at
                // its exact last command throughout the B2-only stand catch.
                // Do NOT retain the captured joint velocity forever: that
                // would continually cancel damping and inject gait-phase
                // energy. Fade it in 0.1 s, and dynamic effort to the known
                // standing effort in 0.3 s, even while a pose blend is paused.
                const float effort=(1.f-effort_blend)*capture_tau[i]+effort_blend*stand_tau[i];
                const float velocity_memory=capture_dq[i]*std::exp(-brake_time/.10);
                const float equivalent=capture_q[i]+(effort+kd*velocity_memory)/kp;
                // Optional local experiment: do not drag planted feet toward
                // an unloaded nominal joint pose. Hold the supported measured
                // geometry with the PRE-walk verified standing effort. Initial
                // total torque is still exactly equivalent; all final support,
                // height, uncertainty, speed and quiet-time gates are unchanged.
                const float settled_target=hold_supported_pose
                    ? capture_q[i]+stand_tau[i]/stand_kp[i] : stand_q[i];
                held.motor_command.q[i]=(1.f-blend)*equivalent+blend*settled_target;
                held.motor_command.dq[i]=0.f;
                held.motor_command.kp[i]=kp;
                held.motor_command.kd[i]=kd;
                held.motor_command.tau[i]=0.f;
            }
            if (progress>=1.) {stage=Stage::Settle; reason="fixed stand; qualifying quiet support";}
        }
        if (stage==Stage::Settle)
        {
            float drift=0.f;
            for (int i:VA15LegIndices)
            {
                drift=std::max(drift,std::fabs(state.motor_state.q[i]-last_q[i]));
            }
            // Load-bearing PD needs nonzero position error. Do not demand
            // convergence to the unloaded nominal joint angles as proof of
            // standing; use the unchanged measured four-foot/height/speed
            // checks plus bounded joint drift instead.
            const bool stable=VA15RealStopSettled(evidence,tilt,gyro,joint_speed,minimum_height,maximum_tilt)
                && drift<=.03f;
            quiet=stable ? quiet+dt : 0.;
            if (quiet+1.e-9>=.5)
            {stage=Stage::ReadyToLower; reason="independent quiet stand confirmed; arm stow may start";}
        }
        last_q=state.motor_state.q;
        command=held;
    }

private:
    RobotCommand<float> entry;
    std::vector<float> capture_q,capture_dq,capture_tau,last_q,stand_tau;
    static bool ValidVector(const std::vector<float>& value)
    {return value.size()==18 && std::all_of(value.begin(),value.end(),[](float v){return std::isfinite(v);});}
    static bool ValidCommand(const RobotCommand<float>& command)
    {
        const auto& c=command.motor_command;
        return ValidVector(c.q)&&ValidVector(c.dq)&&ValidVector(c.kp)&&ValidVector(c.kd)&&ValidVector(c.tau);
    }
    static bool ValidPose(const RobotState<float>& state)
    {
        if (!ValidVector(state.motor_state.q) || !ValidVector(state.motor_state.dq) ||
            state.imu.quaternion.size()!=4 || state.imu.gyroscope.size()!=3) return false;
        float norm=0.f;
        for(float v:state.imu.quaternion) {if(!std::isfinite(v))return false; norm+=v*v;}
        for(float v:state.imu.gyroscope) if(!std::isfinite(v))return false;
        return norm>=.9f && norm<=1.1f;
    }
    bool Capture(const RobotState<float>& state,const RobotCommand<float>& command)
    {
        const auto& c=command.motor_command;
        if (!ValidVector(c.q)||!ValidVector(c.dq)||!ValidVector(c.kp)||
            !ValidVector(c.kd)||!ValidVector(c.tau)) return false;
        entry=command; held=command;
        capture_q=last_q=state.motor_state.q; capture_dq=state.motor_state.dq;
        capture_tau.assign(18,0.f);
        for(int i:VA15LegIndices)
        {
            if(c.kp[i]<=0.f || c.kd[i]<0.f) return false;
            capture_tau[i]=c.kp[i]*(c.q[i]-capture_q[i])+
                c.kd[i]*(c.dq[i]-capture_dq[i])+c.tau[i];
            const float equivalent=capture_q[i]+(capture_tau[i]+c.kd[i]*capture_dq[i])/c.kp[i];
            if(!std::isfinite(equivalent) || std::fabs(equivalent-capture_q[i])>.6f) return false;
            held.motor_command.q[i]=equivalent;
            held.motor_command.dq[i]=0.f; held.motor_command.tau[i]=0.f;
        }
        captured=true;
        return true;
    }
    void Block(const char* text) {stage=Stage::Blocked; reason=text;}
};
