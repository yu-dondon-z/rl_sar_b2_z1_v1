#pragma once
#include "near_visual_guide.hpp"

// Front-camera END SEEN -> original supported stop. This is deliberately NOT
// a rear-TCP arrival controller and does not consume cached unseen distance.
namespace va15_visual_end {
class Stop {
public:
    enum class Stage {Idle,Watching,Confirming,Braking,Standing,Interrupted};
    Stage stage=Stage::Idle;
    unsigned frames=0;
    const char* reason="waiting for explicit visual walking session";
    static constexpr std::uint64_t ConfirmationNs=500000000ULL;
    bool Requested()const{return stage==Stage::Braking||stage==Stage::Standing;}
    bool Stood()const{return stage==Stage::Standing;}
    void Begin(){*this=Stop();stage=Stage::Watching;reason="watching the current tracked crack for a visible cap";}
    void Interrupt(const char* why) {
        if(stage==Stage::Idle||stage==Stage::Standing)return;
        stage=Stage::Interrupted;reason=why;quiet_=0.;
    }
    void Observe(const va15_visual::Packet& p,std::uint64_t now,bool same_accepted_track) {
        if(stage!=Stage::Watching&&stage!=Stage::Confirming)return;
        if(!same_accepted_track||!va15_visual::Valid(p,now)||!p.image_received) {ResetProof();return;}
        if(p.sequence==sequence_)return; // one file is not six independent frames
        if(p.sequence<sequence_||p.capture_ns<=stamp_) {Interrupt("camera sequence reversed; protective stop, not visual completion");return;}
        const bool evidence=p.endpoint_candidate&&std::isfinite(p.endpoint_u)&&std::isfinite(p.endpoint_v)&&
            std::isfinite(p.endpoint_confidence)&&p.endpoint_confidence>=.97&&p.endpoint_confidence<=1.&&
            std::isfinite(p.endpoint_clear_pixels)&&p.endpoint_clear_pixels>=72.&&p.endpoint_clear_pixels<=300.&&
            p.endpoint_u>=420.&&p.endpoint_u<=1500.&&p.endpoint_v>=688.&&p.endpoint_v<=.82*1080.&&
            p.path.size()==p.path_count&&std::hypot(p.endpoint_u-p.path.back()[0],p.endpoint_v-p.path.back()[1])<=3.;
        if(!evidence){sequence_=p.sequence;stamp_=p.capture_ns;ResetProof();return;}
        if(frames&&(p.capture_ns-stamp_>va15_visual::MaximumAgeNs||
           std::fabs(p.endpoint_u-u_)>45.||std::fabs(p.endpoint_v-v_)>45.))ResetProof();
        if(!frames)first_=p.capture_ns;
        sequence_=p.sequence;stamp_=p.capture_ns;u_=p.endpoint_u;v_=p.endpoint_v;
        ++frames;stage=Stage::Confirming;reason="confirming visible end on the same tracked crack";
        if(frames>=6&&stamp_-first_>=ConfirmationNs) {
            stage=Stage::Braking;
            reason="VISUAL END SEEN: decelerating to supported stand; not rear TCP endpoint arrival";
        }
    }
    void Park(double dt,bool healthy,bool original_ready_and_current_quiet) {
        if(stage!=Stage::Braking)return;
        if(!healthy||!std::isfinite(dt)||dt<=0.||dt>.02) {
            Interrupt("state/time fault during visual-end stop; not normal completion");return;
        }
        quiet_=original_ready_and_current_quiet?quiet_+dt:0.;
        if(quiet_+1.e-9>=.5) {
            stage=Stage::Standing;
            reason="VISUAL END STOPPED: original supported stand verified; no automatic getdown/restart";
        }
    }
private:
    std::uint64_t sequence_=0,stamp_=0,first_=0;
    double u_=0.,v_=0.,quiet_=0.;
    void ResetProof(){frames=0;first_=0;stage=Stage::Watching;reason="no confirmed end; ordinary live guide/watchdogs remain active";}
};
}
