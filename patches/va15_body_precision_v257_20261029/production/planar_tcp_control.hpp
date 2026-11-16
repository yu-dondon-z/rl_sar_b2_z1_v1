#pragma once
#include "precision_tcp_control.hpp"
#include "visual_tcp.hpp"

namespace va15_planar_tcp {
class Controller {
public:
    va15_precision::Controller servo;
    double projection_error=NAN;
    bool observed=false,acquired=false,tracking=false;
    unsigned in_tolerance=0;
    bool Update(const va15_tcp::Pose& pose,const va15_tcp::Tracker& tracker,
                const std::vector<float>& measured,std::vector<float>& command,
                double dt,bool enabled) {
        observed=tracking=false;projection_error=NAN;
        Eigen::Vector3d actual,original,closest;
        if(!va15_tcp::MeasuredTCPBase(measured,actual))return false;
        observed=tracker.Projection(pose,actual,projection_error,closest);
        if(!enabled||!observed) {servo.Hold(command);in_tolerance=0;return true;}
        if(!va15_tcp::MeasuredTCPBase(command,original))return false;
        // Correct XY toward the nearest OBSERVED crack center. Keep the
        // actor's original vertical reference; do not turn a 50 mm planar
        // requirement into a new surface height or longitudinal setpoint.
        const Eigen::Vector3d actor_world=pose.p+pose.q*original;
        closest.z()=actor_world.z();
        const Eigen::Vector3d goal=pose.q.conjugate()*(closest-pose.p);
        const auto actor=command;
        if(!servo.Update(measured,actor,goal,dt,command))return false;
        in_tolerance=projection_error<=.05?in_tolerance+1:0;
        tracking=in_tolerance>=100;acquired|=tracking;
        return true;
    }
    void Reset(){*this=Controller();}
};
}
