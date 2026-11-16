// LOCAL HEADLESS SIMULATION ONLY. No Unitree SDK, DDS, UDP, real launcher,
// camera or robot-control endpoint is linked or initialized.
#include "simulation_feedback.hpp"
#include "independent_model_io.hpp"
#include "b_stop_brake.hpp"

// Preserve the archived include guard while substituting only the local
// transition-shape experiment. The archived controller remains unmodified.
#define VA15ControlledStop VA15ArchivedStopNotSelected
#include "../va15_controlled_getdown_20261020/va15_controlled_stop.hpp"
#undef VA15ControlledStop

// Test-only dependency injection into the archived native FSM. Load its
// original feedback definitions BEFORE these substitutions (above). The
// legacy production validator remains unchanged and rejects every frame of
// our bridge because sport_fresh is false. No hardware profile can select it.
#define VA15StopFeedbackValid VA15SimEstimatedStopValid
#define VA15RealStopSettled VA15SimEstimatedStopSettled
#include "stop_response_controller.hpp"
#define main archived_truth_reference_main_not_called
#include "../va15_controlled_getdown_20261020/simulate_controlled_getdown.cpp"
#undef main
#undef VA15StopFeedbackValid
#undef VA15RealStopSettled

namespace independent=va15_independent;
static double Quantile(std::vector<double> values,double q)
{if(values.empty())return independent::NaN;std::sort(values.begin(),values.end());return values[std::size_t((values.size()-1)*q)];}
static void Number(std::ostream& stream,double v){if(std::isfinite(v))stream<<v;else stream<<"null";}

int main(int argc,char** argv)
{
    try
    {
        std::string model_path,output,fault="none";
        double speed=.2,walk=8.,friction=.8,observer_period=.010,tau_scale=1.,b_decel=.12,pose_blend=2.;
        for(int i=1;i<argc;++i)
        {
            if(i+1>=argc)throw std::runtime_error("Missing value");const std::string key=argv[i];
            if(key=="--model")model_path=argv[++i];else if(key=="--output")output=argv[++i];
            else if(key=="--speed")speed=std::stod(argv[++i]);else if(key=="--walk-seconds")walk=std::stod(argv[++i]);
            else if(key=="--friction")friction=std::stod(argv[++i]);else if(key=="--fault")fault=argv[++i];
            else if(key=="--observer-period")observer_period=std::stod(argv[++i]);
            else if(key=="--b-decel")b_decel=std::stod(argv[++i]);
            else if(key=="--pose-blend")pose_blend=std::stod(argv[++i]);
            else if(key=="--tau-scale")tau_scale=std::stod(argv[++i]);else throw std::runtime_error("Unknown option");
        }
        if(model_path.empty()||output.empty()||fs::exists(output)||fs::exists(output+".json")||
            !std::isfinite(speed)||speed<0||speed>.3||!std::isfinite(walk)||walk<0||walk>40||
            !std::isfinite(friction)||friction<.5||friction>1.0||!std::isfinite(tau_scale)||tau_scale<.75||tau_scale>1.25||
            !std::isfinite(pose_blend)||pose_blend<.5||pose_blend>2.||
            (std::fabs(observer_period-.005)>1.e-9&&std::fabs(observer_period-.010)>1.e-9))
            throw std::runtime_error("New output and declared 0..0.3 speed/.5..1.0 friction/5 or 10ms observer required");
        if(fault!="none"&&fault!="stale"&&fault!="tau_nan"&&fault!="imu_sign"&&
           fault!="partial_tau_zero"&&fault!="stale_after_catch")throw std::runtime_error("Unknown observer-stream fault");
        independent::Observer observer(LoadIndependentModel(model_path));
        SimRL rl("/home/dong/rl_sar_b2_z1_old/policy/b2_z1_no_gun/VA15/scene_flat.xml",true,friction,0.);
        const auto ordinary_decel=rl.params.Get<std::vector<float>>("command_decel_limits");
        const auto b_decel_limits=va15_sim_brake::Limits(ordinary_decel,true,b_decel);
        if(!rl.RequiresRealStopFeedback())throw std::runtime_error("Oracle fallback branch must be disabled");
        std::array<int,12> position_ids,velocity_ids;
        const char* legs[]={"FR","FL","RR","RL"};const char* joints[]={"hip","thigh","calf"};
        for(int i=0;i<4;++i)for(int j=0;j<3;++j)
        {
            const auto name=std::string(legs[i])+"_"+joints[j]+"_joint";
            const int joint=mj_name2id(rl.m,mjOBJ_JOINT,name.c_str());if(joint<0)throw std::runtime_error("Missing leg joint");
            position_ids[3*i+j]=rl.m->jnt_qposadr[joint];velocity_ids[3*i+j]=rl.m->jnt_dofadr[joint];
        }
        const int imu=mj_name2id(rl.m,mjOBJ_SENSOR,"imu_acc");if(imu<0)throw std::runtime_error("Missing IMU sensor");
        if(!fs::path(output).parent_path().empty())fs::create_directories(fs::path(output).parent_path());
        std::ofstream csv(output);csv<<std::setprecision(12);
        csv<<"elapsed,state,stop_stage,estimate_valid,stop_review,loads_FR_FL_RR_RL,h_mean,h_lower,v_mean,v_upper,"
              "truth_h,truth_speed,truth_four,tilt,policy_vx,stand_progress,quiet,source_sport_fresh,fault_active,reason,"
              "truth_x,truth_y,truth_vx,truth_vy,path_after_B,xy_distance_from_B,command_decel_x\n";
        const double initial=rl.d->time,start_walk=initial+3.,press_b=start_walk+walk;
        const int observer_stride=std::lround(observer_period/.005);
        auto locomotion=std::dynamic_pointer_cast<RLFSMStateRLLocomotion>(rl.fsm.states_.at("RLFSMStateRLLocomotion"));
        locomotion->controlled_stop.pose_blend_duration=pose_blend;
        bool b_sent=false,captured=false,lowered=false,fell=false,blocked=false;
        bool fault_active=false,fault_observed=false,reference_ready=false;
        double capture=-1.,lower=-1.,prone=-1.,passive_since=-1.,blocked_since=-1.,fault_since=-1.;
        double max_tilt=0.,max_slide=0.,x_at_b=0.,last_received=initial;
        double y_at_b=0.,last_x=0.,last_y=0.,path_after_b=0.,max_xy=0.,max_xy_before_lower=0.;
        double command_at_b=0.,command_small=-1.,command_zero=-1.,truth_quiet=-1.,truth_quiet_window=0.;
        double capture_xy=independent::NaN,capture_path=independent::NaN,truth_speed_at_b=0.;
        double max_tilt_after_b=0.;
        double h_at_lower=independent::NaN,v_at_lower=independent::NaN;
        std::uint32_t last_tick=0;
        unsigned false_catch=0,false_lower=0,sport_frames=0,fault_rows=0,fault_invalid=0;
        unsigned movement_rows=0,valid_movement_rows=0;
        std::vector<double> height_errors,speed_errors;
        std::string reason="simulation deadline";
        for(int step=0;step<int((3.+walk+23.)/.005);++step)
        {
            rl.ReadState(); // copies sensor q/dq/tau/IMU; reference feedback overwritten below
            const auto truth=rl.robot_state.real_stop_feedback; // evaluation ONLY
            const double now=rl.d->time,elapsed=now-initial;
            const bool true_four=std::all_of(truth.contacts.begin(),truth.contacts.end(),[](bool v){return v;});
            const bool press_this_frame=!b_sent&&now>=press_b;
            if(press_this_frame)
            {
                b_sent=true;x_at_b=last_x=rl.d->qpos[0];y_at_b=last_y=rl.d->qpos[1];
                command_at_b=rl.obs.commands[0];truth_speed_at_b=truth.planar_speed;
                // Simulation-only override; the YAML and ordinary command
                // acceleration/deceleration remain unchanged before B.
                rl.params.config_node["command_decel_limits"]=b_decel_limits;
                if(fault!="none"&&fault!="stale_after_catch"){fault_active=true;fault_since=now;}
            }
            if(step%observer_stride==0)
            {
                independent::Sample sample;sample.mode.fill(10);
                for(int i=0;i<12;++i)
                {
                    sample.q[i]=rl.d->qpos[position_ids[i]];sample.dq[i]=rl.d->qvel[velocity_ids[i]];
                    sample.tau[i]=tau_scale*rl.d->qfrc_actuator[velocity_ids[i]];
                }
                sample.quaternion=Eigen::Quaterniond(rl.robot_state.imu.quaternion[0],rl.robot_state.imu.quaternion[1],
                    rl.robot_state.imu.quaternion[2],rl.robot_state.imu.quaternion[3]);
                for(int i=0;i<3;++i)
                {sample.gyro[i]=rl.robot_state.imu.gyroscope[i];sample.specific_force[i]=rl.d->sensordata[rl.m->sensor_adr[imu]+i];}
                const bool freeze=fault_active&&(fault=="stale"||fault=="stale_after_catch");
                if(!freeze){last_tick=static_cast<std::uint32_t>(std::lround(now*1000.));last_received=now;}
                if(fault_active&&fault=="tau_nan")sample.tau[0]=independent::NaN;
                if(fault_active&&fault=="imu_sign")sample.specific_force=-sample.specific_force;
                if(fault_active&&fault=="partial_tau_zero")for(int i=0;i<3;++i)sample.tau[i]=0.;
                observer.Update(sample,last_tick,last_received);
            }
            const auto estimate=observer.Snapshot(now);
            // The entire evidence container is replaced BEFORE any FSM Run,
            // support cache, policy-start permission, catch or lower decision.
            rl.robot_state.real_stop_feedback=va15_sim_stop::Feedback(estimate);
            const auto evidence=rl.robot_state.real_stop_feedback;
            if(evidence.sport_fresh){++sport_frames;throw std::runtime_error("Oracle/sport freshness leak");}
            rl.control.current_keyboard=Input::Keyboard::None;rl.control.current_gamepad=Input::Gamepad::None;
            if(now>=start_walk&&!b_sent)rl.control.x=speed;
            if(press_this_frame)rl.control.current_gamepad=Input::Gamepad::B;
            rl.StateController(&rl.robot_state,&rl.robot_command);
            if(step%4==0)rl.Infer();
            const auto state=rl.fsm.current_state_->GetStateName();const auto& stop=locomotion->controlled_stop;
            reference_ready=reference_ready||stop.stand_reference_valid;
            const double tilt=rl.Tilt();max_tilt=std::max(max_tilt,tilt);
            if(tilt>.5||!std::isfinite(rl.d->qpos[2])){fell=true;reason="physical tilt/nonfinite failure";break;}
            const double xy=std::hypot(rl.d->qpos[0]-x_at_b,rl.d->qpos[1]-y_at_b);
            if(b_sent)
            {
                max_tilt_after_b=std::max(max_tilt_after_b,tilt);
                path_after_b+=std::hypot(rl.d->qpos[0]-last_x,rl.d->qpos[1]-last_y);
                last_x=rl.d->qpos[0];last_y=rl.d->qpos[1];max_xy=std::max(max_xy,xy);
                if(!lowered)max_xy_before_lower=std::max(max_xy_before_lower,xy);
                if(command_small<0.&&std::fabs(rl.obs.commands[0])<=.03)command_small=now-press_b;
                if(command_zero<0.&&std::fabs(rl.obs.commands[0])<=1.e-6)command_zero=now-press_b;
                // Truth is an evaluation metric ONLY, never a permission.
                const bool quiet_truth=true_four&&truth.base_height>=.48&&truth.planar_speed<.04&&tilt<.15;
                truth_quiet_window=quiet_truth?truth_quiet_window+.005:0.;
                if(truth_quiet<0.&&truth_quiet_window>=.5)truth_quiet=now-press_b;
            }
            if(now>=start_walk&&now<press_b)
            {
                ++movement_rows;valid_movement_rows+=estimate.estimate_valid_for_review;
                if(estimate.estimate_valid_for_review)
                {
                    height_errors.push_back(std::fabs(estimate.base_height-truth.base_height));
                    speed_errors.push_back((estimate.velocity.head<2>()-Eigen::Vector2d(rl.d->qvel[0],rl.d->qvel[1])).norm());
                }
            }
            if(stop.captured&&!captured)
            {
                captured=true;capture=now-press_b;
                capture_xy=xy;capture_path=path_after_b;
                if(!true_four||truth.base_height<.48||truth.planar_speed>.1||tilt>.18)++false_catch;
                if(fault=="stale_after_catch"){fault_active=true;fault_since=now;}
            }
            if(state=="RLFSMStateGetDown"&&!lowered)
            {
                lowered=true;lower=now-press_b;h_at_lower=truth.base_height;v_at_lower=truth.planar_speed;
                if(!true_four||truth.base_height<.48||truth.planar_speed>=.04||tilt>.15)++false_lower;
            }
            if(b_sent)max_slide=std::max(max_slide,std::fabs(rl.d->qpos[0]-x_at_b));
            if(fault_active && now>fault_since+.15)
            {
                fault_observed=true;++fault_rows;
                // Partial load can retain a valid two/three-foot estimate,
                // but must not yield a four-foot stopped qualification.
                const bool invalid=!estimate.stop_candidate_for_review&&
                    (fault=="partial_tau_zero"||!estimate.estimate_valid_for_review);
                fault_invalid+=invalid;
            }
            if(stop.stage==VA15ControlledStop::Stage::Blocked)
            {
                blocked=true;reason=stop.reason;if(blocked_since<0.)blocked_since=now;
            }
            if(step%4==0)
            {
                csv<<elapsed<<','<<state<<','<<int(stop.stage)<<','<<estimate.estimate_valid_for_review<<','<<estimate.stop_candidate_for_review<<',';
                for(bool v:estimate.load_candidate)csv<<(v?'1':'0');
                csv<<','<<estimate.base_height<<','<<evidence.base_height<<','<<estimate.planar_speed<<','<<evidence.planar_speed
                   <<','<<truth.base_height<<','<<truth.planar_speed<<','<<true_four<<','<<tilt<<','<<rl.obs.commands[0]<<','
                   <<stop.progress<<','<<stop.quiet<<','<<evidence.sport_fresh<<','<<fault_active<<','<<stop.reason
                   <<','<<rl.d->qpos[0]<<','<<rl.d->qpos[1]<<','<<rl.d->qvel[0]<<','<<rl.d->qvel[1]<<','
                   <<path_after_b<<','<<(b_sent?xy:0.)<<','<<(b_sent?b_decel_limits[0]:ordinary_decel[0])<<'\n';
            }
            if(state=="RLFSMStatePassive"&&lowered&&rl.d->qpos[2]<.22)
            {
                if(passive_since<0.)passive_since=now;
                if(now-passive_since>=2.&&tilt<.3&&truth.planar_speed<.04)
                {prone=now-press_b;reason="estimated support stop -> getdown -> settled prone/passive";break;}
            }
            // Observe dynamics after refusing lowering, not just the first
            // rejection frame. This still is not a full fault recovery claim.
            if(blocked_since>=0.&&now-blocked_since>=1.)break;
            rl.Integrate();
        }
        const bool success=fault=="none" ? (!fell&&lowered&&prone>=0.&&false_catch==0&&false_lower==0&&sport_frames==0) :
            (!fell&&!lowered&&blocked&&fault_observed&&fault_rows>0&&fault_invalid==fault_rows&&sport_frames==0);
        csv.flush();if(!csv)throw std::runtime_error("CSV I/O failure");
        std::ofstream out(output+".json");out<<std::boolalpha<<std::setprecision(12);
        out<<"{\n  \"offline_closed_loop_pass\": "<<success<<",\n  \"hardware_qualified\": false,\n  \"control_authorized\": false,"
            "\n  \"observer_in_stop_control_loop\": true,\n  \"truth_drives_stop_decisions\": false,\n  \"sport_fresh_frames\": "<<sport_frames
           <<",\n  \"actor\": \"unchanged native VA15 model_5000\",\n  \"z1_servo\": \"existing implicit simulation proxy\","
             "\n  \"sensor_fault_scope\": \"observer stream only, not full actor/transport watchdog\",\n  \"speed\": "<<speed
           <<",\n  \"walk_seconds\": "<<walk<<",\n  \"observer_period\": "<<observer_period<<",\n  \"friction\": "<<friction
           <<",\n  \"sensor_torque_scale\": "<<tau_scale<<",\n  \"fault\": \""<<fault
           <<"\",\n  \"fell\": "<<fell<<",\n  \"captured\": "<<captured<<",\n  \"lowered\": "<<lowered<<",\n  \"blocked\": "<<blocked
           <<",\n  \"estimated_stand_reference_recorded\": "<<reference_ready<<",\n  \"capture_after_B\": "<<capture<<",\n  \"lower_after_B\": "<<lower
           <<",\n  \"prone_after_B\": "<<prone<<",\n  \"false_supported_catches\": "<<false_catch<<",\n  \"false_lowering_permissions\": "<<false_lower
           <<",\n  \"max_tilt_deg\": "<<max_tilt*57.2957795<<",\n  \"x_at_B\": "<<x_at_b<<",\n  \"max_travel_after_B\": "<<max_slide
           <<",\n  \"B_decel_limit\": "<<b_decel<<",\n  \"ordinary_decel_limit\": "<<ordinary_decel[0]
           <<",\n  \"pose_blend_duration\": "<<pose_blend
           <<",\n  \"command_at_B\": "<<command_at_b<<",\n  \"truth_speed_at_B\": "<<truth_speed_at_b
           <<",\n  \"command_small_after_B\": "<<command_small<<",\n  \"command_zero_after_B\": "<<command_zero
           <<",\n  \"truth_quiet_half_second_after_B\": "<<truth_quiet
           <<",\n  \"max_xy_after_B\": "<<max_xy<<",\n  \"max_xy_before_lower\": "<<max_xy_before_lower
           <<",\n  \"max_tilt_after_B_deg\": "<<max_tilt_after_b*57.2957795
           <<",\n  \"xy_at_capture\": ";Number(out,capture_xy);out<<",\n  \"path_at_capture\": ";Number(out,capture_path);out
           <<",\n  \"true_height_when_lowering\": ";Number(out,h_at_lower);out<<",\n  \"true_speed_when_lowering\": ";Number(out,v_at_lower);
        out<<",\n  \"walking_estimate_valid_fraction\": "<<(movement_rows?double(valid_movement_rows)/movement_rows:0.)
           <<",\n  \"walking_height_error_p95\": ";Number(out,Quantile(height_errors,.95));out<<",\n  \"walking_speed_error_p95\": ";Number(out,Quantile(speed_errors,.95));
        out<<",\n  \"fault_checked_rows\": "<<fault_rows<<",\n  \"fault_rejected_rows\": "<<fault_invalid<<",\n  \"reason\": \""<<reason<<"\"\n}\n";
        if(!out)throw std::runtime_error("JSON I/O failure");
        std::cout<<"ESTIMATED STOP "<<(success?"PASS":"FAIL")<<" capture="<<capture<<" lower="<<lower<<" prone="<<prone
            <<" false_catch="<<false_catch<<" false_lower="<<false_lower<<" reason="<<reason<<"; NO HARDWARE AUTHORITY\n";
        return success?0:2;
    }
    catch(const std::exception& error){std::cerr<<"ERROR: "<<error.what()<<'\n';return 1;}
}
