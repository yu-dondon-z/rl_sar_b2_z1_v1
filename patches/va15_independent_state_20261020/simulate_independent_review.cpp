// Reuse the prior native VA15 actor/FSM simulator unchanged, but evaluate
// the new observer independently. Oracle state drives the REFERENCE FSM;
// observer measurements are NOT injected into commands in this shadow test.
#include "independent_model_io.hpp"
#define main va15_reference_test_main_not_called
#include "../va15_controlled_getdown_20261020/simulate_controlled_getdown.cpp"
#undef main

namespace indep=va15_independent;
static double Q(std::vector<double> x,double q)
{if(x.empty())return indep::NaN;std::sort(x.begin(),x.end());return x[std::size_t((x.size()-1)*q)];}
static void Num(std::ostream& o,double v){if(std::isfinite(v))o<<v;else o<<"null";}

int main(int argc,char** argv)
{
    try
    {
        std::string model_path,output,fault="none";double speed=.2,walk=8.,friction=.8;
        for(int i=1;i<argc;++i)
        {
            if(i+1>=argc)throw std::runtime_error("Missing value");const std::string arg=argv[i];
            if(arg=="--model")model_path=argv[++i];else if(arg=="--output")output=argv[++i];
            else if(arg=="--speed")speed=std::stod(argv[++i]);else if(arg=="--walk-seconds")walk=std::stod(argv[++i]);
            else if(arg=="--friction")friction=std::stod(argv[++i]);else if(arg=="--fault")fault=argv[++i];else throw std::runtime_error("Unknown option");
        }
        if(output.empty()||fs::exists(output)||fs::exists(output+".json")||!std::isfinite(speed)||!std::isfinite(walk)||!std::isfinite(friction)||speed<0||speed>.3||walk<0||walk>40||friction<.3||friction>1.2)
            throw std::runtime_error("New output and declared .0-.3 speed / .3-1.2 friction required");
        if(fault!="none"&&fault!="stale"&&fault!="tau_nan"&&fault!="imu_sign")throw std::runtime_error("Unknown sensor fault");
        const auto model=LoadIndependentModel(model_path);indep::Observer observer(model);
        SimRL rl("/home/dong/rl_sar_b2_z1_old/policy/b2_z1_no_gun/VA15/scene_flat.xml",true,friction,0.);
        std::array<int,12> qids,vids;
        const char* names[]={"FR","FL","RR","RL"};const char* parts[]={"hip","thigh","calf"};
        for(int i=0;i<4;++i)for(int j=0;j<3;++j)
        {const auto name=std::string(names[i])+"_"+parts[j]+"_joint";const int id=mj_name2id(rl.m,mjOBJ_JOINT,name.c_str());
         if(id<0)throw std::runtime_error("Missing native joint");qids[3*i+j]=rl.m->jnt_qposadr[id];vids[3*i+j]=rl.m->jnt_dofadr[id];}
        const int sensor=mj_name2id(rl.m,mjOBJ_SENSOR,"imu_acc");if(sensor<0)throw std::runtime_error("Missing actual IMU accelerometer sensor");
        fs::create_directories(fs::path(output).parent_path());std::ofstream csv(output);
        csv<<"elapsed,state,review_valid,stop_review,support_count,accepted_feet,truth_h,estimated_h,height_error,speed_error,estimate_vx,truth_vx,load_DDS,true_load40_DDS,forceFR,forceFL,forceRR,forceRL\n";
        const double initial=rl.d->time,walk_start=initial+3.,press_b=walk_start+walk,fault_start=walk_start+2.;
        bool b_sent=false,fell=false,lowered=false;int valid=0,reviewed=0,stance_valid=0,stance_rows=0;
        int air_rows=0,false_load=0,heavy_rows=0,missed_heavy=0,unqualified_stop=0,fault_rows=0,fault_invalid=0;
        std::vector<double> height_errors,speed_errors,normal_errors;double max_tilt=0.;
        for(int step=0;step<int((3.+walk+16.)/.005);++step)
        {
            rl.ReadState();const double now=rl.d->time,elapsed=now-initial;
            indep::Sample s;s.mode.fill(10);
            for(int i=0;i<12;++i){s.q[i]=rl.d->qpos[qids[i]];s.dq[i]=rl.d->qvel[vids[i]];s.tau[i]=rl.d->qfrc_actuator[vids[i]];}
            s.quaternion=Eigen::Quaterniond(rl.d->qpos[3],rl.d->qpos[4],rl.d->qpos[5],rl.d->qpos[6]);
            for(int i=0;i<3;++i){s.gyro[i]=rl.robot_state.imu.gyroscope[i];s.specific_force[i]=rl.d->sensordata[rl.m->sensor_adr[sensor]+i];}
            const bool fault_active=now>=fault_start&&fault!="none";
            if(fault_active&&fault=="tau_nan")s.tau[0]=indep::NaN;
            if(fault_active&&fault=="imu_sign")s.specific_force=-s.specific_force;
            const auto tick=static_cast<std::uint32_t>(std::lround((fault_active&&fault=="stale"?fault_start:now)*1000.));
            const double receive=(fault_active&&fault=="stale"?fault_start:now);
            observer.Update(s,tick,receive);const auto r=observer.Snapshot(now);
            std::array<double,4> truth{};
            for(int k=0;k<rl.d->ncon;++k)
            {
                const auto& c=rl.d->contact[k];const bool ground1=rl.m->geom_bodyid[c.geom1]==0,ground2=rl.m->geom_bodyid[c.geom2]==0;
                if(!ground1&&!ground2)continue;double f[6]{};mj_contactForce(rl.m,rl.d,k,f);
                for(int i=0;i<4;++i)
                {
                    const int native_to_policy[]={1,0,3,2};const int geom=rl.feet[native_to_policy[i]];
                    if(c.geom1!=geom&&c.geom2!=geom)continue;
                    double vertical=0.;for(int axis=0;axis<3;++axis)vertical+=c.frame[3*axis+2]*f[axis];
                    if(c.geom1==geom)vertical=-vertical;truth[i]+=vertical;
                }
            }
            rl.control.current_keyboard=Input::Keyboard::None;rl.control.current_gamepad=Input::Gamepad::None;
            if(now>=walk_start&&!b_sent)rl.control.x=speed;
            if(!b_sent&&now>=press_b){b_sent=true;rl.control.current_gamepad=Input::Gamepad::B;}
            rl.StateController(&rl.robot_state,&rl.robot_command);if(step%4==0)rl.Infer();
            const auto state=rl.fsm.current_state_->GetStateName();const double tilt=rl.Tilt();max_tilt=std::max(max_tilt,tilt);
            if(tilt>.5){fell=true;break;}
            const bool locomotion=state=="RLFSMStateRLLocomotion";
            if(locomotion&&elapsed>1.)
            {
                ++reviewed;valid+=r.estimate_valid_for_review;
                const bool physical_still=now<walk_start;
                if(physical_still){++stance_rows;stance_valid+=r.estimate_valid_for_review;}
                for(int i=0;i<4;++i)
                {
                    if(truth[i]<5.){++air_rows;false_load+=r.load_candidate[i];}
                    if(truth[i]>100.){++heavy_rows;missed_heavy+=!r.load_candidate[i];}
                    if(std::isfinite(r.normal_equivalent[i]))normal_errors.push_back(std::fabs(r.normal_equivalent[i]-truth[i]));
                }
                if(r.estimate_valid_for_review)
                {
                    height_errors.push_back(std::fabs(r.base_height-rl.d->xpos[3*rl.base+2]));
                    speed_errors.push_back((r.velocity.head<2>()-Eigen::Vector2d(rl.d->qvel[0],rl.d->qvel[1])).norm());
                }
                if(r.stop_candidate_for_review&&(!std::all_of(truth.begin(),truth.end(),[](double f){return f>5.;})||
                    rl.d->xpos[3*rl.base+2]<.45||std::hypot(rl.d->qvel[0],rl.d->qvel[1])>.08||tilt>.18))++unqualified_stop;
                if(fault_active&&now>fault_start+.15){++fault_rows;fault_invalid+=!r.estimate_valid_for_review&&!r.stop_candidate_for_review;}
            }
            if(step%4==0)
            {
                csv<<std::setprecision(12)<<elapsed<<','<<state<<','<<r.estimate_valid_for_review<<','<<r.stop_candidate_for_review<<','<<r.support_count<<','<<r.accepted_feet<<','
                   <<rl.d->xpos[3*rl.base+2]<<','<<r.base_height<<','<<std::fabs(r.base_height-rl.d->xpos[3*rl.base+2])<<','
                   <<(r.velocity.head<2>()-Eigen::Vector2d(rl.d->qvel[0],rl.d->qvel[1])).norm()<<','<<r.velocity.x()<<','<<rl.d->qvel[0]<<',';
                for(bool c:r.load_candidate)csv<<(c?'1':'0');csv<<',';for(double f:truth)csv<<(f>40.?'1':'0');for(double f:r.normal_equivalent)csv<<','<<f;csv<<'\n';
            }
            if(state=="RLFSMStateGetDown")lowered=true;
            if(lowered&&state=="RLFSMStatePassive")break;
            rl.Integrate();
        }
        // Declared OFFLINE review criteria, not hardware authority. Retain
        // failures; do not tune thresholds just to manufacture contact truth.
        const double validity=reviewed?double(valid)/reviewed:0.;
        const double false_rate=air_rows?double(false_load)/air_rows:0.;
        const bool pass=fault=="none" ? (!fell&&stance_rows>0&&double(stance_valid)/stance_rows>.90&&
            validity>.50&&Q(height_errors,.95)<.030&&Q(speed_errors,.95)<.10&&false_rate<.04&&unqualified_stop==0) :
            (!fell&&fault_rows>0&&fault_invalid==fault_rows);
        std::ofstream out(output+".json");out<<std::setprecision(12)<<std::boolalpha<<"{\n  \"offline_review_pass\": "<<pass
           <<",\n  \"hardware_qualified\": false,\n  \"control_authorized\": false,\n  \"observer_in_control_loop\": false,\n  \"reference_controller_uses_truth\": true,\n  \"speed\": "<<speed
           <<",\n  \"friction\": "<<friction<<",\n  \"sensor_fault\": \""<<fault<<"\",\n  \"fell\": "<<fell<<",\n  \"reviewed_rows\": "<<reviewed<<",\n  \"review_valid_fraction\": "<<validity
           <<",\n  \"height_error_p95\": ";Num(out,Q(height_errors,.95));out<<",\n  \"speed_error_p95\": ";Num(out,Q(speed_errors,.95));
        out<<",\n  \"force_equivalent_abs_error_p95\": ";Num(out,Q(normal_errors,.95));
        out<<",\n  \"air_leg_false_load_fraction\": "<<false_rate<<",\n  \"heavy_contact_missed_fraction\": "<<(heavy_rows?double(missed_heavy)/heavy_rows:0.)
           <<",\n  \"false_four_load_stop_candidates\": "<<unqualified_stop<<",\n  \"fault_reviewed_rows\": "<<fault_rows<<",\n  \"fault_invalid_rows\": "<<fault_invalid
           <<",\n  \"max_tilt_deg\": "<<max_tilt*57.2957795<<"\n}\n";
        std::cout<<"OBSERVER SHADOW "<<(pass?"PASS":"FAIL")<<" valid="<<validity<<" h95="<<Q(height_errors,.95)
                 <<" v95="<<Q(speed_errors,.95)<<" false_air="<<false_rate<<" false_stop="<<unqualified_stop<<"; no hardware authority\n";
        return pass?0:2;
    }
    catch(const std::exception& e){std::cerr<<"ERROR: "<<e.what()<<'\n';return 1;}
}
