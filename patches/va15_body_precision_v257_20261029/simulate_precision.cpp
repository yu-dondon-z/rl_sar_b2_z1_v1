// SDK-free physics with the VA15 actor/FSM. --camera-vision uses rendered
// RGB and production RHANet/LongTracker; legacy scenarios use synthetic UV.
#include "production/fsm_b2_z1_no_gun.hpp"
#include "production/visual_tcp.hpp"
#include "production/independent_model_io.hpp"
#include "production/foot_seam_geometry.hpp"
#include "production/precision_tcp_control.hpp"
#include "production/body_path_control.hpp"
#include "production/planar_tcp_control.hpp"
#include "production/tcp_priority_session.hpp"
#define main archived_simulation_main_not_called
#include "../va15_controlled_getdown_20261020/simulate_controlled_getdown.cpp"
#undef main
#include <algorithm>
#include <array>
#include <deque>
#include <thread>
#include <vector>
#include <GLFW/glfw3.h>
#include "mujoco_vision.hpp"

static double amplitude=.035,frequency=.8;
static double Path(double x){return amplitude*std::sin(frequency*x);}

class MuJoCoLiveViewer {
public:
    GLFWwindow* window=nullptr;
    mjvCamera camera{}; mjvOption option{}; mjvPerturb perturb{};
    mjvScene scene{}; mjrContext context{};
    int tcp=-1,base=-1,camera_id=-1; bool ready=false;
    const std::vector<unsigned char>* camera_overlay=nullptr;
    ~MuJoCoLiveViewer() {
        if(ready) {mjv_freeScene(&scene);mjr_freeContext(&context);}
        if(window) {glfwDestroyWindow(window);glfwTerminate();}
    }
    bool Init(mjModel* model,int tcp_site,int base_body,int front_camera) {
        if(!glfwInit())return false;
        glfwWindowHint(GLFW_VISIBLE,GLFW_TRUE);
        window=glfwCreateWindow(1280,800,"B2+Z1 VA15 V257 MuJoCo live deployment",nullptr,nullptr);
        if(!window)return false;
        glfwMakeContextCurrent(window);glfwSwapInterval(1);
        mjv_defaultCamera(&camera);mjv_defaultOption(&option);mjv_defaultPerturb(&perturb);
        camera.distance=3.;camera.azimuth=135.;camera.elevation=-22.;
        mjv_makeScene(model,&scene,2000);mjr_defaultContext(&context);
        mjr_makeContext(model,&context,mjFONTSCALE_150);
        mjr_resizeOffscreen(640,360,&context);
        glfwSetKeyCallback(window,[](GLFWwindow* w,int key,int,int action,int){
            if(key==GLFW_KEY_ESCAPE&&action==GLFW_PRESS)glfwSetWindowShouldClose(w,GLFW_TRUE);
        });
        tcp=tcp_site;base=base_body;camera_id=front_camera;ready=true;return true;
    }
    void AddVisuals(mjData* data,bool diagnostics=true) {
        double identity[9]{1.,0.,0.,0.,1.,0.,0.,0.,1.};float black[4]{.015f,.015f,.015f,1.f};
        for(double x=-2.;x<30.&&scene.ngeom+1<scene.maxgeom;x+=.10) {
            const double end=x+.10,dy=Path(end)-Path(x),length=std::hypot(.10,dy),yaw=std::atan2(dy,.10);
            double size[3]{length*.5+.001,.015,.0005},pos[3]{x+.05,.5*(Path(x)+Path(end)),.001};
            double rotation[9]{std::cos(yaw),-std::sin(yaw),0.,std::sin(yaw),std::cos(yaw),0.,0.,0.,1.};
            mjv_initGeom(&scene.geoms[scene.ngeom++],mjGEOM_BOX,size,pos,rotation,black);
        }
        if(!diagnostics||scene.ngeom+1>=scene.maxgeom)return;
        double sphere_size[3]{.025,0.,0.},sphere_pos[3]{
            data->site_xpos[3*tcp],data->site_xpos[3*tcp+1],.008};float green[4]{.1f,.9f,.1f,1.f};
        mjv_initGeom(&scene.geoms[scene.ngeom++],mjGEOM_SPHERE,sphere_size,sphere_pos,identity,green);
    }
    bool Render(mjModel* model,mjData* data) {
        if(!ready||glfwWindowShouldClose(window))return false;
        mjv_updateScene(model,data,&option,&perturb,&camera,mjCAT_ALL,&scene);
        AddVisuals(data);
        camera.lookat[0]=data->xpos[3*base];camera.lookat[1]=data->xpos[3*base+1];
        camera.lookat[2]=data->xpos[3*base+2]-.05;
        int width=0,height=0;glfwGetFramebufferSize(window,&width,&height);
        mjrRect viewport{0,0,width,height};mjr_render(viewport,&scene,&context);
        if(camera_overlay&&camera_overlay->size()==640*360*3&&width>=640&&height>=360) {
            mjrRect inset{width-640,height-360,640,360};
            mjr_drawPixels(camera_overlay->data(),nullptr,inset,&context);
        }
        glfwSwapBuffers(window);glfwPollEvents();return true;
    }
    bool CaptureCamera(mjModel* model,mjData* data,std::vector<unsigned char>& rgb,
                       int width=640,int height=360) {
        if(!ready||camera_id<0||glfwWindowShouldClose(window))return false;
        mjvCamera sensor_camera; mjv_defaultCamera(&sensor_camera);
        sensor_camera.type=mjCAMERA_FIXED;sensor_camera.fixedcamid=camera_id;
        mjv_updateScene(model,data,&option,&perturb,&sensor_camera,mjCAT_ALL,&scene);
        AddVisuals(data,false);
        mjr_setBuffer(mjFB_OFFSCREEN,&context);
        mjrRect viewport{0,0,width,height};mjr_render(viewport,&scene,&context);
        rgb.resize(static_cast<std::size_t>(width)*height*3);
        mjr_readPixels(rgb.data(),nullptr,viewport,&context);
        mjr_setBuffer(mjFB_WINDOW,&context);
        return true;
    }
};

static double ProjectionDistance(double x,double y) {
    double lo=x-.7,hi=x+.7;
    for(int i=0;i<30;++i) {
        const double a=(2.*lo+hi)/3.,b=(lo+2.*hi)/3.;
        const double da=(a-x)*(a-x)+(Path(a)-y)*(Path(a)-y);
        const double db=(b-x)*(b-x)+(Path(b)-y)*(Path(b)-y);
        if(da<db)hi=b;else lo=a;
    }
    const double best=.5*(lo+hi);return std::hypot(best-x,Path(best)-y);
}
static double Pixel(const va15_tcp::Pose& pose,double v) {
    double low=384.,high=1536.;
    for(int i=0;i<45;++i) {
        const double u=.5*(low+high);va15_tcp::V3 world;
        if(!va15_tcp::Project(pose,{u,v},world))return NAN;
        if(world.y()-Path(world.x())>0.)low=u;else high=u;
    }
    return .5*(low+high);
}

static void ConfigureCalibratedCamera(mjModel* model,int camera) {
    if(camera<0)throw std::runtime_error("front_depth_camera missing");
    const auto cfg=YAML::LoadFile((fs::path(__FILE__).parent_path()/"config/b2_builtin_front_camera.yaml").string());
    Eigen::Matrix3d rotation;
    for(int row=0;row<3;++row) {
        model->cam_pos[3*camera+row]=cfg["T_base_camera"][row][3].as<double>();
        for(int col=0;col<3;++col)rotation(row,col)=cfg["T_base_camera"][row][col].as<double>()*(col? -1.:1.);
    }
    const Eigen::Quaterniond q=Eigen::Quaterniond(rotation).normalized();
    model->cam_quat[4*camera]=q.w();model->cam_quat[4*camera+1]=q.x();
    model->cam_quat[4*camera+2]=q.y();model->cam_quat[4*camera+3]=q.z();
    model->cam_fovy[camera]=2.*std::atan(540./cfg["intrinsics"]["fy"].as<double>())*180./std::acos(-1.);
    model->cam_sensorsize[2*camera]=model->cam_sensorsize[2*camera+1]=0.;
}
int main(int argc,char** argv) {
    try {
        if(argc<5||argc>7||fs::exists(argv[2])||fs::exists(std::string(argv[2])+".json"))throw std::runtime_error("scenario new.csv friction fixed|guided|ik|body [--viewer] [--camera-vision]");
        const std::string mode=argv[4];
        if(mode!="fixed"&&mode!="guided"&&mode!="ik"&&mode!="body"&&mode!="planar")throw std::runtime_error("invalid mode");
        bool live_viewer=false,camera_vision=false;
        for(int i=5;i<argc;++i) {
            const std::string flag=argv[i];
            if(flag=="--viewer")live_viewer=true;
            else if(flag=="--camera-vision")camera_vision=true;
            else throw std::runtime_error("only --viewer and --camera-vision are supported as optional arguments");
        }
        if(camera_vision&&!live_viewer)throw std::runtime_error("--camera-vision requires --viewer so MuJoCo can render the front camera");
        const std::string scenario=argv[1];const double mu=std::stod(argv[3]);
        const bool estimated=scenario.rfind("estimated",0)==0;
        if(scenario=="estimated_bend_left"||scenario=="estimated_bend_right") {
            amplitude=scenario=="estimated_bend_left"?.20:-.20;frequency=1.2;
        }
        if(scenario=="estimated_gentle_left"||scenario=="estimated_gentle_right") {amplitude=scenario=="estimated_gentle_left"?.20:-.20;frequency=.4;}
        if(scenario=="estimated_straight"||scenario.rfind("estimated_probe",0)==0)amplitude=0.;
        SimRL rl("/home/dong/rl_sar_b2_z1_old/policy/b2_z1_no_gun/VA15/scene_flat.xml",true,mu,0.);
        rl.params.config_node["isolated_forward_trial"]=true;
        rl.params.config_node["isolated_forward_trial_motion"]=true;
        rl.params.config_node["near_visual_trial"]=true;
        rl.params.config_node["repeatable_visual_session"]=true;
        rl.params.config_node["continuous_visual_session"]=true;
        auto loco=std::dynamic_pointer_cast<RLFSMStateRLLocomotion>(rl.fsm.states_.at("RLFSMStateRLLocomotion"));
        loco->trial_configured=false;loco->Enter();
        va15_visual::Guide guide(true,true,true);va15_tcp::Tracker tracker;va15_tcp::History poses;
        const auto foot_model=LoadIndependentModel((fs::path(__FILE__).parent_path()/"production/b2_independent_model.json").string());
        va15_independent::Observer observer(foot_model,true);
        va15_precision::Controller servo;va15_planar_tcp::Controller planar;va15_body_path::Controller body;
        rl.params.config_node["visual_body_path_control"]=(mode=="body"||mode=="ik"||mode=="planar");
        tracker.training_workspace_recovery=true;
        std::array<int,12> dds_q,dds_v;const char* leg_names[]={"FR","FL","RR","RL"};
        const char* joint_names[]={"hip","thigh","calf"};
        for(int l=0;l<4;++l)for(int j=0;j<3;++j) {
            const auto name=std::string(leg_names[l])+"_"+joint_names[j]+"_joint";
            const int id=mj_name2id(rl.m,mjOBJ_JOINT,name.c_str());
            dds_q[3*l+j]=rl.m->jnt_qposadr[id];dds_v[3*l+j]=rl.m->jnt_dofadr[id];
        }
        const int imu=mj_name2id(rl.m,mjOBJ_SENSOR,"imu_acc");
        std::deque<va15_visual::Packet> delayed;
        const double initial=rl.d->time;std::uint64_t seq=0,last_supported_ns=0;
        bool started=false,fell=false,prone=false,did_track=false,unexpected_stop=false;
        bool yaw_impulse=false,recovery_wait=false,recovery_stood=false;
        double max_tilt=0.,max_target_step=0.,target_y_max=0.,distance=0.,tcp_error_sq=0.,start_x=0.;
        double first_tracking=-1.,steady_error_sq=0.,max_body_lateral=0.;unsigned steady_samples=0;
        double late_body_error_sq=0.;unsigned late_body_samples=0;
        double xyz_error_sq=0.,vertical_error_sq=0.,longitudinal_error_sq=0.,steady_xyz_error_sq=0.;
        unsigned tracked=0;const int tcp=mj_name2id(rl.m,mjOBJ_SITE,"ee_site");
        const int front_camera=mj_name2id(rl.m,mjOBJ_CAMERA,"front_depth_camera");
        MuJoCoLiveViewer viewer;
        if(live_viewer&&!viewer.Init(rl.m,tcp,rl.base,front_camera))throw std::runtime_error("could not create MuJoCo GLFW viewer; run from a graphical desktop");
        bool viewer_closed=false;
        unsigned camera_frames=0,camera_valid_frames=0;
        double max_projection=0.,projection_sum=0.,min_clearance=INFINITY,max_gun_force=0.,max_observed_projection=0.,min_contact_clearance=INFINITY;
        double first_precision=-1.;unsigned projected=0,over_limit=0,foot_crossings=0,gun_contacts=0,loaded_crossings=0;
        double acquisition=-1.,post_acquire_max=0.;unsigned in_tolerance=0,post_acquire_samples=0,post_acquire_violations=0;
        std::vector<double> projections;
        bool active_session=false,pending=false;va15_visual::Packet incoming;
        va15_tcp::V3 previous_position=va15_tcp::V3::Zero();bool have_previous=false;
        std::ofstream csv(argv[2]);csv<<"time,x,y,height,tilt,guide,stop,cache,active,target_x,target_y,target_z,tcp_z,tcp_lateral,course_slip,yaw_command,body_e,body_heading,body_history,rear_crossings,workspace_rejections,rear_gap,estimate_x,estimate_y,yaw,projection_error_m,min_truth_foot_clearance_m,observed_feet,observed_foot_clearance_m,gun_force_n,servo_error_m,policy_request_y,policy_request_yaw,minimum_contact_foot_clearance_m";
        for(int i=0;i<rl.m->nq;++i)csv<<",qpos_"<<i;csv<<'\n';
        for(int step=0;step<18400;++step) {
            rl.ReadState();const double t=rl.d->time-initial;
            // The private FSM explicitly consumes tagged estimate feedback.
            // Only this synthetic-pose physics harness adapts MuJoCo truth;
            // do NOT leave it branded as production SportMode feedback.
            rl.robot_state.real_stop_feedback.sport_fresh=false;
            rl.robot_state.real_stop_feedback.unavailable_reason=va15_trial::Source;
            const auto ns=std::uint64_t((100.+t)*1.e9);
            va15_tcp::Pose pose{ns,va15_tcp::V3(rl.d->qpos[0],rl.d->qpos[1],rl.d->qpos[2]),
                Eigen::Quaterniond(rl.d->qpos[3],rl.d->qpos[4],rl.d->qpos[5],rl.d->qpos[6]).normalized(),true};
            const auto render_pose=pose;
            if(estimated) {
                if(step%2==0) {
                    va15_independent::Sample sample;sample.mode.fill(10);sample.documented_motor_health=true;
                    sample.quaternion=pose.q;
                    for(int i=0;i<12;++i){sample.q[i]=rl.d->qpos[dds_q[i]];sample.dq[i]=rl.d->qvel[dds_v[i]];sample.tau[i]=rl.d->qfrc_actuator[dds_v[i]];}
                    for(int j=0;j<3;++j){sample.gyro[j]=rl.robot_state.imu.gyroscope[j];sample.specific_force[j]=rl.d->sensordata[rl.m->sensor_adr[imu]+j];}
                    observer.Update(sample,std::lround(rl.d->time*1000.),rl.d->time);
                }
                const auto estimate=observer.Snapshot(rl.d->time);
                rl.robot_state.real_stop_feedback=va15_trial::Feedback(estimate);
                if(va15_trial::Healthy(estimate)&&estimate.estimate_valid_for_review)last_supported_ns=ns;
                pose.p=estimate.position;pose.valid=va15_trial::Healthy(estimate)&&last_supported_ns&&ns-last_supported_ns<=250000000ULL;
                if(pose.valid) {
                    if(step%2==0)poses.Add(pose);
                    const auto velocity=(pose.q.conjugate()*estimate.velocity).eval();
                    guide.ObserveBodyVelocity(velocity.x(),velocity.y(),.005);
                }
            } else poses.Add(pose);
            if(!estimated&&have_previous) {
                const auto velocity=(pose.q.conjugate()*((pose.p-previous_position)/.005)).eval();
                guide.ObserveBodyVelocity(velocity.x(),velocity.y(),.005);
            }
            previous_position=pose.p;have_previous=true;
            if(step%30==0) {
                va15_visual::Packet packet;packet.sequence=++seq;
                packet.capture_ns=ns;packet.publish_ns=ns+100000000;
                bool vision_ok=false;
                if(camera_vision) {
                    std::vector<unsigned char> rgb;
                    ++camera_frames;
                    vision_ok=viewer.CaptureCamera(rl.m,rl.d,rgb,packet.sequence)&&
                        DetectRenderedTape(rgb,640,360,ns,ns+100000000,packet.sequence,packet);
                    if(vision_ok)++camera_valid_frames;
                } else {
                    // Legacy synthetic packet path retained for regression scenarios.
                    packet.valid=packet.session_matched=true;
                    const bool adaptive=scenario=="estimated_adaptive"&&int(t*2.)%2;
                    packet.v=972.;packet.look_v=adaptive?852.:820.8;
                    packet.u=Pixel(render_pose,packet.v);packet.look_u=Pixel(render_pose,packet.look_v);
                    packet.confidence=packet.look_confidence=.99;packet.width=30;packet.contrast=.8;packet.rms=1.;
                    for(int v=1044;v>=(adaptive?840:744);v-=6)packet.path.push_back({Pixel(render_pose,v),double(v)});
                    packet.path_count=packet.path.size();packet.path_span=adaptive?204.:300.;
                    vision_ok=true;
                }
                if(!vision_ok) {
                    packet.valid=false;packet.session_matched=true;packet.v=972.;packet.look_v=820.8;
                    packet.path.clear();packet.path_count=0;packet.path_span=0.;
                    packet.confidence=packet.look_confidence=0.;
                }
                if(camera_vision&&packet.sequence>=38&&packet.sequence<=65)
                    std::cerr<<"camera packet "<<packet.sequence<<" valid_contract="<<va15_visual::Valid(packet,packet.publish_ns)
                        <<" u/v="<<packet.u<<'/'<<packet.v<<" look="<<packet.look_u<<'/'<<packet.look_v
                        <<" width="<<packet.width<<" confidence="<<packet.confidence<<" contrast="<<packet.contrast
                        <<" rms="<<packet.rms<<" count/span="<<packet.path_count<<'/'<<packet.path_span<<'\n';
                if(camera_vision&&!vision_ok&&packet.sequence<=140)
                    std::cerr<<"camera detection failed seq="<<packet.sequence<<" t="<<t<<'\n';
                if(scenario.find("dropout")!=std::string::npos&&t>=18.&&t<18.15)packet.valid=false;
                // Inject loss after the long front-to-rear warmup, so the
                // regression exercises an actually active TCP, not a forced
                // early target. No runtime timeout is added to production.
                if(scenario=="estimated_loss"&&t>=55.&&t<57.)packet.valid=false;
                delayed.push_back(packet);
            }
            while(!delayed.empty()&&delayed.front().publish_ns<=ns) {
                incoming=delayed.front();guide.Observe(incoming,ns);va15_tcp::Pose capture;
                if(va15_visual::Valid(guide.latest,ns)&&poses.At(guide.latest.capture_ns,capture))tracker.Observe(guide.latest,capture);
                delayed.pop_front();
            }
            guide.metric_bearing_valid=tracker.Guidance(pose,guide.metric_bearing);
            guide.body_reference_valid=tracker.BodyGuidance(pose,guide.body_lateral,guide.body_heading,guide.body_history_used);
            rl.control.current_keyboard=Input::Keyboard::None;rl.control.current_gamepad=Input::Gamepad::None;
            if(t>=6.&&!started) {
                started=true;start_x=rl.d->qpos[0];
                if(!guide.Start(ns,loco->RealStopSettled(),true,true))throw std::runtime_error(guide.reason);
                tracker.Begin(pose);active_session=true;body.Reset();
                const int gun=mj_name2id(rl.m,mjOBJ_GEOM,"welding_gun");
                rl.m->geom_contype[gun]=2;rl.m->geom_conaffinity[gun]=1;
            }
            if(t>=80.)guide.Stop("explicit X stop");
            if(scenario=="estimated_pose_loss"&&t>=55.&&t<55.4)pose.valid=false;
            if(started&&!pose.valid)guide.Stop("local estimate invalid");
            if(camera_vision&&active_session&&!va15_visual::Valid(incoming,ns))
                std::cerr<<"active packet invalid at t="<<t<<" seq="<<incoming.sequence
                    <<" capture="<<incoming.capture_ns<<" publish="<<incoming.publish_ns
                    <<" age_ms="<<(ns>=incoming.capture_ns?(ns-incoming.capture_ns)/1000000ULL:0)
                    <<" width="<<incoming.width<<" count/span="<<incoming.path_count<<'/'<<incoming.path_span
                    <<" valid="<<incoming.valid<<" matched="<<incoming.session_matched<<'\n';
            auto output=va15_tcp_priority::Tick(guide,incoming,ns,.005,true,false,true,pose.valid,true,active_session,pending);
            if(output.stop)loco->trial.Stop(guide.reason);
            if(started&&t<80.&&output.stop)unexpected_stop=true;
            // Match native ordering: guide before FSM, autonomy AFTER FSM.
            // The previous autonomous request survives into the FSM except
            // when the visual session just stopped.
            if(output.stop)rl.control.x=rl.control.y=rl.control.yaw=0.;
            const auto before=tracker.target;
            tracker.Step(pose,.005,guide.stage==va15_visual::Guide::Stage::Running);
            rl.SetRuntimeTargetCommands({float(tracker.target.x()),float(tracker.target.y()),float(tracker.target.z()),0.,-.707107,0.,.707107});
            max_target_step=std::max(max_target_step,(tracker.target-before).cwiseAbs().maxCoeff());
            target_y_max=std::max(target_y_max,std::fabs(tracker.target.y()));
            if(tracker.active) {
                did_track=true;++tracked;
                const double error=rl.d->site_xpos[3*tcp+1]-Path(rl.d->site_xpos[3*tcp]);tcp_error_sq+=error*error;
                const va15_tcp::V3 actual(rl.d->site_xpos[3*tcp],rl.d->site_xpos[3*tcp+1],rl.d->site_xpos[3*tcp+2]);
                const va15_tcp::V3 desired=render_pose.p+render_pose.q*tracker.target;
                const va15_tcp::V3 error_body=render_pose.q.conjugate()*(actual-desired);
                xyz_error_sq+=(actual-desired).squaredNorm();vertical_error_sq+=std::pow(actual.z()-desired.z(),2);
                longitudinal_error_sq+=error_body.x()*error_body.x();
                if(first_tracking<0.)first_tracking=t;
                if(t-first_tracking>=8.){steady_error_sq+=error*error;steady_xyz_error_sq+=(actual-desired).squaredNorm();++steady_samples;}
            }
            
            rl.StateController(&rl.robot_state,&rl.robot_command);
            if(scenario=="estimated_yaw_recovery"&&loco->controlled_stop.captured&&!yaw_impulse) {
                // OFFLINE physical fault injection ONLY: angular impulse,
                // not a pose reset, fake sensor packet or real-robot command.
                rl.d->qvel[5]+=.72;yaw_impulse=true;
            }
            recovery_wait=recovery_wait||loco->controlled_stop.stage==VA15ControlledStop::Stage::AwaitQuiet;
            recovery_stood=recovery_stood||(recovery_wait&&loco->controlled_stop.Ready());
            if(active_session&&(loco->trial.stage==va15_trial::Supervisor::Stage::Armed||loco->trial.Forward())) {
                rl.control.x=std::min(.15,output.x);rl.control.y=0.;rl.control.yaw=mode=="fixed"?0.:output.yaw;
                if(mode=="body"||mode=="ik"||mode=="planar") {
                    const auto request=body.Step(pose,tracker,.005);
                    if(!request.valid)throw std::runtime_error("body path reference invalid");
                    rl.control.x=request.x;rl.control.y=request.y;rl.control.yaw=request.yaw;
                }
            } else rl.control.x=rl.control.y=rl.control.yaw=0.;
            if(mode=="planar"&&!planar.Update(pose,tracker,rl.robot_state.motor_state.q,
                rl.robot_command.motor_command.q,.005,active_session&&loco->trial.Forward()))
                throw std::runtime_error("planar TCP control invalid");
            va15_tcp::V3 exact_goal;
            if(mode=="ik"&&active_session&&loco->trial.Forward()&&tracker.RearPoint(pose,-.775,.09,exact_goal)) {
                auto original=rl.robot_command.motor_command.q;
                if(!servo.Update(rl.robot_state.motor_state.q,original,exact_goal,.005,rl.robot_command.motor_command.q))
                    throw std::runtime_error("precision solver invalid");
            } else if(mode=="ik"&&servo.engaged)servo.Hold(rl.robot_command.motor_command.q);
            // OFFLINE ONLY: characterize the actor's lateral response while
            // it is already walking. No real launcher enables these probes.
            if(loco->trial.Forward()) {
                if(scenario=="lateral_02")rl.control.y=.02;
                if(scenario=="lateral_04")rl.control.y=.04;
                if(scenario=="lateral_06")rl.control.y=.06;
                if(scenario=="estimated_lateral_02")rl.control.y=.02;
                if(scenario=="estimated_lateral_04")rl.control.y=.04;
                if(scenario=="estimated_lateral_06")rl.control.y=.06;
                if(scenario=="estimated_probe_p15"){rl.control.y=.15;rl.control.yaw=0.;}
                if(scenario=="estimated_probe_n15"){rl.control.y=-.15;rl.control.yaw=0.;}
                if(scenario=="estimated_probe_n06"){rl.control.y=-.06;rl.control.yaw=0.;}
            }
            if(step%4==0)rl.Infer();
            max_tilt=std::max(max_tilt,double(rl.Tilt()));
            if(t<80.){distance=rl.d->qpos[0]-start_x;max_body_lateral=std::max(max_body_lateral,std::fabs(rl.d->qpos[1]-Path(rl.d->qpos[0])));}
            if(t>=70.&&t<80.&&!output.stop){late_body_error_sq+=std::pow(rl.d->qpos[1]-Path(rl.d->qpos[0]),2);++late_body_samples;}
            double clearance=INFINITY,contact_clearance=INFINITY;
            std::array<bool,4> touching{};
            for(int contact=0;contact<rl.d->ncon;++contact) {
                const auto& c=rl.d->contact[contact];
                if(rl.m->geom_bodyid[c.geom1]!=0&&rl.m->geom_bodyid[c.geom2]!=0)continue;
                double force[6]{};mj_contactForce(rl.m,rl.d,contact,force);
                for(int f=0;f<4;++f)if((c.geom1==rl.feet[f]||c.geom2==rl.feet[f])&&force[0]>1.)touching[f]=true;
            }
            int foot_index=0;
            for(int foot:rl.feet) {
                const double distance=ProjectionDistance(rl.d->geom_xpos[3*foot],rl.d->geom_xpos[3*foot+1]);
                const double gap=distance-rl.m->geom_size[3*foot]-.015;
                clearance=std::min(clearance,gap);
                if(touching[foot_index])contact_clearance=std::min(contact_clearance,gap);++foot_index;
            }
            const auto observed_feet=va15_foot_seam::Check(foot_model,rl.robot_state.motor_state.q,pose,tracker);
            double gun_force=0.;const int gun=mj_name2id(rl.m,mjOBJ_GEOM,"welding_gun");
            for(int contact=0;contact<rl.d->ncon;++contact) {
                const auto& c=rl.d->contact[contact];if(c.geom1!=gun&&c.geom2!=gun)continue;
                double force[6]{};mj_contactForce(rl.m,rl.d,contact,force);gun_force=std::max(gun_force,force[0]);
            }
            const double projection=ProjectionDistance(rl.d->site_xpos[3*tcp],rl.d->site_xpos[3*tcp+1]);
            if(t>=6.) {min_clearance=std::min(min_clearance,clearance);min_contact_clearance=std::min(min_contact_clearance,contact_clearance);loaded_crossings+=contact_clearance<=0.;foot_crossings+=clearance<=0.;max_gun_force=std::max(max_gun_force,gun_force);gun_contacts+=gun_force>5.;}
            if(mode=="planar"&&planar.observed&&active_session||tracker.active&&active_session) {
                if(first_precision<0.)first_precision=t;
                ++projected;projections.push_back(projection);projection_sum+=projection*projection;
                max_projection=std::max(max_projection,projection);over_limit+=projection>.05;
                // Operational acquisition is based on the SAME local visual
                // estimate as native, never selected by this truth scorer.
                // The native body controller has no arm acquisition flag.
                // Treat seven seconds after the first rear TCP observation as
                // its stable-entry window; this excludes the measured
                // front-to-rear handoff transient without hiding later error.
                const bool ready=mode=="planar"?planar.tracking:
                    mode=="ik"?servo.tracking:
                    mode=="body"&&first_precision>=0.&&t-first_precision>=7.;
                if(acquisition<0.&&ready)acquisition=t;
                if(acquisition>=0.) {++post_acquire_samples;post_acquire_max=std::max(post_acquire_max,projection);post_acquire_violations+=projection>.05;}
            }
            if(step%10==0) {csv<<t<<','<<rl.d->qpos[0]<<','<<rl.d->qpos[1]<<','<<rl.d->qpos[2]<<','<<rl.Tilt()
                <<','<<int(guide.stage)<<','<<int(loco->controlled_stop.stage)<<','<<tracker.curves.size()<<','<<tracker.active
                <<','<<tracker.target.x()<<','<<tracker.target.y()<<','<<tracker.target.z()<<','<<rl.d->site_xpos[3*tcp+2]
                <<','<<rl.d->site_xpos[3*tcp+1]-Path(rl.d->site_xpos[3*tcp])
                <<','<<guide.course_slip<<','<<output.yaw<<','<<guide.body_lateral<<','<<guide.body_heading<<','<<guide.body_history_used
                <<','<<tracker.rear_crossings<<','<<tracker.workspace_rejections<<','<<tracker.nearest_rear_gap
                <<','<<pose.p.x()<<','<<pose.p.y()<<','<<std::atan2(2.*(pose.q.w()*pose.q.z()+pose.q.x()*pose.q.y()),1.-2.*(pose.q.y()*pose.q.y()+pose.q.z()*pose.q.z()))<<','<<projection<<','<<clearance<<','<<observed_feet.covered<<','<<observed_feet.minimum<<','<<gun_force<<','<<servo.error.norm()<<','<<rl.control.y<<','<<rl.control.yaw<<','<<contact_clearance;
            for(int i=0;i<rl.m->nq;++i)csv<<','<<rl.d->qpos[i];csv<<'\n';}
            if(!std::isfinite(rl.Tilt())||rl.Tilt()>.5){fell=true;break;}
            if(rl.fsm.current_state_->GetStateName()=="RLFSMStatePassive"&&rl.d->qpos[2]<.2){prone=true;break;}
            if(live_viewer&&step%4==0) {
                if(!viewer.Render(rl.m,rl.d)){viewer_closed=true;break;}
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            rl.Integrate();
        }
        std::sort(projections.begin(),projections.end());
        const double p95=projections.empty()?-1.:projections[std::min(projections.size()-1,std::size_t(.95*projections.size()))];
        const bool stopped=loco->controlled_stop.Ready();
        const bool pass=started&&tracked>=1000&&!fell&&!unexpected_stop&&distance>2.&&
            acquisition>=0.&&post_acquire_samples>4000&&post_acquire_max<=.05&&min_contact_clearance>0.&&max_gun_force<5.&&stopped;
        std::ofstream report(std::string(argv[2])+".json");report<<std::boolalpha<<std::setprecision(10)
            <<"{\"pass\":"<<pass<<",\"scenario\":\""<<scenario<<"\",\"mode\":\""<<mode
            <<"\",\"friction\":"<<mu<<",\"fell\":"<<fell<<",\"stop_ready\":"<<stopped
            <<",\"unexpected_stop\":"<<unexpected_stop<<",\"forward_m\":"<<distance
            <<",\"max_tilt_rad\":"<<max_tilt<<",\"tracking_samples\":"<<projected<<",\"first_rear_tracking_s\":"<<first_precision
            <<",\"tcp_ground_projection_max_m\":"<<max_projection<<",\"tcp_ground_projection_p95_m\":"<<p95
            <<",\"tcp_ground_projection_rms_m\":"<<std::sqrt(projection_sum/std::max(1u,projected))
            <<",\"tcp_over_50mm_seconds\":"<<over_limit*.005
            <<",\"acquisition_s\":"<<acquisition<<",\"post_acquisition_max_error_m\":"<<post_acquire_max
            <<",\"post_acquisition_seconds\":"<<post_acquire_samples*.005<<",\"post_acquisition_over_50mm_seconds\":"<<post_acquire_violations*.005
            <<",\"minimum_foot_edge_clearance_m\":"<<min_clearance<<",\"foot_crossing_seconds\":"<<foot_crossings*.005
            <<",\"minimum_loaded_foot_edge_clearance_m\":"<<min_contact_clearance<<",\"loaded_foot_crossing_seconds\":"<<loaded_crossings*.005
            <<",\"gun_collision_enabled_during_session\":true,\"maximum_gun_contact_force_n\":"<<max_gun_force
            <<",\"gun_contact_over_5n_seconds\":"<<gun_contacts*.005
            <<",\"max_body_lateral_m\":"<<max_body_lateral
            <<",\"metric\":\"shortest XY distance from physical TCP to ground-truth centerline; 15mm seam half-width for feet\""
            <<",\"synthetic_rgb\":"<<(!camera_vision)<<",\"camera_vision\":"<<camera_vision
            <<",\"camera_frames\":"<<camera_frames<<",\"camera_valid_frames\":"<<camera_valid_frames
            <<",\"independent_estimated_pose\":"<<estimated
            <<",\"live_viewer\":"<<live_viewer<<",\"viewer_closed\":"<<viewer_closed
            <<",\"hardware_qualified\":false,\"stop_reason\":\""<<guide.reason<<"\"}\n";
        return pass?0:2;
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
