#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <Eigen/SVD>
#include <Eigen/LU>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

// Independent of SportModeState, policy, gait phase and raw force bridges.
// This implementation is an UNQUALIFIED REVIEW CANDIDATE. Force-inferred
// contacts and no-slip/flat-floor state estimates are not sensor truth.
// No runtime option turns review output into robot-control permission.
namespace va15_independent
{
using V3=Eigen::Vector3d;
using M3=Eigen::Matrix3d;
using State=Eigen::Matrix<double,18,1>; // p, v, four world foot centers
using Cov=Eigen::Matrix<double,18,18>;
constexpr double NaN=std::numeric_limits<double>::quiet_NaN();

struct Joint {V3 origin=V3::Zero(),rpy=V3::Zero(),axis=V3::UnitX();double lower=0,upper=0;};
struct Body {double mass=0;V3 com=V3::Zero();M3 inertia=M3::Zero();};
struct Leg {std::array<Joint,3> joints;std::array<Body,4> bodies;V3 tip=V3::Zero();double radius=0;};
struct Model {std::array<Leg,4> legs;V3 imu_position=V3::Zero();std::string source_sha256;};
struct Sample
{
    std::array<double,12> q{},dq{},tau{},reported_ddq{};
    std::array<unsigned,12> mode{},lost{};
    bool documented_motor_health=false; // private candidate only, raw lost is NOT rewritten
    Eigen::Quaterniond quaternion=Eigen::Quaterniond::Identity(); // W <- base
    V3 gyro=V3::Zero(),specific_force=V3::Zero(); // IMU point, base axes
};
struct Foot
{
    V3 position=V3::Zero(),velocity=V3::Zero(),dynamics_torque=V3::Zero();
    M3 jacobian=M3::Zero();
    V3 force_world=V3::Constant(NaN);
    double condition=NaN;
};
struct Result
{
    static constexpr bool control_authorized=false;
    static constexpr bool hardware_qualified=false;
    static constexpr bool common_mode_slip_observable=false;
    bool input_valid=false,sequence_valid=false,fresh=false,initialized=false;
    bool estimate_valid_for_review=false,stop_candidate_for_review=false;
    bool reported_ddq_used=false;
    std::array<bool,4> load_candidate{}; // DDS FR FL RR RL; NOT certified contacts
    std::array<double,4> normal_equivalent{{NaN,NaN,NaN,NaN}};
    // Explanation of the UNCHANGED load criterion, never extra authority.
    std::array<double,4> horizontal_equivalent{{NaN,NaN,NaN,NaN}};
    std::array<double,4> horizontal_limit{{NaN,NaN,NaN,NaN}};
    std::array<double,4> normal_lower{{NaN,NaN,NaN,NaN}};
    std::array<double,4> load_on_seconds{},load_off_seconds{};
    std::array<bool,4> friction_credible{};
    int support_count=0,accepted_feet=0;
    double base_height=NaN,planar_speed=NaN,height_spread=NaN,velocity_spread=NaN;
    V3 position=V3::Constant(NaN),velocity=V3::Constant(NaN);
    V3 gyro_bias=V3::Zero(),accel_bias=V3::Zero();
    double position_sigma=NaN,velocity_sigma=NaN;
    std::string reason="no samples";
};

inline M3 Rpy(const V3& q)
{return (Eigen::AngleAxisd(q.z(),V3::UnitZ())*Eigen::AngleAxisd(q.y(),V3::UnitY())*
         Eigen::AngleAxisd(q.x(),V3::UnitX())).toRotationMatrix();}

class Dynamics
{
public:
    explicit Dynamics(const Model& m):model(m)
    {
        if(model.source_sha256.size()!=64||!model.imu_position.allFinite())throw std::invalid_argument("Invalid model provenance/IMU");
        for(const auto& leg:model.legs)
        {
            if(!leg.tip.allFinite()||!std::isfinite(leg.radius)||leg.radius<=0||leg.radius>.1)
                throw std::invalid_argument("Invalid spherical foot");
            for(const auto& j:leg.joints)
                if(!j.origin.allFinite()||!j.rpy.allFinite()||!j.axis.allFinite()||
                   std::fabs(j.axis.norm()-1.)>1e-6||!std::isfinite(j.lower)||!std::isfinite(j.upper)||j.lower>=j.upper)
                    throw std::invalid_argument("Invalid leg joint");
            for(const auto& b:leg.bodies)
                if(!std::isfinite(b.mass)||b.mass<=0||!b.com.allFinite()||!b.inertia.allFinite()||
                   !b.inertia.isApprox(b.inertia.transpose(),1e-10)||b.inertia.llt().info()!=Eigen::Success)
                    throw std::invalid_argument("Invalid link inertia");
        }
    }
    Foot Evaluate(int index,const Sample& s,const std::array<double,12>& qdd,
                  const V3& angular_acceleration,const V3& gyro_bias=V3::Zero(),
                  const V3& accel_bias=V3::Zero())const
    {
        const auto& leg=model.legs.at(index);
        M3 rotation=M3::Identity();V3 p=V3::Zero();
        V3 omega=s.gyro-gyro_bias,alpha=angular_acceleration;
        // IMU specific force is at its displaced mounting point. Move it
        // back to base_link before propagating link accelerations.
        V3 a=s.specific_force-accel_bias-alpha.cross(model.imu_position)-
             omega.cross(omega.cross(model.imu_position));
        std::array<V3,3> origins,axes;
        std::array<V3,4> centers,forces,moments;
        for(int body=0;body<4;++body)
        {
            const V3 delta=rotation*(body<3?leg.joints[body].origin:leg.tip);
            a+=alpha.cross(delta)+omega.cross(omega.cross(delta));p+=delta;
            if(body<3)
            {
                const auto& j=leg.joints[body];const int k=3*index+body;
                if(!std::isfinite(s.q[k])||s.q[k]<j.lower-1e-6||s.q[k]>j.upper+1e-6)
                    throw std::invalid_argument("Joint outside URDF limits");
                rotation*=Rpy(j.rpy);origins[body]=p;axes[body]=rotation*j.axis;
                alpha+=axes[body]*qdd[k]+omega.cross(axes[body]*s.dq[k]);
                omega+=axes[body]*s.dq[k];
                rotation*=Eigen::AngleAxisd(s.q[k],j.axis).toRotationMatrix();
            }
            const auto& b=leg.bodies[body];const V3 offset=rotation*b.com;
            centers[body]=p+offset;
            forces[body]=b.mass*(a+alpha.cross(offset)+omega.cross(omega.cross(offset)));
            const M3 inertia=rotation*b.inertia*rotation.transpose();
            moments[body]=inertia*alpha+omega.cross(inertia*omega);
        }
        Foot out;out.position=p;
        for(int j=0;j<3;++j)
        {
            out.jacobian.col(j)=axes[j].cross(p-origins[j]);
            for(int b=j;b<4;++b)
                out.dynamics_torque[j]+=axes[j].dot((centers[b]-origins[j]).cross(forces[b])+moments[b]);
        }
        Eigen::JacobiSVD<M3> svd(out.jacobian,Eigen::ComputeFullU|Eigen::ComputeFullV);
        const auto singular=svd.singularValues();out.condition=singular[0]/singular[2];
        if(!std::isfinite(out.condition)||out.condition>30.)throw std::invalid_argument("Ill-conditioned foot Jacobian");
        V3 tau,dq;for(int j=0;j<3;++j){tau[j]=s.tau[3*index+j];dq[j]=s.dq[3*index+j];}
        const M3 world=s.quaternion.normalized().toRotationMatrix();
        out.force_world=world*out.jacobian.transpose().fullPivLu().solve(out.dynamics_torque-tau);
        out.velocity=-world*(out.jacobian*dq+(s.gyro-gyro_bias).cross(out.position));
        if(!out.force_world.allFinite()||!out.velocity.allFinite())throw std::invalid_argument("Nonfinite dynamics");
        return out;
    }
    const Model model;
};

class Observer
{
public:
    explicit Observer(const Model& model,bool require_motor_contract=false):
        dynamics(model),require_motor_contract_(require_motor_contract){Reset();}
    void Reset()
    {
        result={};have=false;initialized=false;previous_tick=0;previous_time=0;warmup=0;quiet=0;
        filtered_qdd.fill(0.);filtered_alpha.setZero();filtered_force.fill(0.);
        contact.fill(false);on_time.fill(0.);off_time.fill(0.);previous_contact.fill(false);
        x.setZero();cov=Cov::Identity();gyro_bias.setZero();accel_bias.setZero();
        sum_gyro.setZero();sum_accel_bias.setZero();bias_samples=0;
    }
    void Update(const Sample& s,std::uint32_t tick,double received)
    {
        if(!std::isfinite(received)){Reset();result.reason="invalid receive time";return;}
        if(have&&received<previous_time){Reset();result.reason="receive clock reversal";return;}
        if(have&&tick==previous_tick)return; // never refresh age with duplicates
        const auto delta=static_cast<std::uint32_t>(tick-previous_tick);
        const double wall_dt=have?received-previous_time:0.;
        const bool continuous=have&&delta>0&&delta<=50&&wall_dt>0&&wall_dt<=.1;
        if(!continuous)
        {
            Reset();have=true;previous=s;previous_tick=tick;previous_time=received;
            result.reason="sequence warmup/gap/reset";return;
        }
        const double dt=delta*.001; // B2 source tick is milliseconds; checked vs receive time
        if(std::fabs(dt-wall_dt)>.035)
        {Reset();result.reason="tick/receive-time disagreement";return;}
        result={};result.sequence_valid=true;
        try
        {
            ValidateSample(s);
            const double derivative_alpha=1.-std::exp(-dt/.025);
            for(int k=0;k<12;++k)
            {
                const double derivative=(s.dq[k]-previous.dq[k])/dt;
                if(!std::isfinite(derivative)||std::fabs(derivative)>1500.)throw std::invalid_argument("Encoder acceleration jump");
                filtered_qdd[k]+=derivative_alpha*(derivative-filtered_qdd[k]);
            }
            const V3 angular_derivative=(s.gyro-previous.gyro)/dt;
            if(angular_derivative.norm()>200.)throw std::invalid_argument("IMU angular acceleration jump");
            filtered_alpha+=derivative_alpha*(angular_derivative-filtered_alpha);
            std::array<Foot,4> feet;
            const M3 rotation=s.quaternion.normalized().toRotationMatrix();
            double max_dq=0;for(double v:s.dq)max_dq=std::max(max_dq,std::fabs(v));
            for(int i=0;i<4;++i)
            {
                feet[i]=dynamics.Evaluate(i,s,filtered_qdd,filtered_alpha,gyro_bias,accel_bias);
                const double fz=feet[i].force_world.z();
                filtered_force[i]+=(1.-std::exp(-dt/.015))*(fz-filtered_force[i]);
                // Heuristic model-error margin, NOT a proven confidence bound.
                const double lower=filtered_force[i]-std::max(30.,.25*std::fabs(filtered_force[i]));
                const bool credible=std::fabs(fz)<1500.&&feet[i].force_world.head<2>().norm()<
                    std::max(30.,.8*std::max(0.,fz));
                if(credible&&lower>80.){on_time[i]+=dt;off_time[i]=0.;}
                else if(!credible||lower<45.){off_time[i]+=dt;on_time[i]=0.;}
                else {on_time[i]=off_time[i]=0.;}
                if(!contact[i]&&on_time[i]>=.030)contact[i]=true;
                if(contact[i]&&off_time[i]>=.005)contact[i]=false;
                result.normal_equivalent[i]=fz;result.load_candidate[i]=contact[i];
                result.horizontal_equivalent[i]=feet[i].force_world.head<2>().norm();
                result.horizontal_limit[i]=std::max(30.,.8*std::max(0.,fz));
                result.normal_lower[i]=lower;result.friction_credible[i]=credible;
                result.load_on_seconds[i]=on_time[i];result.load_off_seconds[i]=off_time[i];
                result.support_count+=contact[i];
            }
            warmup+=dt;
            result.input_valid=true;
            std::array<int,4> selected{};int count=0;
            for(int i=0;i<4;++i)if(contact[i])selected[count++]=i;
            V3 velocity=V3::Zero();double height=0.;
            if(count)
            {
                double min_h=1.e9,max_h=-1.e9;
                for(int j=0;j<count;++j)
                {int i=selected[j];const double h=dynamics.model.legs[i].radius-(rotation*feet[i].position).z();
                 min_h=std::min(min_h,h);max_h=std::max(max_h,h);height+=h;velocity+=feet[i].velocity;}
                height/=count;velocity/=count;result.height_spread=max_h-min_h;result.velocity_spread=0.;
                for(int j=0;j<count;++j)result.velocity_spread=std::max(result.velocity_spread,(feet[selected[j]].velocity-velocity).norm());
            }
            const bool consistent=count>=2&&result.height_spread<.030&&result.velocity_spread<.150;
            // Four *load hypotheses* and a quiet start initialize the review
            // filter. Initialization is not contact/hardware qualification.
            const double orientation_change=2.*std::acos(std::clamp(std::fabs(s.quaternion.normalized().dot(previous.quaternion.normalized())),0.,1.));
            const bool static_start=consistent&&count==4&&height>.4&&height<.8&&max_dq<.05&&
                s.gyro.norm()<.05&&orientation_change/dt<.05&&
                (rotation*s.specific_force-V3(0,0,9.81)).norm()<1.;
            if(!initialized)
            {
                if(static_start)
                {quiet+=dt;sum_gyro+=s.gyro;sum_accel_bias+=s.specific_force-rotation.transpose()*V3(0,0,9.81);++bias_samples;}
                else {quiet=0.;sum_gyro.setZero();sum_accel_bias.setZero();bias_samples=0;}
                if(quiet>=.5&&warmup>=.5)
                {
                    gyro_bias=sum_gyro/bias_samples;accel_bias=sum_accel_bias/bias_samples;
                    x.setZero();x[2]=height;cov=Cov::Identity()*.01;
                    for(int i=0;i<4;++i)x.segment<3>(6+3*i)=x.head<3>()+rotation*feet[i].position;
                    initialized=true;
                }
            }
            else
            {
                const V3 corrected_gyro=s.gyro-gyro_bias;
                const V3 origin_specific=s.specific_force-accel_bias-filtered_alpha.cross(dynamics.model.imu_position)-
                    corrected_gyro.cross(corrected_gyro.cross(dynamics.model.imu_position));
                const V3 acceleration=rotation*origin_specific+V3(0,0,-9.81);
                if(acceleration.norm()>30.)throw std::invalid_argument("Excessive IMU acceleration");
                Cov a=Cov::Identity();a.block<3,3>(0,3)=M3::Identity()*dt;
                x.head<3>()+=x.segment<3>(3)*dt+.5*acceleration*dt*dt;
                x.segment<3>(3)+=acceleration*dt;
                cov=a*cov*a.transpose();
                // Discrete acceleration-sample uncertainty: G Q_a G^T,
                // G=[.5 dt^2 I; dt I]. Do not confuse variance/sample with
                // a continuous-time random-walk density multiplied by dt.
                cov.block<3,3>(0,0)+=M3::Identity()*(.25*dt*dt*dt*dt+1.e-7*dt);
                cov.block<3,3>(0,3)+=M3::Identity()*(.5*dt*dt*dt);
                cov.block<3,3>(3,0)+=M3::Identity()*(.5*dt*dt*dt);
                cov.block<3,3>(3,3)+=M3::Identity()*(dt*dt);
                for(int i=0;i<4;++i)
                {
                    cov.block<3,3>(6+3*i,6+3*i)+=M3::Identity()*(contact[i]?1.e-5:.3)*dt;
                    if(contact[i]&&!previous_contact[i])
                    {
                        x.segment<3>(6+3*i)=x.head<3>()+rotation*feet[i].position;
                        cov.block<3,18>(6+3*i,0).setZero();cov.block<18,3>(0,6+3*i).setZero();
                        cov.block<3,3>(6+3*i,6+3*i)=M3::Identity()*.01;
                    }
                }
                if(consistent)
                {
                    for(int j=0;j<count;++j)
                    {
                        const int i=selected[j];
                        if((feet[i].velocity-velocity).norm()>.12)continue;
                        Eigen::Matrix<double,3,18> h=Eigen::Matrix<double,3,18>::Zero();
                        h.block<3,3>(0,0)=M3::Identity();h.block<3,3>(0,6+3*i)=-M3::Identity();
                        if(!Correct(h,-rotation*feet[i].position,.008,16.3))continue;
                        h.setZero();h.block<3,3>(0,3)=M3::Identity();
                        if(!Correct(h,feet[i].velocity,.06,16.3))continue;
                        // Flat-ground assumption is explicit, not terrain truth.
                        Eigen::Matrix<double,1,18> hz=Eigen::Matrix<double,1,18>::Zero();hz[6+3*i+2]=1.;
                        CorrectScalar(hz,dynamics.model.legs[i].radius,.008);
                        ++result.accepted_feet;
                    }
                }
            }
            result.initialized=initialized;result.gyro_bias=gyro_bias;result.accel_bias=accel_bias;
            if(initialized)
            {
                result.position=x.head<3>();result.velocity=x.segment<3>(3);
                result.base_height=x[2];result.planar_speed=x.segment<2>(3).norm();
                result.position_sigma=std::sqrt(std::max(0.,cov(2,2)));
                result.velocity_sigma=std::sqrt(std::max(0.,cov.block<3,3>(3,3).diagonal().maxCoeff()));
                result.estimate_valid_for_review=consistent&&result.accepted_feet>=2&&
                    result.position_sigma<.05&&result.velocity_sigma<.15&&x.allFinite()&&cov.allFinite();
                const double tilt_sine=std::hypot(rotation(2,0),rotation(2,1));
                result.stop_candidate_for_review=result.estimate_valid_for_review&&count==4&&
                    result.base_height-2.*result.position_sigma>=.48&&
                    result.planar_speed+2.*result.velocity_sigma<.04&&tilt_sine<=.15&&
                    max_dq<.55&&correctedGyro(s).norm()<.18;
            }
            result.reason= !initialized ? "waiting for quiet four-load initialization" :
                (result.estimate_valid_for_review ? "independent flat/no-slip estimate; UNQUALIFIED" : "insufficient/inconsistent observations");
            previous_contact=contact;
        }
        catch(const std::exception& e)
        {const auto reason=std::string(e.what());Reset();result.reason=reason;}
        have=true;previous=s;previous_tick=tick;previous_time=received;
    }
    Result Snapshot(double now)const
    {
        auto r=result;r.fresh=have&&std::isfinite(now)&&now>=previous_time&&now-previous_time<=.1;
        if(!r.fresh)
        {
            r.estimate_valid_for_review=r.stop_candidate_for_review=false;r.load_candidate.fill(false);
            r.support_count=r.accepted_feet=0;r.base_height=r.planar_speed=NaN;
            r.position=r.velocity=V3::Constant(NaN);r.normal_equivalent.fill(NaN);
            r.reason="lowstate missing/stale";
        }
        return r;
    }
    const Dynamics dynamics;
private:
    V3 correctedGyro(const Sample& s)const{return s.gyro-gyro_bias;}
    void ValidateSample(const Sample& s)const
    {
        if(require_motor_contract_&&!s.documented_motor_health)
            throw std::invalid_argument("Documented current motor-health contract not ready");
        if(!s.gyro.allFinite()||!s.specific_force.allFinite()||!s.quaternion.coeffs().allFinite()||
           s.quaternion.squaredNorm()<.9||s.quaternion.squaredNorm()>1.1||s.gyro.norm()>10.||s.specific_force.norm()>80.)
            throw std::invalid_argument("Invalid IMU");
        for(int i=0;i<12;++i)
            if(!std::isfinite(s.q[i])||!std::isfinite(s.dq[i])||!std::isfinite(s.tau[i])||
               std::fabs(s.tau[i])>360.||std::fabs(s.dq[i])>30.||s.mode[i]!=10||
               (s.lost[i]!=0&&!(require_motor_contract_&&s.documented_motor_health)))
                throw std::invalid_argument("Invalid motor/transport state");
    }
    bool Correct(const Eigen::Matrix<double,3,18>& h,const V3& measurement,double sigma,double nis_limit)
    {
        const V3 residual=measurement-h*x;const M3 innovation=h*cov*h.transpose()+M3::Identity()*sigma*sigma;
        const auto factor=innovation.ldlt();if(factor.info()!=Eigen::Success)return false;
        if(residual.dot(factor.solve(residual))>nis_limit)return false;
        const Eigen::Matrix<double,18,3> gain=cov*h.transpose()*factor.solve(M3::Identity());
        x+=gain*residual;const Cov a=Cov::Identity()-gain*h;
        cov=a*cov*a.transpose()+gain*(M3::Identity()*sigma*sigma)*gain.transpose();
        cov=(.5*(cov+cov.transpose())).eval();return true;
    }
    void CorrectScalar(const Eigen::Matrix<double,1,18>& h,double measurement,double sigma)
    {
        const double innovation=(h*cov*h.transpose())(0,0)+sigma*sigma;
        const double residual=measurement-(h*x)(0,0);
        if(!std::isfinite(innovation)||innovation<=0||residual*residual/innovation>10.8)return;
        const Eigen::Matrix<double,18,1> gain=cov*h.transpose()/innovation;
        x+=gain*residual;const Cov a=Cov::Identity()-gain*h;
        cov=a*cov*a.transpose()+gain*(sigma*sigma)*gain.transpose();cov=(.5*(cov+cov.transpose())).eval();
    }
    const bool require_motor_contract_;
    Result result;Sample previous;bool have=false,initialized=false;
    std::uint32_t previous_tick=0;double previous_time=0,warmup=0,quiet=0;
    std::array<double,12> filtered_qdd{};V3 filtered_alpha=V3::Zero();
    std::array<double,4> filtered_force{},on_time{},off_time{};
    std::array<bool,4> contact{},previous_contact{};
    State x=State::Zero();Cov cov=Cov::Identity();
    V3 gyro_bias=V3::Zero(),accel_bias=V3::Zero(),sum_gyro=V3::Zero(),sum_accel_bias=V3::Zero();int bias_samples=0;
};
} // namespace va15_independent
