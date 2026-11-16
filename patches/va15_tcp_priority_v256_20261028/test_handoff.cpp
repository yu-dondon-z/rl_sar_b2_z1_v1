#include "production/va15_policy_handoff.hpp"
#include <cassert>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>

using Command=RobotCommand<float>;
using State=RobotState<float>;
float Effort(const State& s,const Command& c,int i)
{const auto& x=c.motor_command;return x.kp[i]*(x.q[i]-s.motor_state.q[i])+x.kd[i]*(x.dq[i]-s.motor_state.dq[i])+x.tau[i];}
Command Policy(const Command& hold)
{auto c=hold;for(int i:VA15LegIndices){c.motor_command.kp[i]=160.f;c.motor_command.kd[i]=5.f;c.motor_command.q[i]+=.12f;}return c;}
std::vector<float> low(18,-100.f),high(18,100.f),limits(18,200.f);
void Check(const State& s,const Command& old)
{
    VA15PolicyHandoff join;join.Begin(old);
    auto p=Policy(old),out=p;
    for(int k=0;k<8;++k){assert(join.Apply(s,out,false,.005,low,high,limits));assert(out.motor_command.q==old.motor_command.q);assert(out.motor_command.kp==old.motor_command.kp);assert(join.elapsed==0.);out=p;}
    assert(join.Apply(s,out,true,.005,low,high,limits));
    for(int i:VA15LegIndices){assert(out.motor_command.kp[i]==160.f&&out.motor_command.kd[i]==5.f);assert(std::fabs(Effort(s,out,i)-Effort(s,old,i))<.001f);}
    for(int k=0;k<301;++k){out=p;assert(join.Apply(s,out,k%4==0,.005,low,high,limits));}
    assert(!join.Active()&&!join.failed);
    assert(out.motor_command.q==p.motor_command.q&&out.motor_command.kp==p.motor_command.kp&&out.motor_command.kd==p.motor_command.kd);
    // Invalid input fails atomically; an old frame cannot re-arm the join.
    join.Begin(old);out=p;auto bad=s;bad.motor_state.dq[11]=NAN;
    assert(!join.Apply(bad,out,true,.005,low,high,limits));assert(out.motor_command.q==old.motor_command.q);
    out=p;assert(!join.Apply(s,out,true,.005,low,high,limits));assert(out.motor_command.q==old.motor_command.q);
    join.Begin(old);out=p;out.motor_command.q[0]=999.f;
    assert(join.Apply(s,out,true,.005,low,high,limits)); // first frame joins exactly to old
    out=p;out.motor_command.q[0]=1999.f;
    assert(!join.Apply(s,out,true,.005,low,high,limits));
}
std::vector<std::string> Split(const std::string& text)
{std::vector<std::string> v;std::istringstream s(text);std::string t;while(std::getline(s,t,','))v.push_back(t);return v;}
int main(int argc,char** argv)
{
    for(int i:VA15LegIndices){low[i]=i<4?-.6f:i<9?-.15f:-2.3f;high[i]=i<4?.6f:i<9?1.6f:-.45f;if(i>=9)limits[i]=320.f;}
    if(argc==2)
    {
        std::ifstream input(argv[1]);assert(input);std::string line;std::getline(input,line);
        auto names=Split(line);std::map<std::string,int> cols;for(int i=0;i<int(names.size());++i)cols[names[i]]=i;
        std::vector<std::string> previous;int starts=0;float maximum=0.f;
        const int routing[]={1,5,10,0,4,9,3,7,12,2,6,11}; // DDS -> policy
        while(std::getline(input,line))
        {
            auto row=Split(line);const auto value=[&](const std::vector<std::string>& r,const std::string& key){return std::stof(r.at(cols.at(key)));};
            if(!previous.empty()&&value(previous,"sent_kp_dds_2")>600&&value(row,"sent_kp_dds_2")<200&&value(row,"trial_stage")==3)
            {
                State s;Command old;
                for(auto* v:{&s.motor_state.q,&s.motor_state.dq,&old.motor_command.q,&old.motor_command.dq,&old.motor_command.kp,&old.motor_command.kd,&old.motor_command.tau})v->assign(18,0.f);
                for(int j=0;j<12;++j){int i=routing[j];auto suffix=std::to_string(j);s.motor_state.q[i]=value(row,"actual_q_dds_"+suffix);s.motor_state.dq[i]=value(row,"actual_dq_dds_"+suffix);old.motor_command.q[i]=value(previous,"sent_q_dds_"+suffix);old.motor_command.kp[i]=value(previous,"sent_kp_dds_"+suffix);old.motor_command.kd[i]=value(previous,"sent_kd_dds_"+suffix);}
                VA15PolicyHandoff join;join.Begin(old);auto out=Policy(old);assert(join.Apply(s,out,true,.005,low,high,limits));
                for(int i:VA15LegIndices)maximum=std::max(maximum,std::fabs(Effort(s,out,i)-Effort(s,old,i)));
                ++starts;
            }
            previous=std::move(row);
        }
        assert(starts==3&&maximum<.001f);std::cout<<"PASS recorded 3 hardware starts: max instantaneous PD effort difference="<<maximum<<" Nm; offline replay only\n";return 0;
    }
    State s;Command old;
    for(auto* v:{&s.motor_state.q,&s.motor_state.dq,&old.motor_command.q,&old.motor_command.dq,&old.motor_command.kp,&old.motor_command.kd,&old.motor_command.tau})v->assign(18,0.f);
    for(int i:VA15LegIndices){old.motor_command.q[i]=i<4?0.f:i<9?.8f:-1.5f;old.motor_command.kp[i]=i<9?450.f:675.f;old.motor_command.kd[i]=15.f;s.motor_state.q[i]=old.motor_command.q[i]-(i<4?.03f:.10f);s.motor_state.dq[i]=.025f;}
    Check(s,old);
    for(float error:{.04f,.08f,.102f,.16f}){for(int i:VA15LegIndices)s.motor_state.q[i]=old.motor_command.q[i]-error;Check(s,old);}
    std::cout<<"PASS delayed actor hold, loaded gain/reference continuity, full release, finite/envelope faults and no implicit re-arm\n";
}
