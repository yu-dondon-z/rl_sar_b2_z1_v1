// Compile the actual real entrypoint against the candidate FSM, but expose
// ONLY --offline-check in this test executable. No network/motion entry path.
#include "fsm_b2_z1_no_gun.hpp"
#define main va15_candidate_hidden_main
#include "../va15_stop_feedback_20261020/work/rl_real_b2_z1_no_gun.cpp"
#undef main
int main(int argc,char** argv)
{
    if(argc!=2 || std::string(argv[1])!="--offline-check")
    {std::cerr<<"Offline candidate test accepts ONLY --offline-check; no hardware path exposed\n";return 1;}
    return va15_candidate_hidden_main(argc,argv);
}
