#pragma once
#include <cstdint>
extern "C" {
struct VD2State {
    double b2_time, z1_time, command_time;
    double q[18], dq[18], tau[18], quaternion[4], omega[3];
    std::uint32_t tick, b2_flags, motor_fault[12], motor_lost[12];
    std::int32_t z1_mode, z1_error[6], z1_connected[6], foot_force[4];
    std::uint32_t keys, fault;
    double z1_sent_time,z1_sent_source_time,z1_sent_q[6];
    std::uint64_t z1_sends,z1_suppressed,z1_replies;
};
struct VD2Command {
    double q[18], policy_blend;
    std::uint32_t mode; // 0: damping/passive, 1: position
    double leg_kp[18], leg_kd[18]; // actor order; arm entries must be zero
};
const char* vd2_error();
int vd2_open(const char* interface_name, const char* z1_ip, int to_port, int own_port);
int vd2_snapshot(VD2State* state);
int vd2_acquire(); // explicit takeover; never called by preview/preflight
int vd2_command(const VD2Command* command);
void vd2_close();
int vd2_front_open(const char* interface_name);
int vd2_front_image(unsigned char* bytes, int capacity, double* requested, double* received);
int vd2_offline_check(); // does not initialize a robot transport
unsigned long vd2_state_size();
unsigned long vd2_command_size();
}
