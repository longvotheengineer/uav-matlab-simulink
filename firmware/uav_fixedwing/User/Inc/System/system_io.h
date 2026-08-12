#pragma once

#include <stdint.h>

#pragma pack(push, 1)

// state variables - from uav to visualization/guidance/autopilot/wind_model
typedef struct {
    uint8_t sync_1;         // expect 0xAA
    uint8_t sync_2;         // expect 0xFF
    float pn, pe, pd;       // position in inertial frame (m)
    float u, v, w;          // velocity in body frame (m/s)
    float phi, theta, psi;  // Euler angles in vehicle2,1,0 frame (rad)
    float p, q, r;          // angular rates in body frame (rad/s)
} state_var_t;

// guidance commands - from guidance to autopilot
typedef struct {
    float psi_cmd;          // desired heading/yaw (rad)
    float alt_cmd;          // desired altitude (m)
} gdnc_cmd_t;

// autopilot inputs - from uav/guidance to autopilot
typedef struct {
    state_var_t state_var_x;
    gdnc_cmd_t gdnc_cmd;
} atpl_i_t;

// control commands - from autopilot to uav
typedef struct {
    uint8_t sync_1;         // expect 0xAA
    uint8_t sync_2;         // expect 0xFF
    float delta_a;          // aileron (rad)
    float delta_e;          // elevator (rad)
    float delta_r;          // rudder (rad)
    float delta_t;          // throttle (0-1)
} ctrl_cmd_t;

// path visualization - from guidance to visualization
// typedef struct {
//     uint8_t sync_1;         // expect 0xAA
//     uint8_t sync_2;         // expect 0xFF
//     float n_carrot;         // desired north position (m)
//     float e_carrot;         // desired east position (m)
//     float alt_carrot;       // desired altitude position (m)
//     int traj_type;          // trajectory type (1: figure-8, 2: helix)
// } path_visual_t;

#pragma pack(pop)

extern ctrl_cmd_t ctrl_cmd_delta;

extern atpl_i_t atpl_i;