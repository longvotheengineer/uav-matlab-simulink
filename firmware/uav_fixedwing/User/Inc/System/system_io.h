#pragma once

#include <stdint.h>

#pragma pack(push, 1)

// 12 state variables
typedef struct {
    uint8_t sync_1;         // expect 0xAA
    uint8_t sync_2;         // expect 0xFF
    float pn, pe, pd;       // position in inertial frame (m)
    float u, v, w;          // velocity in body frame (m/s)
    float phi, theta, psi;  // Euler angles in vehicle2,1,0 frame (rad)
    float p, q, r;          // angular rates in body frame (rad/s)
} state_var_t;

// 4 control commands
typedef struct {
    uint8_t sync_1;         // expect 0xAA
    uint8_t sync_2;         // expect 0xFF
    float delta_a;          // aileron (rad)
    float delta_e;          // elevator (rad)
    float delta_r;          // rudder (rad)
    float delta_t;          // throttle (0-1)
} ctrl_cmd_t;

// typedef struct {
//     uint8_t sync_1;         // expect 0xAA
//     uint8_t sync_2;         // expect 0xFF
//     float n_carrot;         // desired north position (m)
//     float e_carrot;         // desired east position (m)
//     float alt_carrot;       // desired altitude position (m)
//     int traj_type;          // trajectory type (1: figure-8, 2: helix)
// } path_cmd_t;

#pragma pack(pop)

extern state_var_t state_var_x;
extern ctrl_cmd_t ctrl_cmd_delta;