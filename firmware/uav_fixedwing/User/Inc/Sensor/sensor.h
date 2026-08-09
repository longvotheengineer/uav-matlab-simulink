#pragma once

#include <stdint.h>

#pragma pack(push, 1)

// 12 state variables - receive from Simulink
typedef struct {
    uint8_t sync_1;         // expect 0xAA
    uint8_t sync_2;         // expect 0xFF
    float pn, pe, pd;       // position in inertial frame (m)
    float u, v, w;          // velocity in body frame (m/s)
    float phi, theta, psi;  // Euler angles in vehicle2,1,0 frame (rad)
    float p, q, r;          // angular rates in body frame (rad/s)
} state_variable_t;

// 4 control inputs - send to Simulink
typedef struct {
    uint8_t sync_1;         // expect 0xAA
    uint8_t sync_2;         // expect 0xFF
    float delta_a;          // aileron (rad)
    float delta_e;          // elevator (rad)
    float delta_r;          // rudder (rad)
    float delta_t;          // throttle (0-1)
} control_cmd_t;

#pragma pack(pop)

extern state_variable_t state_var_x;
extern control_cmd_t ctrl_cmd_delta;