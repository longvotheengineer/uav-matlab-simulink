#pragma once

#include "System/system_io.h"

typedef struct {
    float kp;               // proportional gain
    float ki;               // integral gain
    float kd;               // derivative gain
    float trim;             // baseline operating point (trim condition)
    
    float integrator;       // accumulated error
    float prev_error;       // previous error (for derivative)
    
    float limit_out_max;    // max output
    float limit_out_min;    // min output
    
    float limit_int_max;    // max anti-windup
    float limit_int_min;    // min anti-windup
    
    float Ts;               // sampling time
} pid_t;

// autopilot controllers (scl controller)
typedef struct {
    // lateral loops (roll & yaw)
    pid_t roll_from_aileron;
    pid_t course_from_roll;
    
    // longitudinal loops (pitch & altitude)
    pid_t pitch_from_elevator;
    pid_t alt_from_pitch;
    
    // airspeed loop
    pid_t airspeed_from_throttle;

    // directional loop (yaw damper)
    pid_t sideslip_from_rudder;
} slc_t;

extern slc_t autopilot;

void slc_controller_init(void);
void slc_controller_update(const atpl_i_t *input, ctrl_cmd_t *output);