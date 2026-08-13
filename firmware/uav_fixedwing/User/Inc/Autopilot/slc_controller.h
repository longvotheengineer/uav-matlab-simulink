#pragma once

#include "System/system_io.h"

typedef struct {
    float kp;               // proportional gain
    float ki;               // integral gain
    float kd;               // derivative gain
    float trim;             // Baseline operating point (Trim Condition)
    
    float integrator;       // accumulated error
    float prev_error;       // previous error (for derivative)
    
    float limit_out_max;    // Maximum physical output (e.g., max servo angle)
    float limit_out_min;    // Minimum physical output
    
    float limit_int_max;    // Anti-windup max
    float limit_int_min;    // Anti-windup min
    
    float Ts;               // Sample time (0.01s for 100 Hz)
} pid_t;

// 2. The Full Autopilot Controller Instance
typedef struct {
    // ---- Lateral Loops (Roll & Yaw) ----
    pid_t roll_from_aileron;
    pid_t course_from_roll;
    
    // ---- Longitudinal Loops (Pitch & Altitude) ----
    pid_t pitch_from_elevator;
    pid_t alt_from_pitch;
    
    // ---- Airspeed Loop ----
    pid_t airspeed_from_throttle;

    // ---- Directional Loop (Yaw Damper) ----
    pid_t sideslip_from_rudder;
} slc_t;

// 3. Expose the autopilot globally so we can initialize it
extern slc_t autopilot;

// 4. Pure Math Function Prototypes
void slc_controller_init(void);
void slc_controller_update(const atpl_i_t *input, ctrl_cmd_t *output);