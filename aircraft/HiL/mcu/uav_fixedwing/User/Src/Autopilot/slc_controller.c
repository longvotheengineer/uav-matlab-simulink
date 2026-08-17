#include <math.h>

#include "Autopilot/slc_controller.h"

slc_t autopilot;

// wrap angle to [-pi, pi] (used for heading (psi) error only)
static float wrap_angle(float error_in) {
    float val = error_in + M_PI;
    float mod_val = val - (2.0f * M_PI) * floorf(val / (2.0f * M_PI));
    return mod_val - M_PI;
}

// upgraded PID to use physical angle rates (p, q, r)
static float calculate_pid(pid_t *pid, float setpoint, float measured, float rate) {
    float error = setpoint - measured;
    float p_term = pid->kp * error;
    
    pid->integrator += pid->ki * error * pid->Ts;
    // anti-windup
    if (pid->integrator > pid->limit_int_max) pid->integrator = pid->limit_int_max;
    if (pid->integrator < pid->limit_int_min) pid->integrator = pid->limit_int_min;
    float i_term = pid->integrator;
    
    // use the physical angle rate instead of (error - prev_error)/Ts
    float d_term = pid->kd * (-rate); 
    
    float output = p_term + i_term + d_term + pid->trim;
    
    // saturation
    if (output > pid->limit_out_max) output = pid->limit_out_max;
    if (output < pid->limit_out_min) output = pid->limit_out_min;
    
    return output;
}

void slc_controller_init(void) {
    // inner loop (roll from aileron)
    autopilot.roll_from_aileron.kp = 4.766f;
    autopilot.roll_from_aileron.ki = 0.0f;
    autopilot.roll_from_aileron.kd = 0.155f;
    autopilot.roll_from_aileron.trim = 0.0f;
    autopilot.roll_from_aileron.integrator = 0.0f;
    autopilot.roll_from_aileron.limit_out_max = 0.785f;
    autopilot.roll_from_aileron.limit_out_min = -0.785f;
    autopilot.roll_from_aileron.limit_int_max = 0.2f;
    autopilot.roll_from_aileron.limit_int_min = -0.2f;
    autopilot.roll_from_aileron.Ts = 0.01f;

    // outer loop (course from roll)
    autopilot.course_from_roll.kp = 1.08f;
    autopilot.course_from_roll.ki = 0.05f;
    autopilot.course_from_roll.kd = 0.0f;
    autopilot.course_from_roll.trim = 0.0f;
    autopilot.course_from_roll.integrator = 0.0f;
    autopilot.course_from_roll.limit_out_max = 0.523f;
    autopilot.course_from_roll.limit_out_min = -0.523f;
    autopilot.course_from_roll.limit_int_max = 0.1f;
    autopilot.course_from_roll.limit_int_min = -0.1f;
    autopilot.course_from_roll.Ts = 0.01f;

    // inner loop (pitch from elevator)
    autopilot.pitch_from_elevator.kp = -4.923f;
    autopilot.pitch_from_elevator.ki = 0.0f;
    autopilot.pitch_from_elevator.kd = -0.843f;
    autopilot.pitch_from_elevator.trim = -0.58621f;
    autopilot.pitch_from_elevator.integrator = 0.0f;
    autopilot.pitch_from_elevator.limit_out_max = 0.785f;
    autopilot.pitch_from_elevator.limit_out_min = -0.785f;
    autopilot.pitch_from_elevator.limit_int_max = 0.2f;
    autopilot.pitch_from_elevator.limit_int_min = -0.2f;
    autopilot.pitch_from_elevator.Ts = 0.01f;

    // outer loop (altitude from pitch)
    autopilot.alt_from_pitch.kp = 0.028f;
    autopilot.alt_from_pitch.ki = 0.006f;
    autopilot.alt_from_pitch.kd = 0.0f;
    autopilot.alt_from_pitch.trim = 0.14622f;
    autopilot.alt_from_pitch.integrator = 0.0f;
    autopilot.alt_from_pitch.limit_out_max = 0.523f;
    autopilot.alt_from_pitch.limit_out_min = -0.523f;
    autopilot.alt_from_pitch.limit_int_max = 0.2f;
    autopilot.alt_from_pitch.limit_int_min = -0.2f;
    autopilot.alt_from_pitch.Ts = 0.01f;

    // airspeed loop (throttle from velocity)
    autopilot.airspeed_from_throttle.kp = 1.1f;
    autopilot.airspeed_from_throttle.ki = 0.846f;
    autopilot.airspeed_from_throttle.kd = 0.0f;
    autopilot.airspeed_from_throttle.trim = 0.38792f;
    autopilot.airspeed_from_throttle.integrator = 0.0f;
    autopilot.airspeed_from_throttle.limit_out_max = 1.0f;
    autopilot.airspeed_from_throttle.limit_out_min = 0.0f;
    autopilot.airspeed_from_throttle.limit_int_max = 0.5f;
    autopilot.airspeed_from_throttle.limit_int_min = 0.0f;
    autopilot.airspeed_from_throttle.Ts = 0.01f;

    // directional loop (sideslip from rudder)
    autopilot.sideslip_from_rudder.kp = 0.15f;
    autopilot.sideslip_from_rudder.ki = 0.0f;
    autopilot.sideslip_from_rudder.kd = 0.0f;
    autopilot.sideslip_from_rudder.trim = 0.0f;
    autopilot.sideslip_from_rudder.integrator = 0.0f;
    autopilot.sideslip_from_rudder.limit_out_max = 0.523f;
    autopilot.sideslip_from_rudder.limit_out_min = -0.523f;
    autopilot.sideslip_from_rudder.limit_int_max = 0.1f;
    autopilot.sideslip_from_rudder.limit_int_min = -0.1f;
    autopilot.sideslip_from_rudder.Ts = 0.01f;
}

void slc_controller_update(const atpl_i_t *input, ctrl_cmd_t *output) {
    output->sync_1 = 0xAA;
    output->sync_2 = 0xFF;

    // lateral loop (rate = p)
    float psi_error = wrap_angle(input->gdnc_cmd.psi_cmd - input->state_var_x.psi);
    float target_roll = calculate_pid(&autopilot.course_from_roll, psi_error,
                                      0.0f, 0.0f);
    output->delta_a = calculate_pid(&autopilot.roll_from_aileron, target_roll,
                                    input->state_var_x.phi, input->state_var_x.p);

    // longitudinal loop (rate = q)
    float current_altitude = -input->state_var_x.pd;
    
    // feedforward from roll angle to compensate for lift loss in turns
    float phi_feedforward = 0.006f * fabsf(input->state_var_x.phi);
    
    float target_pitch = calculate_pid(&autopilot.alt_from_pitch, input->gdnc_cmd.alt_cmd, 
                                       current_altitude, 0.0f);
    target_pitch += phi_feedforward; // inject feedforward into target pitch
    
    // saturation
    if (target_pitch > autopilot.alt_from_pitch.limit_out_max) 
        target_pitch = autopilot.alt_from_pitch.limit_out_max;
    if (target_pitch < autopilot.alt_from_pitch.limit_out_min) 
        target_pitch = autopilot.alt_from_pitch.limit_out_min;

    output->delta_e = calculate_pid(&autopilot.pitch_from_elevator, target_pitch, 
                                    input->state_var_x.theta, input->state_var_x.q);

    // airspeed loop
    float target_airspeed = 15.0f; 
    output->delta_t = calculate_pid(&autopilot.airspeed_from_throttle, target_airspeed,
                                    input->state_var_x.u, 0.0f);

    // directional loop
    // hardcode rudder to 0.0 to match the constant block on Page 16 and prevent skidding
    output->delta_r = 0.0f;
}