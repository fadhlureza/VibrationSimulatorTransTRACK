#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "panel/panel.h"

enum sys_state_t {
    SYS_CALIBRATING = 0, // IMU boot calibration running (~4 s)
    SYS_STOPPED,         // Idle / stopped; active brake engaged
    SYS_HOMING,          // START pressed; auto-homing to 0.0 deg (flat)
    SYS_RUNNING,         // Homing complete; active mode running (Mode 0, 1, 2)
    SYS_FAULT            // Angle limit exceeded (>20 deg); active brake engaged
};

struct pitch_telemetry_t {
    sys_state_t  sys_state;
    bool         is_homed;
    float        current_pitch_deg;
    float        target_pitch_deg;
    float        slew_pitch_deg;
    int16_t      motor_pwm; // -1023 to +1023
    panel_mode_t active_mode;
    panel_dir_t  active_dir;
    float        dwell_remaining_s;
    float        dwell_total_s;
};

// Initialize pitch control task on Core 1 (100 Hz)
void pitch_control_init();

// Manual triggers (also handled automatically via panel START/STOP buttons)
void pitch_control_start();
void pitch_control_stop();

// State querying
sys_state_t pitch_control_get_state();
const char* pitch_control_get_state_str();
bool pitch_control_is_running();
bool pitch_control_is_homed();

// Thread-safe telemetry snapshot
pitch_telemetry_t pitch_control_get_telemetry();

// Pitch PID tuning
void pitch_control_update_pid(float kp, float ki, float kd);
void pitch_control_get_pid(float *kp, float *ki, float *kd);
