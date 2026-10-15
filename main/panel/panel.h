#pragma once

#include <stdint.h>
#include <stdbool.h>

enum panel_mode_t {
    MODE_STABILIZED = 0, // Mode 0: Holds 0 deg, rejects external tilt disturbances
    MODE_INTERVAL   = 1, // Mode 1: Automated 0 -> target -> 0 -> dwell cycle
    MODE_MANUAL     = 2  // Mode 2: Direct knob tracking
};

enum panel_dir_t {
    DIR_FORWARD = 0, // Right side up (+)
    DIR_REVERSE = 1  // Left side up (-)
};

struct panel_state_t {
    panel_mode_t mode;
    panel_dir_t  dir;

    // Processed control targets (mode-masked)
    float vib_target_g;    // 0..16 g (pot_0 in Mode 0 & 2; locked at 1.000 g in Mode 1)
    float pitch_angle_deg; // 1..15 deg (from pot_1)
    float interval_s;      // 0..300 s (from pot_2; 0.0 in Mode 0 & 2)

    // Raw potentiometer values for telemetry/dashboard
    int   raw_pot0_adc;
    int   raw_pot1_adc;
    int   raw_pot2_adc;
    float raw_pot0_g;
    float raw_pot1_deg;
    float raw_pot2_s;

    // Edge-triggered button event flags
    bool  start_pressed;
    bool  stop_pressed;
};

// Panel
void panel_init();
panel_state_t panel_get_state();
bool panel_check_and_clear_start();
bool panel_check_and_clear_stop();
