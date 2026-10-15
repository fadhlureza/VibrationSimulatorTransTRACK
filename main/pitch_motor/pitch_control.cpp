#include "pitch_control.h"
#include "pitch_motor.h"
#include "constant.h"
#include "imu/imu.h"
#include "panel/panel.h"
#include "module/fastpid/fastpid.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdio.h>

static TaskHandle_t s_pitch_task_handle = NULL;
static portMUX_TYPE s_pitch_mux = portMUX_INITIALIZER_UNLOCKED;

// State machine & tracking
static volatile sys_state_t s_sys_state = SYS_CALIBRATING;
static volatile bool        s_is_homed = false;
static float                s_target_pitch_deg = 0.0f;
static float                s_slew_pitch_deg = 0.0f;
static int16_t              s_motor_pwm = 0;
static int                  s_homed_stable_count = 0;

// Mode 1: Interval cycling sub-states
enum interval_substate_t {
    INTERVAL_TILT_OUT = 0,    // Slewing from 0.0° to target angle (1..15°)
    INTERVAL_TILT_RETURN = 1, // Returning from target back to 0.0° with no top pause
    INTERVAL_DWELL = 2        // Dwelling at 0.0° for pot_2 duration (0..300 s)
};

static interval_substate_t s_interval_substate = INTERVAL_TILT_OUT;
static TickType_t          s_dwell_start_tick = 0;
static float               s_dwell_duration_s = 0.0f;
static float               s_dwell_remaining_s = 0.0f;
static int                 s_peak_catchup_ticks = 0;
static int                 s_flat_catchup_ticks = 0;
static int                 s_still_standby_ticks = 0;

// PID Controller for Pitch Axis
static FastPID s_pitch_pid(PITCH_KP, PITCH_KI, PITCH_KD, PITCH_HZ, 16, true);
static float s_active_pitch_kp = PITCH_KP;
static float s_active_pitch_ki = PITCH_KI;
static float s_active_pitch_kd = PITCH_KD;
static bool s_pid_reconfigure_pending = false;

// Telemetry cache
static pitch_telemetry_t s_telemetry_cache = {
    .sys_state = SYS_CALIBRATING,
    .is_homed = false,
    .current_pitch_deg = 0.0f,
    .target_pitch_deg = 0.0f,
    .slew_pitch_deg = 0.0f,
    .motor_pwm = 0,
    .active_mode = MODE_STABILIZED,
    .active_dir = DIR_FORWARD,
    .dwell_remaining_s = 0.0f,
    .dwell_total_s = 0.0f
};

static void pitch_task_func(void *pvParameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period_ticks = pdMS_TO_TICKS(10); // 100 Hz (10 ms)
    const float dt = 0.01f;
    const float max_step = PITCH_SLEW_RATE_DPS * dt; // 0.05 deg/step at 5 deg/s

    while (1) {
        vTaskDelayUntil(&last_wake_time, period_ticks);

        // Apply any pending PID tuning updates
        if (s_pid_reconfigure_pending) {
            taskENTER_CRITICAL(&s_pitch_mux);
            s_pitch_pid.setCoefficients(s_active_pitch_kp, s_active_pitch_ki, s_active_pitch_kd, PITCH_HZ);
            s_pitch_pid.clear();
            s_pid_reconfigure_pending = false;
            taskEXIT_CRITICAL(&s_pitch_mux);
            printf("[PITCH] Updated PID: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", 
                   s_active_pitch_kp, s_active_pitch_ki, s_active_pitch_kd);
        }

        // Fetch inputs
        imu_snapshot_t imu_snap = imu_get_snapshot();
        panel_state_t panel_st = panel_get_state();
        float current_pitch = imu_snap.pitch_gyro_deg;

        // Check panel START button edge
        if (panel_check_and_clear_start()) {
            if (s_sys_state == SYS_STOPPED) {
                if (!imu_snap.is_calibrated) {
                    printf("[PITCH] START ignored: IMU boot calibration in progress\n");
                } else {
                    s_sys_state = SYS_HOMING;
                    s_is_homed = false;
                    s_slew_pitch_deg = current_pitch; // Start slewing from current position
                    s_target_pitch_deg = 0.0f;
                    s_homed_stable_count = 0;
                    s_interval_substate = INTERVAL_TILT_OUT;
                    s_dwell_remaining_s = 0.0f;
                    s_peak_catchup_ticks = 0;
                    s_flat_catchup_ticks = 0;
                    s_still_standby_ticks = 0;
                    printf("[PITCH] START pressed -> Entering SYS_HOMING (from %.2f deg to 0.0 deg)\n", current_pitch);
                }
            } else if (s_sys_state == SYS_CALIBRATING) {
                printf("[PITCH] START ignored: IMU is calibrating\n");
            }
        }

        // Check panel STOP button edge
        if (panel_check_and_clear_stop()) {
            if (s_sys_state != SYS_STOPPED && s_sys_state != SYS_CALIBRATING) {
                s_sys_state = SYS_STOPPED;
                s_is_homed = false;
                s_target_pitch_deg = current_pitch;
                s_slew_pitch_deg = current_pitch;
                s_motor_pwm = 0;
                s_interval_substate = INTERVAL_TILT_OUT;
                s_dwell_remaining_s = 0.0f;
                s_peak_catchup_ticks = 0;
                s_flat_catchup_ticks = 0;
                s_still_standby_ticks = 0;
                s_pitch_pid.clear();
                pitch_motor_brake();
                printf("[PITCH] STOP pressed -> Motors halted, active brake holding at %.2f deg\n", current_pitch);
            }
        }

        // Software safety limit check (+/- 20.0 deg)
        if (fabsf(current_pitch) > PITCH_MAX_ANGLE_FAULT && s_sys_state != SYS_FAULT && s_sys_state != SYS_CALIBRATING) {
            s_sys_state = SYS_FAULT;
            s_is_homed = false;
            s_motor_pwm = 0;
            s_pitch_pid.clear();
            pitch_motor_brake();
            printf("[PITCH ALARM] Angle limit exceeded (%.2f deg > %.1f deg)! Tripped SYS_FAULT.\n",
                   current_pitch, PITCH_MAX_ANGLE_FAULT);
        }

        // Handle mode transitions while running
        static panel_mode_t s_last_mode = MODE_STABILIZED;
        if (panel_st.mode != s_last_mode) {
            printf("[PITCH] Mode switch changed: Mode %d -> Mode %d\n", (int)s_last_mode, (int)panel_st.mode);
            s_pitch_pid.clear();
            s_interval_substate = INTERVAL_TILT_OUT;
            s_dwell_remaining_s = 0.0f;
            s_peak_catchup_ticks = 0;
            s_flat_catchup_ticks = 0;
            s_last_mode = panel_st.mode;
        }

        // State machine processing
        switch (s_sys_state) {
            case SYS_CALIBRATING:
                pitch_motor_brake();
                s_motor_pwm = 0;
                if (imu_snap.is_calibrated) {
                    s_sys_state = SYS_STOPPED;
                    s_slew_pitch_deg = current_pitch;
                    s_target_pitch_deg = 0.0f;
                    printf("[PITCH] IMU Calibrated -> System READY in SYS_STOPPED (initial pitch: %.2f deg)\n", current_pitch);
                }
                break;

            case SYS_STOPPED:
                pitch_motor_brake();
                s_motor_pwm = 0;
                s_is_homed = false;

                // Standstill drift re-zeroing: gently aligns gyro pitch to accelerometer gravity
                // when table is motionless (rate < 0.2 dps, vibration < 0.04 g for >= 500 ms)
                if (imu_snap.is_calibrated && 
                    fabsf(imu_snap.pitch_rate_dps) < 0.2f && 
                    imu_snap.vibration_g < 0.04f) {
                    s_still_standby_ticks++;
                    if (s_still_standby_ticks >= 50) { // Still for >= 500 ms
                        imu_standstill_realign(0.02f); // 2% blend per 10 ms cycle
                        s_slew_pitch_deg = imu_snap.pitch_gyro_deg;
                        s_target_pitch_deg = imu_snap.pitch_gyro_deg;
                    }
                } else {
                    s_still_standby_ticks = 0;
                }
                break;

            case SYS_HOMING: {
                s_target_pitch_deg = 0.0f;

                // Slew setpoint toward 0.0 deg at 5 deg/s
                if (s_slew_pitch_deg < -max_step) {
                    s_slew_pitch_deg += max_step;
                } else if (s_slew_pitch_deg > max_step) {
                    s_slew_pitch_deg -= max_step;
                } else {
                    s_slew_pitch_deg = 0.0f;
                }

                int16_t sp_centi = (int16_t)(s_slew_pitch_deg * 100.0f);
                int16_t fb_centi = (int16_t)(current_pitch * 100.0f);
                int16_t pwm = s_pitch_pid.step(sp_centi, fb_centi);

                // Deadband and static friction threshold
                if (abs(sp_centi - fb_centi) <= 15) {
                    pwm = 0; // within 0.15 deg deadband
                } else if (pwm > 0 && pwm < PITCH_PWM_MIN_DRIVE) {
                    pwm = PITCH_PWM_MIN_DRIVE;
                } else if (pwm < 0 && pwm > -PITCH_PWM_MIN_DRIVE) {
                    pwm = -PITCH_PWM_MIN_DRIVE;
                }

                s_motor_pwm = pwm;
                pitch_motor_drive(pwm);

                // Check homing convergence: flat within 0.5 deg and slew setpoint reached 0.0
                if (fabsf(current_pitch) < PITCH_HOMED_TOLERANCE && fabsf(s_slew_pitch_deg) < 0.1f) {
                    s_homed_stable_count++;
                    if (s_homed_stable_count >= 10) { // Stable for 100 ms
                        s_sys_state = SYS_RUNNING;
                        s_is_homed = true;
                        printf("[PITCH] Table reached 0.0 deg (flat) -> Auto-homing complete! Starting RUNNING state.\n");
                    }
                } else {
                    s_homed_stable_count = 0;
                }
                break;
            }

            case SYS_RUNNING: {
                float dir_sign = (panel_st.dir == DIR_FORWARD) ? 1.0f : -1.0f;
                float configured_target = dir_sign * panel_st.pitch_angle_deg;

                switch (panel_st.mode) {
                    case MODE_STABILIZED: {
                        // Mode 0: Maintain steady 0.0 deg and actively counter external tilts (CT-3.1)
                        s_target_pitch_deg = 0.0f;
                        s_dwell_remaining_s = 0.0f;
                        s_dwell_duration_s = 0.0f;

                        // Slew setpoint toward 0.0 deg at 5 deg/s (ensures smooth return if user switched from Mode 1 or 2 while tilted)
                        if (s_slew_pitch_deg < -max_step) {
                            s_slew_pitch_deg += max_step;
                        } else if (s_slew_pitch_deg > max_step) {
                            s_slew_pitch_deg -= max_step;
                        } else {
                            s_slew_pitch_deg = 0.0f;
                        }
                        break;
                    }

                    case MODE_INTERVAL: {
                        // Mode 1: Automated 0° -> target (1..15°) -> 0° -> dwell (0..300s) cycle (CT-3.2, CT-2.1, CT-2.2, CT-2.3)
                        s_dwell_duration_s = panel_st.interval_s;

                        switch (s_interval_substate) {
                            case INTERVAL_TILT_OUT: {
                                s_target_pitch_deg = configured_target;
                                s_dwell_remaining_s = 0.0f;

                                // Slew setpoint toward configured target at 5 deg/s
                                if (s_slew_pitch_deg < configured_target - max_step) {
                                    s_slew_pitch_deg += max_step;
                                } else if (s_slew_pitch_deg > configured_target + max_step) {
                                    s_slew_pitch_deg -= max_step;
                                } else {
                                    s_slew_pitch_deg = configured_target;
                                }

                                // Peak target reached: immediately return straight down with NO pause at top (CT-3.2)
                                if (fabsf(s_slew_pitch_deg - configured_target) < 0.05f) {
                                    if (fabsf(current_pitch - configured_target) <= 0.6f || ++s_peak_catchup_ticks >= 50) {
                                        s_peak_catchup_ticks = 0;
                                        s_interval_substate = INTERVAL_TILT_RETURN;
                                        printf("[PITCH] Mode 1: Peak target (%.2f deg) reached -> Immediately returning to 0.0 deg (no top pause)\n", 
                                               configured_target);
                                    }
                                } else {
                                    s_peak_catchup_ticks = 0;
                                }
                                break;
                            }

                            case INTERVAL_TILT_RETURN: {
                                s_target_pitch_deg = 0.0f;
                                s_dwell_remaining_s = 0.0f;

                                // Slew setpoint back to 0.0 deg at 5 deg/s
                                if (s_slew_pitch_deg < -max_step) {
                                    s_slew_pitch_deg += max_step;
                                } else if (s_slew_pitch_deg > max_step) {
                                    s_slew_pitch_deg -= max_step;
                                } else {
                                    s_slew_pitch_deg = 0.0f;
                                }

                                // Flat 0.0 deg reached
                                if (fabsf(s_slew_pitch_deg) < 0.05f) {
                                    if (fabsf(current_pitch) <= PITCH_HOMED_TOLERANCE || ++s_flat_catchup_ticks >= 50) {
                                        s_flat_catchup_ticks = 0;
                                        s_interval_substate = INTERVAL_DWELL;
                                        s_dwell_start_tick = xTaskGetTickCount();
                                        s_dwell_remaining_s = s_dwell_duration_s;
                                        printf("[PITCH] Mode 1: Returned to 0.0 deg -> Dwelling for %.1f s\n", s_dwell_duration_s);
                                    }
                                } else {
                                    s_flat_catchup_ticks = 0;
                                }
                                break;
                            }

                            case INTERVAL_DWELL: {
                                s_target_pitch_deg = 0.0f;
                                s_slew_pitch_deg = 0.0f;

                                if (s_dwell_duration_s <= 0.05f) {
                                    // 0 s dwell = continuous sweep without pause
                                    s_dwell_remaining_s = 0.0f;
                                    s_interval_substate = INTERVAL_TILT_OUT;
                                    printf("[PITCH] Mode 1: 0s dwell (continuous sweep) -> Starting next tilt cycle\n");
                                } else {
                                    float elapsed_s = (float)(xTaskGetTickCount() - s_dwell_start_tick) * (portTICK_PERIOD_MS / 1000.0f);
                                    s_dwell_remaining_s = s_dwell_duration_s - elapsed_s;
                                    if (s_dwell_remaining_s <= 0.0f) {
                                        s_dwell_remaining_s = 0.0f;
                                        s_interval_substate = INTERVAL_TILT_OUT;
                                        printf("[PITCH] Mode 1: Dwell complete -> Starting next tilt cycle\n");
                                    }
                                }
                                break;
                            }
                        }
                        break;
                    }

                    case MODE_MANUAL: {
                        // Mode 2: Direct knob tracking of DIR x pot_1 (1..15 deg) via 5 deg/s slew limiter (CT-3.3, CT-2.1, CT-2.2, CT-2.3)
                        s_target_pitch_deg = configured_target;
                        s_dwell_remaining_s = 0.0f;
                        s_dwell_duration_s = 0.0f;

                        // Slew setpoint toward configured target at 5 deg/s
                        // If DIR is flipped mid-motion, smoothly decelerates and sweeps to opposite side without pausing (CT-2.3)
                        if (s_slew_pitch_deg < configured_target - max_step) {
                            s_slew_pitch_deg += max_step;
                        } else if (s_slew_pitch_deg > configured_target + max_step) {
                            s_slew_pitch_deg -= max_step;
                        } else {
                            s_slew_pitch_deg = configured_target;
                        }
                        break;
                    }

                    default:
                        s_target_pitch_deg = 0.0f;
                        s_slew_pitch_deg = 0.0f;
                        break;
                }

                // Unified FastPID closed-loop calculation across running modes
                int16_t sp_centi = (int16_t)(s_slew_pitch_deg * 100.0f);
                int16_t fb_centi = (int16_t)(current_pitch * 100.0f);
                int16_t pwm = s_pitch_pid.step(sp_centi, fb_centi);

                // Deadband (+/- 0.15 deg) to prevent hunting and minimize actuator wear
                if (abs(sp_centi - fb_centi) <= 15) {
                    pwm = 0;
                } else if (pwm > 0 && pwm < PITCH_PWM_MIN_DRIVE) {
                    pwm = PITCH_PWM_MIN_DRIVE;
                } else if (pwm < 0 && pwm > -PITCH_PWM_MIN_DRIVE) {
                    pwm = -PITCH_PWM_MIN_DRIVE;
                }

                s_motor_pwm = pwm;
                pitch_motor_drive(pwm);
                break;
            }

            case SYS_FAULT:
            default:
                pitch_motor_brake();
                s_motor_pwm = 0;
                s_is_homed = false;
                break;
        }

        // Cache telemetry snapshot under critical section
        taskENTER_CRITICAL(&s_pitch_mux);
        s_telemetry_cache.sys_state = s_sys_state;
        s_telemetry_cache.is_homed = s_is_homed;
        s_telemetry_cache.current_pitch_deg = current_pitch;
        s_telemetry_cache.target_pitch_deg = s_target_pitch_deg;
        s_telemetry_cache.slew_pitch_deg = s_slew_pitch_deg;
        s_telemetry_cache.motor_pwm = s_motor_pwm;
        s_telemetry_cache.active_mode = panel_st.mode;
        s_telemetry_cache.active_dir = panel_st.dir;
        s_telemetry_cache.dwell_remaining_s = s_dwell_remaining_s;
        s_telemetry_cache.dwell_total_s = s_dwell_duration_s;
        taskEXIT_CRITICAL(&s_pitch_mux);
    }
}

void pitch_control_init() {
    s_pitch_pid.configure(PITCH_KP, PITCH_KI, PITCH_KD, PITCH_HZ, 16, true);
    s_pitch_pid.setOutputRange(-PITCH_PWM_MAX_DRIVE, PITCH_PWM_MAX_DRIVE);
    s_pitch_pid.clear();

    // Start pitch task pinned to Core 1 at priority 6
    xTaskCreatePinnedToCore(pitch_task_func, "pitch_task", 4096, NULL, 6, &s_pitch_task_handle, 1);
    printf("Pitch Control Task initialized on Core 1 (100 Hz).\n");
}

void pitch_control_start() {
    taskENTER_CRITICAL(&s_pitch_mux);
    if (s_sys_state == SYS_STOPPED) {
        s_sys_state = SYS_HOMING;
        s_is_homed = false;
        s_homed_stable_count = 0;
    }
    taskEXIT_CRITICAL(&s_pitch_mux);
}

void pitch_control_stop() {
    taskENTER_CRITICAL(&s_pitch_mux);
    s_sys_state = SYS_STOPPED;
    s_is_homed = false;
    taskEXIT_CRITICAL(&s_pitch_mux);
    pitch_motor_brake();
}

sys_state_t pitch_control_get_state() {
    taskENTER_CRITICAL(&s_pitch_mux);
    sys_state_t st = s_sys_state;
    taskEXIT_CRITICAL(&s_pitch_mux);
    return st;
}

const char* pitch_control_get_state_str() {
    sys_state_t st = pitch_control_get_state();
    switch (st) {
        case SYS_CALIBRATING: return "CALIBRATING";
        case SYS_STOPPED:     return "STOPPED";
        case SYS_HOMING:      return "HOMING";
        case SYS_RUNNING:     return "RUNNING";
        case SYS_FAULT:       return "FAULT";
        default:              return "UNKNOWN";
    }
}

bool pitch_control_is_running() {
    return (pitch_control_get_state() == SYS_RUNNING);
}

bool pitch_control_is_homed() {
    taskENTER_CRITICAL(&s_pitch_mux);
    bool homed = s_is_homed;
    taskEXIT_CRITICAL(&s_pitch_mux);
    return homed;
}

pitch_telemetry_t pitch_control_get_telemetry() {
    pitch_telemetry_t telem;
    taskENTER_CRITICAL(&s_pitch_mux);
    telem = s_telemetry_cache;
    taskEXIT_CRITICAL(&s_pitch_mux);
    return telem;
}

void pitch_control_update_pid(float kp, float ki, float kd) {
    taskENTER_CRITICAL(&s_pitch_mux);
    s_active_pitch_kp = kp;
    s_active_pitch_ki = ki;
    s_active_pitch_kd = kd;
    s_pid_reconfigure_pending = true;
    taskEXIT_CRITICAL(&s_pitch_mux);
}

void pitch_control_get_pid(float *kp, float *ki, float *kd) {
    taskENTER_CRITICAL(&s_pitch_mux);
    if (kp) *kp = s_active_pitch_kp;
    if (ki) *ki = s_active_pitch_ki;
    if (kd) *kd = s_active_pitch_kd;
    taskEXIT_CRITICAL(&s_pitch_mux);
}
