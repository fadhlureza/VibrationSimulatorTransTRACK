#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "constant.h"
#include "imu/imu.h"
#include "motor/motor.h"
#include "panel/panel.h"
#include "pitch_motor/pitch_motor.h"
#include "pitch_motor/pitch_control.h"
#include "module/fastpid/fastpid.h"

float g_vibration_g = 0.0f;
float g_calibrated_g = 0.0f;
float g_calibrated_ms2 = 0.0f;
volatile float g_dominant_freq_hz = 0.0;
volatile int g_pot_raw = 0;
volatile int g_pwm_value = 0;
volatile float g_target_g = 0.0f;

volatile float g_active_kp = KP;
volatile float g_active_ki = KI;
volatile float g_active_kd = KD;
volatile bool g_pid_update_flag = false;

volatile float g_ema_alpha = 0.2f;
volatile bool first_ema_read = true;

FastPID myPID(KP, KI, KD, HZ, 8, false);

extern "C" void app_main(void) {
    printf("Initializing...\n");
    
    myPID.setOutputRange(0, 210);

    wifi_init_softap();
    start_webserver();

    if (!imu_init()) {
        printf("IMU init failed - check wiring/I2C\n");
        return;
    }
    imu_calibrate();
    
    motor_init();
    motorStop();

    pitch_motor_init();
    pitch_control_init();
    
    panel_init();

    printf("Motor Control Initialized.\n");

    int counter = 1;

    while (1) {
        if (g_pid_update_flag) {
            g_pid_update_flag = false;
            myPID.setCoefficients(g_active_kp, g_active_ki, g_active_kd, HZ);
            myPID.clear();
            printf("PID coefficients updated: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", g_active_kp, g_active_ki, g_active_kd);
        }

        imu_snapshot_t imu_snap = imu_get_snapshot();

        float vibration = imu_snap.vibration_g;
        float vibration_ms2 = imu_snap.vibration_ms2;
        float vibration_ms2_calibrated = imu_snap.vibration_ms2_calibrated;
        float deltaX = imu_snap.delta_x;
        float deltaY = imu_snap.delta_y;
        float deltaZ = imu_snap.delta_z;
        float freq_hz = imu_snap.dominant_freq_hz;
        float accX_ms2 = imu_snap.acc_x_ms2;
        float accY_ms2 = imu_snap.acc_y_ms2;
        float accZ_ms2 = imu_snap.acc_z_ms2;
        float pitch_gyro = imu_snap.pitch_gyro_deg;
        float pitch_acc = imu_snap.pitch_acc_deg;
        float roll = imu_snap.roll_deg;

        g_vibration_g = vibration;

        float current_raw_ms2 = vibration_ms2_calibrated;

        if (first_ema_read) {
            g_calibrated_ms2 = current_raw_ms2;
            first_ema_read = false;
        } else {
            g_calibrated_ms2 = (g_ema_alpha * current_raw_ms2) + ((1.0f - g_ema_alpha) * g_calibrated_ms2);
        }

        g_calibrated_g = g_calibrated_ms2 / 9.80665f;
        g_dominant_freq_hz = freq_hz;

        // Read panel state & system pitch state
        panel_state_t panel_st = panel_get_state();
        sys_state_t sys_state = pitch_control_get_state();
        pitch_telemetry_t pitch_telem = pitch_control_get_telemetry();

        g_target_g = panel_st.vib_target_g;

        int target_g_scaled = (int)(g_target_g * 1000.0f);
        int actual_g_scaled = (int)(g_calibrated_g * 1000.0f);

        // Vibration motor only runs once pitch motor has homed to 0.0 deg (SYS_RUNNING)
        bool vibrationRunning = (sys_state == SYS_RUNNING);

        if (vibrationRunning) {
            g_pwm_value = myPID.step(target_g_scaled, actual_g_scaled);

            if (target_g_scaled > 0) {
                if (g_pwm_value < 55) {
                    g_pwm_value = 55;
                }
            } else {
                g_pwm_value = 0;
                myPID.clear();
            }

            // Vibration motor always operates forward
            motorForward(g_pwm_value);
        } else {
            g_pwm_value = 0;
            motorStop();
            myPID.clear();
        }

        static int s_log_ticks = 0;
        if (++s_log_ticks >= 100) { // 100 * 10ms = 1000ms (1 Hz)
            s_log_ticks = 0;

            printf("Data Accelerometer BaseLine!!\n");
            if (vibration >= 0.03) {
                printf("GETARAN | ");
            } else {
                printf("DIAM    | ");
            }
            
            printf("counter: %d | ", counter++);

            printf("SysState: %s | Mode: %d | Dir: %s | Target: %.3f g | Aktual: %.3f g\n", 
                   pitch_control_get_state_str(), (int)panel_st.mode, panel_st.dir == DIR_FORWARD ? "FWD" : "REV", g_target_g, g_calibrated_g);
            
            printf("vibration: %.3f g (%.3f m/s2) | calibrated vibration: %.3f m/s2 | dominant frequency: %.2f Hz | dX: %.3f | dY: %.3f | dZ: %.3f\n", 
                   vibration, vibration_ms2, vibration_ms2_calibrated, freq_hz, deltaX, deltaY, deltaZ);
            
            if (panel_st.mode == MODE_INTERVAL && pitch_telem.dwell_total_s > 0.0f) {
                printf("Pitch Axis: Live=%.2f deg | Target=%.2f deg | Slew=%.2f deg | PWM=%d | Dwell: %.1f/%.1fs\n",
                       pitch_telem.current_pitch_deg, pitch_telem.target_pitch_deg, pitch_telem.slew_pitch_deg, pitch_telem.motor_pwm, pitch_telem.dwell_remaining_s, pitch_telem.dwell_total_s);
            } else {
                printf("Pitch Axis: Live=%.2f deg | Target=%.2f deg | Slew=%.2f deg | PWM=%d | Homed=%s\n",
                       pitch_telem.current_pitch_deg, pitch_telem.target_pitch_deg, pitch_telem.slew_pitch_deg, pitch_telem.motor_pwm, pitch_telem.is_homed ? "YES" : "NO");
            }

            printf("Acc X: %.2f m/s2 | Y: %.2f m/s2 | Z: %.2f m/s2 || Pitch(Gyro): %.2f deg | Pitch(Acc): %.2f deg | Roll: %.2f deg\n", 
                   accX_ms2, accY_ms2, accZ_ms2, pitch_gyro, pitch_acc, roll);

            printf("Vibration Motor: %s (PWM: %d) | Pitch Motor: %s\n\n", 
                   vibrationRunning ? "RUNNING" : "STOP",
                   g_pwm_value,
                   (sys_state == SYS_STOPPED || sys_state == SYS_FAULT) ? "BRAKED" : "ACTIVE");
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}