#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "constant.h"
#include "imu/imu.h"
#include "motor/motor.h"
#include "button/button.h"
#include "potentiometer/potentiometer.h"
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
    
    pot_init();
    button_init();

    printf("Motor Control Initialized.\n");

    bool motorRunning = false;
    bool motorForwardDirection = true;
    int counter = 1;

    while (1) {
        if (g_pid_update_flag) {
            g_pid_update_flag = false;
            myPID.setCoefficients(g_active_kp, g_active_ki, g_active_kd, HZ);
            myPID.clear();
            printf("PID coefficients updated: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", g_active_kp, g_active_ki, g_active_kd);
        }


        float vibration, vibration_ms2, vibration_ms2_calibrated, deltaX, deltaY, deltaZ, freq_hz;
        float accX_ms2, accY_ms2, accZ_ms2, accZ_ms2_calibrated, pitch, roll;
        
        imu_read_data(&vibration, &vibration_ms2, &vibration_ms2_calibrated, &deltaX, &deltaY, &deltaZ, 
                      &accX_ms2, &accY_ms2, &accZ_ms2, &pitch, &roll, &freq_hz, &accZ_ms2_calibrated);

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

        int target_g_scaled = pot_read_target();
        g_target_g = (float)target_g_scaled / 1000.0f;

        int actual_g_scaled = (int)(g_calibrated_g * 1000.0f);

        if (start_btn_flag) {
            start_btn_flag = false;
            motorRunning = true;
            printf("START pressed\n");
        }

        if (stop_btn_flag) {
            stop_btn_flag = false;
            motorRunning = false;
            motorStop();
            myPID.clear();
            printf("STOP pressed\n");
        }

        if (dir_btn_flag) {
            dir_btn_flag = false;
            motorRunning = false;
            motorStop();
            myPID.clear();

            printf("DIR pressed - motor stopped before changing direction\n");
            vTaskDelay(1000 / portTICK_PERIOD_MS);

            motorForwardDirection = !motorForwardDirection;
            printf("Direction changed to: %s\n", motorForwardDirection ? "FORWARD" : "REVERSE");
        }

        if (motorRunning) {
            g_pwm_value = myPID.step(target_g_scaled, actual_g_scaled);

            if (target_g_scaled > 0) {
                if (g_pwm_value < 55) {
                    g_pwm_value = 55;
                }
            } else {
                g_pwm_value = 0;
                myPID.clear();
            }

            if (motorForwardDirection) {
                motorForward(g_pwm_value);
            } else {
                motorReverse(g_pwm_value);
            }
        } else {
            g_pwm_value = 0;
            motorStop();
            myPID.clear();
        }

        printf("Data Accelerometer BaseLine!!\n");
        if (vibration >= 0.03) {
            printf("GETARAN | ");
        } else {
            printf("DIAM    | ");
        }
        
        printf("counter: %d | ", counter++);

        printf("Target: %.3f g | Aktual: %.3f g | Output PWM: %d\n", g_target_g, g_calibrated_g, g_pwm_value);
        
        printf("vibration: %.3f g (%.3f m/s2) | calibrated vibration: %.3f m/s2 | dominant frequency: %.2f Hz | dX: %.3f | dY: %.3f | dZ: %.3f\n", 
               vibration, vibration_ms2, vibration_ms2_calibrated, freq_hz, deltaX, deltaY, deltaZ);
        
        printf("Data Accelerometer m/s², pitch, and roll!!\n");
        printf("Acc X: %.2f m/s2 | Y: %.2f m/s2 | Z: %.2f m/s2 || Pitch: %.2f deg | Roll: %.2f deg\n", 
               accX_ms2, accY_ms2, accZ_ms2, pitch, roll);

        printf("Motor: %s | Direction: %s\n\n", 
               motorRunning ? "RUNNING" : "STOP", motorForwardDirection ? "FORWARD" : "REVERSE");

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}