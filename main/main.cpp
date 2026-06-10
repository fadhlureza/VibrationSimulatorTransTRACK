#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "constant.h"
#include "imu/imu.h"
#include "motor/motor.h"
#include "button/button.h"
#include "potentiometer/potentiometer.h"

float g_vibration_g = 0.0f;
float g_calibrated_g = 0.0f;
float g_calibrated_ms2 = 0.0f;
volatile float g_dominant_freq_hz = 0.0;
volatile int g_pot_raw = 0;
volatile int g_pwm_value = 0;

extern "C" void app_main(void) {

    printf("Initializing...\n");
    
    wifi_init_softap();
    start_webserver();

    imu_init();
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
        float vibration, vibration_ms2, vibration_ms2_calibrated, deltaX, deltaY, deltaZ, freq_hz;
        float accX_ms2, accY_ms2, accZ_ms2, pitch, roll;
        
        imu_read_data(&vibration, &vibration_ms2, &vibration_ms2_calibrated, &deltaX, &deltaY, &deltaZ, 
                      &accX_ms2, &accY_ms2, &accZ_ms2, &pitch, &roll, &freq_hz);

        g_vibration_g = vibration;
        g_calibrated_ms2 = vibration_ms2_calibrated;
        g_calibrated_g = vibration_ms2_calibrated / 9.80665f;
        g_dominant_freq_hz = freq_hz;

        int pwmValue = pot_read_pwm();

        if (start_btn_flag) {
            start_btn_flag = false;
            motorRunning = true;
            printf("START pressed\n");
        }

        if (stop_btn_flag) {
            stop_btn_flag = false;
            motorRunning = false;
            motorStop();
            printf("STOP pressed\n");
        }

        if (dir_btn_flag) {
            dir_btn_flag = false;
            motorRunning = false;
            motorStop();

            printf("DIR pressed - motor stopped before changing direction\n");
            vTaskDelay(1000 / portTICK_PERIOD_MS);

            motorForwardDirection = !motorForwardDirection;
            printf("Direction changed to: %s\n", motorForwardDirection ? "FORWARD" : "REVERSE");
        }

        if (motorRunning) {
            if (motorForwardDirection) {
                motorForward(pwmValue);
            } else {
                motorReverse(pwmValue);
            }
        } else {
            motorStop();
        }

        printf("Data Accelerometer BaseLine!!\n");
        if (vibration >= 0.03) {
            printf("GETARAN | ");
        } else {
            printf("DIAM    | ");
        }
        
        printf("counter: %d | ", counter++);

        printf("vibration: %.3f g (%.3f m/s2) | calibrated vibration: %.3f m/s2 | dominant frequency: %.2f Hz | dX: %.3f | dY: %.3f | dZ: %.3f\n", vibration, vibration_ms2, vibration_ms2_calibrated, freq_hz, deltaX, deltaY, deltaZ);
        
        printf("Data Accelerometer m/s², pitch, and roll!!\n");
        printf("Acc X: %.2f m/s2 | Y: %.2f m/s2 | Z: %.2f m/s2 || Pitch: %.2f deg | Roll: %.2f deg\n", 
            accX_ms2, accY_ms2, accZ_ms2, pitch, roll);

        printf("PWM: %d | Motor: %s | Direction: %s\n\n", 
            pwmValue, motorRunning ? "RUNNING" : "STOP", motorForwardDirection ? "FORWARD" : "REVERSE");

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}