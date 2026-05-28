#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "imu.h"
#include "motor.h"
#include "button.h"
#include "potentiometer.h"

extern "C" void app_main(void) {

    printf("Initializing...\n");
    
    imu_init();
    imu_calibrate();
    
    motor_init();
    motorStop();
    
    pot_init();
    button_init();

    printf("Motor Control Initialized.\n");

    bool motorRunning = false;
    bool motorForwardDirection = true;

    while (1) {
        float vibration, deltaX, deltaY, deltaZ;
        float accX_ms2, accY_ms2, accZ_ms2, pitch, roll;
        
        imu_read_data(&vibration, &deltaX, &deltaY, &deltaZ, 
                      &accX_ms2, &accY_ms2, &accZ_ms2, &pitch, &roll);

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

        printf("vibration: %.3f g | dX: %.3f | dY: %.3f | dZ: %.3f\n", vibration, deltaX, deltaY, deltaZ);
        
        printf("Data Accelerometer m/s², pitch, and roll!!\n");
        printf("Acc X: %.2f m/s2 | Y: %.2f m/s2 | Z: %.2f m/s2 || Pitch: %.2f deg | Roll: %.2f deg\n", 
            accX_ms2, accY_ms2, accZ_ms2, pitch, roll);

        printf("PWM: %d | Motor: %s | Direction: %s\n\n", 
            pwmValue, motorRunning ? "RUNNING" : "STOP", motorForwardDirection ? "FORWARD" : "REVERSE");

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}