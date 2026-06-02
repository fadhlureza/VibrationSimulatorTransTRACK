#pragma once

#include <stdint.h>

// Button flags
extern volatile bool start_btn_flag;
extern volatile bool stop_btn_flag;
extern volatile bool dir_btn_flag;

void button_init();

// Motor
void motor_init();
void motorStop();
void motorForward(int speedPWM);
void motorReverse(int speedPWM);

// Potentiometer
void pot_init();
int pot_read_pwm();

// IMU
void imu_init();
void imu_calibrate();
void imu_read_data(float* vibration, float* vibration_ms2, float* vibration_ms2_calibrated, float* deltaX, float* deltaY, float* deltaZ, 
                    float* accX_ms2, float* accY_ms2, float* accZ_ms2,
                    float* pitch, float* roll);

#ifdef __cplusplus
extern "C" {
#endif

// Wi-Fi AP
void wifi_init_softap(void);

// Web Server
void start_webserver(void);

// Shared IMU data
extern float g_vibration_g;
extern float g_calibrated_g;
extern float g_calibrated_ms2;

#ifdef __cplusplus
}
#endif
