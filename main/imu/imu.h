#pragma once

#include <stdint.h>

// IMU
void imu_init();
void imu_calibrate();
void imu_read_data(float* vibration, float* vibration_ms2, float* vibration_ms2_calibrated, float* deltaX, float* deltaY, float* deltaZ, 
                    float* accX_ms2, float* accY_ms2, float* accZ_ms2,
                    float* pitch, float* roll);