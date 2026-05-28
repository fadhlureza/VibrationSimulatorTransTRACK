#pragma once

#include <stdint.h>

void imu_init();
void imu_calibrate();
void imu_read_data(float* vibration, float* vibration_ms2, float* deltaX, float* deltaY, float* deltaZ, 
                   float* accX_ms2, float* accY_ms2, float* accZ_ms2, 
                   float* pitch, float* roll);
