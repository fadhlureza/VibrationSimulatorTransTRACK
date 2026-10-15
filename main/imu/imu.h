#pragma once

#include <stdint.h>
#include <stdbool.h>

struct imu_snapshot_t {
    // Pitch angles (deg)
    float pitch_gyro_deg;           // Pure integrated gyro pitch angle (control feedback)
    float pitch_acc_deg;            // Static accelerometer pitch angle (reference / drift monitoring)
    float roll_deg;                 // Accelerometer roll
    float pitch_rate_dps;           // Live pitch angular velocity (deg/s)

    // Accelerations (m/s^2)
    float acc_x_ms2;
    float acc_y_ms2;
    float acc_z_ms2;
    float acc_z_ms2_calibrated;

    // Vibration metrics (dynamic gravity-tilt compensated)
    float vibration_g;              // Unfiltered dynamic vibration magnitude (g)
    float vibration_ms2;            // Dynamic vibration in m/s^2
    float vibration_ms2_calibrated; // Calibrated vibration m/s^2

    // Directional deviations from tilt-compensated baseline
    float delta_x;
    float delta_y;
    float delta_z;

    // Spectral analysis
    float dominant_freq_hz;

    // Health & status
    bool  is_calibrated;
    bool  sensor_ok;
};

// IMU Lifecycle & Snapshot API
bool imu_init();
void imu_calibrate();
imu_snapshot_t imu_get_snapshot();

// Standstill drift re-zeroing: gently aligns integrated gyro angle to accelerometer gravity
void imu_standstill_realign(float alpha);

// Backward-compatible signature
void imu_read_data(float* vibration, float* vibration_ms2, float* vibration_ms2_calibrated, float* deltaX, float* deltaY, float* deltaZ, 
                    float* accX_ms2, float* accY_ms2, float* accZ_ms2,
                    float* pitch, float* roll, float* output_freq_hz, float* accZ_ms2_calibrated);