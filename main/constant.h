#pragma once

#include <stdint.h>
#include "module/fastpid/fastpid.h"

// Panel GPIO inputs
#define START_BTN_PIN       GPIO_NUM_8
#define STOP_BTN_PIN        GPIO_NUM_3
#define DIR_SWITCH_PIN      GPIO_NUM_9    // Low = FWD (+), High = REV (-)
#define MODE_SWITCH_PIN_A   GPIO_NUM_10   // Low = Mode 0 (Stabilized)
#define MODE_SWITCH_PIN_B   GPIO_NUM_21   // Low = Mode 2 (Manual); Both High = Mode 1 (Interval)

// Vibration Motor GPIO pins (BTS7960)
#define RPWM_PIN  13
#define LPWM_PIN  14
#define R_EN_PIN  11
#define L_EN_PIN  12

// Vibration Motor LEDC (PWM) parameters
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_R_CHANNEL          LEDC_CHANNEL_0
#define LEDC_L_CHANNEL          LEDC_CHANNEL_1
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT
#define LEDC_FREQUENCY          20000 // 20 kHz

// Pitch Motor GPIO pins (BTS7960, placeholder pins)
#define PITCH_RPWM_PIN          15
#define PITCH_LPWM_PIN          16
#define PITCH_R_EN_PIN          17
#define PITCH_L_EN_PIN          18

// Pitch Motor LEDC (PWM) parameters
#define LEDC_PITCH_TIMER        LEDC_TIMER_1
#define LEDC_PITCH_R_CH         LEDC_CHANNEL_2
#define LEDC_PITCH_L_CH         LEDC_CHANNEL_3
#define LEDC_PITCH_DUTY_RES     LEDC_TIMER_10_BIT
#define LEDC_PITCH_FREQUENCY    20000

// Pitch PID defaults & frequency
#define PITCH_KP                20.0f
#define PITCH_KI                2.0f
#define PITCH_KD                1.0f
#define PITCH_HZ                100.0f

// Pitch motion limits & safety
#define PITCH_SLEW_RATE_DPS     5.0f   // Default/safe slew rate in degrees/sec (5 deg/s) for Homing & Mode 2
#define PITCH_SLEW_GAIN_DEFAULT 0.50f  // Default Slew Gain for Mode 1 (50% of POT2 duration)
#define PITCH_SLEW_GAIN_MIN     0.10f  // Minimum allowable gain (10%)
#define PITCH_SLEW_GAIN_MAX     5.00f  // Maximum allowable gain (500%)
#define PITCH_SLEW_TIME_MIN_S   1.00f  // Hardware protection minimum slew duration (1.0 s)
#define PITCH_MAX_SLEW_RATE_DPS 15.0f  // Hardware safety ceiling for slew velocity (15 deg/s)
#define PITCH_HOMED_TOLERANCE   0.5f   // Flat tolerance (+/- 0.5 deg)
#define PITCH_MAX_ANGLE_FAULT   20.0f  // Hardware safety trip threshold (+/- deg)
#define PITCH_PWM_MIN_DRIVE     120    // Min PWM to overcome friction on 10-bit (0..1023)
#define PITCH_PWM_MAX_DRIVE     1023   // Max 10-bit PWM output

// Potentiometer ADC parameters (ADC1 only, Wi-Fi safe)
#define POT0_ADC_UNIT           ADC_UNIT_1
#define POT0_ADC_CHANNEL        ADC_CHANNEL_0 // GPIO 1: Vibration target (0-16 g)

#define POT1_ADC_UNIT           ADC_UNIT_1
#define POT1_ADC_CHANNEL        ADC_CHANNEL_5 // GPIO 6: Pitch tilt angle (1-15 deg)

#define POT2_ADC_UNIT           ADC_UNIT_1
#define POT2_ADC_CHANNEL        ADC_CHANNEL_6 // GPIO 7: Pitch interval (0-300 s)

// Legacy alias
#define POT_ADC_UNIT            POT0_ADC_UNIT
#define POT_ADC_CHANNEL         POT0_ADC_CHANNEL

// Potentiometer ranges & constants
#define POT0_MAX_G              16.0f
#define POT1_MIN_DEG            1.0f
#define POT1_MAX_DEG            15.0f
#define POT2_MIN_SEC            0.0f
#define POT2_MAX_SEC            300.0f
#define MODE1_LOCKED_G          1.000f

// Vibration Safety Configuration
#define VIB_SAFETY              1       // 1 = Safety feature active & shown to user; 0 = Safety off & completely hidden from user
#define VIB_SAFETY_MAX_G        1.000f  // Maximum allowable target vibration in G across all modes
#define VIB_SAFETY_OVER_G_TRIP  1.800f  // Emergency trip cutoff if measured vibration exceeds this

// IMU parameters
#define BMI160_ADDR             0x68
#define BMI160_REG_CHIP_ID      0x00
#define BMI160_CHIP_ID          0xD1
#define BMI160_REG_STATUS       0x1B
#define BMI160_STATUS_DRDY_ACC  0x80
#define CMD_REG                 0x7E
#define BMI160_CMD_SOFT_RESET   0xB6
#define BMI160_CMD_ACC_NORMAL   0x11
#define BMI160_CMD_GYR_NORMAL   0x15
#define GYR_X_LSB               0x0C
#define GYR_CONF                0x42
#define GYR_RANGE               0x43
#define ACC_X_LSB               0x12
#define ACC_RANGE               0x41
#define ACC_CONF                0x40
#define FFT_SAMPLES             128

// Gyro sensitivity scale factor for +/- 500 deg/s range (32768 / 500 = 65.536 LSB / deg/s)
#define GYRO_LSB_PER_DPS        65.536f

// IMU Pitch axis orientation configuration
// Set PITCH_AXIS_IS_Y: 0 if pitch tilts around IMU X-axis, 1 if around IMU Y-axis
// PITCH_SIGN: +1.0f or -1.0f to match table positive tilt direction
#define PITCH_AXIS_IS_Y         0
#define PITCH_SIGN              (1.0f)

// I2C parameters
#define I2C_MASTER_SCL_IO           4
#define I2C_MASTER_SDA_IO           5
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000

// PID
const float KP = 0.13;
const float KI = 0.3;
const float KD = 0.0;
const float HZ = 100.0;

extern volatile float g_target_g;

// Wi-Fi AP
void wifi_init_softap(void);
#define ESP_WIFI_SSID      "Vibration_Simulator"
#define ESP_WIFI_PASS      "12345678"
#define ESP_WIFI_CHANNEL   1
#define ESP_MAX_STA_CONN   4

// Web Server
void start_webserver(void);

// Shared IMU data
extern float g_vibration_g;
extern float g_calibrated_g;
extern float g_calibrated_ms2;
extern volatile float g_dominant_freq_hz;

// Shared Potentiometer data
extern volatile int g_pot_raw;
extern volatile int g_pwm_value;