#pragma once

#include <stdint.h>

// Button flags
extern volatile bool start_btn_flag;
extern volatile bool stop_btn_flag;
extern volatile bool dir_btn_flag;

// Button GPIO pins
#define START_BTN_PIN  GPIO_NUM_8
#define STOP_BTN_PIN   GPIO_NUM_3
#define DIR_BTN_PIN    GPIO_NUM_46

//Motor GPIO pins
#define RPWM_PIN  13
#define LPWM_PIN  14
#define R_EN_PIN  11
#define L_EN_PIN  12

// LEDC (PWM) parameters
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_R_CHANNEL          LEDC_CHANNEL_0
#define LEDC_L_CHANNEL          LEDC_CHANNEL_1
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT
#define LEDC_FREQUENCY          1000

// Potentiometer ADC parameters
#define POT_ADC_UNIT    ADC_UNIT_1
#define POT_ADC_CHANNEL ADC_CHANNEL_0

// IMU parameters
#define BMI160_ADDR             0x68
#define BMI160_REG_CHIP_ID      0x00
#define BMI160_CHIP_ID          0xD1
#define BMI160_REG_STATUS       0x1B
#define BMI160_STATUS_DRDY_ACC  0x80
#define CMD_REG                 0x7E
#define BMI160_CMD_SOFT_RESET   0xB6
#define ACC_X_LSB               0x12
#define ACC_RANGE               0x41
#define FFT_SAMPLES             128

// I2C parameters
#define I2C_MASTER_SCL_IO           4
#define I2C_MASTER_SDA_IO           5
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000

// Wi-Fi AP
void wifi_init_softap(void);
#define ESP_WIFI_SSID      "IMU_Sensor_AP"
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