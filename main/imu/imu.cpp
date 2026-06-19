#include "constant.h"
#include "imu.h"
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_timer.h"

static float baseX = 0;
static float baseY = 0;
static float baseZ = 0;

static float vReal[FFT_SAMPLES];
static float vImag[FFT_SAMPLES];
static int sampleIndex = 0;
static int64_t first_sample_time = 0;
static float dominant_freq_hz = 0.0;

static void compute_FFT(float *vR, float *vI, uint16_t samples) {
    uint16_t j = 0;
    for (uint16_t i = 0; i < (samples - 1); i++) {
        if (i < j) {
            float tempR = vR[j];
            float tempI = vI[j];
            vR[j] = vR[i];
            vI[j] = vI[i];
            vR[i] = tempR;
            vI[i] = tempI;
        }
        uint16_t k = (samples >> 1);
        while (k <= j) {
            j -= k;
            k >>= 1;
        }
        j += k;
    }

    float c1 = -1.0, c2 = 0.0;
    uint16_t l2 = 1;
    for (uint16_t l = 0; (1 << l) < samples; l++) {
        uint16_t l1 = l2;
        l2 <<= 1;
        float u1 = 1.0;
        float u2 = 0.0;
        for (uint16_t j = 0; j < l1; j++) {
            for (uint16_t i = j; i < samples; i += l2) {
                uint16_t i1 = i + l1;
                float t1 = u1 * vR[i1] - u2 * vI[i1];
                float t2 = u1 * vI[i1] + u2 * vR[i1];
                vR[i1] = vR[i] - t1;
                vI[i1] = vI[i] - t2;
                vR[i] += t1;
                vI[i] += t2;
            }
            float z = (u1 * c1) - (u2 * c2);
            u2 = (u1 * c2) + (u2 * c1);
            u1 = z;
        }
        c2 = sqrt((1.0 - c1) / 2.0);
        if (l == 0) c2 = -c2;
        c1 = sqrt((1.0 + c1) / 2.0);
    }
}

static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    
    i2c_param_config(I2C_MASTER_NUM, &conf);
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static void writeRegister(uint8_t reg, uint8_t data) {
    uint8_t write_buf[2] = {reg, data};
    i2c_master_write_to_device(I2C_MASTER_NUM, BMI160_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static int16_t read16(uint8_t reg) {
    uint8_t data[2] = {0, 0};
    i2c_master_write_read_device(I2C_MASTER_NUM, BMI160_ADDR, &reg, 1, data, 2, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    return (int16_t)((data[1] << 8) | data[0]);
}

static float rawToG(int16_t raw) {
    return raw / 2048.0f;
}

void imu_init() {
    i2c_master_init();
    
    writeRegister(CMD_REG, 0x11);
    writeRegister(ACC_CONF, 0x2C);
    writeRegister(ACC_RANGE, 0x0C);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    printf("BMI160 Started\n");
}

void imu_calibrate() {
    const int sampleCount = 100;

    float sumX = 0;
    float sumY = 0;
    float sumZ = 0;

    printf("Calibrating IMU baseline...\n");
    printf("Jangan gerakkan sensor...\n");

    for (int i = 0; i < sampleCount; i++) {
        int16_t accX_raw = read16(ACC_X_LSB);
        int16_t accY_raw = read16(ACC_X_LSB + 2);
        int16_t accZ_raw = read16(ACC_X_LSB + 4);

        sumX += rawToG(accX_raw);
        sumY += rawToG(accY_raw);
        sumZ += rawToG(accZ_raw);

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    baseX = sumX / sampleCount;
    baseY = sumY / sampleCount;
    baseZ = sumZ / sampleCount;

    printf("Baseline selesai:\n");
    printf("baseX: %.3f g | baseY: %.3f g | baseZ: %.3f g\n", baseX, baseY, baseZ);
}

void imu_read_data(float* vibration, float* vibration_ms2, float* vibration_ms2_calibrated, float* deltaX, float* deltaY, float* deltaZ, 
                   float* accX_ms2, float* accY_ms2, float* accZ_ms2, 
                   float* pitch, float* roll, float* output_freq_hz, float* accZ_ms2_calibrated) {
    int16_t accX_raw = read16(ACC_X_LSB);
    int16_t accY_raw = read16(ACC_X_LSB + 2);
    int16_t accZ_raw = read16(ACC_X_LSB + 4);

    float accX_g = rawToG(accX_raw);
    float accY_g = rawToG(accY_raw);
    float accZ_g = rawToG(accZ_raw);

    *deltaX = accX_g - baseX;
    *deltaY = accY_g - baseY;
    *deltaZ = accZ_g - baseZ;

    *vibration = sqrt((*deltaX) * (*deltaX) + (*deltaY) * (*deltaY) + (*deltaZ) * (*deltaZ));
    *vibration_ms2 = ((*vibration) * 9.80665);
    *vibration_ms2_calibrated = ((*vibration_ms2) * 1.226) + 0.145;

    *accZ_ms2_calibrated = (*deltaZ * 9.80665) * 1.226;

    *roll = atan2(accY_g, accZ_g) * 180.0 / M_PI;
    *pitch = atan2(-accX_g, sqrt(accY_g * accY_g + accZ_g * accZ_g)) * 180.0 / M_PI;

    *accX_ms2 = accX_g * 9.80665;
    *accY_ms2 = accY_g * 9.80665;
    *accZ_ms2 = accZ_g * 9.80665;

    if (sampleIndex == 0) {
        first_sample_time = esp_timer_get_time();
    }

    vReal[sampleIndex] = *accZ_ms2_calibrated;
    vImag[sampleIndex] = 0.0;
    sampleIndex++;

    if (sampleIndex >= FFT_SAMPLES) {
        int64_t end_time = esp_timer_get_time();
        
        float time_elapsed_sec = (end_time - first_sample_time) / 1000000.0;
        float sampling_freq = FFT_SAMPLES / time_elapsed_sec;

        compute_FFT(vReal, vImag, FFT_SAMPLES);

        float max_magnitude = 0.0;
        int peak_index = 0;
        
        for (int i = 3; i < (FFT_SAMPLES / 2); i++) { 
            float magnitude = sqrt((vReal[i] * vReal[i]) + (vImag[i] * vImag[i]));
            if (magnitude > max_magnitude) {
                max_magnitude = magnitude;
                peak_index = i;
            }
        }

        dominant_freq_hz = (peak_index * sampling_freq) / FFT_SAMPLES;
        
        sampleIndex = 0; 
    }

    *output_freq_hz = dominant_freq_hz;
}
