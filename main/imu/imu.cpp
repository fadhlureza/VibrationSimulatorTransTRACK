#include "constant.h"
#include "imu.h"
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "BMI160";

static const int I2C_RETRY_COUNT = 3;
static const int RECOVERY_COOLDOWN_MS = 5000;
static const int HEALTH_CHECK_INTERVAL = 100;
static const int RECOVERY_FAIL_THRESHOLD = 3;

static float baseX = 0;
static float baseY = 0;
static float baseZ = 0;

static float vReal[FFT_SAMPLES];
static float vImag[FFT_SAMPLES];
static int sampleIndex = 0;
static int64_t first_sample_time = 0;
static float dominant_freq_hz = 0.0;

static struct {
    float accX_g, accY_g, accZ_g;
    float vibration, vibration_ms2, vibration_ms2_calibrated;
    float deltaX, deltaY, deltaZ;
    float accX_ms2, accY_ms2, accZ_ms2;
    float pitch, roll;
    bool valid;
} last_valid = {};

static int read_cycle_count = 0;
static int consecutive_failures = 0;
static int64_t last_recovery_attempt_us = 0;

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

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
    }
    return err;
}

static esp_err_t writeRegister(uint8_t reg, uint8_t data) {
    uint8_t write_buf[2] = {reg, data};
    esp_err_t err = i2c_master_write_to_device(
        I2C_MASTER_NUM, BMI160_ADDR, write_buf, sizeof(write_buf),
        I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Write reg 0x%02X failed: %s", reg, esp_err_to_name(err));
    }
    return err;
}

static esp_err_t readRegister(uint8_t reg, uint8_t *buf, size_t len) {
    esp_err_t err = i2c_master_write_read_device(
        I2C_MASTER_NUM, BMI160_ADDR, &reg, 1, buf, len,
        I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Read reg 0x%02X failed: %s", reg, esp_err_to_name(err));
    }
    return err;
}

static esp_err_t read_accel_burst(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t data[6] = {0};
    esp_err_t err = readRegister(ACC_X_LSB, data, sizeof(data));
    if (err != ESP_OK) {
        return err;
    }

    *x = (int16_t)((data[1] << 8) | data[0]);
    *y = (int16_t)((data[3] << 8) | data[2]);
    *z = (int16_t)((data[5] << 8) | data[4]);
    return ESP_OK;
}

static float rawToG(int16_t raw) {
    return raw / 8192.0f;
}

static esp_err_t bmi160_check_who_am_i(void) {
    uint8_t chip_id = 0;
    esp_err_t err = readRegister(BMI160_REG_CHIP_ID, &chip_id, 1);
    if (err != ESP_OK) {
        return err;
    }
    if (chip_id != BMI160_CHIP_ID) {
        ESP_LOGE(TAG, "Unexpected CHIP_ID: 0x%02X (expected 0x%02X)", chip_id, BMI160_CHIP_ID);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t bmi160_configure(void) {
    esp_err_t err = writeRegister(CMD_REG, 0x11);
    if (err != ESP_OK) {
        return err;
    }

    err = writeRegister(ACC_RANGE, 0x05);
    if (err != ESP_OK) {
        return err;
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
    return ESP_OK;
}

static esp_err_t read_accel_burst_with_retry(int16_t *x, int16_t *y, int16_t *z) {
    esp_err_t err = ESP_FAIL;
    for (int attempt = 0; attempt < I2C_RETRY_COUNT; attempt++) {
        err = read_accel_burst(x, y, z);
        if (err == ESP_OK) {
            return ESP_OK;
        }
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
    return err;
}

static void copy_last_valid_outputs(
    float *vibration, float *vibration_ms2, float *vibration_ms2_calibrated,
    float *deltaX, float *deltaY, float *deltaZ,
    float *accX_ms2, float *accY_ms2, float *accZ_ms2,
    float *pitch, float *roll, float *output_freq_hz) {
    *vibration = last_valid.vibration;
    *vibration_ms2 = last_valid.vibration_ms2;
    *vibration_ms2_calibrated = last_valid.vibration_ms2_calibrated;
    *deltaX = last_valid.deltaX;
    *deltaY = last_valid.deltaY;
    *deltaZ = last_valid.deltaZ;
    *accX_ms2 = last_valid.accX_ms2;
    *accY_ms2 = last_valid.accY_ms2;
    *accZ_ms2 = last_valid.accZ_ms2;
    *pitch = last_valid.pitch;
    *roll = last_valid.roll;
    *output_freq_hz = dominant_freq_hz;
}

static void update_fft(float vibration_ms2_calibrated) {
    if (sampleIndex == 0) {
        first_sample_time = esp_timer_get_time();
    }

    vReal[sampleIndex] = vibration_ms2_calibrated;
    vImag[sampleIndex] = 0.0;
    sampleIndex++;

    if (sampleIndex >= FFT_SAMPLES) {
        int64_t end_time = esp_timer_get_time();

        float time_elapsed_sec = (end_time - first_sample_time) / 1000000.0;
        float sampling_freq = FFT_SAMPLES / time_elapsed_sec;

        compute_FFT(vReal, vImag, FFT_SAMPLES);

        float max_magnitude = 0.0;
        int peak_index = 0;

        for (int i = 1; i < (FFT_SAMPLES / 2); i++) {
            float magnitude = sqrt((vReal[i] * vReal[i]) + (vImag[i] * vImag[i]));
            if (magnitude > max_magnitude) {
                max_magnitude = magnitude;
                peak_index = i;
            }
        }

        dominant_freq_hz = (peak_index * sampling_freq) / FFT_SAMPLES;
        sampleIndex = 0;
    }
}

static esp_err_t imu_recover(void) {
    int64_t now_us = esp_timer_get_time();
    int64_t cooldown_us = (int64_t)RECOVERY_COOLDOWN_MS * 1000;
    if ((now_us - last_recovery_attempt_us) < cooldown_us) {
        return ESP_ERR_INVALID_STATE;
    }
    last_recovery_attempt_us = now_us;

    ESP_LOGW(TAG, "Attempting BMI160 recovery (soft reset)...");

    esp_err_t err = writeRegister(CMD_REG, BMI160_CMD_SOFT_RESET);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Soft reset write failed: %s", esp_err_to_name(err));
        return err;
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);

    err = bmi160_configure();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Reconfigure after reset failed: %s", esp_err_to_name(err));
        return err;
    }

    err = bmi160_check_who_am_i();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WHO_AM_I check after recovery failed");
        return err;
    }

    sampleIndex = 0;
    imu_calibrate();
    consecutive_failures = 0;

    ESP_LOGI(TAG, "BMI160 recovery successful, baseline re-calibrated");
    return ESP_OK;
}

bool imu_init(void) {
    esp_err_t err = i2c_master_init();
    if (err != ESP_OK) {
        return false;
    }

    err = bmi160_configure();
    if (err != ESP_OK) {
        return false;
    }

    err = bmi160_check_who_am_i();
    if (err != ESP_OK) {
        return false;
    }

    ESP_LOGI(TAG, "BMI160 initialized (CHIP_ID 0x%02X)", BMI160_CHIP_ID);
    return true;
}

void imu_calibrate(void) {
    const int sampleCount = 100;

    float sumX = 0;
    float sumY = 0;
    float sumZ = 0;
    int validSamples = 0;

    printf("Calibrating IMU baseline...\n");
    printf("Jangan gerakkan sensor...\n");

    for (int i = 0; i < sampleCount; i++) {
        int16_t accX_raw, accY_raw, accZ_raw;
        esp_err_t err = read_accel_burst_with_retry(&accX_raw, &accY_raw, &accZ_raw);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Calibration sample %d failed, skipping", i);
            vTaskDelay(10 / portTICK_PERIOD_MS);
            continue;
        }

        sumX += rawToG(accX_raw);
        sumY += rawToG(accY_raw);
        sumZ += rawToG(accZ_raw);
        validSamples++;

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    if (validSamples == 0) {
        ESP_LOGE(TAG, "Calibration failed: no valid samples");
        return;
    }

    baseX = sumX / validSamples;
    baseY = sumY / validSamples;
    baseZ = sumZ / validSamples;

    printf("Baseline selesai:\n");
    printf("baseX: %.3f g | baseY: %.3f g | baseZ: %.3f g\n", baseX, baseY, baseZ);
}

void imu_read_data(float* vibration, float* vibration_ms2, float* vibration_ms2_calibrated, float* deltaX, float* deltaY, float* deltaZ,
                   float* accX_ms2, float* accY_ms2, float* accZ_ms2,
                   float* pitch, float* roll, float* output_freq_hz) {
    read_cycle_count++;
    if (read_cycle_count >= HEALTH_CHECK_INTERVAL) {
        read_cycle_count = 0;
        esp_err_t health_err = bmi160_check_who_am_i();
        if (health_err != ESP_OK) {
            ESP_LOGW(TAG, "Periodic WHO_AM_I health check failed");
        }
    }

    int16_t accX_raw, accY_raw, accZ_raw;
    esp_err_t err = read_accel_burst_with_retry(&accX_raw, &accY_raw, &accZ_raw);

    if (err != ESP_OK) {
        consecutive_failures++;
        ESP_LOGW(TAG, "Accel read failed (%d consecutive failures)", consecutive_failures);

        if (consecutive_failures >= RECOVERY_FAIL_THRESHOLD) {
            imu_recover();
        }

        if (last_valid.valid) {
            copy_last_valid_outputs(
                vibration, vibration_ms2, vibration_ms2_calibrated,
                deltaX, deltaY, deltaZ,
                accX_ms2, accY_ms2, accZ_ms2,
                pitch, roll, output_freq_hz);
        } else {
            *output_freq_hz = dominant_freq_hz;
        }
        return;
    }

    consecutive_failures = 0;

    float accX_g = rawToG(accX_raw);
    float accY_g = rawToG(accY_raw);
    float accZ_g = rawToG(accZ_raw);

    float out_deltaX = accX_g - baseX;
    float out_deltaY = accY_g - baseY;
    float out_deltaZ = accZ_g - baseZ;

    float out_vibration = sqrt(out_deltaX * out_deltaX + out_deltaY * out_deltaY + out_deltaZ * out_deltaZ);
    float out_vibration_ms2 = out_vibration * 9.80665f;
    float out_vibration_ms2_calibrated = (out_vibration_ms2 * 1.226f) + 0.145f;

    float out_roll = atan2(accY_g, accZ_g) * 180.0f / (float)M_PI;
    float out_pitch = atan2(-accX_g, sqrt(accY_g * accY_g + accZ_g * accZ_g)) * 180.0f / (float)M_PI;

    float out_accX_ms2 = accX_g * 9.80665f;
    float out_accY_ms2 = accY_g * 9.80665f;
    float out_accZ_ms2 = accZ_g * 9.80665f;

    last_valid.accX_g = accX_g;
    last_valid.accY_g = accY_g;
    last_valid.accZ_g = accZ_g;
    last_valid.vibration = out_vibration;
    last_valid.vibration_ms2 = out_vibration_ms2;
    last_valid.vibration_ms2_calibrated = out_vibration_ms2_calibrated;
    last_valid.deltaX = out_deltaX;
    last_valid.deltaY = out_deltaY;
    last_valid.deltaZ = out_deltaZ;
    last_valid.accX_ms2 = out_accX_ms2;
    last_valid.accY_ms2 = out_accY_ms2;
    last_valid.accZ_ms2 = out_accZ_ms2;
    last_valid.pitch = out_pitch;
    last_valid.roll = out_roll;
    last_valid.valid = true;

    *deltaX = out_deltaX;
    *deltaY = out_deltaY;
    *deltaZ = out_deltaZ;
    *vibration = out_vibration;
    *vibration_ms2 = out_vibration_ms2;
    *vibration_ms2_calibrated = out_vibration_ms2_calibrated;
    *roll = out_roll;
    *pitch = out_pitch;
    *accX_ms2 = out_accX_ms2;
    *accY_ms2 = out_accY_ms2;
    *accZ_ms2 = out_accZ_ms2;

    update_fft(out_vibration_ms2_calibrated);
    *output_freq_hz = dominant_freq_hz;
}
