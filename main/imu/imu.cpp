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
static const int HEALTH_CHECK_INTERVAL = 200;
static const int RECOVERY_FAIL_THRESHOLD = 5;

// Calibration baselines
static float baseX = 0.0f;
static float baseY = 0.0f;
static float baseZ = 0.0f;
static float s_flat_baseX = 0.0f;
static float s_flat_baseY = 0.0f;
static float s_flat_baseZ = 1.0f;
static float bias_gx = 0.0f;
static float bias_gy = 0.0f;
static float bias_gz = 0.0f;
static float s_initial_pitch_deg = 0.0f;
static volatile float s_integrated_pitch_deg = 0.0f;
static volatile bool s_is_calibrated = false;

// FFT state (128 samples)
static float vReal[FFT_SAMPLES];
static float vImag[FFT_SAMPLES];
static int sampleIndex = 0;
static int64_t first_sample_time = 0;
static float dominant_freq_hz = 0.0f;

// Thread-safe snapshot state
static imu_snapshot_t s_latest_snapshot = {};
static portMUX_TYPE s_imu_mux = portMUX_INITIALIZER_UNLOCKED;

static TaskHandle_t s_imu_task_handle = NULL;
static esp_timer_handle_t s_imu_timer = NULL;

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

    float c1 = -1.0f, c2 = 0.0f;
    uint16_t l2 = 1;
    for (uint16_t l = 0; (1 << l) < samples; l++) {
        uint16_t l1 = l2;
        l2 <<= 1;
        float u1 = 1.0f;
        float u2 = 0.0f;
        for (uint16_t m = 0; m < l1; m++) {
            for (uint16_t i = m; i < samples; i += l2) {
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
        c2 = sqrtf((1.0f - c1) / 2.0f);
        if (l == 0) c2 = -c2;
        c1 = sqrtf((1.0f + c1) / 2.0f);
    }
}

static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
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

static esp_err_t read_motion_burst(int16_t *gx, int16_t *gy, int16_t *gz,
                                  int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t data[12] = {0};
    esp_err_t err = readRegister(GYR_X_LSB, data, sizeof(data));
    if (err != ESP_OK) {
        return err;
    }

    *gx = (int16_t)((data[1] << 8) | data[0]);
    *gy = (int16_t)((data[3] << 8) | data[2]);
    *gz = (int16_t)((data[5] << 8) | data[4]);
    *ax = (int16_t)((data[7] << 8) | data[6]);
    *ay = (int16_t)((data[9] << 8) | data[8]);
    *az = (int16_t)((data[11] << 8) | data[10]);
    return ESP_OK;
}

static inline float rawToG(int16_t raw) {
    return (float)raw / 2048.0f; // ±16g range
}

static inline float rawToDPS(int16_t raw) {
    return (float)raw / GYRO_LSB_PER_DPS; // ±500 deg/s range (65.536 LSB/deg/s)
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
    // Settle delay after I2C bus initialization
    vTaskDelay(pdMS_TO_TICKS(10));

    // 1. Power on Accelerometer in normal mode
    esp_err_t err = writeRegister(CMD_REG, BMI160_CMD_ACC_NORMAL);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(10)); // Allow ACC PMU to settle (min 3.8 ms per datasheet)

    // 2. Power on Gyroscope in normal mode
    err = writeRegister(CMD_REG, BMI160_CMD_GYR_NORMAL);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(60)); // Allow Gyro analog PLL to stabilize (min 55-80 ms per datasheet)

    // 3. Accelerometer config: 200 Hz ODR, normal bandwidth
    err = writeRegister(ACC_CONF, 0x2C);
    if (err != ESP_OK) return err;

    // 4. Accelerometer range: ±16g
    err = writeRegister(ACC_RANGE, 0x0C);
    if (err != ESP_OK) return err;

    // 5. Gyroscope config: 200 Hz ODR, normal bandwidth
    err = writeRegister(GYR_CONF, 0x29);
    if (err != ESP_OK) return err;

    // 6. Gyroscope range: ±500 deg/s
    err = writeRegister(GYR_RANGE, 0x02);
    if (err != ESP_OK) return err;

    // Allow internal digital filters and sensor registers to stabilize
    vTaskDelay(pdMS_TO_TICKS(50));
    return ESP_OK;
}

static esp_err_t read_motion_burst_with_retry(int16_t *gx, int16_t *gy, int16_t *gz,
                                             int16_t *ax, int16_t *ay, int16_t *az) {
    esp_err_t err = ESP_FAIL;
    for (int attempt = 0; attempt < I2C_RETRY_COUNT; attempt++) {
        err = read_motion_burst(gx, gy, gz, ax, ay, az);
        if (err == ESP_OK) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return err;
}

static void update_fft(float vibration_calibrated) {
    if (sampleIndex == 0) {
        first_sample_time = esp_timer_get_time();
    }

    vReal[sampleIndex] = vibration_calibrated;
    vImag[sampleIndex] = 0.0f;
    sampleIndex++;

    if (sampleIndex >= FFT_SAMPLES) {
        int64_t end_time = esp_timer_get_time();
        float time_elapsed_sec = (float)(end_time - first_sample_time) / 1000000.0f;
        float sampling_freq = (time_elapsed_sec > 0.001f) ? ((float)FFT_SAMPLES / time_elapsed_sec) : 200.0f;

        compute_FFT(vReal, vImag, FFT_SAMPLES);

        float max_magnitude = 0.0f;
        int peak_index = 0;

        for (int i = 1; i < (FFT_SAMPLES / 2); i++) {
            float magnitude = sqrtf((vReal[i] * vReal[i]) + (vImag[i] * vImag[i]));
            if (magnitude > max_magnitude) {
                max_magnitude = magnitude;
                peak_index = i;
            }
        }

        dominant_freq_hz = ((float)peak_index * sampling_freq) / (float)FFT_SAMPLES;
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

    vTaskDelay(pdMS_TO_TICKS(50));

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
    consecutive_failures = 0;

    ESP_LOGI(TAG, "BMI160 recovery successful");
    return ESP_OK;
}

// 200 Hz Timer callback
static void IRAM_ATTR imu_timer_callback(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (s_imu_task_handle != NULL) {
        vTaskNotifyGiveFromISR(s_imu_task_handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// Dedicated 200 Hz IMU processing task on Core 1
static void imu_task(void *pvParameters) {
    int64_t last_time_us = esp_timer_get_time();

    while (1) {
        // Wait for deterministic 200 Hz (5 ms) timer notification
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        int64_t now_us = esp_timer_get_time();
        float dt = (float)(now_us - last_time_us) / 1000000.0f;
        last_time_us = now_us;

        // Guard against abnormal dt (e.g. after long debug delay)
        if (dt <= 0.0001f || dt > 0.05f) {
            dt = 0.005f; // Fallback to 5 ms (200 Hz)
        }

        read_cycle_count++;
        if (read_cycle_count >= HEALTH_CHECK_INTERVAL) {
            read_cycle_count = 0;
            if (bmi160_check_who_am_i() != ESP_OK) {
                ESP_LOGW(TAG, "Periodic WHO_AM_I check failed");
            }
        }

        int16_t gx_raw, gy_raw, gz_raw, ax_raw, ay_raw, az_raw;
        esp_err_t err = read_motion_burst_with_retry(&gx_raw, &gy_raw, &gz_raw, &ax_raw, &ay_raw, &az_raw);

        if (err != ESP_OK) {
            consecutive_failures++;
            if (consecutive_failures >= RECOVERY_FAIL_THRESHOLD) {
                imu_recover();
            }
            portENTER_CRITICAL(&s_imu_mux);
            s_latest_snapshot.sensor_ok = false;
            portEXIT_CRITICAL(&s_imu_mux);
            continue;
        }

        consecutive_failures = 0;

        // Convert raw values to physical units
        float gx_dps = rawToDPS(gx_raw);
        float gy_dps = rawToDPS(gy_raw);
        (void)gz_raw; // Z gyro rate reserved for future yaw estimation

        float accX_g = rawToG(ax_raw);
        float accY_g = rawToG(ay_raw);
        float accZ_g = rawToG(az_raw);

        // Compute live pitch rate with resting bias removed
        float raw_pitch_rate = (PITCH_AXIS_IS_Y ? (gy_dps - bias_gy) : (gx_dps - bias_gx));
        float pitch_rate_dps = raw_pitch_rate * PITCH_SIGN;

        // Integrate pure gyro pitch angle
        if (s_is_calibrated) {
            s_integrated_pitch_deg += pitch_rate_dps * dt;
        }

        // Accelerometer tilt angle calculations (reference / static)
        float acc_roll = atan2f(accY_g, accZ_g) * 180.0f / (float)M_PI;
        float acc_pitch = atan2f(-accX_g, sqrtf(accY_g * accY_g + accZ_g * accZ_g)) * 180.0f / (float)M_PI;

        // --- Dynamic Gravity Tilt Compensation ---
        // Expected gravity vector under current table tilt angle relative to true Earth level (0.0 deg)
        float expected_gx = s_flat_baseX;
        float expected_gy = s_flat_baseY;
        float expected_gz = s_flat_baseZ;

        if (s_is_calibrated) {
            float pitch_rad = s_integrated_pitch_deg * ((float)M_PI / 180.0f);
            float cos_p = cosf(pitch_rad);
            float sin_p = sinf(pitch_rad);

            #if PITCH_AXIS_IS_Y
            expected_gx = (s_flat_baseX * cos_p) + (s_flat_baseZ * sin_p);
            expected_gy = s_flat_baseY;
            expected_gz = (-s_flat_baseX * sin_p) + (s_flat_baseZ * cos_p);
            #else
            // Standard pitch rotation around X-axis
            expected_gx = (s_flat_baseX * cos_p) - (s_flat_baseZ * sin_p);
            expected_gy = s_flat_baseY;
            expected_gz = (s_flat_baseX * sin_p) + (s_flat_baseZ * cos_p);
            #endif
        }

        // Subtract the expected static gravity vector: pure dynamic vibration remains
        float deltaX = accX_g - expected_gx;
        float deltaY = accY_g - expected_gy;
        float deltaZ = accZ_g - expected_gz;

        float vibration = sqrtf(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
        float vibration_ms2 = vibration * 9.80665f;
        float vibration_ms2_calibrated = (vibration_ms2 * 1.226f) + 0.145f;
        float accZ_ms2_calibrated = (accZ_g * 9.80665f * 1.226f) + 0.145f;

        float accX_ms2 = accX_g * 9.80665f;
        float accY_ms2 = accY_g * 9.80665f;
        float accZ_ms2 = accZ_g * 9.80665f;

        // Update FFT with dynamic calibrated vibration
        update_fft(vibration_ms2_calibrated);

        // Update thread-safe snapshot
        portENTER_CRITICAL(&s_imu_mux);
        s_latest_snapshot.pitch_gyro_deg = s_integrated_pitch_deg;
        s_latest_snapshot.pitch_acc_deg = acc_pitch;
        s_latest_snapshot.roll_deg = acc_roll;
        s_latest_snapshot.pitch_rate_dps = pitch_rate_dps;

        s_latest_snapshot.acc_x_ms2 = accX_ms2;
        s_latest_snapshot.acc_y_ms2 = accY_ms2;
        s_latest_snapshot.acc_z_ms2 = accZ_ms2;
        s_latest_snapshot.acc_z_ms2_calibrated = accZ_ms2_calibrated;

        s_latest_snapshot.vibration_g = vibration;
        s_latest_snapshot.vibration_ms2 = vibration_ms2;
        s_latest_snapshot.vibration_ms2_calibrated = vibration_ms2_calibrated;

        s_latest_snapshot.delta_x = deltaX;
        s_latest_snapshot.delta_y = deltaY;
        s_latest_snapshot.delta_z = deltaZ;

        s_latest_snapshot.dominant_freq_hz = dominant_freq_hz;
        s_latest_snapshot.is_calibrated = s_is_calibrated;
        s_latest_snapshot.sensor_ok = true;
        portEXIT_CRITICAL(&s_imu_mux);
    }
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

    // Create 200 Hz IMU processing task pinned to Core 1
    xTaskCreatePinnedToCore(imu_task, "imu_task", 4096, NULL, 6, &s_imu_task_handle, 1);

    // Create and start periodic 200 Hz (5000 us) hardware timer
    esp_timer_create_args_t timer_args = {};
    timer_args.callback = &imu_timer_callback;
    timer_args.arg = NULL;
    timer_args.dispatch_method = ESP_TIMER_TASK;
    timer_args.name = "imu_200hz_timer";
    timer_args.skip_unhandled_events = false;

    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &s_imu_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_imu_timer, 5000)); // 5 ms = 200 Hz

    ESP_LOGI(TAG, "BMI160 initialized with Gyroscope enabled (CHIP_ID 0x%02X)", BMI160_CHIP_ID);
    return true;
}

void imu_calibrate(void) {
    const int discardCount = 50;  // 50 * 10ms = 0.5s settling
    const int sampleCount = 350;  // 350 * 10ms = 3.5s sampling (total ~4.0s)

    printf("Calibrating IMU baseline & gyro resting bias (approx 4 sec)...\n");
    printf("Pastikan meja simulator diam dan rata...\n");

    // Phase 1: Discard initial samples while gyro analog PLL settles
    for (int i = 0; i < discardCount; i++) {
        int16_t gx, gy, gz, ax, ay, az;
        read_motion_burst_with_retry(&gx, &gy, &gz, &ax, &ay, &az);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Phase 2: Accumulate high-precision resting baseline
    float sum_ax = 0.0f, sum_ay = 0.0f, sum_az = 0.0f;
    float sum_gx = 0.0f, sum_gy = 0.0f, sum_gz = 0.0f;
    int validSamples = 0;

    for (int i = 0; i < sampleCount; i++) {
        int16_t gx_raw, gy_raw, gz_raw, ax_raw, ay_raw, az_raw;
        esp_err_t err = read_motion_burst_with_retry(&gx_raw, &gy_raw, &gz_raw, &ax_raw, &ay_raw, &az_raw);
        if (err == ESP_OK) {
            sum_ax += rawToG(ax_raw);
            sum_ay += rawToG(ay_raw);
            sum_az += rawToG(az_raw);

            sum_gx += rawToDPS(gx_raw);
            sum_gy += rawToDPS(gy_raw);
            sum_gz += rawToDPS(gz_raw);
            validSamples++;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (validSamples == 0) {
        ESP_LOGE(TAG, "Calibration failed: no valid I2C samples");
        return;
    }

    baseX = sum_ax / (float)validSamples;
    baseY = sum_ay / (float)validSamples;
    baseZ = sum_az / (float)validSamples;

    bias_gx = sum_gx / (float)validSamples;
    bias_gy = sum_gy / (float)validSamples;
    bias_gz = sum_gz / (float)validSamples;

    // Calculate initial resting pitch angle from baseline accelerometer gravity vector
    s_initial_pitch_deg = atan2f(-baseX, sqrtf(baseY * baseY + baseZ * baseZ)) * 180.0f / (float)M_PI;
    s_integrated_pitch_deg = s_initial_pitch_deg;

    // De-rotate measured gravity vector to true Earth horizontal level (0.0 deg)
    // so vibration baseline is referenced to true level even if booted while tilted
    float boot_pitch_rad = s_initial_pitch_deg * ((float)M_PI / 180.0f);
    float cos_boot = cosf(-boot_pitch_rad);
    float sin_boot = sinf(-boot_pitch_rad);

    #if PITCH_AXIS_IS_Y
    s_flat_baseX = (baseX * cos_boot) + (baseZ * sin_boot);
    s_flat_baseY = baseY;
    s_flat_baseZ = (-baseX * sin_boot) + (baseZ * cos_boot);
    #else
    s_flat_baseX = (baseX * cos_boot) - (baseZ * sin_boot);
    s_flat_baseY = baseY;
    s_flat_baseZ = (baseX * sin_boot) + (baseZ * cos_boot);
    #endif

    s_is_calibrated = true;

    printf("Kalibrasi selesai:\n");
    printf("Baseline Accel -> X: %.3f g | Y: %.3f g | Z: %.3f g\n", baseX, baseY, baseZ);
    printf("Level Accel Ref-> X: %.3f g | Y: %.3f g | Z: %.3f g\n", s_flat_baseX, s_flat_baseY, s_flat_baseZ);
    printf("Bias Gyro      -> X: %.2f dps | Y: %.2f dps | Z: %.2f dps\n", bias_gx, bias_gy, bias_gz);
    printf("Starting Pitch -> %.2f deg (motor will home to 0.00 deg)\n", s_initial_pitch_deg);
}

imu_snapshot_t imu_get_snapshot(void) {
    imu_snapshot_t copy;
    portENTER_CRITICAL(&s_imu_mux);
    copy = s_latest_snapshot;
    portEXIT_CRITICAL(&s_imu_mux);
    return copy;
}

void imu_standstill_realign(float alpha) {
    if (alpha <= 0.0f) return;
    if (alpha > 1.0f) alpha = 1.0f;

    portENTER_CRITICAL(&s_imu_mux);
    if (s_is_calibrated) {
        float target = s_latest_snapshot.pitch_acc_deg;
        s_integrated_pitch_deg = ((1.0f - alpha) * s_integrated_pitch_deg) + (alpha * target);
        s_latest_snapshot.pitch_gyro_deg = s_integrated_pitch_deg;
    }
    portEXIT_CRITICAL(&s_imu_mux);
}

// Backward-compatible interface
void imu_read_data(float* vibration, float* vibration_ms2, float* vibration_ms2_calibrated, float* deltaX, float* deltaY, float* deltaZ, 
                    float* accX_ms2, float* accY_ms2, float* accZ_ms2,
                    float* pitch, float* roll, float* output_freq_hz, float* accZ_ms2_calibrated) {
    imu_snapshot_t snap = imu_get_snapshot();

    if (vibration)                *vibration = snap.vibration_g;
    if (vibration_ms2)            *vibration_ms2 = snap.vibration_ms2;
    if (vibration_ms2_calibrated) *vibration_ms2_calibrated = snap.vibration_ms2_calibrated;
    if (deltaX)                   *deltaX = snap.delta_x;
    if (deltaY)                   *deltaY = snap.delta_y;
    if (deltaZ)                   *deltaZ = snap.delta_z;
    if (accX_ms2)                 *accX_ms2 = snap.acc_x_ms2;
    if (accY_ms2)                 *accY_ms2 = snap.acc_y_ms2;
    if (accZ_ms2)                 *accZ_ms2 = snap.acc_z_ms2;
    if (pitch)                    *pitch = snap.pitch_gyro_deg; // Control pitch is integrated gyro
    if (roll)                     *roll = snap.roll_deg;
    if (output_freq_hz)           *output_freq_hz = snap.dominant_freq_hz;
    if (accZ_ms2_calibrated)      *accZ_ms2_calibrated = snap.acc_z_ms2_calibrated;
}
