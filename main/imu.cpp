#include "imu.h"
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

#define BMI160_ADDR 0x68
#define CMD_REG    0x7E
#define ACC_X_LSB  0x12

#define I2C_MASTER_SCL_IO           41
#define I2C_MASTER_SDA_IO           40
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000

static float baseX = 0;
static float baseY = 0;
static float baseZ = 0;

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
    return raw / 16384.0f;
}

void imu_init() {
    i2c_master_init();
    
    writeRegister(CMD_REG, 0x11);
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

void imu_read_data(float* vibration, float* vibration_ms2, float* deltaX, float* deltaY, float* deltaZ, 
                   float* accX_ms2, float* accY_ms2, float* accZ_ms2, 
                   float* pitch, float* roll) {
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
    *vibration_ms2 = (*vibration) * 9.80665;

    *roll = atan2(accY_g, accZ_g) * 180.0 / M_PI;
    *pitch = atan2(-accX_g, sqrt(accY_g * accY_g + accZ_g * accZ_g)) * 180.0 / M_PI;

    *accX_ms2 = accX_g * 9.80665;
    *accY_ms2 = accY_g * 9.80665;
    *accZ_ms2 = accZ_g * 9.80665;
}
