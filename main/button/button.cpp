#include "constant.h"
#include "button.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

volatile bool start_btn_flag = false;
volatile bool stop_btn_flag = false;
volatile bool dir_btn_flag = false;

static void button_task(void *pvParameters) {
    int start_cnt = 0;
    int stop_cnt = 0;
    int dir_cnt = 0;

    while (1) {
        if (gpio_get_level((gpio_num_t)START_BTN_PIN) == 0) start_cnt++; else start_cnt = 0;
        if (gpio_get_level((gpio_num_t)STOP_BTN_PIN) == 0) stop_cnt++; else stop_cnt = 0;
        if (gpio_get_level((gpio_num_t)DIR_BTN_PIN) == 0) dir_cnt++; else dir_cnt = 0;

        if (start_cnt == 20) { start_btn_flag = true; start_cnt = 31; }
        if (stop_cnt == 20) { stop_btn_flag = true; stop_cnt = 31; }
        if (dir_cnt == 20) { dir_btn_flag = true; dir_cnt = 31; }

        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}

void button_init() {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << START_BTN_PIN) | (1ULL << STOP_BTN_PIN) | (1ULL << DIR_BTN_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    xTaskCreatePinnedToCore(button_task, "button_task", 2048, NULL, 5, NULL, 1);
}