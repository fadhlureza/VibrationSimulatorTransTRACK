#include "button.h"
#include "esp_attr.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#define START_BTN_PIN  GPIO_NUM_35
#define STOP_BTN_PIN   GPIO_NUM_36
#define DIR_BTN_PIN    GPIO_NUM_37

volatile bool start_btn_flag = false;
volatile bool stop_btn_flag = false;
volatile bool dir_btn_flag = false;

volatile int64_t last_isr_time = 0;

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t)(uintptr_t) arg;
    int64_t current_time = esp_timer_get_time();
    
    if (current_time - last_isr_time > 250000) {
        if (gpio_num == START_BTN_PIN) {
            start_btn_flag = true;
        } else if (gpio_num == STOP_BTN_PIN) {
            stop_btn_flag = true;
        } else if (gpio_num == DIR_BTN_PIN) {
            dir_btn_flag = true;
        }
        last_isr_time = current_time;
    }
}

void button_init() {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << START_BTN_PIN) | (1ULL << STOP_BTN_PIN) | (1ULL << DIR_BTN_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    
    gpio_isr_handler_add((gpio_num_t)START_BTN_PIN, gpio_isr_handler, (void*) START_BTN_PIN);
    gpio_isr_handler_add((gpio_num_t)STOP_BTN_PIN, gpio_isr_handler, (void*) STOP_BTN_PIN);
    gpio_isr_handler_add((gpio_num_t)DIR_BTN_PIN, gpio_isr_handler, (void*) DIR_BTN_PIN);
}