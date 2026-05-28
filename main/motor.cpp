#include "motor.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#define RPWM_PIN  13
#define LPWM_PIN  14
#define R_EN_PIN  11
#define L_EN_PIN  12

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_R_CHANNEL          LEDC_CHANNEL_0
#define LEDC_L_CHANNEL          LEDC_CHANNEL_1
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT
#define LEDC_FREQUENCY          1000

void motor_init() {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << R_EN_PIN) | (1ULL << L_EN_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    
    gpio_set_level((gpio_num_t)R_EN_PIN, 1);
    gpio_set_level((gpio_num_t)L_EN_PIN, 1);

    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode       = LEDC_MODE;
    ledc_timer.timer_num        = LEDC_TIMER;
    ledc_timer.duty_resolution  = LEDC_DUTY_RES;
    ledc_timer.freq_hz          = LEDC_FREQUENCY;  
    ledc_timer.clk_cfg          = LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_ch_r = {};
    ledc_ch_r.speed_mode     = LEDC_MODE;
    ledc_ch_r.channel        = LEDC_R_CHANNEL;
    ledc_ch_r.timer_sel      = LEDC_TIMER;
    ledc_ch_r.intr_type      = LEDC_INTR_DISABLE;
    ledc_ch_r.gpio_num       = RPWM_PIN;
    ledc_ch_r.duty           = 0;
    ledc_ch_r.hpoint         = 0;
    ledc_channel_config(&ledc_ch_r);

    ledc_channel_config_t ledc_ch_l = {};
    ledc_ch_l.speed_mode     = LEDC_MODE;
    ledc_ch_l.channel        = LEDC_L_CHANNEL;
    ledc_ch_l.timer_sel      = LEDC_TIMER;
    ledc_ch_l.intr_type      = LEDC_INTR_DISABLE;
    ledc_ch_l.gpio_num       = LPWM_PIN;
    ledc_ch_l.duty           = 0;
    ledc_ch_l.hpoint         = 0;
    ledc_channel_config(&ledc_ch_l);
}

void motorStop() {
    ledc_set_duty(LEDC_MODE, LEDC_R_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, LEDC_R_CHANNEL);
    ledc_set_duty(LEDC_MODE, LEDC_L_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, LEDC_L_CHANNEL);
}

void motorForward(int speedPWM) {
    ledc_set_duty(LEDC_MODE, LEDC_R_CHANNEL, speedPWM);
    ledc_update_duty(LEDC_MODE, LEDC_R_CHANNEL);
    ledc_set_duty(LEDC_MODE, LEDC_L_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, LEDC_L_CHANNEL);
}

void motorReverse(int speedPWM) {
    ledc_set_duty(LEDC_MODE, LEDC_R_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, LEDC_R_CHANNEL);
    ledc_set_duty(LEDC_MODE, LEDC_L_CHANNEL, speedPWM);
    ledc_update_duty(LEDC_MODE, LEDC_L_CHANNEL);
}
