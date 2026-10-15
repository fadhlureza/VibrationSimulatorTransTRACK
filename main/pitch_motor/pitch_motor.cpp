#include "pitch_motor.h"
#include "constant.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

void pitch_motor_init() {
    // Configure Enable Pins as outputs
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << PITCH_R_EN_PIN) | (1ULL << PITCH_L_EN_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    
    gpio_set_level((gpio_num_t)PITCH_R_EN_PIN, 1);
    gpio_set_level((gpio_num_t)PITCH_L_EN_PIN, 1);

    // Configure LEDC Timer 1 (20 kHz, 10-bit resolution)
    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode       = LEDC_MODE;
    ledc_timer.timer_num        = LEDC_PITCH_TIMER;
    ledc_timer.duty_resolution  = LEDC_PITCH_DUTY_RES;
    ledc_timer.freq_hz          = LEDC_PITCH_FREQUENCY;  
    ledc_timer.clk_cfg          = LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_timer);

    // Configure RPWM Channel (LEDC Channel 2)
    ledc_channel_config_t ledc_ch_r = {};
    ledc_ch_r.speed_mode     = LEDC_MODE;
    ledc_ch_r.channel        = LEDC_PITCH_R_CH;
    ledc_ch_r.timer_sel      = LEDC_PITCH_TIMER;
    ledc_ch_r.intr_type      = LEDC_INTR_DISABLE;
    ledc_ch_r.gpio_num       = PITCH_RPWM_PIN;
    ledc_ch_r.duty           = 0;
    ledc_ch_r.hpoint         = 0;
    ledc_channel_config(&ledc_ch_r);

    // Configure LPWM Channel (LEDC Channel 3)
    ledc_channel_config_t ledc_ch_l = {};
    ledc_ch_l.speed_mode     = LEDC_MODE;
    ledc_ch_l.channel        = LEDC_PITCH_L_CH;
    ledc_ch_l.timer_sel      = LEDC_PITCH_TIMER;
    ledc_ch_l.intr_type      = LEDC_INTR_DISABLE;
    ledc_ch_l.gpio_num       = PITCH_LPWM_PIN;
    ledc_ch_l.duty           = 0;
    ledc_ch_l.hpoint         = 0;
    ledc_channel_config(&ledc_ch_l);

    pitch_motor_brake();
}

void pitch_motor_brake() {
    // Active low-side dynamic brake: both enable pins HIGH, both PWM duties 0.
    // Grounds both motor leads through the lower MOSFETs of BTS7960 half-bridges.
    gpio_set_level((gpio_num_t)PITCH_R_EN_PIN, 1);
    gpio_set_level((gpio_num_t)PITCH_L_EN_PIN, 1);

    ledc_set_duty(LEDC_MODE, LEDC_PITCH_R_CH, 0);
    ledc_update_duty(LEDC_MODE, LEDC_PITCH_R_CH);
    ledc_set_duty(LEDC_MODE, LEDC_PITCH_L_CH, 0);
    ledc_update_duty(LEDC_MODE, LEDC_PITCH_L_CH);
}

void pitch_motor_drive(int duty) {
    if (duty > PITCH_PWM_MAX_DRIVE) duty = PITCH_PWM_MAX_DRIVE;
    if (duty < -PITCH_PWM_MAX_DRIVE) duty = -PITCH_PWM_MAX_DRIVE;

    if (duty > 0) {
        gpio_set_level((gpio_num_t)PITCH_R_EN_PIN, 1);
        gpio_set_level((gpio_num_t)PITCH_L_EN_PIN, 1);

        ledc_set_duty(LEDC_MODE, LEDC_PITCH_R_CH, duty);
        ledc_update_duty(LEDC_MODE, LEDC_PITCH_R_CH);
        ledc_set_duty(LEDC_MODE, LEDC_PITCH_L_CH, 0);
        ledc_update_duty(LEDC_MODE, LEDC_PITCH_L_CH);
    } else if (duty < 0) {
        gpio_set_level((gpio_num_t)PITCH_R_EN_PIN, 1);
        gpio_set_level((gpio_num_t)PITCH_L_EN_PIN, 1);

        ledc_set_duty(LEDC_MODE, LEDC_PITCH_R_CH, 0);
        ledc_update_duty(LEDC_MODE, LEDC_PITCH_R_CH);
        ledc_set_duty(LEDC_MODE, LEDC_PITCH_L_CH, -duty);
        ledc_update_duty(LEDC_MODE, LEDC_PITCH_L_CH);
    } else {
        pitch_motor_brake();
    }
}

void pitch_motor_stop() {
    pitch_motor_brake();
}
