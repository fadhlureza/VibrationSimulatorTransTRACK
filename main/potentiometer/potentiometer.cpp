#include "constant.h"
#include "esp_adc/adc_oneshot.h"

static adc_oneshot_unit_handle_t adc1_handle;

void pot_init() {
    adc_oneshot_unit_init_cfg_t init_config1 = {};
    init_config1.unit_id = POT_ADC_UNIT;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {};
    config.bitwidth = ADC_BITWIDTH_DEFAULT;
    config.atten = ADC_ATTEN_DB_12; 
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, POT_ADC_CHANNEL, &config));
}

int pot_read_pwm() {
    int potValue = 0;
    adc_oneshot_read(adc1_handle, POT_ADC_CHANNEL, &potValue);
    
    int pwmValue = (potValue * 255) / 4095;
    if(pwmValue > 255) pwmValue = 255;

    g_pot_raw = potValue;
    g_pwm_value = pwmValue;
    
    return pwmValue;
}