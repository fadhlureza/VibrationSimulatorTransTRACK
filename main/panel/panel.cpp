#include "panel.h"
#include "constant.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

static const char *TAG = "PANEL";

static adc_oneshot_unit_handle_t s_adc1_handle = NULL;
static panel_state_t s_panel_state = {};
static portMUX_TYPE s_panel_mux = portMUX_INITIALIZER_UNLOCKED;

// Debounce settings
static const int BTN_POLL_MS = 50;
static const int BTN_HOLD_COUNT = 40; // 40 * 50ms = 2000ms (2.0s hold time, as in original firmware)

// Exponential filter weight for analog smoothing
static const float ADC_EMA_ALPHA = 0.25f;

static void panel_task(void *pvParameters) {
    int start_cnt = 0;
    int stop_cnt = 0;
    bool start_latched = false;
    bool stop_latched = false;

    float filtered_pot0 = 0.0f;
    float filtered_pot1 = 0.0f;
    float filtered_pot2 = 0.0f;
    bool first_adc_read = true;

    while (1) {
        // --- 1. Debounce START Button (Active Low) ---
        if (gpio_get_level(START_BTN_PIN) == 0) {
            if (!start_latched) {
                start_cnt++;
                if (start_cnt >= BTN_HOLD_COUNT) {
                    portENTER_CRITICAL(&s_panel_mux);
                    s_panel_state.start_pressed = true;
                    portEXIT_CRITICAL(&s_panel_mux);
                    start_latched = true;
                    ESP_LOGI(TAG, "START button pressed");
                }
            }
        } else {
            start_cnt = 0;
            start_latched = false;
        }

        // --- 2. Debounce STOP Button (Active Low) ---
        if (gpio_get_level(STOP_BTN_PIN) == 0) {
            if (!stop_latched) {
                stop_cnt++;
                if (stop_cnt >= BTN_HOLD_COUNT) {
                    portENTER_CRITICAL(&s_panel_mux);
                    s_panel_state.stop_pressed = true;
                    portEXIT_CRITICAL(&s_panel_mux);
                    stop_latched = true;
                    ESP_LOGI(TAG, "STOP button pressed");
                }
            }
        } else {
            stop_cnt = 0;
            stop_latched = false;
        }

        // --- 3. Read DIR Switch with contact debounce ---
        static panel_dir_t s_debounced_dir = DIR_FORWARD;
        static panel_dir_t s_candidate_dir = DIR_FORWARD;
        static int s_dir_stable_cnt = 0;

        panel_dir_t raw_dir = (gpio_get_level(DIR_SWITCH_PIN) == 0) ? DIR_FORWARD : DIR_REVERSE;
        if (raw_dir == s_candidate_dir) {
            if (++s_dir_stable_cnt >= 2) { // 2 * 50ms = 100ms stable
                s_debounced_dir = raw_dir;
            }
        } else {
            s_candidate_dir = raw_dir;
            s_dir_stable_cnt = 0;
        }
        panel_dir_t current_dir = s_debounced_dir;

        // --- 4. Read 3-Position MODE Switch with contact debounce ---
        // Pin A Low  -> Mode 0 (Stabilized)
        // Pin B Low  -> Mode 2 (Full Manual)
        // Both High  -> Mode 1 (Interval)
        static panel_mode_t s_debounced_mode = MODE_STABILIZED;
        static panel_mode_t s_candidate_mode = MODE_STABILIZED;
        static int s_mode_stable_cnt = 0;

        int mode_a = gpio_get_level(MODE_SWITCH_PIN_A);
        int mode_b = gpio_get_level(MODE_SWITCH_PIN_B);
        panel_mode_t raw_mode;
        if (mode_a == 0) {
            raw_mode = MODE_STABILIZED;
        } else if (mode_b == 0) {
            raw_mode = MODE_MANUAL;
        } else {
            raw_mode = MODE_INTERVAL;
        }

        if (raw_mode == s_candidate_mode) {
            if (++s_mode_stable_cnt >= 2) { // 2 * 50ms = 100ms stable (filters break-before-make transition)
                s_debounced_mode = raw_mode;
            }
        } else {
            s_candidate_mode = raw_mode;
            s_mode_stable_cnt = 0;
        }
        panel_mode_t current_mode = s_debounced_mode;

        // --- 5. Read Potentiometers (ADC1) ---
        int raw0 = 0, raw1 = 0, raw2 = 0;
        if (s_adc1_handle != NULL) {
            adc_oneshot_read(s_adc1_handle, POT0_ADC_CHANNEL, &raw0);
            adc_oneshot_read(s_adc1_handle, POT1_ADC_CHANNEL, &raw1);
            adc_oneshot_read(s_adc1_handle, POT2_ADC_CHANNEL, &raw2);
        }

        if (first_adc_read) {
            filtered_pot0 = (float)raw0;
            filtered_pot1 = (float)raw1;
            filtered_pot2 = (float)raw2;
            first_adc_read = false;
        } else {
            filtered_pot0 = (ADC_EMA_ALPHA * (float)raw0) + ((1.0f - ADC_EMA_ALPHA) * filtered_pot0);
            filtered_pot1 = (ADC_EMA_ALPHA * (float)raw1) + ((1.0f - ADC_EMA_ALPHA) * filtered_pot1);
            filtered_pot2 = (ADC_EMA_ALPHA * (float)raw2) + ((1.0f - ADC_EMA_ALPHA) * filtered_pot2);
        }

        // Scale raw ADC (0..4095) to physical engineering units
        float pot0_g = (filtered_pot0 * POT0_MAX_G) / 4095.0f;
        if (pot0_g < 0.0f) pot0_g = 0.0f;
        if (pot0_g > POT0_MAX_G) pot0_g = POT0_MAX_G;

        float pot1_deg = POT1_MIN_DEG + ((filtered_pot1 * (POT1_MAX_DEG - POT1_MIN_DEG)) / 4095.0f);
        if (pot1_deg < POT1_MIN_DEG) pot1_deg = POT1_MIN_DEG;
        if (pot1_deg > POT1_MAX_DEG) pot1_deg = POT1_MAX_DEG;

        float pot2_s = POT2_MIN_SEC + ((filtered_pot2 * (POT2_MAX_SEC - POT2_MIN_SEC)) / 4095.0f);
        if (pot2_s < POT2_MIN_SEC) pot2_s = POT2_MIN_SEC;
        if (pot2_s > POT2_MAX_SEC) pot2_s = POT2_MAX_SEC;

        // Legacy global sync
        g_pot_raw = raw0;

        // --- 6. Apply Per-Mode Input Masking ---
        float active_vib_g = 0.0f;
        float active_pitch_deg = 0.0f;
        float active_interval_s = 0.0f;

        switch (current_mode) {
            case MODE_STABILIZED:
                active_vib_g = pot0_g;
                active_pitch_deg = 0.0f;
                active_interval_s = 0.0f;
                break;

            case MODE_INTERVAL:
                active_vib_g = MODE1_LOCKED_G; // Locked at 1.000 g
                active_pitch_deg = pot1_deg;
                active_interval_s = pot2_s;
                break;

            case MODE_MANUAL:
                active_vib_g = pot0_g;
                active_pitch_deg = pot1_deg;
                active_interval_s = 0.0f;      // Disabled (no cycle)
                break;
        }

        // --- 7. Commit state atomically ---
        portENTER_CRITICAL(&s_panel_mux);
        s_panel_state.mode = current_mode;
        s_panel_state.dir = current_dir;
        s_panel_state.vib_target_g = active_vib_g;
        s_panel_state.pitch_angle_deg = active_pitch_deg;
        s_panel_state.interval_s = active_interval_s;
        s_panel_state.raw_pot0_adc = raw0;
        s_panel_state.raw_pot1_adc = raw1;
        s_panel_state.raw_pot2_adc = raw2;
        s_panel_state.raw_pot0_g = pot0_g;
        s_panel_state.raw_pot1_deg = pot1_deg;
        s_panel_state.raw_pot2_s = pot2_s;
        portEXIT_CRITICAL(&s_panel_mux);

        vTaskDelay(pdMS_TO_TICKS(BTN_POLL_MS));
    }
}

void panel_init(void) {
    // 1. Configure digital input GPIOs with pull-ups
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << START_BTN_PIN) |
                           (1ULL << STOP_BTN_PIN)  |
                           (1ULL << DIR_SWITCH_PIN) |
                           (1ULL << MODE_SWITCH_PIN_A) |
                           (1ULL << MODE_SWITCH_PIN_B);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // 2. Configure ADC1 unit for potentiometers
    adc_oneshot_unit_init_cfg_t init_config1 = {};
    init_config1.unit_id = ADC_UNIT_1;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &s_adc1_handle));

    adc_oneshot_chan_cfg_t chan_config = {};
    chan_config.bitwidth = ADC_BITWIDTH_DEFAULT;
    chan_config.atten = ADC_ATTEN_DB_12;

    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc1_handle, POT0_ADC_CHANNEL, &chan_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc1_handle, POT1_ADC_CHANNEL, &chan_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc1_handle, POT2_ADC_CHANNEL, &chan_config));

    // 3. Spawn background panel_task on Core 1
    xTaskCreatePinnedToCore(panel_task, "panel_task", 3072, NULL, 5, NULL, 1);
    ESP_LOGI(TAG, "Panel module initialized successfully");
}

panel_state_t panel_get_state(void) {
    panel_state_t copy;
    portENTER_CRITICAL(&s_panel_mux);
    copy = s_panel_state;
    portEXIT_CRITICAL(&s_panel_mux);
    return copy;
}

bool panel_check_and_clear_start(void) {
    bool pressed = false;
    portENTER_CRITICAL(&s_panel_mux);
    if (s_panel_state.start_pressed) {
        pressed = true;
        s_panel_state.start_pressed = false;
    }
    portEXIT_CRITICAL(&s_panel_mux);
    return pressed;
}

bool panel_check_and_clear_stop(void) {
    bool pressed = false;
    portENTER_CRITICAL(&s_panel_mux);
    if (s_panel_state.stop_pressed) {
        pressed = true;
        s_panel_state.stop_pressed = false;
    }
    portEXIT_CRITICAL(&s_panel_mux);
    return pressed;
}
