#include <esp_http_server.h>
#include <esp_log.h>
#include <stdio.h>
#include "constant.h"
#include "cJSON.h"
#include "panel/panel.h"
#include "pitch_motor/pitch_control.h"
#include "imu/imu.h"

static const char *TAG = "web_server";

extern volatile float g_active_kp;
extern volatile float g_active_ki;
extern volatile float g_active_kd;
extern volatile bool g_pid_update_flag;

extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");
extern const uint8_t style_css_start[]  asm("_binary_style_css_start");
extern const uint8_t style_css_end[]    asm("_binary_style_css_end");
extern const uint8_t script_js_start[]  asm("_binary_script_js_start");
extern const uint8_t script_js_end[]    asm("_binary_script_js_end");
extern const uint8_t full_logo_png_start[] asm("_binary_full_logo_png_start");
extern const uint8_t full_logo_png_end[]   asm("_binary_full_logo_png_end");

static esp_err_t index_html_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    const size_t index_html_size = (index_html_end - index_html_start);
    httpd_resp_send(req, (const char *)index_html_start, index_html_size);
    return ESP_OK;
}

static esp_err_t style_css_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/css");
    const size_t style_css_size = (style_css_end - style_css_start);
    httpd_resp_send(req, (const char *)style_css_start, style_css_size);
    return ESP_OK;
}

static esp_err_t script_js_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/javascript");
    const size_t script_js_size = (script_js_end - script_js_start);
    httpd_resp_send(req, (const char *)script_js_start, script_js_size);
    return ESP_OK;
}

static esp_err_t full_logo_png_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "image/png");
    httpd_resp_set_hdr(req, "Cache-Control", "public, max-age=86400");
    const size_t logo_size = (full_logo_png_end - full_logo_png_start);
    httpd_resp_send(req, (const char *)full_logo_png_start, logo_size);
    return ESP_OK;
}

static esp_err_t data_handler(httpd_req_t *req)
{
    panel_state_t panel_st = panel_get_state();
    pitch_telemetry_t pitch_telem = pitch_control_get_telemetry();
    imu_snapshot_t imu_snap = imu_get_snapshot();

    const char* mode_names[] = {"Stabilized", "Interval", "Full Manual"};
    const char* mode_str = (panel_st.mode <= MODE_MANUAL) ? mode_names[panel_st.mode] : "Unknown";

    char resp_str[768];
    snprintf(resp_str, sizeof(resp_str),
        "{"
        "\"target_g\": %.3f,"
        "\"vibration_g\": %.3f,"
        "\"calibrated_g\": %.3f,"
        "\"calibrated_ms2\": %.3f,"
        "\"dominant_freq_hz\": %.2f,"
        "\"pwm_value\": %d,"
        "\"sys_state\": \"%s\","
        "\"is_homed\": %d,"
        "\"mode\": %d,"
        "\"mode_str\": \"%s\","
        "\"dir\": \"%s\","
        "\"pitch_target\": %.2f,"
        "\"pitch_slew\": %.2f,"
        "\"pitch_angle\": %.2f,"
        "\"pitch_acc\": %.2f,"
        "\"pitch_pwm\": %d,"
        "\"pot0_g\": %.3f,"
        "\"pot1_deg\": %.2f,"
        "\"pot2_s\": %.1f,"
        "\"dwell_rem\": %.1f,"
        "\"dwell_total\": %.1f,"
        "\"imu_calibrated\": %d,"
        "\"vib_safety\": %d,"
        "\"vib_safety_max_g\": %.3f"
        "}",
        g_target_g, g_vibration_g, g_calibrated_g, g_calibrated_ms2, g_dominant_freq_hz, g_pwm_value,
        pitch_control_get_state_str(),
        pitch_telem.is_homed ? 1 : 0,
        (int)panel_st.mode,
        mode_str,
        (panel_st.dir == DIR_FORWARD) ? "FWD" : "REV",
        pitch_telem.target_pitch_deg,
        pitch_telem.slew_pitch_deg,
        pitch_telem.current_pitch_deg,
        imu_snap.pitch_acc_deg,
        pitch_telem.motor_pwm,
        panel_st.raw_pot0_g,
        panel_st.raw_pot1_deg,
        panel_st.raw_pot2_s,
        pitch_telem.dwell_remaining_s,
        pitch_telem.dwell_total_s,
        imu_snap.is_calibrated ? 1 : 0,
        VIB_SAFETY ? 1 : 0,
        VIB_SAFETY_MAX_G
    );
            
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t update_pid_handler(httpd_req_t *req)
{
    char buf[128];
    int ret, remaining = req->content_len;

    if (remaining >= sizeof(buf)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Payload too large");
        return ESP_FAIL;
    }

    if ((ret = httpd_req_recv(req, buf, remaining)) <= 0) {
        return ESP_FAIL;
    }
    buf[ret] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (root == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON *kp_json = cJSON_GetObjectItem(root, "kp");
    cJSON *ki_json = cJSON_GetObjectItem(root, "ki");
    cJSON *kd_json = cJSON_GetObjectItem(root, "kd");

    if (kp_json && ki_json && kd_json) {
        g_active_kp = (float)kp_json->valuedouble;
        g_active_ki = (float)ki_json->valuedouble;
        g_active_kd = (float)kd_json->valuedouble;
        g_pid_update_flag = true;
    }

    cJSON_Delete(root);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\": \"Vibration PID coefficients updated\"}");
    return ESP_OK;
}

static esp_err_t update_pitch_pid_handler(httpd_req_t *req)
{
    char buf[128];
    int ret, remaining = req->content_len;

    if (remaining >= sizeof(buf)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Payload too large");
        return ESP_FAIL;
    }

    if ((ret = httpd_req_recv(req, buf, remaining)) <= 0) {
        return ESP_FAIL;
    }
    buf[ret] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (root == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON *kp_json = cJSON_GetObjectItem(root, "kp");
    cJSON *ki_json = cJSON_GetObjectItem(root, "ki");
    cJSON *kd_json = cJSON_GetObjectItem(root, "kd");

    if (kp_json && ki_json && kd_json) {
        float kp = (float)kp_json->valuedouble;
        float ki = (float)ki_json->valuedouble;
        float kd = (float)kd_json->valuedouble;
        pitch_control_update_pid(kp, ki, kd);
    }

    cJSON_Delete(root);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\": \"Pitch PID coefficients updated\"}");
    return ESP_OK;
}

static esp_err_t get_pids_handler(httpd_req_t *req)
{
    float pitch_kp = 0, pitch_ki = 0, pitch_kd = 0;
    pitch_control_get_pid(&pitch_kp, &pitch_ki, &pitch_kd);

    char resp_str[192];
    snprintf(resp_str, sizeof(resp_str),
        "{\"vib_kp\": %.4f, \"vib_ki\": %.4f, \"vib_kd\": %.4f, \"pitch_kp\": %.2f, \"pitch_ki\": %.2f, \"pitch_kd\": %.2f}",
        g_active_kp, g_active_ki, g_active_kd, pitch_kp, pitch_ki, pitch_kd);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static const httpd_uri_t uri_index = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = index_html_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t uri_style = {
    .uri       = "/style.css",
    .method    = HTTP_GET,
    .handler   = style_css_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t uri_script = {
    .uri       = "/script.js",
    .method    = HTTP_GET,
    .handler   = script_js_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t uri_data = {
    .uri       = "/api/data",
    .method    = HTTP_GET,
    .handler   = data_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t uri_update_pid = {
    .uri       = "/api/update_pid",
    .method    = HTTP_POST,
    .handler   = update_pid_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t uri_update_pitch_pid = {
    .uri       = "/api/update_pitch_pid",
    .method    = HTTP_POST,
    .handler   = update_pitch_pid_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t uri_get_pids = {
    .uri       = "/api/pids",
    .method    = HTTP_GET,
    .handler   = get_pids_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t uri_logo = {
    .uri       = "/full-logo.png",
    .method    = HTTP_GET,
    .handler   = full_logo_png_handler,
    .user_ctx  = NULL
};

void start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.max_uri_handlers = 12;
    
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &uri_index);
        httpd_register_uri_handler(server, &uri_style);
        httpd_register_uri_handler(server, &uri_script);
        httpd_register_uri_handler(server, &uri_logo);
        httpd_register_uri_handler(server, &uri_data);
        httpd_register_uri_handler(server, &uri_update_pid);
        httpd_register_uri_handler(server, &uri_update_pitch_pid);
        httpd_register_uri_handler(server, &uri_get_pids);
        
        ESP_LOGI(TAG, "Web server started on port %d", config.server_port);
    } else {
        ESP_LOGI(TAG, "Error starting server!");
    }
}