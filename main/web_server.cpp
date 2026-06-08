#include <esp_http_server.h>
#include <esp_log.h>
#include <stdio.h>
#include "constant.h"

static const char *TAG = "web_server";

extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");
extern const uint8_t style_css_start[]  asm("_binary_style_css_start");
extern const uint8_t style_css_end[]    asm("_binary_style_css_end");
extern const uint8_t script_js_start[]  asm("_binary_script_js_start");
extern const uint8_t script_js_end[]    asm("_binary_script_js_end");

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

static esp_err_t data_handler(httpd_req_t *req)
{
    char resp_str[256];
    snprintf(resp_str, sizeof(resp_str), "{\"vibration_g\": %.3f, \"calibrated_g\": %.3f, \"calibrated_ms2\": %.3f, \"pot_raw\": %d, \"pwm_value\": %d}", g_vibration_g, g_calibrated_g, g_calibrated_ms2, g_pot_raw, g_pwm_value);
            
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

void start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &uri_index);
        httpd_register_uri_handler(server, &uri_style);
        httpd_register_uri_handler(server, &uri_script);
        httpd_register_uri_handler(server, &uri_data);
        
        ESP_LOGI(TAG, "Web server started on port %d", config.server_port);
    } else {
        ESP_LOGI(TAG, "Error starting server!");
    }
}