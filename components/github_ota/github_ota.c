#include "github_ota.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "esp_crt_bundle.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"

#define TAG "GITHUB_OTA"

#ifndef GITHUB_OTA_ENABLE
#define GITHUB_OTA_ENABLE 1
#endif

#ifndef GITHUB_OTA_CURRENT_BUILD
#define GITHUB_OTA_CURRENT_BUILD 2
#endif

#ifndef GITHUB_OTA_VERSION_URL
#define GITHUB_OTA_VERSION_URL \
    "https://raw.githubusercontent.com/kenhkythuat/monitor-water-flow/main/releases/esp32s3/version.json"
#endif

#ifndef GITHUB_OTA_DEFAULT_FIRMWARE_URL
#define GITHUB_OTA_DEFAULT_FIRMWARE_URL \
    "https://raw.githubusercontent.com/kenhkythuat/monitor-water-flow/main/releases/esp32s3/gateway_charging_station.bin"
#endif

#define GITHUB_OTA_EXPECTED_TARGET "esp32s3"
#define GITHUB_OTA_EXPECTED_PROJECT "gateway_apartment"
#define GITHUB_OTA_CHECK_INTERVAL_MS 30000
#define GITHUB_OTA_FIRST_CHECK_DELAY_MS 5000
#define GITHUB_OTA_HTTP_TIMEOUT_MS 10000
#define GITHUB_OTA_VERSION_JSON_MAX_SIZE 1024
#define GITHUB_OTA_URL_MAX_SIZE 256
#define GITHUB_OTA_CHECK_TASK_STACK_SIZE 6144
#define GITHUB_OTA_UPDATE_TASK_STACK_SIZE 8192
#define GITHUB_OTA_NVS_NAMESPACE "storage"
#define GITHUB_OTA_NVS_BUILD_KEY "ota_build"

static volatile bool s_ota_in_progress;
static volatile bool s_ota_version_check_started;
static uint32_t s_pending_build;

static uint32_t github_ota_get_current_build(void)
{
    uint32_t build = GITHUB_OTA_CURRENT_BUILD;
    nvs_handle_t nvs = 0;

    if (nvs_open(GITHUB_OTA_NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
        uint32_t saved_build = 0;
        if (nvs_get_u32(nvs, GITHUB_OTA_NVS_BUILD_KEY, &saved_build) == ESP_OK &&
            saved_build > build) {
            build = saved_build;
        }
        nvs_close(nvs);
    }

    return build;
}

static void github_ota_set_current_build(uint32_t build)
{
    nvs_handle_t nvs = 0;

    if (nvs_open(GITHUB_OTA_NVS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK) {
        ESP_LOGW(TAG, "Cannot open NVS to save OTA build");
        return;
    }

    if (nvs_set_u32(nvs, GITHUB_OTA_NVS_BUILD_KEY, build) == ESP_OK) {
        esp_err_t err = nvs_commit(nvs);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Cannot commit OTA build: %s", esp_err_to_name(err));
        }
    }
    nvs_close(nvs);
}

void github_ota_mark_app_valid_after_boot(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;

    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK &&
        ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
        esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Running OTA app marked valid");
        } else {
            ESP_LOGE(TAG, "Failed to mark OTA app valid: %s", esp_err_to_name(err));
        }
    }
}

static esp_err_t github_ota_validate_image_header(esp_app_desc_t *new_app_info)
{
    if (new_app_info == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_app_desc_t running_app_info;
    if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK) {
        ESP_LOGI(TAG, "Running firmware version: %s", running_app_info.version);
    }

    ESP_LOGI(TAG, "New firmware version: %s", new_app_info->version);
    return ESP_OK;
}

static void github_ota_update_task(void *arg)
{
    char *firmware_url = (char *)arg;
    esp_https_ota_handle_t ota_handle = NULL;
    esp_err_t ota_finish_err = ESP_OK;
    esp_err_t err = ESP_OK;

    if (firmware_url == NULL || firmware_url[0] == '\0') {
        s_ota_in_progress = false;
        free(firmware_url);
        vTaskDelete(NULL);
        return;
    }

    esp_http_client_config_t http_config = {
        .url = firmware_url,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = GITHUB_OTA_HTTP_TIMEOUT_MS,
        .keep_alive_enable = true,
    };
    esp_https_ota_config_t ota_config = {
        .http_config = &http_config,
    };

    ESP_LOGI(TAG, "Starting OTA from URL: %s", firmware_url);
    err = esp_https_ota_begin(&ota_config, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_https_ota_begin failed: %s", esp_err_to_name(err));
        goto fail_no_abort;
    }

    esp_app_desc_t app_desc;
    err = esp_https_ota_get_img_desc(ota_handle, &app_desc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_https_ota_get_img_desc failed: %s", esp_err_to_name(err));
        goto fail;
    }

    err = github_ota_validate_image_header(&app_desc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Image validation failed: %s", esp_err_to_name(err));
        goto fail;
    }

    while (1) {
        err = esp_https_ota_perform(ota_handle);
        if (err != ESP_ERR_HTTPS_OTA_IN_PROGRESS) {
            break;
        }
        ESP_LOGD(TAG, "OTA bytes read: %d", esp_https_ota_get_image_len_read(ota_handle));
    }

    if (!esp_https_ota_is_complete_data_received(ota_handle)) {
        ESP_LOGE(TAG, "Complete OTA image was not received");
        goto fail;
    }

    ota_finish_err = esp_https_ota_finish(ota_handle);
    ota_handle = NULL;
    if (err == ESP_OK && ota_finish_err == ESP_OK) {
        ESP_LOGI(TAG, "OTA successful. build=%lu. Rebooting...",
                 (unsigned long)s_pending_build);
        if (s_pending_build > 0) {
            github_ota_set_current_build(s_pending_build);
        }
        free(firmware_url);
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }

    ESP_LOGE(TAG, "OTA failed: perform=%s finish=0x%x",
             esp_err_to_name(err),
             ota_finish_err);

fail:
    if (ota_handle != NULL) {
        esp_https_ota_abort(ota_handle);
    }
fail_no_abort:
    free(firmware_url);
    s_ota_in_progress = false;
    vTaskDelete(NULL);
}

void github_ota_update(const char *url)
{
#if GITHUB_OTA_ENABLE
    if (url == NULL || url[0] == '\0') {
        ESP_LOGE(TAG, "Invalid OTA URL");
        return;
    }

    if (s_ota_in_progress) {
        ESP_LOGW(TAG, "OTA already in progress");
        return;
    }

    char *task_url = (char *)malloc(GITHUB_OTA_URL_MAX_SIZE);
    if (task_url == NULL) {
        ESP_LOGE(TAG, "Cannot allocate OTA URL");
        return;
    }

    strlcpy(task_url, url, GITHUB_OTA_URL_MAX_SIZE);
    s_ota_in_progress = true;

    BaseType_t ok = xTaskCreate(github_ota_update_task,
                                "github_ota_update",
                                GITHUB_OTA_UPDATE_TASK_STACK_SIZE,
                                task_url,
                                5,
                                NULL);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Cannot create OTA update task");
        s_ota_in_progress = false;
        free(task_url);
    }
#else
    (void)url;
#endif
}

static esp_err_t github_ota_fetch_version_json(char *json_buf, size_t json_buf_size)
{
    if (json_buf == NULL || json_buf_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_http_client_config_t config = {
        .url = GITHUB_OTA_VERSION_URL,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = GITHUB_OTA_HTTP_TIMEOUT_MS,
        .keep_alive_enable = true,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        return ESP_FAIL;
    }

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Cannot open version.json: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return err;
    }

    (void)esp_http_client_fetch_headers(client);

    int total_read = 0;
    while (total_read < (int)json_buf_size - 1) {
        int read_len = esp_http_client_read(client,
                                            json_buf + total_read,
                                            (int)json_buf_size - 1 - total_read);
        if (read_len <= 0) {
            break;
        }
        total_read += read_len;
    }
    json_buf[total_read] = '\0';

    int status_code = esp_http_client_get_status_code(client);
    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    if (status_code != 200 || total_read == 0) {
        ESP_LOGE(TAG, "Invalid version.json response: status=%d bytes=%d",
                 status_code,
                 total_read);
        return ESP_FAIL;
    }

    return ESP_OK;
}

static bool github_ota_parse_version_json(const char *json,
                                          uint32_t *build_out,
                                          char *firmware_url_out,
                                          size_t firmware_url_size)
{
    bool ok = false;
    cJSON *root = cJSON_Parse(json);
    if (root == NULL) {
        ESP_LOGE(TAG, "Cannot parse version.json");
        return false;
    }

    const cJSON *build_item = cJSON_GetObjectItemCaseSensitive(root, "build");
    const cJSON *url_item = cJSON_GetObjectItemCaseSensitive(root, "firmware_url");
    const cJSON *version_item = cJSON_GetObjectItemCaseSensitive(root, "version");
    const cJSON *target_item = cJSON_GetObjectItemCaseSensitive(root, "target");
    const cJSON *project_item = cJSON_GetObjectItemCaseSensitive(root, "project");

    if (!cJSON_IsNumber(build_item) || build_item->valuedouble < 0) {
        ESP_LOGE(TAG, "version.json missing valid build");
        goto done;
    }

    if (cJSON_IsString(target_item) && target_item->valuestring != NULL &&
        strcmp(target_item->valuestring, GITHUB_OTA_EXPECTED_TARGET) != 0) {
        ESP_LOGE(TAG, "version.json target mismatch: %s", target_item->valuestring);
        goto done;
    }

    if (cJSON_IsString(project_item) && project_item->valuestring != NULL &&
        strcmp(project_item->valuestring, GITHUB_OTA_EXPECTED_PROJECT) != 0 &&
        strcmp(project_item->valuestring, "gateway_charging_station") != 0) {
        ESP_LOGE(TAG, "version.json project mismatch: %s", project_item->valuestring);
        goto done;
    }

    if (cJSON_IsString(url_item) && url_item->valuestring != NULL &&
        url_item->valuestring[0] != '\0') {
        strlcpy(firmware_url_out, url_item->valuestring, firmware_url_size);
    } else {
        strlcpy(firmware_url_out, GITHUB_OTA_DEFAULT_FIRMWARE_URL, firmware_url_size);
    }

    *build_out = (uint32_t)build_item->valuedouble;
    ESP_LOGI(TAG, "GitHub version.json: version=%s build=%lu url=%s",
             cJSON_IsString(version_item) ? version_item->valuestring : "unknown",
             (unsigned long)*build_out,
             firmware_url_out);
    ok = true;

done:
    cJSON_Delete(root);
    return ok;
}

static void github_ota_version_check_task(void *arg)
{
    (void)arg;
    char version_json[GITHUB_OTA_VERSION_JSON_MAX_SIZE];
    char firmware_url[GITHUB_OTA_URL_MAX_SIZE];

    vTaskDelay(pdMS_TO_TICKS(GITHUB_OTA_FIRST_CHECK_DELAY_MS));

    for (;;) {
        if (!s_ota_in_progress) {
            uint32_t remote_build = 0;
            uint32_t current_build = github_ota_get_current_build();

            if (github_ota_fetch_version_json(version_json, sizeof(version_json)) == ESP_OK &&
                github_ota_parse_version_json(version_json,
                                              &remote_build,
                                              firmware_url,
                                              sizeof(firmware_url))) {
                if (remote_build > current_build) {
                    ESP_LOGW(TAG, "New firmware available: current=%lu remote=%lu",
                             (unsigned long)current_build,
                             (unsigned long)remote_build);
                    s_pending_build = remote_build;
                    github_ota_update(firmware_url);
                } else {
                    ESP_LOGI(TAG, "Firmware is up to date: current=%lu remote=%lu",
                             (unsigned long)current_build,
                             (unsigned long)remote_build);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(GITHUB_OTA_CHECK_INTERVAL_MS));
    }
}

esp_err_t github_ota_start_version_check(void)
{
#if GITHUB_OTA_ENABLE
    if (s_ota_version_check_started) {
        return ESP_OK;
    }

    s_ota_version_check_started = true;
    BaseType_t ok = xTaskCreate(github_ota_version_check_task,
                                "github_ota_check",
                                GITHUB_OTA_CHECK_TASK_STACK_SIZE,
                                NULL,
                                4,
                                NULL);
    if (ok != pdPASS) {
        s_ota_version_check_started = false;
        ESP_LOGE(TAG, "Cannot create OTA check task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "GitHub OTA check started, interval=%d ms, url=%s",
             GITHUB_OTA_CHECK_INTERVAL_MS,
             GITHUB_OTA_VERSION_URL);
    return ESP_OK;
#else
    ESP_LOGI(TAG, "GitHub OTA disabled");
    return ESP_OK;
#endif
}
