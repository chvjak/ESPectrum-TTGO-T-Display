/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "bt.h"

#include "nvs.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_log.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"

#include "AySound.h"

#include <vector>
#include <string>
#include <mutex>

#define BT_AV_TAG             "BT_AV"
#define LOCAL_DEVICE_NAME     "ESP_A2DP_SRC"

static esp_bd_addr_t s_peer_bda = {0};
static uint8_t s_peer_bdname[ESP_BT_GAP_MAX_BDNAME_LEN + 1];

// --- Bluetooth device discovery/connection state ---
static std::vector<BTDevice> g_discovered_devices;
static std::mutex g_bt_mutex;
static uint8_t g_connected_mac[6] = {0};
static std::string g_connected_name;

IRAM_ATTR static int32_t audio_data_cb(uint8_t *data, int32_t len)
{
    if (data == NULL || len < 0) return 0;
    int samples = len / 2; // 16-bit samples
    return AySound::getAudioSamples((int16_t*)data, samples) * 2;
}

static char *bda2str(esp_bd_addr_t bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
    return str;
}

static bool get_name_from_eir(uint8_t *eir, uint8_t *bdname, uint8_t *bdname_len)
{
    uint8_t *rmt_bdname = NULL;
    uint8_t rmt_bdname_len = 0;
    if (!eir) return false;
    rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &rmt_bdname_len);
    if (!rmt_bdname) {
        rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &rmt_bdname_len);
    }
    if (rmt_bdname) {
        if (rmt_bdname_len > ESP_BT_GAP_MAX_BDNAME_LEN) rmt_bdname_len = ESP_BT_GAP_MAX_BDNAME_LEN;
        if (bdname) {
            memcpy(bdname, rmt_bdname, rmt_bdname_len);
            bdname[rmt_bdname_len] = '\0';
        }
        if (bdname_len) *bdname_len = rmt_bdname_len;
        return true;
    }
    return false;
}

#define A2DP_SINK_UUID16 0x110B

bool device_is_a2dp_sink(uint8_t *eir) {
    if (!eir) return false;
    // Get the list of 16-bit UUIDs from EIR
    uint8_t uuid_len = 0;
    uint8_t *uuid_list = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_16BITS_UUID, &uuid_len);
    if (!uuid_list) {
        uuid_list = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_INCMPL_16BITS_UUID, &uuid_len);
    }
    if (uuid_list && uuid_len >= 2) {
        for (int i = 0; i < uuid_len; i += 2) {
            uint16_t uuid = uuid_list[i] | (uuid_list[i+1] << 8);
            if (uuid == A2DP_SINK_UUID16) {
                return true;
            }
        }
    }
    return false;
}

// --- Device discovery implementation ---
std::vector<BTDevice> bt_discover_devices() {
    g_discovered_devices.clear();

    // Start discovery (blocking, 10s)
    bt_init(); // includes start_discovery()
    // Wait for discovery to finish (polling for DISCOVERY_STOPPED)
    // In a real implementation, use an event group or callback to signal completion.
    int wait_ms = 0;
    while (wait_ms < 12000) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
        wait_ms += 100;
        // Check if discovery stopped (not implemented here, would need a flag from GAP callback)
        // For now, just wait 10s
    }
    esp_bt_gap_cancel_discovery();

    // In a real implementation, g_discovered_devices would be filled in the GAP callback.
    // Here, we return whatever was found (if any).
    return g_discovered_devices;
}

// --- Connect to device by MAC ---
bool bt_connect_device(const uint8_t mac[6]) {
    std::lock_guard<std::mutex> lock(g_bt_mutex);
    memcpy(g_connected_mac, mac, 6);
    // Try to connect (non-blocking, real connection handled by A2DP callback)
    esp_err_t err = esp_a2d_source_connect((uint8_t*)mac);
    if (err == ESP_OK) {
        // Save name if known
        for (const auto& dev : g_discovered_devices) {
            if (memcmp(dev.mac, mac, 6) == 0) {
                g_connected_name = dev.name;
                break;
            }
        }
        return true;
    }
    return false;
}

// --- Get currently connected device MAC ---
bool bt_get_connected_mac(uint8_t mac[6]) {
    std::lock_guard<std::mutex> lock(g_bt_mutex);
    if (memcmp(g_connected_mac, "\0\0\0\0\0\0", 6) == 0) return false;
    memcpy(mac, g_connected_mac, 6);
    return true;
}

// --- Get currently connected device name ---
std::string bt_get_connected_name() {
    std::lock_guard<std::mutex> lock(g_bt_mutex);
    return g_connected_name;
}

static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    if (event == ESP_BT_GAP_DISC_RES_EVT) {
        printf("ESP_BT_GAP_DISC_RES_EVT: Found device \n");
        char bda_str[18];
        uint8_t *eir = NULL;
        for (int i = 0; i < param->disc_res.num_prop; i++) {
            if (param->disc_res.prop[i].type == ESP_BT_GAP_DEV_PROP_EIR) {
                eir = (uint8_t *)(param->disc_res.prop[i].val);
            }
        }
        if (eir && device_is_a2dp_sink(eir)) {
            uint8_t bdname[ESP_BT_GAP_MAX_BDNAME_LEN + 1] = {0};
            get_name_from_eir(eir, bdname, NULL);
            BTDevice dev;
            memcpy(dev.mac, param->disc_res.bda, 6);
            dev.name = (char*)bdname;
            std::lock_guard<std::mutex> lock(g_bt_mutex);
            // Avoid duplicates
            bool found = false;
            for (const auto& d : g_discovered_devices) {
                if (memcmp(d.mac, dev.mac, 6) == 0) { found = true; break; }
            }
            if (!found) g_discovered_devices.push_back(dev);
        }
    } else if (event == ESP_BT_GAP_DISC_STATE_CHANGED_EVT) {
        if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
            printf("Discovery started\n");
            ESP_LOGI(BT_AV_TAG, "Discovery started");
        } else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
            printf("Discovery stopped\n");
            ESP_LOGI(BT_AV_TAG, "Discovery stopped");
        }
    }
}

void a2dp_event_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    esp_a2d_cb_param_t *a2d = NULL;
    char bda_str[18] = {0};
    switch (event) {
        case ESP_A2D_CONNECTION_STATE_EVT:
            ESP_LOGW(BT_AV_TAG, "A2DP connection state: %d", param->conn_stat.state);
            a2d = (esp_a2d_cb_param_t *)(param);
            switch (a2d->conn_stat.state) {
              case ESP_A2D_CONNECTION_STATE_CONNECTING:
                ESP_LOGW(BT_AV_TAG, "A2DP connecting to %s", bda2str(a2d->conn_stat.remote_bda, bda_str, sizeof(bda_str)));
                break;
              case ESP_A2D_CONNECTION_STATE_CONNECTED:
                printf("A2DP connected to %s\n", bda2str(a2d->conn_stat.remote_bda, bda_str, sizeof(bda_str)));
                esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
                ESP_LOGW(BT_AV_TAG, "A2DP connected to %s", bda2str(a2d->conn_stat.remote_bda, bda_str, sizeof(bda_str)));
                esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY);
                break;
              case ESP_A2D_CONNECTION_STATE_DISCONNECTING:
              case ESP_A2D_CONNECTION_STATE_DISCONNECTED:
                break;
            }
            break;

        case ESP_A2D_AUDIO_STATE_EVT:
            ESP_LOGW(BT_AV_TAG, "A2DP audio state: %d", param->audio_stat.state);
            break;

        case ESP_A2D_AUDIO_CFG_EVT: {
            ESP_LOGW(BT_AV_TAG, "A2DP audio config done, ready to start");
            break;
        }
        case ESP_A2D_MEDIA_CTRL_ACK_EVT:
            ESP_LOGW(BT_AV_TAG, "A2DP media control ack: cmd %d, status %d", param->media_ctrl_stat.cmd, param->media_ctrl_stat.status);
            printf("A2DP media control ack: cmd %d, status %d\n", param->media_ctrl_stat.cmd, param->media_ctrl_stat.status);
            a2d = (esp_a2d_cb_param_t *)(param);
            if (a2d->media_ctrl_stat.cmd == ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY &&
                    a2d->media_ctrl_stat.status == ESP_A2D_MEDIA_CTRL_ACK_SUCCESS) {
                ESP_LOGW(BT_AV_TAG, "a2dp media ready, starting ...");

                vTaskDelay(6000 / portTICK_PERIOD_MS); // wait for audio data to be ready
                esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_START); // should be called after emul-init
            }
            break;
        // handle all required enum values
        case ESP_A2D_PROF_STATE_EVT:
            ESP_LOGD(BT_AV_TAG, "Unhandled A2DP event: %d", event);
            break;
        default:
            ESP_LOGW(BT_AV_TAG, "Unknown A2DP event: %d", event);
            break;
    }
}

void bt_init(void)
{
    printf("Initializing Bluetooth...\n");

    char bda_str[18] = {0};
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));

    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);

    ESP_LOGI(BT_AV_TAG, "Own address:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));

    esp_bt_dev_set_device_name(LOCAL_DEVICE_NAME);
    esp_bt_gap_register_callback(bt_app_gap_cb);


    esp_avrc_rn_evt_cap_mask_t evt_set = {0};
    esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &evt_set, ESP_AVRC_RN_VOLUME_CHANGE);
    ESP_ERROR_CHECK(esp_avrc_tg_set_rn_evt_cap(&evt_set));

    ESP_ERROR_CHECK(esp_a2d_source_init());
    ESP_ERROR_CHECK(esp_a2d_register_callback(a2dp_event_cb));
    ESP_ERROR_CHECK(esp_a2d_source_register_data_callback(audio_data_cb));

    ESP_LOGI(BT_AV_TAG, "Starting device discovery...");
    printf("Starting Bluetooth discovery...\n");
    esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
}
