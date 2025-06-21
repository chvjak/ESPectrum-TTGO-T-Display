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

#define BT_AV_TAG             "BT_AV"
#define LOCAL_DEVICE_NAME     "ESP_A2DP_SRC"

static const char remote_device_name[] = "EPOS ADAPT 560";
static esp_bd_addr_t s_peer_bda = {0};
static uint8_t s_peer_bdname[ESP_BT_GAP_MAX_BDNAME_LEN + 1];

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
        if (eir) {
            get_name_from_eir(eir, s_peer_bdname, NULL);
            printf("Found device: %s", s_peer_bdname);
            if (strcmp((char *)s_peer_bdname, remote_device_name) == 0) {
                ESP_LOGI(BT_AV_TAG, "Found target device: %s", s_peer_bdname);
                memcpy(s_peer_bda, param->disc_res.bda, ESP_BD_ADDR_LEN);
                esp_bt_gap_cancel_discovery();
                esp_a2d_source_connect(s_peer_bda);
            }
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
            ESP_LOGI(BT_AV_TAG, "A2DP connection state: %d", param->conn_stat.state);
            a2d = (esp_a2d_cb_param_t *)(param);
            switch (a2d->conn_stat.state) {
              case ESP_A2D_CONNECTION_STATE_CONNECTING:
                ESP_LOGI(BT_AV_TAG, "A2DP connecting to %s", bda2str(a2d->conn_stat.remote_bda, bda_str, sizeof(bda_str)));
                break;
              case ESP_A2D_CONNECTION_STATE_CONNECTED:
                printf("A2DP connected to %s\n", bda2str(a2d->conn_stat.remote_bda, bda_str, sizeof(bda_str)));
                esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
                ESP_LOGI(BT_AV_TAG, "A2DP connected to %s", bda2str(a2d->conn_stat.remote_bda, bda_str, sizeof(bda_str)));
                esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY);
                break;
              case ESP_A2D_CONNECTION_STATE_DISCONNECTING:
              case ESP_A2D_CONNECTION_STATE_DISCONNECTED:
                break;
            }
            break;

        case ESP_A2D_AUDIO_STATE_EVT:
            ESP_LOGI(BT_AV_TAG, "A2DP audio state: %d", param->audio_stat.state);
            break;

        case ESP_A2D_AUDIO_CFG_EVT: {
            ESP_LOGI(BT_AV_TAG, "A2DP audio config done, ready to start");
            break;
        }
        case ESP_A2D_MEDIA_CTRL_ACK_EVT:
            ESP_LOGI(BT_AV_TAG, "A2DP media control ack: cmd %d, status %d", param->media_ctrl_stat.cmd, param->media_ctrl_stat.status);
            printf("A2DP media control ack: cmd %d, status %d\n", param->media_ctrl_stat.cmd, param->media_ctrl_stat.status);
            a2d = (esp_a2d_cb_param_t *)(param);
            if (a2d->media_ctrl_stat.cmd == ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY &&
                    a2d->media_ctrl_stat.status == ESP_A2D_MEDIA_CTRL_ACK_SUCCESS) {
                ESP_LOGI(BT_AV_TAG, "a2dp media ready, starting ...");
                esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_START);
            }
            break;
        // handle all required enum values
        case ESP_A2D_PROF_STATE_EVT:
            ESP_LOGD(BT_AV_TAG, "Unhandled A2DP event: %d", event);
            break;
        default:
            ESP_LOGI(BT_AV_TAG, "Unknown A2DP event: %d", event);
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