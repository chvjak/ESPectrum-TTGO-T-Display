// menu_bt.cpp
#include "driver/gpio.h"
#include "tft.h"
#include "bt.h"
#include "esp_bt_defs.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <vector>
#include <string>
#include <algorithm>
#include <cstring>

#define MENU_KEY_NEXT GPIO_NUM_0
#define MENU_KEY_SELECT GPIO_NUM_35

// Helper to read button state (active low)
static inline bool button_pressed(gpio_num_t gpio) {
    return gpio_get_level(gpio) == 0;
}

// NVS helpers for MAC and device name
static bool load_bt_device(uint8_t mac[6], std::string& name) {
    nvs_handle_t nvs;
    if (nvs_open("btcfg", NVS_READONLY, &nvs) != ESP_OK) return false;
    size_t len = 6;
    esp_err_t err = nvs_get_blob(nvs, "btmac", mac, &len);
    if (err != ESP_OK || len != 6) {
        nvs_close(nvs);
        return false;
    }
    char namebuf[64] = {0};
    len = sizeof(namebuf);
    err = nvs_get_str(nvs, "btname", namebuf, &len);
    nvs_close(nvs);
    if (err == ESP_OK) name = namebuf;
    else name.clear();
    return true;
}

static void save_bt_device(const uint8_t mac[6], const std::string& name) {
    nvs_handle_t nvs;
    if (nvs_open("btcfg", NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_blob(nvs, "btmac", mac, 6);
        nvs_set_str(nvs, "btname", name.c_str());
        nvs_commit(nvs);
        nvs_close(nvs);
    }
}

// Helper to format MAC as string
static std::string mac_to_str(const uint8_t mac[6]) {
    char buf[18];
    snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return std::string(buf);
}

bool showBTMenu() {
    TFTDisplay::begin();
    LGFX &lcd = TFTDisplay::lcd;

    // Configure buttons as input with pull-up
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << MENU_KEY_NEXT) | (1ULL << MENU_KEY_SELECT);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    // Level 1: Show connected device (if any) and "Discover Devices"
    uint8_t saved_mac[6] = {0};
    std::string saved_name;
    bool has_saved_device = load_bt_device(saved_mac, saved_name);

    int sel = 0;
    int menu_items = has_saved_device ? 3 : 2; // Discover, Connect, Cancel

    bt_init();
    while (true) {
        lcd.startWrite();
        lcd.clear(TFT_BLACK);
        int y = 0;
        lcd.setTextColor(TFT_WHITE, TFT_BLACK);
        lcd.setCursor(0, y);
        lcd.printf("Bluetooth Menu\n");
        y += 16;
        if (has_saved_device) {
            lcd.setTextColor(sel == 0 ? TFT_YELLOW : TFT_WHITE, TFT_BLACK);
            lcd.setCursor(0, y);
            lcd.printf("Connect to: %s", saved_name.c_str());
            y += 16;
        }

        // Discover Devices
        lcd.setTextColor(sel == (has_saved_device ? 1 : 0) ? TFT_YELLOW : TFT_WHITE, TFT_BLACK);
        lcd.setCursor(0, y);
        lcd.printf("Discover Devices\n");
        y += 16;

        // Cancel
        lcd.setTextColor(sel == (menu_items - 1) ? TFT_YELLOW : TFT_WHITE, TFT_BLACK);
        lcd.setCursor(0, y);
        lcd.printf("Cancel\n");
        lcd.endWrite();

        // Wait for button press
        bool next_pressed = false, select_pressed = false;
        while (!next_pressed && !select_pressed) {
            next_pressed = button_pressed(MENU_KEY_NEXT);
            select_pressed = button_pressed(MENU_KEY_SELECT);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }

        // Debounce: wait for release
        while (button_pressed(MENU_KEY_NEXT) || button_pressed(MENU_KEY_SELECT)) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }

        if (next_pressed) {
            sel = (sel + 1) % menu_items;
        } else if (select_pressed) {
            if (sel == has_saved_device ? 1 : 0) break; // Discover Devices
            else if (has_saved_device && sel == 0) {
                // Connect to saved device
                lcd.startWrite();
                lcd.clear(TFT_BLACK);
                lcd.setTextColor(TFT_WHITE, TFT_BLACK);
                lcd.setCursor(0, 0);
                lcd.printf("Connecting to %s...\n", !saved_name.empty() ? saved_name.c_str() : mac_to_str(saved_mac).c_str());
                lcd.endWrite();
                if (bt_connect_device(saved_mac)) {
                    lcd.startWrite();
                    lcd.setTextColor(TFT_GREEN, TFT_BLACK);
                    lcd.setCursor(0, 16);
                    lcd.printf("Connected!\n");
                    lcd.endWrite();
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                    return true;
                } else {
                    lcd.startWrite();
                    lcd.setTextColor(TFT_RED, TFT_BLACK);
                    lcd.setCursor(0, 16);
                    lcd.printf("Failed to connect!\n");
                    lcd.endWrite();
                    vTaskDelay(2000 / portTICK_PERIOD_MS);
                    return false;
                }
            } else {
                return false; // Cancel
            }
        }
    }

    // Level 2: Discover and list devices
    lcd.startWrite();
    lcd.clear(TFT_BLACK);
    lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    lcd.setCursor(0, 0);
    lcd.printf("Scanning for devices...\n");
    lcd.endWrite();

    std::vector<BTDevice> devices = bt_discover_devices();
    if (devices.empty()) {
        lcd.startWrite();
        lcd.setTextColor(TFT_RED, TFT_BLACK);
        lcd.setCursor(0, 16);
        lcd.printf("No devices found!\n");
        lcd.endWrite();
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        return false;
    }

    int dev_sel = 0;
    while (true) {
        lcd.startWrite();
        lcd.clear(TFT_BLACK);
        int y = 0;
        lcd.setTextColor(TFT_WHITE, TFT_BLACK);
        lcd.setCursor(0, y);
        lcd.printf("Select device:\n");
        y += 16;
        for (size_t i = 0; i < devices.size(); ++i) {
            if ((int)i == dev_sel)
                lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
            else
                lcd.setTextColor(TFT_WHITE, TFT_BLACK);
            lcd.setCursor(0, y);
            lcd.printf("%s [%s]\n", devices[i].name.c_str(), mac_to_str(devices[i].mac).c_str());
            y += 16;
        }
        lcd.endWrite();
        // Wait for button press
        bool next_pressed = false, select_pressed = false;
        while (!next_pressed && !select_pressed) {
            next_pressed = button_pressed(MENU_KEY_NEXT);
            select_pressed = button_pressed(MENU_KEY_SELECT);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        // Debounce: wait for release
        while (button_pressed(MENU_KEY_NEXT) || button_pressed(MENU_KEY_SELECT)) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        if (next_pressed) {
            dev_sel = (dev_sel + 1) % devices.size();
        } else if (select_pressed) {
            // Try to connect
            lcd.startWrite();
            lcd.clear(TFT_BLACK);
            lcd.setTextColor(TFT_WHITE, TFT_BLACK);
            lcd.setCursor(0, 0);
            lcd.printf("Connecting to %s...\n", devices[dev_sel].name.c_str());
            lcd.endWrite();
            if (bt_connect_device(devices[dev_sel].mac)) {
                save_bt_device(devices[dev_sel].mac, devices[dev_sel].name);
                lcd.startWrite();
                lcd.setTextColor(TFT_GREEN, TFT_BLACK);
                lcd.setCursor(0, 16);
                lcd.printf("Connected!\n");
                lcd.endWrite();
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                return true;
            } else {
                lcd.startWrite();
                lcd.setTextColor(TFT_RED, TFT_BLACK);
                lcd.setCursor(0, 16);
                lcd.printf("Failed to connect!\n");
                lcd.endWrite();
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                return false;
            }
        }
    }
}