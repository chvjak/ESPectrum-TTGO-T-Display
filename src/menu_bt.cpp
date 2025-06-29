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

// NVS helpers for MAC
static bool load_bt_mac(uint8_t mac[6]) {
    nvs_handle_t nvs;
    if (nvs_open("btcfg", NVS_READONLY, &nvs) != ESP_OK) return false;
    size_t len = 6;
    esp_err_t err = nvs_get_blob(nvs, "btmac", mac, &len);
    nvs_close(nvs);
    return err == ESP_OK && len == 6;
}
static void save_bt_mac(const uint8_t mac[6]) {
    nvs_handle_t nvs;
    if (nvs_open("btcfg", NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_blob(nvs, "btmac", mac, 6);
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
    bool has_saved_mac = load_bt_mac(saved_mac);
    uint8_t connected_mac[6] = {0};
    bool is_connected = bt_get_connected_mac(connected_mac);
    std::string connected_name = is_connected ? bt_get_connected_name() : "";

    int sel = 0;
    while (true) {
        lcd.startWrite();
        lcd.clear(TFT_BLACK);
        int y = 0;
        lcd.setTextColor(TFT_WHITE, TFT_BLACK);
        lcd.setCursor(0, y);
        lcd.printf("Bluetooth Menu\n");
        y += 16;
        if (is_connected) {
            lcd.setTextColor(TFT_GREEN, TFT_BLACK);
            lcd.setCursor(0, y);
            lcd.printf("Connected: %s", connected_name.c_str());
            y += 16;
            lcd.setTextColor(TFT_WHITE, TFT_BLACK);
            lcd.setCursor(0, y);
            lcd.printf("MAC: %s", mac_to_str(connected_mac).c_str());
            y += 16;
        } else if (has_saved_mac) {
            lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
            lcd.setCursor(0, y);
            lcd.printf("Saved device: %s", mac_to_str(saved_mac).c_str());
            y += 16;
        } else {
            lcd.setTextColor(TFT_RED, TFT_BLACK);
            lcd.setCursor(0, y);
            lcd.printf("No device connected");
            y += 16;
        }
        lcd.setTextColor(sel == 0 ? TFT_YELLOW : TFT_WHITE, TFT_BLACK);
        lcd.setCursor(0, y);
        lcd.printf("Discover Devices\n");
        y += 16;
        lcd.setTextColor(sel == 1 ? TFT_YELLOW : TFT_WHITE, TFT_BLACK);
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
            sel = (sel + 1) % 2;
        } else if (select_pressed) {
            if (sel == 0) break; // Discover Devices
            else return false;   // Cancel
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
                save_bt_mac(devices[dev_sel].mac);
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