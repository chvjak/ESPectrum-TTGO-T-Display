#include "driver/gpio.h"
#include "tft.h"
#include "FileUtils.h"

#include <dirent.h>
#include <vector>
#include <algorithm>
#include <string>

#define MENU_KEY_NEXT GPIO_NUM_0   // Button 1: move to next file
#define MENU_KEY_SELECT GPIO_NUM_35 // Button 2: select file

// Helper to read button state (active low)
static inline bool button_pressed(gpio_num_t gpio) {
    return gpio_get_level(gpio) == 0;
}

// Refactored: Use TFTDisplay* directly, no Graphics abstraction
std::string showZ80FileMenu() {
    TFTDisplay::begin();

    LGFX &lcd = TFTDisplay::lcd;

    std::vector<std::string> files;
    DIR* dir = opendir("/spiffs");
    if (dir) {
        printf("[MENU] Opened /spiffs directory\n");
        struct dirent* ent;
        while ((ent = readdir(dir)) != nullptr) {
            std::string fname = ent->d_name;
            if (fname.size() > 4 && fname.substr(fname.size()-4) == ".z80") {
                printf("[MENU] Found file: %s\n", fname.c_str());
                files.push_back(fname);
            }
        }
        closedir(dir);
    } else {
        printf("[MENU] Failed to open /spiffs directory!\n");
    }
    std::sort(files.begin(), files.end());
    printf("[MENU] Total .z80 files found: %d\n", (int)files.size());
    if (files.empty()) {
        // Show message on display
        lcd.startWrite();
        lcd.clear(TFT_BLACK);
        lcd.setTextColor(TFT_RED, TFT_BLACK);
        lcd.setCursor(0, 0);
        lcd.printf("No .z80 files found!\n");
        lcd.setTextColor(TFT_WHITE, TFT_BLACK);
        lcd.printf("Check /spiffs\n");
        lcd.endWrite();
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        return "";
    }
    int sel = 0;

    // Configure buttons as input with pull-up
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << MENU_KEY_NEXT) | (1ULL << MENU_KEY_SELECT);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    while (true) {
        lcd.startWrite();
        lcd.clear(TFT_BLACK); // Use LovyanGFX clear
        int y = 0;
        lcd.setTextColor(TFT_WHITE, TFT_BLACK);
        lcd.setCursor(0, y);
        lcd.printf("Select .z80 file:\n");
        y += 16;
        for (size_t i = 0; i < files.size(); ++i) {
            if ((int)i == sel)
                lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
            else
                lcd.setTextColor(TFT_WHITE, TFT_BLACK);
            lcd.setCursor(0, y);
            lcd.printf("%s\n", files[i].c_str());
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
            sel = (sel + 1) % files.size();
        } else if (select_pressed) {
            break;
        }
    }
    return std::string("/spiffs/") + files[sel];
}
