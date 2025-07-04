#define LGFX_TTGO_TDISPLAY                 // TTGO T-Display
#define LGFX_AUTODETECT // 自動認識 (D-duino-32 XS, WT32-SC01, PyBadge はパネルID読取りが出来ないため自動認識の対象から外れています)

// 複数機種の定義を行うか、LGFX_AUTODETECTを定義することで、実行時にボードを自動認識します。


// ヘッダをincludeします。
#include <LovyanGFX.hpp>

#include <LGFX_AUTODETECT.hpp>  // クラス"LGFX"を準備します
#include <cstdint>
#include "tft.h"

#include "esp_log.h"

using LGFX = lgfx::LGFX;

static int width = 240;
static int height = 135;

// vga resolution?
static int speccy_width = 320;
static int speccy_height = 200;

static int vofs = (speccy_height - height) / 2; // Vertical offset for the display
static int hofs = (speccy_width - width) / 2; // Horizontal

static int vofs_inc = 1;

IRAM_BSS_ATTR static uint8_t pattern[240];

void TFTDisplay::begin() {
    lcd.init();
    lcd.setColorDepth(16);     // RGB332
    lcd.setSwapBytes(false);  // No swap for 8-bit
    lcd.setRotation(1);
    lcd.setBrightness(255);
    lcd.clear(TFT_DARKGREY);

    hofs = (speccy_width - width) / 2;  // Horizontal offset for the display
}

uint8_t convertR2G2B2S2_to_RGB332(uint8_t color) {
    // Extract 2 bits per channel
    uint8_t r = (color & 0b00000011);        // bits 0-1
    uint8_t g = (color & 0b00001100) >> 2;   // bits 2-3
    uint8_t b = (color & 0b00110000) >> 4;   // bits 4-5

    // Expand 2 bits to 3 bits for R and G, 2 bits for B stays as is
    uint8_t r3 = (r * 0x7) / 0x3; // 0..3 -> 0..7
    uint8_t g3 = (g * 0x7) / 0x3; // 0..3 -> 0..7
    uint8_t b2 = b;               // 0..3 -> 0..3

    return (r3 << 5) | (g3 << 2) | b2;
}

IRAM_ATTR void TFTDisplay::sendLine(int speccy_y, uint8_t* lineBuffer) {
    if (speccy_y - vofs < 0 || speccy_y - vofs >= height) {
        return; // Invalid line number
    }
    int y = speccy_y - vofs; // Adjust the line number based on vertical offset
    lcd.setAddrWindow(0, y, width, 1);
    for (int x = 0; x < width; ++x) {
        pattern[x] = convertR2G2B2S2_to_RGB332(lineBuffer[(x + hofs) ^ 2]);
    }

    lcd.writePixels(pattern, width);
}

IRAM_ATTR void TFTDisplay::startWrite() {
    lcd.startWrite();
}

IRAM_ATTR void TFTDisplay::endWrite() {
    lcd.endWrite();
}

LGFX TFTDisplay::lcd;
