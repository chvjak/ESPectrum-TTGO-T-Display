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

static LGFX lcd;
static int width = 240;
static int height = 135;

// vga resolution?
static int speccy_width = 320;
static int speccy_height = 200;


static int vofs = 0;
static int hofs = 0;

static int vofs_inc = 1;

static uint8_t pattern[240];

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

void TFTDisplay::sendFrameBuffer(uint8_t** frameBuffer) {
    lcd.startWrite();
    for (int y = 0; y < height; ++y) {
        lcd.setAddrWindow(0, y, width, 1);
        // Convert each pixel in the line to RGB332 and store in pattern
        for (int x = 0; x < width; ++x) {
            pattern[x] = convertR2G2B2S2_to_RGB332(frameBuffer[(speccy_height - y - vofs)][(x + hofs) ^ 2]);
        }
        lcd.writePixels(pattern, width);
    }

    vofs += vofs_inc;
    if (vofs >= (speccy_height - height)) {
        vofs = (speccy_height - height);
        vofs_inc = -1;
    }
    if (vofs <= 0) {
        vofs = 0;
        vofs_inc = 1;
    }

    lcd.endWrite();
    lcd.display();
}
