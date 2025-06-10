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

static int speccy_width = 256;
static int speccy_height = 192;


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

void TFTDisplay::sendFrameBuffer(uint8_t** frameBuffer) {
    lcd.startWrite();
    for (int y = 0; y < height; ++y) {
        lcd.setAddrWindow(0, y, width, 1);
        lcd.writePixels(frameBuffer[(speccy_height - y - vofs)] + hofs, width);
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
