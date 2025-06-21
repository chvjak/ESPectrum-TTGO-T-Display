#include <cstdint>
#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>  // This defines the LGFX class

class TFTDisplay {
  public:
    static void begin();
    static void sendFrameBuffer(uint8_t **frameBuffer);
    static void sendLine(int speccy_y, uint8_t* lineBuffer);

    static void startWrite();
    static void endWrite();

    static LGFX lcd; //IRAM_ATTR?
};
