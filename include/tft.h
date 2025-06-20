#include <cstdint>

class TFTDisplay {
  public:
    static void begin();
    static void sendFrameBuffer(uint8_t **frameBuffer);
    static void sendLine(int speccy_y, uint8_t* lineBuffer);

    static void startWrite();
    static void endWrite();
};
