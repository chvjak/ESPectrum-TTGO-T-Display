#include <cstdint>

class TFTDisplay {
  public:
    static void begin();
    static void sendFrameBuffer(uint8_t **frameBuffer);
};
