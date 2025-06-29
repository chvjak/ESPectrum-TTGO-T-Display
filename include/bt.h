#include <stdint.h>
#include <vector>
#include <string>

static int32_t audio_data_cb(uint8_t *data, int32_t len);

void bt_init(void);

void start_discovery(void);

struct BTDevice {
    uint8_t mac[6];
    std::string name;
};

// Start discovery, returns vector of BTDevice (blocking or with timeout)
std::vector<BTDevice> bt_discover_devices();
// Connect to device by MAC, returns true on success
bool bt_connect_device(const uint8_t mac[6]);
