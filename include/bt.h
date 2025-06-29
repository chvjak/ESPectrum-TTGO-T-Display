#include <stdint.h>
#include <vector>
#include <string>

static int32_t audio_data_cb(uint8_t *data, int32_t len);

void bt_init(void);

struct BTDevice {
    uint8_t mac[6];
    std::string name;
};

// Start discovery, returns vector of BTDevice (blocking or with timeout)
std::vector<BTDevice> bt_discover_devices();
// Connect to device by MAC, returns true on success
bool bt_connect_device(const uint8_t mac[6]);
// Get currently connected device MAC, returns true if connected
bool bt_get_connected_mac(uint8_t mac[6]);
// Get currently connected device name, returns empty if not connected
std::string bt_get_connected_name();
