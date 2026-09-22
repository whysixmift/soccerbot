#include "robot/espnow_receiver.h"
#include "common/logging.h"
#include <string.h>

#ifdef ARDUINO
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#endif

static const char* TAG = "ESPNOW_RX";
static EspNowReceiver* s_instance = nullptr;

#ifdef ARDUINO
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
static void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (s_instance && info) {
        int8_t rssi = info->rx_ctrl ? info->rx_ctrl->rssi : 0;
        s_instance->handleIncomingPacket(info->src_addr, data, len, rssi);
    }
}
#else
static void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
    if (s_instance) {
        // Core 2.x callback does not provide rx_ctrl directly in standard callback
        s_instance->handleIncomingPacket(mac, data, len, 0);
    }
}
#endif
#endif // ARDUINO

EspNowReceiver::EspNowReceiver()
    : _robot_id(1),
      _wifi_channel(1),
      _failsafe(nullptr),
      _last_sequence(0),
      _has_new_cmd(false) {
    memset(&_latest_cmd, 0, sizeof(_latest_cmd));
    memset(&_stats, 0, sizeof(_stats));
    s_instance = this;
}

bool EspNowReceiver::init(uint8_t expected_robot_id, uint8_t wifi_channel, FailsafeManager* failsafe_mgr) {
    _robot_id = expected_robot_id;
    _wifi_channel = wifi_channel;
    _failsafe = failsafe_mgr;
    _last_sequence = 0;
    _has_new_cmd = false;
    memset(&_stats, 0, sizeof(_stats));
    s_instance = this;

    LOG_INFO(TAG, "Initializing ESP-NOW Receiver (Expected Robot ID: %d, WiFi Channel: %d)...",
             _robot_id, _wifi_channel);

#ifdef ARDUINO
    // 1. Initialize Wi-Fi in Station mode
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // 2. Set exact Wi-Fi channel
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(_wifi_channel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    // 3. Disable Wi-Fi power saving for minimal latency (crucial for real-time control)
    esp_wifi_set_ps(WIFI_PS_NONE);

    // 4. Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        LOG_ERROR(TAG, "ESP-NOW initialization failed!");
        return false;
    }

    // 5. Register receive callback
    esp_now_register_recv_cb(onDataRecv);

    printMacAddress();
#endif

    LOG_INFO(TAG, "ESP-NOW Receiver initialized successfully.");
    return true;
}

void EspNowReceiver::printMacAddress() const {
#ifdef ARDUINO
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    LOG_INFO(TAG, ">>> Robot ESP32 MAC Address: %02X:%02X:%02X:%02X:%02X:%02X <<<",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
#endif
}

void EspNowReceiver::handleIncomingPacket(const uint8_t* mac_addr, const uint8_t* data, int len, int8_t rssi) {
    _stats.total_received++;
    _stats.last_rssi = rssi;

    const char* err_reason = nullptr;
    RobotCommand cmd;
    bool valid = protocol_validate_command(data, (size_t)len, _robot_id, _last_sequence, &cmd, &err_reason);

    if (!valid) {
        if (err_reason) {
            if (strcmp(err_reason, "Magic mismatch") == 0) _stats.dropped_magic++;
            else if (strcmp(err_reason, "Protocol version mismatch") == 0) _stats.dropped_version++;
            else if (strcmp(err_reason, "Robot ID mismatch") == 0) _stats.dropped_id++;
            else if (strcmp(err_reason, "CRC mismatch") == 0) _stats.dropped_crc++;
            else if (strcmp(err_reason, "Stale or duplicate sequence") == 0) _stats.dropped_sequence++;
            else _stats.dropped_size++;

            LOG_DEBUG(TAG, "Dropped packet: %s (len: %d)", err_reason, len);
        }
        return;
    }

    // Packet is valid, fresh, and targeted to this robot
    _stats.valid_packets++;
    _last_sequence = cmd.sequence;
    _latest_cmd = cmd;
    _has_new_cmd = true;

    // Notify failsafe manager of fresh packet arrival
    if (_failsafe) {
        _failsafe->onValidPacketReceived(cmd, millis());
    }
}

bool EspNowReceiver::getLatestCommand(RobotCommand* out_cmd) {
    if (!_has_new_cmd) return false;

    if (out_cmd) {
        *out_cmd = _latest_cmd;
    }
    _has_new_cmd = false;
    return true;
}
