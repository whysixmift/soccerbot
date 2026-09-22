#include "controller/espnow_sender.h"
#include "common/logging.h"
#include <string.h>

#ifdef ARDUINO
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#endif

static const char* TAG = "ESPNOW_TX";
static EspNowSender* s_sender_instance = nullptr;

#ifdef ARDUINO
static void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (s_sender_instance) {
        s_sender_instance->handleSendStatus(status == ESP_NOW_SEND_SUCCESS);
    }
}
#endif

EspNowSender::EspNowSender()
    : _target_robot_id(1),
      _wifi_channel(1),
      _sequence(0),
      _peer_registered(false) {
    memset(_target_mac, 0xFF, sizeof(_target_mac));
    memset(&_stats, 0, sizeof(_stats));
    s_sender_instance = this;
}

bool EspNowSender::init(uint8_t target_robot_id, const uint8_t* target_mac, uint8_t wifi_channel) {
    _target_robot_id = target_robot_id;
    _wifi_channel = wifi_channel;
    _sequence = 0;
    _peer_registered = false;
    memset(&_stats, 0, sizeof(_stats));
    s_sender_instance = this;

    if (target_mac) {
        memcpy(_target_mac, target_mac, 6);
    }

    LOG_INFO(TAG, "Initializing ESP-NOW Sender (Target Robot ID: %d, WiFi Channel: %d)...",
             _target_robot_id, _wifi_channel);

#ifdef ARDUINO
    // 1. Initialize Wi-Fi Station mode
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // 2. Set exact Wi-Fi channel
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(_wifi_channel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    // 3. Disable power saving
    esp_wifi_set_ps(WIFI_PS_NONE);

    // 4. Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        LOG_ERROR(TAG, "ESP-NOW Sender init failed!");
        return false;
    }

    // 5. Register send callback
    esp_now_register_send_cb(onDataSent);

    // 6. Register unicast peer
    registerPeer();
#endif

    LOG_INFO(TAG, "ESP-NOW Sender ready. Target MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             _target_mac[0], _target_mac[1], _target_mac[2],
             _target_mac[3], _target_mac[4], _target_mac[5]);
    return true;
}

bool EspNowSender::registerPeer() {
#ifdef ARDUINO
    if (esp_now_is_peer_exist(_target_mac)) {
        esp_now_del_peer(_target_mac);
    }

    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, _target_mac, 6);
    peerInfo.channel = _wifi_channel;
    peerInfo.encrypt = false;

    esp_err_t res = esp_now_add_peer(&peerInfo);
    if (res == ESP_OK) {
        _peer_registered = true;
        LOG_INFO(TAG, "Registered peer successfully.");
        return true;
    } else {
        LOG_ERROR(TAG, "Failed to register peer (error: %d)", (int)res);
        _peer_registered = false;
        return false;
    }
#else
    return true;
#endif
}

bool EspNowSender::setTargetMac(const uint8_t* target_mac) {
    if (!target_mac) return false;
    memcpy(_target_mac, target_mac, 6);
    return registerPeer();
}

bool EspNowSender::sendCommand(const ControllerInputState& input) {
    _sequence++;

    RobotCommand cmd;
    protocol_prepare_command(&cmd,
                             _target_robot_id,
                             input.lx,
                             input.ly,
                             input.rx,
                             input.ry,
                             input.buttons,
                             input.kick,
                             input.dribble,
                             _sequence);

    _stats.packets_sent++;

#ifdef ARDUINO
    esp_err_t res = esp_now_send(_target_mac, (const uint8_t*)&cmd, sizeof(RobotCommand));
    if (res != ESP_OK) {
        _stats.send_fail++;
        LOG_DEBUG(TAG, "esp_now_send error: %d (seq: %lu)", (int)res, (unsigned long)_sequence);
        return false;
    }
    return true;
#else
    return true;
#endif
}

void EspNowSender::handleSendStatus(bool success) {
    if (success) {
        _stats.send_success++;
    } else {
        _stats.send_fail++;
    }
}
