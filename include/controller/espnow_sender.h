#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "common/protocol.h"
#include "controller/controller_input.h"

struct EspNowSenderStats {
    uint32_t packets_sent;
    uint32_t send_success;
    uint32_t send_fail;
};

class EspNowSender {
public:
    EspNowSender();

    /**
     * @brief Initializes Wi-Fi in Station mode, configures Wi-Fi channel,
     * initializes ESP-NOW, and registers the target robot unicast peer.
     *
     * @param target_robot_id Robot ID (1 for BOT_1, 2 for BOT_2)
     * @param target_mac Target Robot 6-byte MAC address
     * @param wifi_channel 2.4GHz Wi-Fi channel (1..13)
     * @return true on success.
     */
    bool init(uint8_t target_robot_id, const uint8_t* target_mac, uint8_t wifi_channel);

    /**
     * @brief Formats, checksums, and transmits a unicast RobotCommand packet over ESP-NOW.
     *
     * @param input Current normalized controller input state
     * @return true if packet was handed off to Wi-Fi stack successfully.
     */
    bool sendCommand(const ControllerInputState& input);

    /**
     * @brief Dynamically updates the target MAC address.
     */
    bool setTargetMac(const uint8_t* target_mac);

    const EspNowSenderStats& getStats() const { return _stats; }
    uint32_t getSequence() const { return _sequence; }
    uint8_t getTargetRobotId() const { return _target_robot_id; }
    const uint8_t* getTargetMac() const { return _target_mac; }

    void handleSendStatus(bool success);

private:
    uint8_t           _target_robot_id;
    uint8_t           _target_mac[6];
    uint8_t           _wifi_channel;
    uint32_t          _sequence;
    bool              _peer_registered;
    EspNowSenderStats _stats;

    bool registerPeer();
};
