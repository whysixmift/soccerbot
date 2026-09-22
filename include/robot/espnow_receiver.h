#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "common/protocol.h"
#include "robot/failsafe.h"

struct EspNowReceiverStats {
    uint32_t total_received;
    uint32_t valid_packets;
    uint32_t dropped_magic;
    uint32_t dropped_version;
    uint32_t dropped_id;
    uint32_t dropped_crc;
    uint32_t dropped_sequence;
    uint32_t dropped_size;
    int8_t   last_rssi;
};

class EspNowReceiver {
public:
    EspNowReceiver();

    /**
     * @brief Initializes Wi-Fi in station mode, sets Wi-Fi channel, disables sleep,
     * initializes ESP-NOW peripheral, and registers unicast reception callback.
     *
     * @param expected_robot_id Robot ID to filter for (e.g. 1 or 2)
     * @param wifi_channel Wi-Fi channel (1..13)
     * @param failsafe_mgr Pointer to FailsafeManager to notify on valid packets
     * @return true on successful initialization.
     */
    bool init(uint8_t expected_robot_id, uint8_t wifi_channel, FailsafeManager* failsafe_mgr);

    /**
     * @brief Checks if a new valid command has arrived.
     * @param out_cmd Output buffer to receive latest command.
     * @return true if fresh command was available and copied; false otherwise.
     */
    bool getLatestCommand(RobotCommand* out_cmd);

    /**
     * @brief Retrieves receiver statistics.
     */
    const EspNowReceiverStats& getStats() const { return _stats; }

    /**
     * @brief Prints robot MAC address to serial for pairing.
     */
    void printMacAddress() const;

    // Internal callback handler (public for C callback bridge)
    void handleIncomingPacket(const uint8_t* mac_addr, const uint8_t* data, int len, int8_t rssi);

private:
    uint8_t             _robot_id;
    uint8_t             _wifi_channel;
    FailsafeManager*    _failsafe;
    uint32_t            _last_sequence;
    RobotCommand        _latest_cmd;
    bool                _has_new_cmd;
    EspNowReceiverStats _stats;
};
