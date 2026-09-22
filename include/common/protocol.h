#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "config_common.h"

#pragma pack(push, 1)
/**
 * @brief Compact binary robot control command packet for ESP-NOW unicast transmission.
 * Total size: 22 bytes. Fixed-size, deterministic, versioned, checksummed.
 */
struct RobotCommand {
    uint16_t magic;            // Protocol magic (0x5342)
    uint8_t  protocol_version; // Protocol version (0x01)
    uint8_t  robot_id;         // Target robot ID (e.g. 1 for BOT 1, 2 for BOT 2)

    int16_t  lx;               // Strafe command: -1000 (left) to +1000 (right)
    int16_t  ly;               // Forward/backward command: -1000 (reverse) to +1000 (forward)
    int16_t  rx;               // Rotation command: -1000 (CCW) to +1000 (CW)
    int16_t  ry;               // Auxiliary axis: -1000 to +1000

    uint16_t buttons;          // Button bitmask (BTN_KICK_TRIGGER, BTN_DRIBBLE_TOGGLE, etc.)

    uint8_t  kick;             // Kick trigger / strength (0..255)
    uint8_t  dribble;          // Dribbler motor power (0..255)

    uint32_t sequence;         // Monotonically increasing sequence number
    uint16_t crc;              // CRC-16 checksum covering all preceding 20 bytes
};
#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Computes CRC-16-CCITT (poly 0x1021, init 0xFFFF) across a buffer.
 */
uint16_t protocol_crc16(const uint8_t* data, size_t length);

/**
 * @brief Populates and checksums a RobotCommand structure.
 */
void protocol_prepare_command(RobotCommand* cmd,
                              uint8_t robot_id,
                              int16_t lx,
                              int16_t ly,
                              int16_t rx,
                              int16_t ry,
                              uint16_t buttons,
                              uint8_t kick,
                              uint8_t dribble,
                              uint32_t sequence);

/**
 * @brief Validates an incoming raw packet against magic, version, robot ID, CRC, and sequence freshness.
 *
 * @param data Pointer to received packet data
 * @param len Length in bytes of received packet
 * @param expected_robot_id Expected target robot ID (0 allows any matching ID)
 * @param last_sequence Last valid sequence number (for freshness checking)
 * @param out_cmd Output pointer where validated command will be copied (can be NULL)
 * @param out_err_reason Optional pointer to receive static error string on failure
 * @return true if packet is valid, fresh, and addressed to this robot; false otherwise.
 */
bool protocol_validate_command(const uint8_t* data,
                               size_t len,
                               uint8_t expected_robot_id,
                               uint32_t last_sequence,
                               RobotCommand* out_cmd,
                               const char** out_err_reason);

#ifdef __cplusplus
}
#endif
