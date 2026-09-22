#include "common/protocol.h"
#include <string.h>

// CRC-16-CCITT implementation (polynomial 0x1021, init 0xFFFF)
uint16_t protocol_crc16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static inline int16_t clamp_axis(int16_t val) {
    if (val < AXIS_MIN_VALUE) return AXIS_MIN_VALUE;
    if (val > AXIS_MAX_VALUE) return AXIS_MAX_VALUE;
    return val;
}

void protocol_prepare_command(RobotCommand* cmd,
                              uint8_t robot_id,
                              int16_t lx,
                              int16_t ly,
                              int16_t rx,
                              int16_t ry,
                              uint16_t buttons,
                              uint8_t kick,
                              uint8_t dribble,
                              uint32_t sequence) {
    if (!cmd) return;

    cmd->magic            = SOCCERBOT_PROTOCOL_MAGIC;
    cmd->protocol_version = SOCCERBOT_PROTOCOL_VERSION;
    cmd->robot_id         = robot_id;

    cmd->lx = clamp_axis(lx);
    cmd->ly = clamp_axis(ly);
    cmd->rx = clamp_axis(rx);
    cmd->ry = clamp_axis(ry);

    cmd->buttons = buttons;
    cmd->kick    = kick;
    cmd->dribble = dribble;
    cmd->sequence = sequence;

    // Compute CRC over all struct bytes up to, but not including, the crc field
    const size_t crc_payload_len = sizeof(RobotCommand) - sizeof(uint16_t);
    cmd->crc = protocol_crc16((const uint8_t*)cmd, crc_payload_len);
}

bool protocol_validate_command(const uint8_t* data,
                               size_t len,
                               uint8_t expected_robot_id,
                               uint32_t last_sequence,
                               RobotCommand* out_cmd,
                               const char** out_err_reason) {
    if (!data) {
        if (out_err_reason) *out_err_reason = "Null data buffer";
        return false;
    }

    if (len != sizeof(RobotCommand)) {
        if (out_err_reason) *out_err_reason = "Invalid packet size";
        return false;
    }

    const RobotCommand* cmd = (const RobotCommand*)data;

    if (cmd->magic != SOCCERBOT_PROTOCOL_MAGIC) {
        if (out_err_reason) *out_err_reason = "Magic mismatch";
        return false;
    }

    if (cmd->protocol_version != SOCCERBOT_PROTOCOL_VERSION) {
        if (out_err_reason) *out_err_reason = "Protocol version mismatch";
        return false;
    }

    if (expected_robot_id != 0 && cmd->robot_id != expected_robot_id) {
        if (out_err_reason) *out_err_reason = "Robot ID mismatch";
        return false;
    }

    const size_t crc_payload_len = sizeof(RobotCommand) - sizeof(uint16_t);
    uint16_t computed_crc = protocol_crc16(data, crc_payload_len);
    if (computed_crc != cmd->crc) {
        if (out_err_reason) *out_err_reason = "CRC mismatch";
        return false;
    }

    // Freshness check:
    // If last_sequence is non-zero, check that sequence is strictly newer (handling unsigned 32-bit wrap)
    if (last_sequence != 0) {
        int32_t diff = (int32_t)(cmd->sequence - last_sequence);
        if (diff <= 0) {
            if (out_err_reason) *out_err_reason = "Stale or duplicate sequence";
            return false;
        }
    }

    // Verify axis boundaries
    if (cmd->lx < AXIS_MIN_VALUE || cmd->lx > AXIS_MAX_VALUE ||
        cmd->ly < AXIS_MIN_VALUE || cmd->ly > AXIS_MAX_VALUE ||
        cmd->rx < AXIS_MIN_VALUE || cmd->rx > AXIS_MAX_VALUE ||
        cmd->ry < AXIS_MIN_VALUE || cmd->ry > AXIS_MAX_VALUE) {
        if (out_err_reason) *out_err_reason = "Axis range overflow";
        return false;
    }

    if (out_cmd) {
        memcpy(out_cmd, cmd, sizeof(RobotCommand));
    }

    if (out_err_reason) *out_err_reason = nullptr;
    return true;
}
