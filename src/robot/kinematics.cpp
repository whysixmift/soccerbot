#include "robot/kinematics.h"
#include <stdlib.h>
#include <algorithm>

static inline int32_t abs32(int32_t v) {
    return v < 0 ? -v : v;
}

void Kinematics::applyDeadzone(int16_t& lx, int16_t& ly, int16_t& rx, int16_t deadzone) {
    // Radial deadzone for translation stick (LX, LY)
    int32_t trans_mag_sq = (int32_t)lx * (int32_t)lx + (int32_t)ly * (int32_t)ly;
    int32_t dz_sq = (int32_t)deadzone * (int32_t)deadzone;

    if (trans_mag_sq < dz_sq) {
        lx = 0;
        ly = 0;
    }

    // Deadzone for rotation stick (RX)
    if (abs32(rx) < deadzone) {
        rx = 0;
    }
}

MecanumWheelSpeeds Kinematics::computeMecanum(int16_t lx,
                                             int16_t ly,
                                             int16_t rx,
                                             const KinematicsConfig& config) {
    // 1. Apply deadzones
    applyDeadzone(lx, ly, rx, config.deadzone);

    // 2. Apply axis inversions
    int32_t vx = config.invert_lx ? -lx : lx;
    int32_t vy = config.invert_ly ? -ly : ly;
    int32_t omega = config.invert_rx ? -rx : rx;

    // 3. Mecanum inverse kinematics equations:
    // front_left  = ly + lx + rx
    // front_right = ly - lx - rx
    // rear_left   = ly - lx + rx
    // rear_right  = ly + lx - rx
    int32_t raw_fl = vy + vx + omega;
    int32_t raw_fr = vy - vx - omega;
    int32_t raw_rl = vy - vx + omega;
    int32_t raw_rr = vy + vx - omega;

    // 4. Find maximum absolute wheel output for vector proportional scaling
    int32_t max_val = abs32(raw_fl);
    if (abs32(raw_fr) > max_val) max_val = abs32(raw_fr);
    if (abs32(raw_rl) > max_val) max_val = abs32(raw_rl);
    if (abs32(raw_rr) > max_val) max_val = abs32(raw_rr);

    // 5. Proportional normalization to prevent clipping and preserve trajectory angle
    if (max_val > 1000) {
        raw_fl = (raw_fl * 1000) / max_val;
        raw_fr = (raw_fr * 1000) / max_val;
        raw_rl = (raw_rl * 1000) / max_val;
        raw_rr = (raw_rr * 1000) / max_val;
    }

    // 6. Apply individual wheel inversions
    MecanumWheelSpeeds speeds;
    speeds.fl = (int16_t)(config.invert_fl ? -raw_fl : raw_fl);
    speeds.fr = (int16_t)(config.invert_fr ? -raw_fr : raw_fr);
    speeds.rl = (int16_t)(config.invert_rl ? -raw_rl : raw_rl);
    speeds.rr = (int16_t)(config.invert_rr ? -raw_rr : raw_rr);

    return speeds;
}

DualChannelSpeeds Kinematics::computeDualChannel(int16_t lx,
                                                int16_t ly,
                                                int16_t rx,
                                                const KinematicsConfig& config) {
    // 1. Apply deadzones
    applyDeadzone(lx, ly, rx, config.deadzone);

    // 2. Apply axis inversions (supports steering on Right Stick RX or Left Stick LX)
    int16_t turn_input = (abs32(rx) >= abs32(lx)) ? rx : lx;
    int32_t vy = config.invert_ly ? -ly : ly;
    int32_t omega = config.invert_rx ? -turn_input : turn_input;

    // 3. Apply speed scaling
    if (config.linear_scale > 0 && config.linear_scale <= 1000) {
        vy = (vy * config.linear_scale) / 1000;
    }
    if (config.turn_scale > 0 && config.turn_scale <= 1000) {
        omega = (omega * config.turn_scale) / 1000;
    }

    // 4. Compute 2-channel differential drive:
    // Left channel  = vy + omega
    // Right channel = vy - omega
    int32_t raw_left  = vy + omega;
    int32_t raw_right = vy - omega;

    // 4. Proportional normalization
    int32_t max_val = std::max(abs32(raw_left), abs32(raw_right));
    if (max_val > 1000) {
        raw_left  = (raw_left * 1000) / max_val;
        raw_right = (raw_right * 1000) / max_val;
    }

    // 5. Apply channel inversions
    DualChannelSpeeds speeds;
    speeds.ch1_left  = (int16_t)(config.invert_ch1 ? -raw_left : raw_left);
    speeds.ch2_right = (int16_t)(config.invert_ch2 ? -raw_right : raw_right);

    return speeds;
}
