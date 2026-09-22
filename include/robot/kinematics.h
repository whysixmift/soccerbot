#pragma once

#include <stdint.h>
#include <stdbool.h>

struct KinematicsConfig {
    int16_t deadzone;     // Deadzone threshold (e.g. 80 for 8% on -1000..+1000 scale)
    bool invert_lx;       // Invert strafe (LX)
    bool invert_ly;       // Invert forward/back (LY)
    bool invert_rx;       // Invert rotation (RX)

    bool invert_fl;       // Invert Front-Left wheel
    bool invert_fr;       // Invert Front-Right wheel
    bool invert_rl;       // Invert Rear-Left wheel
    bool invert_rr;       // Invert Rear-Right wheel

    bool invert_ch1;      // Invert Channel 1 (Left)
    bool invert_ch2;      // Invert Channel 2 (Right)
};

struct MecanumWheelSpeeds {
    int16_t fl; // Front Left
    int16_t fr; // Front Right
    int16_t rl; // Rear Left
    int16_t rr; // Rear Right
};

struct DualChannelSpeeds {
    int16_t ch1_left;  // Motor driver Channel 1 (Left)
    int16_t ch2_right; // Motor driver Channel 2 (Right)
};

class Kinematics {
public:
    /**
     * @brief Computes 4-wheel mecanum inverse kinematics from body velocity commands (LX, LY, RX).
     * Applies deadzone filtering, axis inversions, inverse kinematics equations,
     * wheel inversions, and vector proportional normalization.
     */
    static MecanumWheelSpeeds computeMecanum(int16_t lx,
                                            int16_t ly,
                                            int16_t rx,
                                            const KinematicsConfig& config);

    /**
     * @brief Computes 2-channel motor speeds for the physical 2-channel robot hardware.
     * Maps forward/backward (LY) and rotation (RX) with optional strafe mixing and normalization.
     */
    static DualChannelSpeeds computeDualChannel(int16_t lx,
                                                int16_t ly,
                                                int16_t rx,
                                                const KinematicsConfig& config);

    /**
     * @brief Applies radial and rotational deadzone filtering to input axes.
     */
    static void applyDeadzone(int16_t& lx, int16_t& ly, int16_t& rx, int16_t deadzone);
};
