#pragma once

#include <stdint.h>
#include <stdbool.h>

struct ControllerInputState {
    int16_t  lx;              // Normalized Strafe (-1000..+1000)
    int16_t  ly;              // Normalized Forward/Back (-1000..+1000)
    int16_t  rx;              // Normalized Rotation (-1000..+1000)
    int16_t  ry;              // Normalized Auxiliary (-1000..+1000)
    uint16_t buttons;         // Button state bitmask
    uint8_t  kick;            // Kick power (0..255)
    uint8_t  dribble;         // Dribbler speed (0..255)
    bool     connected;       // True if game controller is actively connected
    uint32_t last_update_ms;  // Timestamp of last received input report
};

class ControllerInputProcessor {
public:
    ControllerInputProcessor();

    void init(int16_t deadzone,
              bool invert_lx,
              bool invert_ly,
              bool invert_rx,
              bool invert_ry);

    /**
     * @brief Normalizes raw stick inputs, applies deadband, and handles polarity inversions.
     */
    void processInputs(int16_t raw_lx,
                       int16_t raw_ly,
                       int16_t raw_rx,
                       int16_t raw_ry,
                       uint16_t buttons,
                       uint8_t kick,
                       uint8_t dribble,
                       bool connected,
                       ControllerInputState* out_state);

    /**
     * @brief Maps standard 8-bit unsigned HID joystick axis (0..255, 128=center) to [-1000, +1000].
     */
    static int16_t map8BitAxis(uint8_t raw_val, bool invert);

    /**
     * @brief Maps standard 16-bit signed HID joystick axis (-32768..+32767) to [-1000, +1000].
     */
    static int16_t map16BitAxis(int16_t raw_val, bool invert);

private:
    int16_t _deadzone;
    bool    _invert_lx;
    bool    _invert_ly;
    bool    _invert_rx;
    bool    _invert_ry;
};
