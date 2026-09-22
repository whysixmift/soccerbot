#include "controller/controller_input.h"
#include <stdlib.h>

#ifdef ARDUINO
#include <Arduino.h>
#else
#include "common/logging.h"
#endif

ControllerInputProcessor::ControllerInputProcessor()
    : _deadzone(80),
      _invert_lx(false),
      _invert_ly(false),
      _invert_rx(false),
      _invert_ry(false) {}

void ControllerInputProcessor::init(int16_t deadzone,
                                    bool invert_lx,
                                    bool invert_ly,
                                    bool invert_rx,
                                    bool invert_ry) {
    _deadzone = deadzone;
    _invert_lx = invert_lx;
    _invert_ly = invert_ly;
    _invert_rx = invert_rx;
    _invert_ry = invert_ry;
}

int16_t ControllerInputProcessor::map8BitAxis(uint8_t raw_val, bool invert) {
    // 0 -> -1000, 128 -> 0, 255 -> +1000
    int32_t val = (int32_t)raw_val - 128;
    int32_t scaled = (val * 1000) / 127;
    if (scaled < -1000) scaled = -1000;
    if (scaled > 1000) scaled = 1000;
    return (int16_t)(invert ? -scaled : scaled);
}

int16_t ControllerInputProcessor::map16BitAxis(int16_t raw_val, bool invert) {
    int32_t scaled = ((int32_t)raw_val * 1000) / 32767;
    if (scaled < -1000) scaled = -1000;
    if (scaled > 1000) scaled = 1000;
    return (int16_t)(invert ? -scaled : scaled);
}

void ControllerInputProcessor::processInputs(int16_t raw_lx,
                                             int16_t raw_ly,
                                             int16_t raw_rx,
                                             int16_t raw_ry,
                                             uint16_t buttons,
                                             uint8_t kick,
                                             uint8_t dribble,
                                             bool connected,
                                             ControllerInputState* out_state) {
    if (!out_state) return;

    if (!connected) {
        out_state->lx = 0;
        out_state->ly = 0;
        out_state->rx = 0;
        out_state->ry = 0;
        out_state->buttons = 0;
        out_state->kick = 0;
        out_state->dribble = 0;
        out_state->connected = false;
        out_state->last_update_ms = millis();
        return;
    }

    // Apply polarity inversions
    int16_t lx = _invert_lx ? -raw_lx : raw_lx;
    int16_t ly = _invert_ly ? -raw_ly : raw_ly;
    int16_t rx = _invert_rx ? -raw_rx : raw_rx;
    int16_t ry = _invert_ry ? -raw_ry : raw_ry;

    // Apply translation radial deadzone
    int32_t trans_sq = (int32_t)lx * (int32_t)lx + (int32_t)ly * (int32_t)ly;
    int32_t dz_sq = (int32_t)_deadzone * (int32_t)_deadzone;
    if (trans_sq < dz_sq) {
        lx = 0;
        ly = 0;
    }

    // Apply rotation deadzone
    if (abs(rx) < _deadzone) {
        rx = 0;
    }

    // Apply auxiliary deadzone
    if (abs(ry) < _deadzone) {
        ry = 0;
    }

    out_state->lx = lx;
    out_state->ly = ly;
    out_state->rx = rx;
    out_state->ry = ry;
    out_state->buttons = buttons;
    out_state->kick = kick;
    out_state->dribble = dribble;
    out_state->connected = true;
    out_state->last_update_ms = millis();
}
