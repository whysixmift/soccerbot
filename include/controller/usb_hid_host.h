#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "controller/controller_input.h"

/**
 * @brief Abstract interface for USB Game Controller HID Driver.
 * Decouples physical USB Host hardware layer from robot command translation.
 */
class UsbHidHost {
public:
    UsbHidHost();

    /**
     * @brief Initializes the USB Host subsystem and HID parser.
     * Uses ESP32-S2 built-in USB PHY (GPIO19=D-, GPIO20=D+).
     *
     * @param processor Pointer to input calibration processor
     * @return true on successful initialization.
     */
    bool init(ControllerInputProcessor* processor);

    /**
     * @brief Non-blocking task loop to poll USB Host events and decode pending reports.
     */
    void update(uint32_t now_ms);

    /**
     * @brief Returns current controller input state.
     */
    const ControllerInputState& getState() const { return _current_state; }

    /**
     * @brief Returns true if a physical or simulated controller is connected.
     */
    bool isConnected() const { return _current_state.connected; }

    /**
     * @brief Generic HID Report Decoder.
     * Parses standard USB HID gamepad/joystick reports of various lengths (4 to 64 bytes).
     *
     * @param report Pointer to raw HID report buffer
     * @param len Length in bytes of the report
     */
    void parseRawHidReport(const uint8_t* report, size_t len);

    /**
     * @brief Injects simulated input from Serial / Test runner (for bench testing).
     */
    void injectTestInput(int16_t lx, int16_t ly, int16_t rx, uint16_t buttons, uint8_t kick, uint8_t dribble);

private:
    ControllerInputProcessor* _processor;
    ControllerInputState      _current_state;
    bool                      _initialized;
    uint32_t                  _last_report_time;
};
