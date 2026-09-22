#include "controller/usb_hid_host.h"
#include "controller/config_controller.h"
#include "common/logging.h"
#include <string.h>

static const char* TAG = "USB_HID";

UsbHidHost::UsbHidHost()
    : _processor(nullptr),
      _initialized(false),
      _last_report_time(0) {
    memset(&_current_state, 0, sizeof(_current_state));
}

bool UsbHidHost::init(ControllerInputProcessor* processor) {
    _processor = processor;
    _initialized = true;
    _last_report_time = 0;
    memset(&_current_state, 0, sizeof(_current_state));

    LOG_INFO(TAG, "Initializing ESP32-S2 USB Host HID subsystem...");
    LOG_INFO(TAG, "Hardware PHY: Built-in USB OTG (D- = GPIO%d, D+ = GPIO%d)",
             USB_HOST_PIN_DM, USB_HOST_PIN_DP);

    // Initial disconnected state
    if (_processor) {
        _processor->processInputs(0, 0, 0, 0, 0, 0, 0, false, &_current_state);
    }

    LOG_INFO(TAG, "USB Host HID parser ready. Awaiting USB Gamepad connection or Serial Input...");
    return true;
}

void UsbHidHost::update(uint32_t now_ms) {
    if (!_initialized) return;

    // Check for controller disconnect timeout (1000 ms of inactivity)
    if (_current_state.connected && (now_ms - _last_report_time > 1000)) {
        LOG_WARN(TAG, "USB Game Controller disconnected (timeout).");
        if (_processor) {
            _processor->processInputs(0, 0, 0, 0, 0, 0, 0, false, &_current_state);
        }
    }
}

void UsbHidHost::parseRawHidReport(const uint8_t* report, size_t len) {
    if (!_initialized || !report || len < 4) return;

    _last_report_time = millis();

    int16_t raw_lx = 0;
    int16_t raw_ly = 0;
    int16_t raw_rx = 0;
    int16_t raw_ry = 0;
    uint16_t buttons = 0;
    uint8_t  kick = 0;
    uint8_t  dribble = 0;

    if (len >= 12) {
        // Modern 12+ byte HID Gamepad format (e.g. 16-bit axes or multi-axis descriptor)
        // Check if report has 16-bit or 8-bit fields
        raw_lx = ControllerInputProcessor::map8BitAxis(report[0], false);
        raw_ly = ControllerInputProcessor::map8BitAxis(report[1], false);
        raw_rx = ControllerInputProcessor::map8BitAxis(report[2], false);
        raw_ry = ControllerInputProcessor::map8BitAxis(report[3], false);

        // Buttons in bytes 4..7
        buttons = (uint16_t)report[4] | ((uint16_t)report[5] << 8);

        // Triggers / Dribble / Kick in bytes 6..8 if present
        if (len >= 8) {
            kick = report[6];    // Right trigger (R2/RT)
            dribble = report[7]; // Left trigger (L2/LT)
        }
    } else {
        // Standard compact 4..8 byte HID Joystick format:
        // Byte 0: X (Left/Right)
        // Byte 1: Y (Forward/Back)
        // Byte 2: Z / Rotate X
        // Byte 3: Rz / Rotate Y
        // Byte 4..5: Buttons & Hat switch
        raw_lx = ControllerInputProcessor::map8BitAxis(report[0], false);
        raw_ly = ControllerInputProcessor::map8BitAxis(report[1], false);
        if (len > 2) raw_rx = ControllerInputProcessor::map8BitAxis(report[2], false);
        if (len > 3) raw_ry = ControllerInputProcessor::map8BitAxis(report[3], false);
        if (len > 4) buttons = (uint16_t)report[4];
        if (len > 5) buttons |= ((uint16_t)report[5] << 8);
    }

    if (buttons & 0x01) { // Button 1 -> Kick
        kick = 255;
    }
    if (buttons & 0x02) { // Button 2 -> Dribble toggle
        dribble = 255;
    }

    if (_processor) {
        _processor->processInputs(raw_lx, raw_ly, raw_rx, raw_ry, buttons, kick, dribble, true, &_current_state);
    }
}

void UsbHidHost::injectTestInput(int16_t lx, int16_t ly, int16_t rx, uint16_t buttons, uint8_t kick, uint8_t dribble) {
    _last_report_time = millis();
    if (_processor) {
        _processor->processInputs(lx, ly, rx, 0, buttons, kick, dribble, true, &_current_state);
    }
}
