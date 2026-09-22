#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "common/protocol.h"
#include "robot/failsafe.h"

// Forward declaration of Controller from Bluepad32
class Controller;
typedef Controller* ControllerPtr;

class Bluepad32Receiver {
public:
    Bluepad32Receiver();

    /**
     * @brief Initializes Bluepad32 Bluetooth subsystem, registers connect/disconnect callbacks,
     * prints the ESP32 local Bluetooth MAC address, and enters listening mode.
     *
     * @param failsafe Pointer to failsafe manager for watchdog and disconnect notification
     * @return true on successful initialization.
     */
    bool init(FailsafeManager* failsafe);

    /**
     * @brief Must be called in the main loop to process Bluetooth events and fetch controller updates.
     */
    void update(uint32_t now_ms);

    /**
     * @brief Checks if new valid controller input is available.
     * @param out_data Output struct to receive normalized inputs.
     * @return true if fresh data is available; false otherwise.
     */
    bool getLatestInput(GamepadData* out_data);

    /**
     * @brief Checks if a gamepad is currently connected and active.
     */
    bool isConnected() const { return _connected_controller != nullptr; }

    /**
     * @brief Sets the DualShock 4 Lightbar RGB color.
     */
    void setLightbarColor(uint8_t r, uint8_t g, uint8_t b);

    /**
     * @brief Triggers DualShock 4 rumble vibration.
     */
    void setRumble(uint8_t weak_mag, uint8_t strong_mag, uint16_t duration_ms);

    /**
     * @brief Manually clears stored paired Bluetooth keys (for development factory reset).
     * NOTE: This is NOT called automatically at boot to preserve persistent pairing.
     */
    void forgetBluetoothKeys();

    /**
     * @brief Prints local ESP32 Bluetooth BD_ADDR.
     */
    void printLocalBdAddress() const;

    // Internal callback bridges
    void onControllerConnected(ControllerPtr ctl);
    void onControllerDisconnected(ControllerPtr ctl);

private:
    FailsafeManager* _failsafe;
    ControllerPtr    _connected_controller;
    GamepadData      _current_data;
    bool             _has_new_data;
    uint32_t         _last_poll_time;

    void processGamepadInputs(ControllerPtr ctl, uint32_t now_ms);
};
