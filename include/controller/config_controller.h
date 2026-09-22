#pragma once

#include "common/config_common.h"

// ==============================================================================
// ESP32-S2 CONTROLLER GATEWAY CONFIGURATION
// ==============================================================================

// Target Robot ID (Must match the robot's ROBOT_ID: 1 for BOT_1, 2 for BOT_2)
#ifndef TARGET_ROBOT_ID
#define TARGET_ROBOT_ID         1
#endif

// Target Robot Physical MAC Address (ESP-NOW Unicast)
// IMPORTANT: Replace with your robot's exact MAC address (printed on robot boot)
#ifndef TARGET_ROBOT_MAC
#define TARGET_ROBOT_MAC        { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF } // Default placeholder / broadcast for unconfigured
#endif

// Controller Joystick Deadzone (8% default)
#define CONTROLLER_DEADZONE     80

// Axis Inversions (calibrate according to physical game controller stick polarities)
#define CONTROLLER_INVERT_LX    false   // Invert strafe axis
#define CONTROLLER_INVERT_LY    false   // Invert forward/backward axis (stick forward is typically negative in raw HID)
#define CONTROLLER_INVERT_RX    false   // Invert rotation axis
#define CONTROLLER_INVERT_RY    false   // Invert auxiliary axis

// USB Host Hardware Notes:
// ESP32-S2 features built-in USB OTG peripheral on:
//   GPIO19 -> USB D- (Data Minus)
//   GPIO20 -> USB D+ (Data Plus)
//   5V / VBUS -> Provides 5V power to the connected USB Game Controller
#define USB_HOST_PIN_DM         19
#define USB_HOST_PIN_DP         20
