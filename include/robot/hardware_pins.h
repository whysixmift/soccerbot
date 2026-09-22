#pragma once

#include <stdint.h>

// ==============================================================================
// VERIFIED PHYSICAL PIN DEFINITIONS (ESP32 DevKit V1)
// ==============================================================================
// DO NOT INVENT GPIO assignments.
// Board label mapping verified from physical hardware:
// D13 = GPIO13
// D14 = GPIO14
// D27 = GPIO27
// D26 = GPIO26
// D4  = GPIO4
// RX2 = GPIO16  <-- Configured as ordinary GPIO, NOT UART2!
// TX2 = GPIO17  <-- Configured as ordinary GPIO, NOT UART2!
// D18 = GPIO18

#define PIN_D13   13
#define PIN_D14   14
#define PIN_D27   27
#define PIN_D26   26
#define PIN_D4    4
#define PIN_RX2   16  // GPIO16 - NEVER use for UART2
#define PIN_TX2   17  // GPIO17 - NEVER use for UART2
#define PIN_D18   18

/**
 * @brief Pin configuration for a single BTS7960 / IBT-2 dual half-bridge driver channel.
 */
struct BTS7960ChannelPins {
    int lpwm; // Left PWM pin (Forward)
    int rpwm; // Right PWM pin (Reverse)
    int lis;  // Left current alarm / sense pin
    int ris;  // Right current alarm / sense pin
};

/**
 * @brief Robot motor driver pin assignment for both channels.
 */
struct RobotPinConfig {
    BTS7960ChannelPins ch1_left;
    BTS7960ChannelPins ch2_right;
};

// Define variant configurations
#if defined(BOT_VARIANT_BOT_2)
    #define CURRENT_BOT_VARIANT_NAME "BOT_2"
    static const RobotPinConfig HARDWARE_PINS = {
        // Channel 1 — LEFT (BOT 2: LPWM=GPIO26, RPWM=GPIO27, IS pins unused)
        .ch1_left = {
            .lpwm = PIN_D26, // GPIO26 -> LPWM
            .rpwm = PIN_D27, // GPIO27 -> RPWM
            .lis  = -1,      // Not connected
            .ris  = -1       // Not connected
        },
        // Channel 2 — RIGHT (BOT 2: LPWM=GPIO18, RPWM=GPIO17, IS pins unused)
        .ch2_right = {
            .lpwm = PIN_D18, // GPIO18 -> LPWM
            .rpwm = PIN_TX2, // GPIO17 -> RPWM (TX2 pin used as ordinary GPIO)
            .lis  = -1,      // Not connected
            .ris  = -1       // Not connected
        }
    };
#else
    // Default to BOT_1
    #define CURRENT_BOT_VARIANT_NAME "BOT_1"
    static const RobotPinConfig HARDWARE_PINS = {
        // Channel 1 — LEFT (BOT 1: GPIO13 -> L_IS, GPIO14 -> R_IS)
        .ch1_left = {
            .lpwm = PIN_D26, // GPIO26 -> L_PWM
            .rpwm = PIN_D27, // GPIO27 -> R_PWM
            .lis  = PIN_D13, // GPIO13 -> L_IS
            .ris  = PIN_D14  // GPIO14 -> R_IS
        },
        // Channel 2 — RIGHT (BOT 1: GPIO4 -> L_IS, GPIO16 -> R_IS)
        .ch2_right = {
            .lpwm = PIN_D18, // GPIO18 -> LPWM
            .rpwm = PIN_TX2, // GPIO17 -> RPWM (TX2 pin used as GPIO)
            .lis  = PIN_D4,  // GPIO4  -> L_IS
            .ris  = PIN_RX2  // GPIO16 -> R_IS (RX2 pin used as GPIO)
        }
    };
#endif
