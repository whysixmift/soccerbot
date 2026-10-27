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
#define PIN_D25   25
#define PIN_D26   26
#define PIN_D27   27
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
#if defined(BOT_VARIANT_C3_SUPERMINI)
    #define CURRENT_BOT_VARIANT_NAME "C3_SUPERMINI"
    static const RobotPinConfig HARDWARE_PINS = {
        // Channel 1 — LEFT (C3 SuperMini: LPWM=GPIO0, RPWM=GPIO1)
        .ch1_left = {
            .lpwm = 0,   // GPIO0 -> LPWM
            .rpwm = 1,   // GPIO1 -> RPWM
            .lis  = -1,  // Not connected
            .ris  = -1   // Not connected
        },
        // Channel 2 — RIGHT (C3 SuperMini: LPWM=GPIO3, RPWM=GPIO4)
        .ch2_right = {
            .lpwm = 3,   // GPIO3 -> LPWM
            .rpwm = 4,   // GPIO4 -> RPWM
            .lis  = -1,  // Not connected
            .ris  = -1   // Not connected
        }
    };
#else
    // Authoritative hardware mapping:
    // Motor kiri:  RPWM = GPIO13, LPWM = GPIO14
    // Motor kanan: RPWM = GPIO25, LPWM = GPIO26
    #if defined(BOT_VARIANT_BOT_2)
        #define CURRENT_BOT_VARIANT_NAME "BOT_2"
    #else
        #define CURRENT_BOT_VARIANT_NAME "BOT_1"
    #endif

    static const RobotPinConfig HARDWARE_PINS = {
        // Channel 1 — LEFT (BTS7960: LPWM = GPIO14, RPWM = GPIO13)
        .ch1_left = {
            .lpwm = PIN_D14, // GPIO14 -> LPWM
            .rpwm = PIN_D13, // GPIO13 -> RPWM
            .lis  = -1,      // Not connected
            .ris  = -1       // Not connected
        },
        // Channel 2 — RIGHT (BTS7960: LPWM = GPIO26, RPWM = GPIO25)
        .ch2_right = {
            .lpwm = PIN_D26, // GPIO26 -> LPWM
            .rpwm = PIN_D25, // GPIO25 -> RPWM
            .lis  = -1,      // Not connected
            .ris  = -1       // Not connected
        }
    };
#endif
