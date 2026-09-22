#pragma once

#include <stdint.h>

// ==============================================================================
// GLOBAL WIRELESS & PROTOCOL CONFIGURATION
// ==============================================================================

// ESP-NOW operates on 2.4 GHz Wi-Fi.
// The ESP32-S2 controller and ESP32 DevKit robot MUST use the exact same channel.
#ifndef ESPNOW_WIFI_CHANNEL
#define ESPNOW_WIFI_CHANNEL 1
#endif

// Protocol Magic & Version identifiers
#define SOCCERBOT_PROTOCOL_MAGIC    0x5342  // ASCII 'S', 'B' (SoccerBot)
#define SOCCERBOT_PROTOCOL_VERSION  0x01

// Control update rate (Hz) and period (ms)
#define CONTROL_RATE_HZ             50
#define CONTROL_PERIOD_MS           (1000 / CONTROL_RATE_HZ)  // 20 ms

// Communication timeout in milliseconds for safety failsafe activation
#ifndef CONTROL_TIMEOUT_MS
#define CONTROL_TIMEOUT_MS          150  // 150 ms timeout (~7-8 dropped packets at 50Hz)
#endif

// Axis normalization bounds
#define AXIS_MIN_VALUE              (-1000)
#define AXIS_MAX_VALUE              (+1000)
#define AXIS_DEADZONE_DEFAULT       80   // 80 / 1000 = 8% (0.08)

// Button mask flags
#define BTN_KICK_TRIGGER            (1 << 0)
#define BTN_DRIBBLE_TOGGLE          (1 << 1)
#define BTN_BOOST_MODE              (1 << 2)
#define BTN_EMERGENCY_STOP          (1 << 3)
#define BTN_MODE_SELECT             (1 << 4)
#define BTN_CALIBRATE               (1 << 5)
