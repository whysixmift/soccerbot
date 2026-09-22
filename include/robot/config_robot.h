#pragma once

#include "common/config_common.h"
#include "robot/hardware_pins.h"

// ==============================================================================
// ROBOT FIRMWARE CONFIGURATION (DUALSHOCK 4 + BLUEPAD32)
// ==============================================================================

#ifndef ROBOT_ID
#if defined(BOT_VARIANT_BOT_2)
#define ROBOT_ID 2
#else
#define ROBOT_ID 1
#endif
#endif

// Controller Watchdog Timeout (ms)
// If no controller update is received within this time, motors immediately stop.
// 350ms provides resilience against 2.4GHz RF packet drops while ensuring rapid safety cutoff
#ifndef CONTROLLER_TIMEOUT_MS
#define CONTROLLER_TIMEOUT_MS   350
#endif

// Optional Target DualShock 4 Controller Bluetooth MAC Allow-list
// If defined, the robot will reject any controller whose MAC does not match.
// Example: #define ALLOWED_CONTROLLER_MAC { 0x1C, 0xA0, 0xB8, 0x12, 0x34, 0x56 }
// If left undefined, the robot will accept the first paired DualShock 4 controller.
// #define ALLOWED_CONTROLLER_MAC { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }

// Motor PWM Parameters (LEDC)
#define MOTOR_PWM_FREQ_HZ       20000   // 20 kHz ultrasonic frequency (prevents audible whine)
#define MOTOR_PWM_RESOLUTION_BITS 10     // 10-bit resolution (duty: 0 to 1023)
#define MOTOR_PWM_MAX_DUTY      ((1 << MOTOR_PWM_RESOLUTION_BITS) - 1) // 1023

// Motor Speed Limits (-1000 to +1000 range)
#define MOTOR_MAX_ALLOWED_SPEED 1000    // Top speed limit (1000 = 100%)

// Acceleration & Deceleration Slew Rate Limiter (Speed units per second)
// Eliminates jerky movements ("kaku"), wheel slip, and inductive back-EMF spikes
#define MOTOR_ACCEL_RAMP_RATE   2000    // Smooth acceleration (0 to 380 in ~190ms)
#define MOTOR_DECEL_RAMP_RATE   3000    // Responsive braking (380 to 0 in ~125ms)

// Kinematics and Joystick Deadzones (Tuned for responsive, smooth micro-control)
#define JOYSTICK_DEADZONE       60      // 6% deadzone: suppresses stick drift at center
#define JOYSTICK_EXPO_PERCENT   25      // 25% exponential curve: gentle micro-aiming near center

// Speed and Sensitivity Scaling (0 to 1000 range)
#define LINEAR_SPEED_SCALE      400     // 40% standard cruising speed
#define TURN_SPEED_SCALE        200     // 20% smooth turning speed (tidak terlalu sensitif / anti-twitch)
#define BOOST_LINEAR_SPEED_SCALE 1000   // 100% full power when R2 / Boost is pulled
#define BOOST_TURN_SPEED_SCALE   450    // 45% max turning speed in Boost mode


// Axis Inversions (Standard reference frame: Stick UP = +1000, Stick RIGHT = +1000)
#define INVERT_AXIS_LX          false   // Left Stick X (Strafe)
#define INVERT_AXIS_LY          false   // Left Stick Y (Forward/Backward)
#define INVERT_AXIS_RX          false   // Right Stick X (Rotation)

// Motor Channel Inversions (Physical orientation calibration)
// In a 2-wheel differential chassis, motors face opposite directions.
// Setting CH1 (Left) = true and CH2 (Right) = false aligns forward rotation with forward joystick.
#ifndef INVERT_CH1_LEFT
#define INVERT_CH1_LEFT         true
#endif

#ifndef INVERT_CH2_RIGHT
#define INVERT_CH2_RIGHT        false
#endif

// Mecanum Individual Wheel Inversions (if used in 4-wheel kinematics)
#define INVERT_WHEEL_FL         false
#define INVERT_WHEEL_FR         false
#define INVERT_WHEEL_RL         false
#define INVERT_WHEEL_RR         false

// Motor Diagnostic Test Mode (Disabled by default for safety)
#define MOTOR_TEST_ON_BOOT      false   // MUST be false for safety
#define MOTOR_TEST_SAFE_DUTY    200     // ~20% PWM for bench testing
#define MOTOR_TEST_STEP_MS      1000    // 1 second per phase

