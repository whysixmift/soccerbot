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
#ifndef CONTROLLER_TIMEOUT_MS
#define CONTROLLER_TIMEOUT_MS   150
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

// Kinematics and Joystick Deadzones
#define JOYSTICK_DEADZONE       80      // 8% deadzone (80 / 1000)

// Axis Inversions (calibrate according to stick polarities)
#define INVERT_AXIS_LX          false   // Invert strafe axis (Left Stick X)
#define INVERT_AXIS_LY          false   // Invert forward/back axis (Left Stick Y)
#define INVERT_AXIS_RX          false   // Invert rotation axis (Right Stick X)

// Motor Channel Inversions (Physical orientation calibration)
// In a 2-wheel differential chassis, left and right motors are physically mounted facing opposite directions.
// Setting INVERT_CH2_RIGHT = true aligns both wheels so that positive linear velocity drives both wheels forward.
#ifndef INVERT_CH1_LEFT
#define INVERT_CH1_LEFT         false
#endif

#ifndef INVERT_CH2_RIGHT
#define INVERT_CH2_RIGHT        true
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
