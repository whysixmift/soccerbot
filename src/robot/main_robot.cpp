#ifdef TARGET_ROBOT

#include <Arduino.h>
#include <WiFi.h>
#include "common/logging.h"
#include "common/protocol.h"
#include "robot/config_robot.h"
#include "robot/hardware_pins.h"
#include "robot/motor_controller.h"
#include "robot/kinematics.h"
#include "robot/failsafe.h"

#include "robot/bluepad32_receiver.h"

static const char* TAG = "ROBOT_MAIN";

// Core subsystems
static MotorController    s_motor_ctrl;
static FailsafeManager    s_failsafe;
static Bluepad32Receiver  s_rx;
static KinematicsConfig   s_kinematics_cfg;

#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(BOT_VARIANT_BOT_2) || defined(BOT_VARIANT_C3_SUPERMINI)
#define RECEIVER_TYPE_NAME "Bluetooth BLE Gamepad Host (Bluepad32 BLE)"
#else
#define RECEIVER_TYPE_NAME "Bluetooth Classic Gamepad Host (DualShock 4)"
#endif


// Periodic telemetry timer
static uint32_t s_last_telemetry_time = 0;

static void printBanner() {
    LOG_INFO(TAG, "==================================================");
    LOG_INFO(TAG, "  MINI SOCCER ROBOT - MULTI-CONTROLLER FIRMWARE   ");
    LOG_INFO(TAG, "==================================================");
    LOG_INFO(TAG, " Bot Variant   : %s", CURRENT_BOT_VARIANT_NAME);
    LOG_INFO(TAG, " Robot ID      : %d", ROBOT_ID);
    LOG_INFO(TAG, " Control Mode  : %s", RECEIVER_TYPE_NAME);
    LOG_INFO(TAG, " Safe Timeout  : %d ms", CONTROLLER_TIMEOUT_MS);
    LOG_INFO(TAG, " PWM Freq/Res  : %d Hz, %d bits", MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION_BITS);
    LOG_INFO(TAG, " Max Speed     : %d / 1000", MOTOR_MAX_ALLOWED_SPEED);
    LOG_INFO(TAG, " Hardware Map  : Verified BTS7960 Pinout");
    LOG_INFO(TAG, " Ch1 (Left)    : LPWM=GPIO%d, RPWM=GPIO%d",
             HARDWARE_PINS.ch1_left.lpwm, HARDWARE_PINS.ch1_left.rpwm);
    LOG_INFO(TAG, " Ch2 (Right)   : LPWM=GPIO%d, RPWM=GPIO%d",
             HARDWARE_PINS.ch2_right.lpwm, HARDWARE_PINS.ch2_right.rpwm);
    LOG_INFO(TAG, "==================================================");
}

static void handleSerialCommands() {
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        switch (c) {
            case 't':
            case 'T':
                LOG_INFO(TAG, "Serial command: Start Motor Diagnostic Test");
                s_motor_ctrl.startMotorTest();
                break;
            case 'x':
            case 'X':
                LOG_INFO(TAG, "Serial command: Emergency Stop");
                s_motor_ctrl.stopMotorTest();
                s_failsafe.triggerEmergencyStop();
                break;
            case 'c':
            case 'C':
                LOG_INFO(TAG, "Serial command: Clear Emergency Stop");
                s_failsafe.clearEmergencyStop();
                break;
            case 'f':
            case 'F':
#if !defined(BOT_VARIANT_C3_SUPERMINI)
                LOG_WARN(TAG, "Serial command: Forget Bluetooth Keys (Factory Reset)");
                s_rx.forgetBluetoothKeys();
#endif
                break;
            case 's':
            case 'S': {
                LOG_INFO(TAG, "--- Detailed Status ---");
                LOG_INFO(TAG, " State: %s, Gamepad Connected: %s, Failsafe Triggers: %lu",
                         s_failsafe.getStateString(),
                         s_rx.isConnected() ? "YES" : "NO",
                         (unsigned long)s_failsafe.getFailsafeCount());
                LOG_INFO(TAG, " Motor Ch1 Speed: %d, Ch2 Speed: %d",
                         s_motor_ctrl.getCh1Speed(), s_motor_ctrl.getCh2Speed());
                break;
            }
            case 'h':
            case '?':
                LOG_INFO(TAG, "Commands: [t]=Motor Test, [x]=E-Stop, [c]=Clear E-Stop, [f]=Forget BT Keys, [s]=Status, [h]=Help");
                break;
            default:
                break;
        }
    }
}

void setup() {
    // Explicitly shut down WiFi radio to dedicate 100% of RF frontend and antenna time-slices to Bluetooth Classic
    WiFi.mode(WIFI_OFF);

    // 1. Initialize Serial communication for telemetry (UART0 on standard pins)
    // GPIO16/GPIO17 are strictly reserved for motor control and NEVER assigned to UART
    Serial.begin(115200);
    delay(200);

    printBanner();

    // 2. Initialize Kinematics configuration
    s_kinematics_cfg.deadzone     = JOYSTICK_DEADZONE;
    s_kinematics_cfg.linear_scale = LINEAR_SPEED_SCALE;
    s_kinematics_cfg.turn_scale   = TURN_SPEED_SCALE;
    s_kinematics_cfg.expo_percent = JOYSTICK_EXPO_PERCENT;
    s_kinematics_cfg.invert_lx    = INVERT_AXIS_LX;
    s_kinematics_cfg.invert_ly    = INVERT_AXIS_LY;
    s_kinematics_cfg.invert_rx    = INVERT_AXIS_RX;
    s_kinematics_cfg.invert_fl    = INVERT_WHEEL_FL;
    s_kinematics_cfg.invert_fr    = INVERT_WHEEL_FR;
    s_kinematics_cfg.invert_rl    = INVERT_WHEEL_RL;
    s_kinematics_cfg.invert_rr    = INVERT_WHEEL_RR;
    s_kinematics_cfg.invert_ch1   = false;
    s_kinematics_cfg.invert_ch2   = false;

    // 3. Initialize Motor Controller (Guaranteed safe 0 RPM boot & hardware polarity alignment)
    bool motor_ok = s_motor_ctrl.init(HARDWARE_PINS,
                                     INVERT_CH1_LEFT,
                                     INVERT_CH2_RIGHT,
                                     MOTOR_MAX_ALLOWED_SPEED,
                                     MOTOR_ACCEL_RAMP_RATE,
                                     MOTOR_DECEL_RAMP_RATE);
    if (!motor_ok) {
        LOG_ERROR(TAG, "CRITICAL: Motor initialization failed! Halting.");
        while (1) { delay(1000); }
    }

    // 4. Initialize Failsafe Watchdog Manager
    s_failsafe.init(&s_motor_ctrl, CONTROLLER_TIMEOUT_MS);

    // 5. Initialize Wireless Controller Host (Dabble BLE for C3 or Bluepad32 DS4 for Classic)
    bool rx_ok = s_rx.init(&s_failsafe);
    if (!rx_ok) {
        LOG_ERROR(TAG, "CRITICAL: Controller receiver initialization failed! Halting.");
        while (1) { delay(1000); }
    }

    LOG_INFO(TAG, "Robot initialized successfully. Standing by for %s connection...", RECEIVER_TYPE_NAME);
}

void loop() {
    uint32_t now = millis();

    // 1. Process bench test serial commands
    handleSerialCommands();

    // 2. Poll Bluetooth / BLE stack for controller updates
    s_rx.update(now);

    // 3. Evaluate safety watchdog / failsafe state machine
    s_failsafe.checkTimeout(now);

    // 4. Update motor outputs & smooth ramping
    if (s_motor_ctrl.isMotorTestRunning()) {
        s_motor_ctrl.updateMotorTest(now);
    } else if (s_failsafe.isActive() && s_rx.isConnected()) {
        GamepadData data;
        if (s_rx.getLatestInput(&data) && data.connected) {
            // Full power boost via R2 trigger (analog throttle 0..255) or Square button
            if (data.kick > 10) {
                // Progressive full throttle boost on R2 squeeze up to BOOST_LINEAR_SPEED_SCALE
                int32_t r2_factor = (int32_t)data.kick;
                s_kinematics_cfg.linear_scale = (int16_t)(LINEAR_SPEED_SCALE + ((BOOST_LINEAR_SPEED_SCALE - LINEAR_SPEED_SCALE) * r2_factor) / 255);
                s_kinematics_cfg.turn_scale   = (int16_t)(TURN_SPEED_SCALE + ((BOOST_TURN_SPEED_SCALE - TURN_SPEED_SCALE) * r2_factor) / 255);
            } else if (data.buttons & BTN_BOOST_MODE) {
                // Instant full power boost on Square/Triangle button
                s_kinematics_cfg.linear_scale = BOOST_LINEAR_SPEED_SCALE;
                s_kinematics_cfg.turn_scale   = BOOST_TURN_SPEED_SCALE;
            } else {
                // Standard smooth cruising speed
                s_kinematics_cfg.linear_scale = LINEAR_SPEED_SCALE;
                s_kinematics_cfg.turn_scale   = TURN_SPEED_SCALE;
            }


            DualChannelSpeeds speeds = Kinematics::computeDualChannel(data.lx, data.ly, data.rx, s_kinematics_cfg);
            s_motor_ctrl.setSpeeds(speeds.ch1_left, speeds.ch2_right);
        }
        s_motor_ctrl.update(now);
    } else {
        s_motor_ctrl.stopAll();
    }

    // 5. Throttled periodic telemetry output (1 Hz)
    if (now - s_last_telemetry_time >= 1000) {
        s_last_telemetry_time = now;
        GamepadData data;
        s_rx.getLatestInput(&data);

        LOG_INFO(TAG, "[TELEMETRY] State: %-22s | Gamepad: %s | Sticks (LX:%4d LY:%4d RX:%4d) | Motor L:%4d R:%4d",
                 s_failsafe.getStateString(),
                 s_rx.isConnected() ? "CONNECTED" : "DISCONNECTED",
                 data.lx, data.ly, data.rx,
                 s_motor_ctrl.getCh1Speed(),
                 s_motor_ctrl.getCh2Speed());
    }
}


#endif // TARGET_ROBOT
