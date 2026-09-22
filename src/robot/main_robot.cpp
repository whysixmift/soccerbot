#ifdef TARGET_ROBOT

#include <Arduino.h>
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
static Bluepad32Receiver  s_bp32_rx;
static KinematicsConfig   s_kinematics_cfg;

// Periodic telemetry timer
static uint32_t s_last_telemetry_time = 0;

static void printBanner() {
    LOG_INFO(TAG, "==================================================");
    LOG_INFO(TAG, "  MINI SOCCER ROBOT - DUALSHOCK 4 & BLUEPAD32     ");
    LOG_INFO(TAG, "==================================================");
    LOG_INFO(TAG, " Bot Variant   : %s", CURRENT_BOT_VARIANT_NAME);
    LOG_INFO(TAG, " Robot ID      : %d", ROBOT_ID);
    LOG_INFO(TAG, " Control Mode  : Bluetooth Classic Direct (DS4)");
    LOG_INFO(TAG, " Safe Timeout  : %d ms", CONTROLLER_TIMEOUT_MS);
    LOG_INFO(TAG, " PWM Freq/Res  : %d Hz, %d bits", MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION_BITS);
    LOG_INFO(TAG, " Max Speed     : %d / 1000", MOTOR_MAX_ALLOWED_SPEED);
    LOG_INFO(TAG, " Hardware Map  : Verified BTS7960 Pinout");
    LOG_INFO(TAG, " Ch1 (Left)    : LPWM=GPIO%d, RPWM=GPIO%d, L_IS=GPIO%d, R_IS=GPIO%d",
             HARDWARE_PINS.ch1_left.lpwm, HARDWARE_PINS.ch1_left.rpwm,
             HARDWARE_PINS.ch1_left.lis, HARDWARE_PINS.ch1_left.ris);
    LOG_INFO(TAG, " Ch2 (Right)   : LPWM=GPIO%d, RPWM=GPIO%d, L_IS=GPIO%d, R_IS=GPIO%d",
             HARDWARE_PINS.ch2_right.lpwm, HARDWARE_PINS.ch2_right.rpwm,
             HARDWARE_PINS.ch2_right.lis, HARDWARE_PINS.ch2_right.ris);
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
                LOG_WARN(TAG, "Serial command: Forget Bluetooth Keys (Factory Reset)");
                s_bp32_rx.forgetBluetoothKeys();
                break;
            case 's':
            case 'S': {
                LOG_INFO(TAG, "--- Detailed Status ---");
                LOG_INFO(TAG, " State: %s, DS4 Connected: %s, Failsafe Triggers: %lu",
                         s_failsafe.getStateString(),
                         s_bp32_rx.isConnected() ? "YES" : "NO",
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
    // 1. Initialize Serial communication for telemetry (UART0 on standard pins)
    // GPIO16/GPIO17 are strictly reserved for motor control and NEVER assigned to UART
    Serial.begin(115200);
    delay(200);

    printBanner();

    // 2. Initialize Kinematics configuration
    s_kinematics_cfg.deadzone   = JOYSTICK_DEADZONE;
    s_kinematics_cfg.invert_lx  = INVERT_AXIS_LX;
    s_kinematics_cfg.invert_ly  = INVERT_AXIS_LY;
    s_kinematics_cfg.invert_rx  = INVERT_AXIS_RX;
    s_kinematics_cfg.invert_fl  = INVERT_WHEEL_FL;
    s_kinematics_cfg.invert_fr  = INVERT_WHEEL_FR;
    s_kinematics_cfg.invert_rl  = INVERT_WHEEL_RL;
    s_kinematics_cfg.invert_rr  = INVERT_WHEEL_RR;
    s_kinematics_cfg.invert_ch1 = false;
    s_kinematics_cfg.invert_ch2 = false;

    // 3. Initialize Motor Controller (Guaranteed safe 0 RPM boot & hardware polarity alignment)
    bool motor_ok = s_motor_ctrl.init(HARDWARE_PINS,
                                     INVERT_CH1_LEFT,
                                     INVERT_CH2_RIGHT,
                                     MOTOR_MAX_ALLOWED_SPEED);
    if (!motor_ok) {
        LOG_ERROR(TAG, "CRITICAL: Motor initialization failed! Halting.");
        while (1) { delay(1000); }
    }

    // 4. Initialize Failsafe Watchdog Manager
    s_failsafe.init(&s_motor_ctrl, CONTROLLER_TIMEOUT_MS);

    // 5. Initialize Bluepad32 Bluetooth Gamepad Host
    bool bp32_ok = s_bp32_rx.init(&s_failsafe);
    if (!bp32_ok) {
        LOG_ERROR(TAG, "CRITICAL: Bluepad32 initialization failed! Halting.");
        while (1) { delay(1000); }
    }

    LOG_INFO(TAG, "Robot initialized successfully. Standing by for DualShock 4 connection...");
}

void loop() {
    uint32_t now = millis();

    // 1. Process bench test serial commands
    handleSerialCommands();

    // 2. Poll Bluepad32 Bluetooth stack for controller updates
    s_bp32_rx.update(now);

    // 3. Evaluate safety watchdog / failsafe state machine
    s_failsafe.checkTimeout(now);

    // 4. Update motor outputs
    if (s_motor_ctrl.isMotorTestRunning()) {
        s_motor_ctrl.updateMotorTest(now);
    } else if (s_failsafe.isActive() && s_bp32_rx.isConnected()) {
        GamepadData data;
        if (s_bp32_rx.getLatestInput(&data) && data.connected) {
            DualChannelSpeeds speeds = Kinematics::computeDualChannel(data.lx, data.ly, data.rx, s_kinematics_cfg);
            s_motor_ctrl.setSpeeds(speeds.ch1_left, speeds.ch2_right);
        }
    } else {
        s_motor_ctrl.stopAll();
    }

    // 5. Throttled periodic telemetry output (1 Hz)
    if (now - s_last_telemetry_time >= 1000) {
        s_last_telemetry_time = now;
        GamepadData data;
        s_bp32_rx.getLatestInput(&data);

        LOG_INFO(TAG, "[TELEMETRY] State: %-22s | DS4: %s | Sticks (LX:%4d LY:%4d RX:%4d) | Motor L:%4d R:%4d",
                 s_failsafe.getStateString(),
                 s_bp32_rx.isConnected() ? "CONNECTED" : "DISCONNECTED",
                 data.lx, data.ly, data.rx,
                 s_motor_ctrl.getCh1Speed(),
                 s_motor_ctrl.getCh2Speed());
    }
}

#endif // TARGET_ROBOT
