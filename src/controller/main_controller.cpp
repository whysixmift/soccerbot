#ifdef TARGET_CONTROLLER

#include <Arduino.h>
#include "common/logging.h"
#include "common/protocol.h"
#include "controller/config_controller.h"
#include "controller/controller_input.h"
#include "controller/usb_hid_host.h"
#include "controller/espnow_sender.h"

static const char* TAG = "CTRL_MAIN";

// Core subsystems
static ControllerInputProcessor s_input_processor;
static UsbHidHost               s_usb_host;
static EspNowSender             s_espnow_tx;

// Transmission timing
static uint32_t s_last_send_time = 0;
static uint32_t s_last_telemetry_time = 0;

// Default configured target MAC address
static const uint8_t s_default_robot_mac[6] = TARGET_ROBOT_MAC;

static void printBanner() {
    LOG_INFO(TAG, "==================================================");
    LOG_INFO(TAG, "   MINI SOCCER ROBOT CONTROLLER GATEWAY (ESP32-S2)");
    LOG_INFO(TAG, "==================================================");
    LOG_INFO(TAG, " Target Robot ID: %d", TARGET_ROBOT_ID);
    LOG_INFO(TAG, " WiFi Channel   : %d", ESPNOW_WIFI_CHANNEL);
    LOG_INFO(TAG, " Control Rate   : %d Hz (%d ms interval)", CONTROL_RATE_HZ, CONTROL_PERIOD_MS);
    LOG_INFO(TAG, " Target MAC     : %02X:%02X:%02X:%02X:%02X:%02X",
             s_default_robot_mac[0], s_default_robot_mac[1], s_default_robot_mac[2],
             s_default_robot_mac[3], s_default_robot_mac[4], s_default_robot_mac[5]);
    LOG_INFO(TAG, " USB Host PHY   : Built-in Native USB (D-=GPIO%d, D+=GPIO%d)",
             USB_HOST_PIN_DM, USB_HOST_PIN_DP);
    LOG_INFO(TAG, "==================================================");
}

static void handleSerialCommands() {
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        switch (c) {
            // Simulated Gamepad Inputs (Bench Testing)
            case 'w':
            case 'W':
                LOG_INFO(TAG, "[SIMULATOR] Forward (+500)");
                s_usb_host.injectTestInput(0, 500, 0, 0, 0, 0);
                break;
            case 's':
            case 'S':
                LOG_INFO(TAG, "[SIMULATOR] Reverse (-500)");
                s_usb_host.injectTestInput(0, -500, 0, 0, 0, 0);
                break;
            case 'a':
            case 'A':
                LOG_INFO(TAG, "[SIMULATOR] Strafe Left (-500)");
                s_usb_host.injectTestInput(-500, 0, 0, 0, 0, 0);
                break;
            case 'd':
            case 'D':
                LOG_INFO(TAG, "[SIMULATOR] Strafe Right (+500)");
                s_usb_host.injectTestInput(500, 0, 0, 0, 0, 0);
                break;
            case 'q':
            case 'Q':
                LOG_INFO(TAG, "[SIMULATOR] Rotate CCW (-500)");
                s_usb_host.injectTestInput(0, 0, -500, 0, 0, 0);
                break;
            case 'e':
            case 'E':
                LOG_INFO(TAG, "[SIMULATOR] Rotate CW (+500)");
                s_usb_host.injectTestInput(0, 0, 500, 0, 0, 0);
                break;
            case ' ':
                LOG_INFO(TAG, "[SIMULATOR] KICK!");
                s_usb_host.injectTestInput(0, 0, 0, BTN_KICK_TRIGGER, 255, 0);
                break;
            case 'x':
            case 'X':
                LOG_INFO(TAG, "[SIMULATOR] STOP / NEUTRAL");
                s_usb_host.injectTestInput(0, 0, 0, 0, 0, 0);
                break;
            case '?':
            case 'h':
                LOG_INFO(TAG, "Simulator: [W/S]=Fwd/Rev, [A/D]=Strafe, [Q/E]=Turn, [Space]=Kick, [X]=Stop");
                break;
            default:
                break;
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(200);

    printBanner();

    // 1. Initialize Input Processor & Calibrations
    s_input_processor.init(CONTROLLER_DEADZONE,
                           CONTROLLER_INVERT_LX,
                           CONTROLLER_INVERT_LY,
                           CONTROLLER_INVERT_RX,
                           CONTROLLER_INVERT_RY);

    // 2. Initialize USB HID Host Driver
    bool usb_ok = s_usb_host.init(&s_input_processor);
    if (!usb_ok) {
        LOG_ERROR(TAG, "CRITICAL: USB Host initialization failed!");
    }

    // 3. Initialize ESP-NOW Unicast Sender
    bool tx_ok = s_espnow_tx.init(TARGET_ROBOT_ID, s_default_robot_mac, ESPNOW_WIFI_CHANNEL);
    if (!tx_ok) {
        LOG_ERROR(TAG, "CRITICAL: ESP-NOW Sender initialization failed!");
    }

    LOG_INFO(TAG, "Controller Gateway initialized. Ready for control stream.");
}

void loop() {
    uint32_t now = millis();

    // 1. Process bench test keyboard commands
    handleSerialCommands();

    // 2. Process USB Host HID state
    s_usb_host.update(now);

    // 3. 50 Hz Control Transmission Loop (every 20 ms)
    if (now - s_last_send_time >= CONTROL_PERIOD_MS) {
        s_last_send_time = now;
        const ControllerInputState& input = s_usb_host.getState();
        s_espnow_tx.sendCommand(input);
    }

    // 4. Throttled Telemetry output (1 Hz)
    if (now - s_last_telemetry_time >= 1000) {
        s_last_telemetry_time = now;
        const EspNowSenderStats& st = s_espnow_tx.getStats();
        const ControllerInputState& inp = s_usb_host.getState();

        LOG_INFO(TAG, "[TELEMETRY] USB: %s | Sent: %lu (OK: %lu, Fail: %lu) | LX: %4d, LY: %4d, RX: %4d",
                 inp.connected ? "CONNECTED" : "DISCONNECTED",
                 (unsigned long)st.packets_sent,
                 (unsigned long)st.send_success,
                 (unsigned long)st.send_fail,
                 inp.lx, inp.ly, inp.rx);
    }
}

#endif // TARGET_CONTROLLER
