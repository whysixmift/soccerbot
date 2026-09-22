#include "robot/bluepad32_receiver.h"
#include "robot/config_robot.h"
#include "common/logging.h"
#include <string.h>
#include <Bluepad32.h>

#ifdef ARDUINO
#include <esp_bt.h>
#endif

static const char* TAG = "BLUEPAD32";
static Bluepad32Receiver* s_bp32_instance = nullptr;

// C-style callback wrappers for Bluepad32 C API
static void onConnectedWrapper(ControllerPtr ctl) {
    if (s_bp32_instance) {
        s_bp32_instance->onControllerConnected(ctl);
    }
}

static void onDisconnectedWrapper(ControllerPtr ctl) {
    if (s_bp32_instance) {
        s_bp32_instance->onControllerDisconnected(ctl);
    }
}

Bluepad32Receiver::Bluepad32Receiver()
    : _failsafe(nullptr),
      _connected_controller(nullptr),
      _has_new_data(false),
      _last_poll_time(0) {
    memset(&_current_data, 0, sizeof(_current_data));
    s_bp32_instance = this;
}

bool Bluepad32Receiver::init(FailsafeManager* failsafe) {
    _failsafe = failsafe;
    _connected_controller = nullptr;
    _has_new_data = false;
    _last_poll_time = 0;
    memset(&_current_data, 0, sizeof(_current_data));
    s_bp32_instance = this;

    LOG_INFO(TAG, "Initializing Bluepad32 Bluetooth Gamepad Host (v%s)...", BP32.firmwareVersion());

#ifdef ARDUINO
    // Set Bluetooth Classic & BLE RF TX power to MAXIMUM (+9 dBm)
    // Minimizes packet loss and disconnects caused by metal chassis, motors, or 2.4GHz interference
    esp_bredr_tx_power_set(ESP_PWR_LVL_P9, ESP_PWR_LVL_P9);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P9);
#endif

    printLocalBdAddress();

    // Setup Bluepad32 connect and disconnect callbacks
    BP32.setup(&onConnectedWrapper, &onDisconnectedWrapper);

    // Disable virtual mouse/touchpad device on DualShock 4
    BP32.enableVirtualDevice(false);

    LOG_INFO(TAG, "Bluepad32 initialized. Put DualShock 4 into pairing mode (Hold SHARE + PS button until lightbar flashes).");
    return true;
}


void Bluepad32Receiver::printLocalBdAddress() const {
    const uint8_t* addr = BP32.localBdAddress();
    if (addr) {
        LOG_INFO(TAG, ">>> ESP32 Local Bluetooth Address (BD_ADDR): %02X:%02X:%02X:%02X:%02X:%02X <<<",
                 addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
    }
}

void Bluepad32Receiver::forgetBluetoothKeys() {
    LOG_WARN(TAG, "Forgetting stored Bluetooth pairing keys (Factory Reset). Re-pairing required.");
    BP32.forgetBluetoothKeys();
}

void Bluepad32Receiver::setLightbarColor(uint8_t r, uint8_t g, uint8_t b) {
    if (_connected_controller && _connected_controller->isConnected()) {
        _connected_controller->setColorLED(r, g, b);
    }
}

void Bluepad32Receiver::setRumble(uint8_t weak_mag, uint8_t strong_mag, uint16_t duration_ms) {
    if (_connected_controller && _connected_controller->isConnected()) {
        _connected_controller->playDualRumble(0, duration_ms, weak_mag, strong_mag);
    }
}

void Bluepad32Receiver::onControllerConnected(ControllerPtr ctl) {
    if (!ctl) return;

    ControllerProperties props = ctl->getProperties();
    LOG_INFO(TAG, "Bluetooth Controller Connected: Model='%s', VID=0x%04X, PID=0x%04X, MAC=%02X:%02X:%02X:%02X:%02X:%02X",
             ctl->getModelName().c_str(),
             props.vendor_id,
             props.product_id,
             props.btaddr[0], props.btaddr[1], props.btaddr[2],
             props.btaddr[3], props.btaddr[4], props.btaddr[5]);

#ifdef ALLOWED_CONTROLLER_MAC
    const uint8_t allowed_mac[6] = ALLOWED_CONTROLLER_MAC;
    bool is_allowed = true;
    for (int i = 0; i < 6; i++) {
        if (allowed_mac[i] != 0 && allowed_mac[i] != props.btaddr[i]) {
            is_allowed = false;
            break;
        }
    }
    if (!is_allowed) {
        LOG_WARN(TAG, "Controller rejected: MAC does not match ALLOWED_CONTROLLER_MAC configuration.");
        ctl->disconnect();
        return;
    }
#endif

    if (_connected_controller == nullptr) {
        _connected_controller = ctl;
        LOG_INFO(TAG, "Active Gamepad Assigned (Index: %d). Setting DS4 Lightbar to BLUE.", ctl->index());

        // Visual and haptic feedback on successful connection
        ctl->setColorLED(0, 80, 255); // Blue
        ctl->playDualRumble(0, 300, 0x80, 0x40);
    } else {
        LOG_WARN(TAG, "Additional controller connected (Index %d) but primary controller is already active.", ctl->index());
    }
}

void Bluepad32Receiver::onControllerDisconnected(ControllerPtr ctl) {
    if (!ctl) return;

    LOG_WARN(TAG, "Bluetooth Controller Disconnected (Index: %d)", ctl->index());

    if (_connected_controller == ctl) {
        _connected_controller = nullptr;
        memset(&_current_data, 0, sizeof(_current_data));
        _current_data.connected = false;

        LOG_WARN(TAG, "Primary gamepad disconnected. Standing by for seamless reconnect...");
        if (_failsafe) {
            _failsafe->onControllerDisconnected();
        }
    }
}

// Maps Bluepad32 axis (-511 to +512) to normalized [-1000, +1000]
static inline int16_t mapBp32Axis(int32_t val, bool invert) {
    // Clamp to valid range
    if (val < -511) val = -511;
    if (val > 512)  val = 512;

    int32_t scaled = (val * 1000) / 511;
    if (scaled < -1000) scaled = -1000;
    if (scaled > 1000)  scaled = 1000;

    return (int16_t)(invert ? -scaled : scaled);
}

void Bluepad32Receiver::processGamepadInputs(ControllerPtr ctl, uint32_t now_ms) {
    if (!ctl || !ctl->isConnected()) {
        _current_data.connected = false;
        return;
    }

    _current_data.connected = true;
    _current_data.last_update_ms = now_ms;

    // 1. Analog Sticks (Standard cartesian coordinates: UP=+1000, RIGHT=+1000):
    // LX: Left stick horizontal (Strafe: Left=-1000, Right=+1000)
    // LY: Left stick vertical (Forward: Bluepad32 stick UP is negative (-511), so invert=true maps UP to +1000)
    // RX: Right stick horizontal (Rotation: Left=-1000, Right=+1000)
    // RY: Right stick vertical (Auxiliary: Down=-1000, Up=+1000)
    _current_data.lx = mapBp32Axis(ctl->axisX(), false);
    _current_data.ly = mapBp32Axis(ctl->axisY(), true);  // Invert raw Y so stick forward = +1000
    _current_data.rx = mapBp32Axis(ctl->axisRX(), false);
    _current_data.ry = mapBp32Axis(ctl->axisRY(), true);

    // 2. Triggers (0..1023 mapped to 0..255):
    int32_t raw_l2 = ctl->brake();
    int32_t raw_r2 = ctl->throttle();
    _current_data.dribble = (uint8_t)((raw_l2 * 255) / 1023);
    _current_data.kick    = (uint8_t)((raw_r2 * 255) / 1023);

    // 3. Digital Buttons:
    uint16_t buttons = 0;
    if (ctl->a()) { // Cross (X) on DS4
        buttons |= BTN_KICK_TRIGGER;
        _current_data.kick = 255;
    }
    if (ctl->b()) { // Circle on DS4
        buttons |= BTN_DRIBBLE_TOGGLE;
    }
    if (ctl->x()) { // Square on DS4
        buttons |= BTN_BOOST_MODE;
    }
    if (ctl->y()) { // Triangle on DS4
        buttons |= BTN_MODE_SELECT;
    }
    if (ctl->miscSystem()) { // PS Button on DS4 -> Emergency Stop
        buttons |= BTN_EMERGENCY_STOP;
    }

    _current_data.buttons = buttons;
    _has_new_data = true;

    // Feed Failsafe watchdog
    if (_failsafe) {
        RobotCommand cmd;
        cmd.magic = SOCCERBOT_PROTOCOL_MAGIC;
        cmd.protocol_version = SOCCERBOT_PROTOCOL_VERSION;
        cmd.robot_id = ROBOT_ID;
        cmd.lx = _current_data.lx;
        cmd.ly = _current_data.ly;
        cmd.rx = _current_data.rx;
        cmd.ry = _current_data.ry;
        cmd.buttons = _current_data.buttons;
        cmd.kick = _current_data.kick;
        cmd.dribble = _current_data.dribble;
        cmd.sequence = (uint32_t)(now_ms / 20); // Sequence derived from ticks
        cmd.crc = 0;

        _failsafe->onValidPacketReceived(cmd, now_ms);
    }
}

void Bluepad32Receiver::update(uint32_t now_ms) {
    // 1. Fetch Bluepad32 Bluetooth updates
    bool dataUpdated = BP32.update();

    if (_connected_controller != nullptr && _connected_controller->isConnected()) {
        if (dataUpdated && _connected_controller->hasData()) {
            if (_connected_controller->isGamepad()) {
                processGamepadInputs(_connected_controller, now_ms);
            }
        }
    } else {
        _current_data.connected = false;
    }
}

bool Bluepad32Receiver::getLatestInput(GamepadData* out_data) {
    if (!_connected_controller || !_connected_controller->isConnected() || !_current_data.connected) {
        return false;
    }

    if (out_data) {
        *out_data = _current_data;
    }
    return true;
}

