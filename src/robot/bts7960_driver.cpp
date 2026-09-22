#include "robot/bts7960_driver.h"
#include "common/logging.h"

#ifdef ARDUINO
#include <Arduino.h>
#include <esp_idf_version.h>
#endif

static const char* TAG = "BTS7960";

BTS7960Channel::BTS7960Channel()
    : _lpwm_ch(0),
      _rpwm_ch(0),
      _max_duty(1023),
      _inverted(false),
      _initialized(false),
      _current_speed(0),
      _current_lpwm_duty(0),
      _current_rpwm_duty(0) {
    _pins.lpwm = -1;
    _pins.rpwm = -1;
    _pins.lis  = -1;
    _pins.ris  = -1;
}

bool BTS7960Channel::init(const BTS7960ChannelPins& pins,
                          uint8_t lpwm_ch,
                          uint8_t rpwm_ch,
                          uint32_t freq_hz,
                          uint8_t resolution_bits,
                          bool inverted) {
    _pins = pins;
    _lpwm_ch = lpwm_ch;
    _rpwm_ch = rpwm_ch;
    _inverted = inverted;
    _max_duty = (1UL << resolution_bits) - 1;
    _current_speed = 0;
    _current_lpwm_duty = 0;
    _current_rpwm_duty = 0;

    LOG_INFO(TAG, "Initializing BTS7960 Channel: LPWM=%d (ch%d), RPWM=%d (ch%d), L_IS=%d, R_IS=%d, Invert=%d",
             _pins.lpwm, _lpwm_ch, _pins.rpwm, _rpwm_ch, _pins.lis, _pins.ris, (int)_inverted);

#ifdef ARDUINO
    // 1. Configure diagnostic / current-sense pins as inputs with pulldown
    if (_pins.lis >= 0) {
        pinMode(_pins.lis, INPUT_PULLDOWN);
    }
    if (_pins.ris >= 0) {
        pinMode(_pins.ris, INPUT_PULLDOWN);
    }

    // 2. Configure PWM outputs safely (ensuring output remains LOW during init)
    pinMode(_pins.lpwm, OUTPUT);
    digitalWrite(_pins.lpwm, LOW);
    pinMode(_pins.rpwm, OUTPUT);
    digitalWrite(_pins.rpwm, LOW);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    // Arduino-ESP32 Core 3.x API
    ledcAttach(_pins.lpwm, freq_hz, resolution_bits);
    ledcAttach(_pins.rpwm, freq_hz, resolution_bits);
    ledcWrite(_pins.lpwm, 0);
    ledcWrite(_pins.rpwm, 0);
#else
    // Arduino-ESP32 Core 2.x API
    ledcSetup(_lpwm_ch, freq_hz, resolution_bits);
    ledcAttachPin(_pins.lpwm, _lpwm_ch);
    ledcWrite(_lpwm_ch, 0);

    ledcSetup(_rpwm_ch, freq_hz, resolution_bits);
    ledcAttachPin(_pins.rpwm, _rpwm_ch);
    ledcWrite(_rpwm_ch, 0);
#endif

#endif // ARDUINO

    _initialized = true;
    stop();
    return true;
}

void BTS7960Channel::writePwm(uint32_t lpwm_duty, uint32_t rpwm_duty) {
    if (!_initialized) return;

    if (lpwm_duty > _max_duty) lpwm_duty = _max_duty;
    if (rpwm_duty > _max_duty) rpwm_duty = _max_duty;

    _current_lpwm_duty = lpwm_duty;
    _current_rpwm_duty = rpwm_duty;

#ifdef ARDUINO
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWrite(_pins.lpwm, lpwm_duty);
    ledcWrite(_pins.rpwm, rpwm_duty);
#else
    ledcWrite(_lpwm_ch, lpwm_duty);
    ledcWrite(_rpwm_ch, rpwm_duty);
#endif
#endif
}

void BTS7960Channel::setSpeed(int16_t speed) {
    if (!_initialized) return;

    // Clamp speed to [-1000, 1000]
    if (speed < -1000) speed = -1000;
    if (speed > 1000) speed = 1000;

    _current_speed = speed;

    if (_inverted) {
        speed = -speed;
    }

    if (speed == 0) {
        // Safe Stop: both half bridges disabled (LPWM=0, RPWM=0)
        writePwm(0, 0);
    } else if (speed > 0) {
        // Forward: LPWM modulated with speed duty, RPWM held at 0
        uint32_t duty = (uint32_t)((int32_t)speed * _max_duty / 1000);
        writePwm(duty, 0);
    } else {
        // Reverse: RPWM modulated with speed duty, LPWM held at 0
        uint32_t duty = (uint32_t)((int32_t)(-speed) * _max_duty / 1000);
        writePwm(0, duty);
    }
}

void BTS7960Channel::stop() {
    _current_speed = 0;
    writePwm(0, 0);
}

bool BTS7960Channel::hasFault() const {
#ifdef ARDUINO
    bool fault = false;
    if (_pins.lis >= 0 && digitalRead(_pins.lis) == HIGH) {
        fault = true;
    }
    if (_pins.ris >= 0 && digitalRead(_pins.ris) == HIGH) {
        fault = true;
    }
    return fault;
#else
    return false;
#endif
}
