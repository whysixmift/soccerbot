#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "robot/hardware_pins.h"

/**
 * @brief Low-level driver for BTS7960 / IBT-2 dual half-bridge H-bridge driver.
 *
 * Provides safe PWM output using ESP32 LEDC peripheral, direction switching,
 * stop/brake control, and optional current-sense / alarm monitoring.
 */
class BTS7960Channel {
public:
    BTS7960Channel();

    /**
     * @brief Initializes GPIOs and LEDC PWM channels for the driver.
     * Guaranteed to keep PWM output at 0% during and after initialization.
     *
     * @param pins Struct containing LPWM, RPWM, L_IS, R_IS pin assignments.
     * @param lpwm_ch LEDC channel index for LPWM (Core 2.x)
     * @param rpwm_ch LEDC channel index for RPWM (Core 2.x)
     * @param freq_hz PWM frequency (e.g. 20000 Hz)
     * @param resolution_bits PWM resolution bits (e.g. 10 bits -> 0..1023)
     * @param inverted True if motor direction should be inverted.
     * @return true on successful initialization.
     */
    bool init(const BTS7960ChannelPins& pins,
              uint8_t lpwm_ch,
              uint8_t rpwm_ch,
              uint32_t freq_hz,
              uint8_t resolution_bits,
              bool inverted);

    /**
     * @brief Sets motor output speed and direction.
     *
     * @param speed Range -1000 (Full Reverse) to +1000 (Full Forward). 0 = Stop.
     */
    void setSpeed(int16_t speed);

    /**
     * @brief Immediately stops the motor by clearing both LPWM and RPWM to 0.
     */
    void stop();

    /**
     * @brief Reads the L_IS and R_IS diagnostic/alarm pins.
     * @return true if either IS pin indicates a fault/overcurrent alarm.
     */
    bool hasFault() const;

    /**
     * @brief Gets current raw duty applied to LPWM.
     */
    uint32_t getLpwmDuty() const { return _current_lpwm_duty; }

    /**
     * @brief Gets current raw duty applied to RPWM.
     */
    uint32_t getRpwmDuty() const { return _current_rpwm_duty; }

    /**
     * @brief Gets current commanded speed (-1000 to +1000).
     */
    int16_t getCommandedSpeed() const { return _current_speed; }

private:
    BTS7960ChannelPins _pins;
    uint8_t  _lpwm_ch;
    uint8_t  _rpwm_ch;
    uint32_t _max_duty;
    bool     _inverted;
    bool     _initialized;
    int16_t  _current_speed;
    uint32_t _current_lpwm_duty;
    uint32_t _current_rpwm_duty;

    void writePwm(uint32_t lpwm_duty, uint32_t rpwm_duty);
};
