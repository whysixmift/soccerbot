#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "robot/bts7960_driver.h"
#include "robot/hardware_pins.h"

enum MotorId : uint8_t {
    MOTOR_ID_CH1_LEFT  = 0,
    MOTOR_ID_CH2_RIGHT = 1
};

class MotorController {
public:
    MotorController();

    /**
     * @brief Initializes both BTS7960 motor driver channels.
     * Guaranteed safe: all PWM channels initialized to 0% duty cycle.
     */
    bool init(const RobotPinConfig& pins,
              bool invert_ch1,
              bool invert_ch2,
              int16_t max_speed,
              int16_t accel_rate = 2000,
              int16_t decel_rate = 3000);

    /**
     * @brief Set target speed for an individual motor channel (ramped during update).
     *
     * @param motor Motor ID (MOTOR_ID_CH1_LEFT or MOTOR_ID_CH2_RIGHT)
     * @param speed Target speed from -1000 (Reverse) to +1000 (Forward). 0 = Stop.
     */
    void setMotor(MotorId motor, int16_t speed);

    /**
     * @brief Set target speeds for both channels simultaneously (ramped during update).
     */
    void setSpeeds(int16_t ch1_speed, int16_t ch2_speed);

    /**
     * @brief Periodic update function: performs smooth acceleration/deceleration ramping.
     */
    void update(uint32_t now_ms);

    /**
     * @brief Immediately stops all motors with zero deceleration delay.
     */
    void stopAll();

    /**
     * @brief Starts the automated safe motor diagnostic test sequence.
     */
    void startMotorTest();

    /**
     * @brief Cancels the motor diagnostic test sequence and stops motors.
     */
    void stopMotorTest();

    /**
     * @brief Updates the motor test state machine (non-blocking).
     */
    void updateMotorTest(uint32_t now_ms);

    /**
     * @brief Returns true if the motor diagnostic test is actively running.
     */
    bool isMotorTestRunning() const { return _test_running; }

    int16_t getCh1Speed() const { return _current_ch1; }
    int16_t getCh2Speed() const { return _current_ch2; }
    int16_t getCh1Target() const { return _target_ch1; }
    int16_t getCh2Target() const { return _target_ch2; }

    BTS7960Channel& getCh1() { return _ch1; }
    BTS7960Channel& getCh2() { return _ch2; }

private:
    int16_t rampValue(int16_t current, int16_t target, uint32_t dt_ms);

    BTS7960Channel _ch1;
    BTS7960Channel _ch2;
    int16_t        _max_speed_limit;
    int16_t        _accel_rate;       // Units per second
    int16_t        _decel_rate;       // Units per second
    int16_t        _target_ch1;
    int16_t        _target_ch2;
    int16_t        _current_ch1;
    int16_t        _current_ch2;
    uint32_t       _last_update_ms;
    bool           _test_running;
    uint8_t        _test_step;
    uint32_t       _test_step_timer;
};

