#include "robot/motor_controller.h"
#include "robot/config_robot.h"
#include "common/logging.h"

static const char* TAG = "MOTOR_CTRL";

MotorController::MotorController()
    : _max_speed_limit(MOTOR_MAX_ALLOWED_SPEED),
      _accel_rate(MOTOR_ACCEL_RAMP_RATE),
      _decel_rate(MOTOR_DECEL_RAMP_RATE),
      _target_ch1(0),
      _target_ch2(0),
      _current_ch1(0),
      _current_ch2(0),
      _last_update_ms(0),
      _test_running(false),
      _test_step(0),
      _test_step_timer(0) {}

bool MotorController::init(const RobotPinConfig& pins,
                           bool invert_ch1,
                           bool invert_ch2,
                           int16_t max_speed,
                           int16_t accel_rate,
                           int16_t decel_rate) {
    LOG_INFO(TAG, "Initializing Motor Controller (Variant: %s, MaxSpeed: %d, AccelRate: %d, DecelRate: %d)",
             CURRENT_BOT_VARIANT_NAME, max_speed, accel_rate, decel_rate);

    _max_speed_limit = (max_speed > 0 && max_speed <= 1000) ? max_speed : 1000;
    _accel_rate = (accel_rate > 0) ? accel_rate : 2000;
    _decel_rate = (decel_rate > 0) ? decel_rate : 3000;
    _target_ch1 = 0;
    _target_ch2 = 0;
    _current_ch1 = 0;
    _current_ch2 = 0;
    _last_update_ms = 0;
    _test_running = false;
    _test_step = 0;

    // LEDC Channel allocation:
    // Ch1 (Left):  LPWM -> LEDC 0, RPWM -> LEDC 1
    // Ch2 (Right): LPWM -> LEDC 2, RPWM -> LEDC 3
    bool ok1 = _ch1.init(pins.ch1_left, 0, 1, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION_BITS, invert_ch1);
    bool ok2 = _ch2.init(pins.ch2_right, 2, 3, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION_BITS, invert_ch2);

    stopAll();

    if (ok1 && ok2) {
        LOG_INFO(TAG, "Motor Controller initialized successfully. Motors locked at 0 RPM.");
        return true;
    } else {
        LOG_ERROR(TAG, "Motor Controller initialization failed!");
        return false;
    }
}

static inline int16_t clamp_speed(int16_t val, int16_t limit) {
    if (val < -limit) return -limit;
    if (val > limit) return limit;
    return val;
}

int16_t MotorController::rampValue(int16_t current, int16_t target, uint32_t dt_ms) {
    if (current == target) return target;

    bool is_decel = false;
    if (target == 0) {
        is_decel = true;
    } else if (current > 0 && target < current) {
        is_decel = true;
    } else if (current < 0 && target > current) {
        is_decel = true;
    }

    int32_t rate = is_decel ? _decel_rate : _accel_rate;
    int32_t max_step = (rate * (int32_t)dt_ms) / 1000;
    if (max_step < 1) max_step = 1;

    if (target > current) {
        int32_t next = (int32_t)current + max_step;
        return (next > target) ? target : (int16_t)next;
    } else {
        int32_t next = (int32_t)current - max_step;
        return (next < target) ? target : (int16_t)next;
    }
}

void MotorController::setMotor(MotorId motor, int16_t speed) {
    if (_test_running) return;

    int16_t clamped = clamp_speed(speed, _max_speed_limit);

    if (motor == MOTOR_ID_CH1_LEFT) {
        _current_ch1 = clamped;
        _ch1.setSpeed(clamped);
    } else if (motor == MOTOR_ID_CH2_RIGHT) {
        _current_ch2 = clamped;
        _ch2.setSpeed(clamped);
    }
}

void MotorController::setSpeeds(int16_t ch1_speed, int16_t ch2_speed) {
    if (_test_running) return;

    _current_ch1 = clamp_speed(ch1_speed, _max_speed_limit);
    _current_ch2 = clamp_speed(ch2_speed, _max_speed_limit);
    _ch1.setSpeed(_current_ch1);
    _ch2.setSpeed(_current_ch2);
}

void MotorController::update(uint32_t now_ms) {
    if (_test_running) {
        updateMotorTest(now_ms);
    }
}

void MotorController::stopAll() {
    _target_ch1 = 0;
    _target_ch2 = 0;
    _current_ch1 = 0;
    _current_ch2 = 0;
    _ch1.stop();
    _ch2.stop();
}


void MotorController::startMotorTest() {
    LOG_WARN(TAG, "Starting Safe Motor Diagnostic Test Sequence...");
    stopAll();
    _test_running = true;
    _test_step = 0;
    _test_step_timer = millis();
}

void MotorController::stopMotorTest() {
    if (_test_running) {
        LOG_INFO(TAG, "Motor Diagnostic Test stopped.");
        _test_running = false;
        _test_step = 0;
        stopAll();
    }
}

void MotorController::updateMotorTest(uint32_t now_ms) {
    if (!_test_running) return;

    const uint32_t step_duration = MOTOR_TEST_STEP_MS;
    const int16_t test_speed = MOTOR_TEST_SAFE_DUTY;

    if (now_ms - _test_step_timer < step_duration) {
        return;
    }

    _test_step_timer = now_ms;
    _test_step++;

    switch (_test_step) {
        case 1:
            LOG_INFO(TAG, "[TEST 1/6] CH1 (LEFT) FORWARD (%d/1000)", test_speed);
            _ch1.setSpeed(test_speed);
            _ch2.stop();
            break;
        case 2:
            LOG_INFO(TAG, "[TEST 2/6] CH1 (LEFT) PAUSE");
            stopAll();
            break;
        case 3:
            LOG_INFO(TAG, "[TEST 3/6] CH1 (LEFT) REVERSE (%d/1000)", -test_speed);
            _ch1.setSpeed(-test_speed);
            _ch2.stop();
            break;
        case 4:
            LOG_INFO(TAG, "[TEST 4/6] CH2 (RIGHT) FORWARD (%d/1000)", test_speed);
            _ch1.stop();
            _ch2.setSpeed(test_speed);
            break;
        case 5:
            LOG_INFO(TAG, "[TEST 5/6] CH2 (RIGHT) PAUSE");
            stopAll();
            break;
        case 6:
            LOG_INFO(TAG, "[TEST 6/6] CH2 (RIGHT) REVERSE (%d/1000)", -test_speed);
            _ch1.stop();
            _ch2.setSpeed(-test_speed);
            break;
        default:
            LOG_INFO(TAG, "Motor Diagnostic Test completed. Returning to normal standby.");
            stopMotorTest();
            break;
    }
}
