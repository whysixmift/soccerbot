#include "robot/failsafe.h"
#include "common/logging.h"

static const char* TAG = "FAILSAFE";

FailsafeManager::FailsafeManager()
    : _motor_ctrl(nullptr),
      _state(STATE_BOOT),
      _timeout_ms(150),
      _last_valid_packet_time(0),
      _failsafe_trigger_count(0),
      _last_state_log_time(0) {}

void FailsafeManager::init(MotorController* motor_controller, uint32_t timeout_ms) {
    _motor_ctrl = motor_controller;
    _timeout_ms = timeout_ms;
    _last_valid_packet_time = 0;
    _failsafe_trigger_count = 0;

    // Guaranteed safe initial state
    if (_motor_ctrl) {
        _motor_ctrl->stopAll();
    }
    transitionTo(STATE_WAITING_FOR_CONTROLLER);
    LOG_INFO(TAG, "Failsafe Manager initialized. Timeout set to %lu ms. Awaiting controller...", (unsigned long)_timeout_ms);
}

const char* FailsafeManager::getStateString() const {
    switch (_state) {
        case STATE_BOOT:                     return "BOOT";
        case STATE_WAITING_FOR_CONTROLLER:   return "WAITING_FOR_CONTROLLER";
        case STATE_ACTIVE:                   return "ACTIVE";
        case STATE_FAILSAFE_TIMEOUT:         return "FAILSAFE_TIMEOUT";
        case STATE_EMERGENCY_STOP:           return "EMERGENCY_STOP";
        default:                             return "UNKNOWN";
    }
}

void FailsafeManager::transitionTo(RobotState new_state) {
    if (_state == new_state) return;

    RobotState old_state = _state;
    _state = new_state;

    LOG_WARN(TAG, "State Transition: [%s] -> [%s]",
             (old_state == STATE_BOOT ? "BOOT" :
              old_state == STATE_WAITING_FOR_CONTROLLER ? "WAITING_FOR_CONTROLLER" :
              old_state == STATE_ACTIVE ? "ACTIVE" :
              old_state == STATE_FAILSAFE_TIMEOUT ? "FAILSAFE_TIMEOUT" : "ESTOP"),
             getStateString());

    if (_state != STATE_ACTIVE) {
        if (_motor_ctrl) {
            _motor_ctrl->stopAll();
        }
    }
}

void FailsafeManager::onValidPacketReceived(const RobotCommand& cmd, uint32_t now_ms) {
    _last_valid_packet_time = now_ms;

    // Check emergency stop bit in packet buttons
    if (cmd.buttons & BTN_EMERGENCY_STOP) {
        if (_state != STATE_EMERGENCY_STOP) {
            LOG_ERROR(TAG, "Emergency Stop received via controller button!");
            transitionTo(STATE_EMERGENCY_STOP);
        }
        return;
    }

    if (_state == STATE_EMERGENCY_STOP) {
        // Must stay in E-STOP unless explicitly cleared
        return;
    }

    if (_state != STATE_ACTIVE) {
        LOG_INFO(TAG, "Communication link established/recovered with controller. Entering ACTIVE state.");
        transitionTo(STATE_ACTIVE);
    }
}

void FailsafeManager::checkTimeout(uint32_t now_ms) {
    if (_state == STATE_ACTIVE) {
        if (now_ms - _last_valid_packet_time > _timeout_ms) {
            _failsafe_trigger_count++;
            LOG_ERROR(TAG, "SAFETY TIMEOUT! No valid packet for %lu ms (limit: %lu ms). Triggering FAILSAFE STOP!",
                      (unsigned long)(now_ms - _last_valid_packet_time), (unsigned long)_timeout_ms);
            transitionTo(STATE_FAILSAFE_TIMEOUT);
        }
    } else if (_state == STATE_FAILSAFE_TIMEOUT || _state == STATE_WAITING_FOR_CONTROLLER) {
        // Keep ensuring motors remain stopped
        if (_motor_ctrl && !_motor_ctrl->isMotorTestRunning()) {
            _motor_ctrl->stopAll();
        }
    }
}

void FailsafeManager::onControllerDisconnected() {
    LOG_WARN(TAG, "Controller connection lost. Entering WAITING_FOR_CONTROLLER state. Motors safely stopped.");
    transitionTo(STATE_WAITING_FOR_CONTROLLER);
}

void FailsafeManager::triggerEmergencyStop() {
    LOG_ERROR(TAG, "Manual Emergency Stop triggered!");
    transitionTo(STATE_EMERGENCY_STOP);
}

void FailsafeManager::clearEmergencyStop() {
    LOG_INFO(TAG, "Emergency Stop cleared.");
    transitionTo(STATE_WAITING_FOR_CONTROLLER);
}
