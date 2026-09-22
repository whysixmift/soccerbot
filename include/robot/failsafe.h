#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "common/protocol.h"
#include "robot/motor_controller.h"

enum RobotState : uint8_t {
    STATE_BOOT = 0,
    STATE_WAITING_FOR_CONTROLLER,
    STATE_ACTIVE,
    STATE_FAILSAFE_TIMEOUT,
    STATE_EMERGENCY_STOP
};

class FailsafeManager {
public:
    FailsafeManager();

    /**
     * @brief Initializes failsafe manager with pointer to motor controller and timeout value.
     */
    void init(MotorController* motor_controller, uint32_t timeout_ms);

    /**
     * @brief Called immediately upon receiving a cryptographically/CRC-validated, fresh control packet.
     * Updates timestamps and transitions out of failsafe into ACTIVE state.
     */
    void onValidPacketReceived(const RobotCommand& cmd, uint32_t now_ms);

    /**
     * @brief Periodic check to evaluate timeout. Must be called every loop iteration.
     * If timeout elapsed, triggers immediate STOP on all motors and enters FAILSAFE state.
     */
    void checkTimeout(uint32_t now_ms);

    /**
     * @brief Called when the controller disconnects cleanly or is lost.
     * Transitions to WAITING_FOR_CONTROLLER (safe stop) while allowing automatic reconnection.
     */
    void onControllerDisconnected();

    /**
     * @brief Manually triggers Emergency Stop (locks motors at 0).
     */
    void triggerEmergencyStop();

    /**
     * @brief Clears Emergency Stop.
     */
    void clearEmergencyStop();

    RobotState getState() const { return _state; }
    const char* getStateString() const;
    uint32_t getLastValidPacketTime() const { return _last_valid_packet_time; }
    uint32_t getFailsafeCount() const { return _failsafe_trigger_count; }
    bool isActive() const { return _state == STATE_ACTIVE; }

private:
    MotorController* _motor_ctrl;
    RobotState       _state;
    uint32_t         _timeout_ms;
    uint32_t         _last_valid_packet_time;
    uint32_t         _failsafe_trigger_count;
    uint32_t         _last_state_log_time;

    void transitionTo(RobotState new_state);
};
