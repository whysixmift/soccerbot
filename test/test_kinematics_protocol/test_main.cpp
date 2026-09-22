#include <stdio.h>
#include <assert.h>
#include <string.h>
#include "common/protocol.h"
#include "robot/kinematics.h"
#include "robot/failsafe.h"

// Stubs for native host test execution
void MotorController::stopAll() {}

static int s_tests_passed = 0;
static int s_tests_failed = 0;

#define TEST_ASSERT(cond, msg) \
    do { \
        if (cond) { \
            printf("[PASS] %s\n", msg); \
            s_tests_passed++; \
        } else { \
            printf("[FAIL] %s (Line %d)\n", msg, __LINE__); \
            s_tests_failed++; \
        } \
    } while (0)

void test_protocol_crc_and_validation() {
    printf("\n--- Running Protocol Tests ---\n");

    RobotCommand cmd;
    protocol_prepare_command(&cmd, 1, 300, 500, -200, 0, 0x05, 255, 100, 42);

    TEST_ASSERT(cmd.magic == SOCCERBOT_PROTOCOL_MAGIC, "Magic word is 0x5342");
    TEST_ASSERT(cmd.protocol_version == SOCCERBOT_PROTOCOL_VERSION, "Protocol version is 0x01");
    TEST_ASSERT(cmd.robot_id == 1, "Robot ID is 1");
    TEST_ASSERT(cmd.lx == 300 && cmd.ly == 500 && cmd.rx == -200, "Axes populated correctly");
    TEST_ASSERT(cmd.sequence == 42, "Sequence number is 42");

    // Test valid packet validation
    RobotCommand parsed;
    const char* err = nullptr;
    bool ok = protocol_validate_command((const uint8_t*)&cmd, sizeof(RobotCommand), 1, 41, &parsed, &err);
    TEST_ASSERT(ok == true && err == nullptr, "Valid packet validates successfully");
    TEST_ASSERT(parsed.lx == 300 && parsed.ly == 500 && parsed.rx == -200, "Parsed fields match");

    // Test corrupted CRC
    RobotCommand corrupt_cmd = cmd;
    corrupt_cmd.lx = 301; // altered payload without recomputing CRC
    ok = protocol_validate_command((const uint8_t*)&corrupt_cmd, sizeof(RobotCommand), 1, 41, &parsed, &err);
    TEST_ASSERT(ok == false && strcmp(err, "CRC mismatch") == 0, "Corrupted CRC packet is rejected");

    // Test wrong robot ID
    ok = protocol_validate_command((const uint8_t*)&cmd, sizeof(RobotCommand), 2, 41, &parsed, &err);
    TEST_ASSERT(ok == false && strcmp(err, "Robot ID mismatch") == 0, "Wrong Robot ID packet is rejected");

    // Test stale sequence number (replay attack / delayed packet)
    ok = protocol_validate_command((const uint8_t*)&cmd, sizeof(RobotCommand), 1, 50, &parsed, &err);
    TEST_ASSERT(ok == false && strcmp(err, "Stale or duplicate sequence") == 0, "Stale sequence packet is rejected");

    // Test truncated packet size
    ok = protocol_validate_command((const uint8_t*)&cmd, sizeof(RobotCommand) - 4, 1, 41, &parsed, &err);
    TEST_ASSERT(ok == false && strcmp(err, "Invalid packet size") == 0, "Truncated packet is rejected");
}

void test_kinematics() {
    printf("\n--- Running Kinematics Tests ---\n");

    KinematicsConfig cfg;
    cfg.deadzone     = 80; // 8%
    cfg.linear_scale = 1000;
    cfg.turn_scale   = 1000;
    cfg.invert_lx    = false;
    cfg.invert_ly    = false;
    cfg.invert_rx    = false;
    cfg.invert_fl    = false;
    cfg.invert_fr    = false;
    cfg.invert_rl    = false;
    cfg.invert_rr    = false;
    cfg.invert_ch1   = false;
    cfg.invert_ch2   = false;

    // 1. Pure Forward Motion (LY = +1000, LX = 0, RX = 0)
    MecanumWheelSpeeds m_fwd = Kinematics::computeMecanum(0, 1000, 0, cfg);
    TEST_ASSERT(m_fwd.fl == 1000 && m_fwd.fr == 1000 && m_fwd.rl == 1000 && m_fwd.rr == 1000,
                "Mecanum Pure Forward: all wheels +1000");

    DualChannelSpeeds d_fwd = Kinematics::computeDualChannel(0, 1000, 0, cfg);
    TEST_ASSERT(d_fwd.ch1_left == 1000 && d_fwd.ch2_right == 1000,
                "Dual Channel Pure Forward: Ch1=+1000, Ch2=+1000");

    // 2. Pure Strafe Right (LX = +1000, LY = 0, RX = 0)
    // front_left = ly + lx + rx = 1000
    // front_right = ly - lx - rx = -1000
    // rear_left = ly - lx + rx = -1000
    // rear_right = ly + lx - rx = 1000
    MecanumWheelSpeeds m_strafe = Kinematics::computeMecanum(1000, 0, 0, cfg);
    TEST_ASSERT(m_strafe.fl == 1000 && m_strafe.fr == -1000 && m_strafe.rl == -1000 && m_strafe.rr == 1000,
                "Mecanum Pure Strafe Right: FL=+1000, FR=-1000, RL=-1000, RR=+1000");

    // 3. Pure Rotate CW (RX = +1000, LY = 0, LX = 0)
    // front_left = +1000, front_right = -1000, rear_left = +1000, rear_right = -1000
    MecanumWheelSpeeds m_rot = Kinematics::computeMecanum(0, 0, 1000, cfg);
    TEST_ASSERT(m_rot.fl == 1000 && m_rot.fr == -1000 && m_rot.rl == 1000 && m_rot.rr == -1000,
                "Mecanum Pure Rotate CW: Left side +1000, Right side -1000");

    DualChannelSpeeds d_rot = Kinematics::computeDualChannel(0, 0, 1000, cfg);
    TEST_ASSERT(d_rot.ch1_left == 1000 && d_rot.ch2_right == -1000,
                "Dual Channel Pure Rotate CW: Ch1=+1000, Ch2=-1000");

    // 4. Combined Movement with Proportional Scaling (LY = +1000, RX = +1000)
    // raw: left = 2000, right = 0 -> normalized: left = 1000, right = 0
    DualChannelSpeeds d_comb = Kinematics::computeDualChannel(0, 1000, 1000, cfg);
    TEST_ASSERT(d_comb.ch1_left == 1000 && d_comb.ch2_right == 0,
                "Dual Channel Normalized Combined (Fwd+Turn): Ch1=+1000, Ch2=0");

    // 5. Deadzone Filtering
    MecanumWheelSpeeds m_dz = Kinematics::computeMecanum(50, 40, 60, cfg);
    TEST_ASSERT(m_dz.fl == 0 && m_dz.fr == 0 && m_dz.rl == 0 && m_dz.rr == 0,
                "Deadzone filtering suppresses sub-threshold stick noise to 0");
}

void test_failsafe_state_machine() {
    printf("\n--- Running Failsafe State Machine Tests ---\n");

    FailsafeManager fm;
    fm.init(nullptr, 150); // 150ms timeout

    TEST_ASSERT(fm.getState() == STATE_WAITING_FOR_CONTROLLER, "Initial state is WAITING_FOR_CONTROLLER");
    TEST_ASSERT(!fm.isActive(), "Is not active on boot");

    // Feed a valid packet at t=1000
    RobotCommand cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.robot_id = 1;
    fm.onValidPacketReceived(cmd, 1000);

    TEST_ASSERT(fm.getState() == STATE_ACTIVE, "Transitions to STATE_ACTIVE on valid packet");
    TEST_ASSERT(fm.isActive(), "isActive() returns true");

    // Check timeout at t=1100 (100ms later <= 150ms)
    fm.checkTimeout(1100);
    TEST_ASSERT(fm.getState() == STATE_ACTIVE, "Remains active within timeout window (100ms)");

    // Check timeout at t=1151 (151ms later > 150ms)
    fm.checkTimeout(1151);
    TEST_ASSERT(fm.getState() == STATE_FAILSAFE_TIMEOUT, "Transitions to STATE_FAILSAFE_TIMEOUT on elapsed watchdog");
    TEST_ASSERT(!fm.isActive(), "isActive() returns false after timeout");
    TEST_ASSERT(fm.getFailsafeCount() == 1, "Failsafe counter incremented to 1");

    // Reconnection at t=2000
    fm.onValidPacketReceived(cmd, 2000);
    TEST_ASSERT(fm.getState() == STATE_ACTIVE, "Recovers to STATE_ACTIVE upon packet resumption");

    // Emergency stop trigger via button
    cmd.buttons = BTN_EMERGENCY_STOP;
    fm.onValidPacketReceived(cmd, 2050);
    TEST_ASSERT(fm.getState() == STATE_EMERGENCY_STOP, "Transitions to STATE_EMERGENCY_STOP on E-Stop packet");

    // Subsequent normal packets should NOT clear emergency stop
    cmd.buttons = 0;
    fm.onValidPacketReceived(cmd, 2100);
    TEST_ASSERT(fm.getState() == STATE_EMERGENCY_STOP, "E-Stop persists across normal packets");

    // Explicit manual clear
    fm.clearEmergencyStop();
    TEST_ASSERT(fm.getState() == STATE_WAITING_FOR_CONTROLLER, "E-Stop clear returns to WAITING_FOR_CONTROLLER");
}

int main() {
    printf("========================================\n");
    printf("  SOCCERBOT EMBEDDED UNIT TEST SUITE    \n");
    printf("========================================\n");

    test_protocol_crc_and_validation();
    test_kinematics();
    test_failsafe_state_machine();

    printf("\n========================================\n");
    printf("TEST RESULTS: %d Passed, %d Failed\n", s_tests_passed, s_tests_failed);
    printf("========================================\n");

    return (s_tests_failed == 0) ? 0 : 1;
}

