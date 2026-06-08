/*
 * Straight-line chassis compatibility test.
 *
 * Purpose:
 * - Do not run the competition route.
 * - Do not initialize vision or arm actions.
 * - Only test whether motor directions, const-speed control, and encoders
 *   match the real chassis.
 */

#include <stm32h7xx_hal.h>
#include <SetMotor.h>
#include <SetMotorDirection.h>
#include <SetMotorConstSpeed.h>
#include <SetMotorCode.h>
#include <GetMotorCode.h>
#include <GetSysTime.h>
#include <SetDisplayString.h>
#include <SetDisplayVar.h>
#include <SetWaitForTime.h>
#include <SetFontSize.h>

#include "hardware_config.c"
#include "HardwareInfo.c"
#include "settingRobot.c"

#define TEST_SPEED 12
#define TEST_RUN_TIME_MS 2500
#define TEST_SAMPLE_PERIOD_SEC 0.05

double Kp = 0;
double Ki = 0;
double Kd = 0;
double g_carD = 0;
double g_wheel2R = 0;
double g_motorencoders = 0;
double rate_encoder = 0;

static void stop_all_motors(void)
{
    SetMotorConstSpeed(_M1_, 0);
    SetMotorConstSpeed(_M2_, 0);
    SetMotorConstSpeed(_M3_, 0);
    SetMotorConstSpeed(_M4_, 0);
}

static void configure_drive_base(void)
{
    SetMotorDirection(_M1_, ROBOT_MOTOR_DIR_M1);
    SetMotorDirection(_M2_, ROBOT_MOTOR_DIR_M2);
    SetMotorDirection(_M3_, ROBOT_MOTOR_DIR_M3);
    SetMotorDirection(_M4_, ROBOT_MOTOR_DIR_M4);

    settingRobot(ROBOT_WHEEL_DIAMETER_CM, ROBOT_TRACK_WIDTH_CM, ROBOT_ENCODER_TICKS_PER_REV);
    stop_all_motors();
}

static void reset_motor_codes(void)
{
    SetMotorCode(_M1_);
    SetMotorCode(_M2_);
    SetMotorCode(_M3_);
    SetMotorCode(_M4_);
}

static void show_motor_codes(void)
{
    SetDisplayVar(1, GetMotorCode(_M1_), YELLOW, BLACK);
    SetDisplayVar(3, GetMotorCode(_M2_), YELLOW, BLACK);
    SetDisplayVar(5, GetMotorCode(_M3_), YELLOW, BLACK);
    SetDisplayVar(7, GetMotorCode(_M4_), YELLOW, BLACK);
}

static void run_straight_test(void)
{
    long start_time = 0;

    SetDisplayString(1, "STRAIGHT TEST", 0xFFE0, 0x0000);
    SetDisplayString(8, "M1 M2 M3 M4", 0xFFE0, 0x0000);

    reset_motor_codes();
    SetWaitForTime(0.3);
    show_motor_codes();

    start_time = GetSysTime();
    SetMotorConstSpeed(_M1_, TEST_SPEED);
    SetMotorConstSpeed(_M2_, TEST_SPEED);
    SetMotorConstSpeed(_M3_, TEST_SPEED);
    SetMotorConstSpeed(_M4_, TEST_SPEED);

    while ((GetSysTime() - start_time) < TEST_RUN_TIME_MS)
    {
        show_motor_codes();
        SetWaitForTime(TEST_SAMPLE_PERIOD_SEC);
    }

    stop_all_motors();
    SetWaitForTime(0.5);
    show_motor_codes();
    SetDisplayString(8, "TEST DONE", 0x07E0, 0x0000);
}

int main(void)
{
    E7RCU_Init();
    SetFontSize(1);
    configure_drive_base();
    SetWaitForTime(1);

    run_straight_test();

    while (1)
    {
        show_motor_codes();
        SetWaitForTime(0.5);
    }
}
