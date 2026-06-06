/*
 * File: 2026v1.c
 * Purpose: Baseline Mars exploration robot program.
 *
 * This copied version keeps the original tuned route order, but separates
 * hardware configuration, PID loading, mechanism preparation, and route stages.
 */

#include <stm32h7xx_hal.h>
#include <SetMotor.h>
#include <SetDisplayVar.h>
#include <GetData.h>
#include "hardware_config.c"
#include "fun.c"

typedef struct
{
    double kp;
    double ki;
    double kd;
    double prev_error;
    double integral;
} PID_Controller;

PID_Controller pidX = {0, 0, 0.0, 0, 0};
PID_Controller pidY = {0, 0, 0.0, 0, 0};
PID_Controller pidW = {0, 0, 0.0, 0, 0};

#include "go_to_tag.c"
#include "HardwareInfo.c"
#include "JMLib.c"
#include <SetMotorDirection.h>
#include "settingRobot.c"
#include "speed_control.c"
#include <SetFontSize.h>
#include "init.c"
#include <SetWaitAICamCmd.h>
#include "kinematic.c"
#include "paw_control.c"
#include "go_bmp.c"
#include <SetWaitForTime.h>
#include <SetDisplayString.h>
#include "go_times.c"
#include "set_back.c"
#include "turn_o.c"
#include "photo_scan.c"

double Kp = 0;
double Ki = 0;
double Kd = 0;
double g_carD = 0;
double g_wheel2R = 0;
double g_motorencoders = 0;
double rate_encoder = 0;

static void configure_drive_base(void)
{
    SetMotorDirection(_M1_, ROBOT_MOTOR_DIR_M1);
    SetMotorDirection(_M2_, ROBOT_MOTOR_DIR_M2);
    SetMotorDirection(_M3_, ROBOT_MOTOR_DIR_M3);
    SetMotorDirection(_M4_, ROBOT_MOTOR_DIR_M4);

    settingRobot(ROBOT_WHEEL_DIAMETER_CM, ROBOT_TRACK_WIDTH_CM, ROBOT_ENCODER_TICKS_PER_REV);
    speed_control(0, 0);
}

static void configure_vision(void)
{
    SetWaitAICamCmd(ROBOT_AI_CAM_PORT, ROBOT_AI_CAM_MODEL);
}

static void load_navigation_pid(void)
{
    pidW.kp = (double)GetData(ROBOT_PIDW_KP_ADDR) / ROBOT_PID_DATA_SCALE;
    pidW.ki = (double)GetData(ROBOT_PIDW_KI_ADDR) / ROBOT_PID_DATA_SCALE;
    pidW.kd = (double)GetData(ROBOT_PIDW_KD_ADDR) / ROBOT_PID_DATA_SCALE;
}

static void prepare_mechanism(void)
{
    SetDisplayString(7, "ARM PREP", 0xFFE0, 0x0000);
    kinematic(0, 120, 30, 1000);
    paw_control(ROBOT_PAW_OPEN, 1000);
}

static char trace_go_to_tag(const char *stage, double targetX, double targetY, double targetA, unsigned long timeout_ms, char catch_type)
{
    char ok = 0;

    SetDisplayString(7, stage, 0xFFE0, 0x0000);
    ok = go_to_tag(targetX, targetY, targetA, timeout_ms, catch_type);
    SetDisplayString(8, ok ? "TAG OK" : "TAG FAIL", ok ? 0x07E0 : 0xF800, 0x0000);

    return ok;
}

static void run_baseline_stage_1(void)
{
    SetDisplayString(7, "STAGE 1", 0xFFE0, 0x0000);
    go_bmp(-40, 53, 2);
    SetWaitForTime(0.2);

    go_times(-30, 1, 1);
    SetWaitForTime(0.2);

    go_bmp(40, 1.5, 1);
    SetWaitForTime(0.2);

    trace_go_to_tag("TAG A1", 180, 100, 0, 12000, 0);
    SetWaitForTime(0.5);

    go_bmp(30, 7, 1);
    SetWaitForTime(0.2);

    kinematic(0, 190, -10, 1000);
    paw_control(ROBOT_PAW_GRIP, 1000);
    SetWaitForTime(0.2);

    kinematic(0, 150, 50, 1000);
    go_bmp(30, 5.5, 1);
    SetWaitForTime(0.2);

    set_back(1680, 2000, 1650);
    SetWaitForTime(0.2);

    paw_control(ROBOT_PAW_OPEN, 1000);
    SetWaitForTime(0.2);

    kinematic(0, 120, 30, 2000);
}

static void run_baseline_stage_2(void)
{
    SetDisplayString(7, "STAGE 2", 0xFFE0, 0x0000);
    go_bmp(-40, 68, 2);
    SetWaitForTime(0.2);
    go_bmp(40, 39, 1);
    SetWaitForTime(0.2);
    go_bmp(40, 48, 2);
    SetWaitForTime(0.2);
    go_bmp(-30, 6, 1);
    SetWaitForTime(0.2);

    trace_go_to_tag("TAG A2", 210, 80, 358, 12000, 0);
    SetWaitForTime(0.2);

    go_bmp(30, 5, 1);
    SetWaitForTime(0.2);

    kinematic(0, 230, -10, 1000);
    paw_control(1580, 1200);
    SetWaitForTime(0.2);

    kinematic(0, 110, 30, 2000);
    SetWaitForTime(0.2);

    go_bmp(-40, 53, 2);
    SetWaitForTime(0.2);

    turn_o(9.5, -40, 40);
    SetWaitForTime(0.2);

    go_bmp(40, 108, 1);
    SetWaitForTime(0.2);

    go_bmp(40, 66, 2);
    SetWaitForTime(0.3);

    turn_o(180, -40, 40);
    SetWaitForTime(0.3);

    go_bmp(-30, 16, 2);
    SetWaitForTime(0.3);

    go_bmp(-30, 10, 1);
    SetWaitForTime(1);

    trace_go_to_tag("TAG A3", 65, 60, 179, 12000, 0);
    SetWaitForTime(1);
}

static void run_baseline_stage_3(void)
{
    SetDisplayString(7, "STAGE 3", 0xFFE0, 0x0000);
    go_bmp(30, 7, 1);
    SetWaitForTime(0.2);

    kinematic(0, 200, 100, 1200);
    SetWaitForTime(0.2);

    kinematic(0, 200, 0, 1200);
    paw_control(ROBOT_PAW_OPEN, 1000);
    SetWaitForTime(0.2);

    set_back(1680, 2000, 1660);
    SetWaitForTime(0.2);
    paw_control(ROBOT_PAW_GRIP, 1500);
    SetWaitForTime(0.2);

    set_back(1680, 1200, 1400);
    SetWaitForTime(0.2);
    kinematic(0, 200, 100, 1200);
    SetWaitForTime(0.2);

    kinematic(0, 190, 30, 1200);
    paw_control(ROBOT_PAW_OPEN, 1200);
    SetWaitForTime(0.2);

    kinematic(0, 180, 180, 1000);
    SetWaitForTime(0.2);
}

static void run_baseline_stage_4(void)
{
    SetDisplayString(7, "STAGE 4", 0xFFE0, 0x0000);
    go_bmp(40, 2, 3);
    SetWaitForTime(0.2);
    go_bmp(60, 80, 2);
    SetWaitForTime(0.2);
    go_bmp(40, 16, 1);
    SetWaitForTime(0.2);
    go_bmp(60, 30, 2);
    SetWaitForTime(0.2);
    go_bmp(60, 11, 3);
    SetWaitForTime(1);
    go_bmp(60, 10, 3);
    SetWaitForTime(0.2);
    go_bmp(60, 20, 2);
    SetWaitForTime(0.2);

    kinematic(0, 150, 50, 1000);
    trace_go_to_tag("TAG END", 180, 100, 0, 15000, 1);

    kinematic(0, 230, -10, 1000);
    paw_control(ROBOT_PAW_GRIP, 1200);
    SetWaitForTime(1);
    kinematic(0, 230, 270, 1000);
    go_bmp(60, 35, 3);
    SetWaitForTime(0.3);
    go_bmp(60, 20, 1);
    SetWaitForTime(1);
    kinematic(0, 320, 180, 1000);
    SetWaitForTime(1);
    paw_control(ROBOT_PAW_OPEN, 1000);
    SetWaitForTime(1);
    go_bmp(-60, 15, 1);
    kinematic(0, 150, 50, 1000);
}

static void run_baseline_route(void)
{
    run_baseline_stage_1();
    run_baseline_stage_2();
    run_baseline_stage_3();
    run_baseline_stage_4();
}

int main(void)
{
    E7RCU_Init();

    SetDisplayString(1, "Running", 0xFFE0, 0x0000);
    SetWaitForTime(1);

    configure_drive_base();
    SetFontSize(1);

    init();
    configure_vision();
    load_navigation_pid();

    prepare_mechanism();
    run_baseline_route();

    while (1)
        ;
}
