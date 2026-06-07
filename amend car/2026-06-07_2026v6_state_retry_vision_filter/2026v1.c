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
#include <GetAICam.h>
#include <GetAICamIDData.h>
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

typedef enum
{
    ROBOT_STAGE_IDLE = 0,
    ROBOT_STAGE_1 = 1,
    ROBOT_STAGE_2 = 2,
    ROBOT_STAGE_3 = 3,
    ROBOT_STAGE_4 = 4,
    ROBOT_STAGE_DONE = 5
} RobotStageId;

typedef char (*RobotStageRunner)(void);

static RobotStageId g_current_stage = ROBOT_STAGE_IDLE;

static double clamp_double(double value, double min_value, double max_value)
{
    if (value > max_value)
        return max_value;
    if (value < min_value)
        return min_value;
    return value;
}

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
    SetWaitForTime(0.5);
}

static void load_navigation_pid(void)
{
    pidW.kp = clamp_double((double)GetData(ROBOT_PIDW_KP_ADDR) / ROBOT_PID_DATA_SCALE, 0.0, ROBOT_PID_KP_MAX);
    pidW.ki = clamp_double((double)GetData(ROBOT_PIDW_KI_ADDR) / ROBOT_PID_DATA_SCALE, 0.0, ROBOT_PID_KI_MAX);
    pidW.kd = clamp_double((double)GetData(ROBOT_PIDW_KD_ADDR) / ROBOT_PID_DATA_SCALE, 0.0, ROBOT_PID_KD_MAX);
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
    int attempt = 0;

    for (attempt = 0; attempt <= ROBOT_TAG_RETRY_COUNT; attempt++)
    {
        SetDisplayString(7, stage, 0xFFE0, 0x0000);
        SetDisplayVar(ROBOT_STAGE_DISPLAY_LINE, attempt + 1, YELLOW, BLACK);
        ok = go_to_tag(targetX, targetY, targetA, timeout_ms, catch_type);
        if (ok)
            break;

        speed_control(0, 0);
        SetDisplayString(8, "TAG RETRY", 0xFFE0, 0x0000);
        SetWaitForTime(0.3);
    }

    SetDisplayString(8, ok ? "TAG OK" : "TAG FAIL", ok ? 0x07E0 : 0xF800, 0x0000);

    return ok;
}

static double route_limited_speed(double speed, double limit)
{
    if (speed > limit)
        return limit;
    if (speed < -limit)
        return -limit;
    return speed;
}

static double route_abs_distance(double distance)
{
    return (distance < 0) ? -distance : distance;
}

static char route_go(double speed, double distance, char direction)
{
    double remaining = route_abs_distance(distance);
    double limited_speed = route_limited_speed(speed, ROBOT_ROUTE_MAX_SPEED);

    while (remaining > 0.001)
    {
        double step = (remaining > ROBOT_ROUTE_SEGMENT_CM) ? ROBOT_ROUTE_SEGMENT_CM : remaining;
        char ok = go_bmp(limited_speed, step, direction);

        speed_control(0, 0);
        SetWaitForTime(ROBOT_ROUTE_SETTLE_SEC);
        if (!ok)
            return 0;

        remaining -= step;
    }

    return 1;
}

static char route_turn(double angle, double left_speed, double right_speed)
{
    char ok = turn_o(angle,
                     route_limited_speed(left_speed, ROBOT_ROUTE_TURN_SPEED_LIMIT),
                     route_limited_speed(right_speed, ROBOT_ROUTE_TURN_SPEED_LIMIT));
    speed_control(0, 0);
    SetWaitForTime(ROBOT_ROUTE_SETTLE_SEC);
    return ok;
}

static char route_fail(const char *stage)
{
    speed_control(0, 0);
    SetDisplayString(7, stage, 0xF800, 0x0000);
    SetDisplayString(8, "ROUTE FAIL", 0xF800, 0x0000);
    return 0;
}

#define TRY_ROUTE_GO(stage, speed, distance, direction) \
    do { if (!route_go((speed), (distance), (direction))) return route_fail((stage)); } while (0)

#define TRY_ROUTE_TURN(stage, angle, left_speed, right_speed) \
    do { if (!route_turn((angle), (left_speed), (right_speed))) return route_fail((stage)); } while (0)

#define TRY_ARM_MOVE(stage, x, y, z, time) \
    do { if (kinematic((x), (y), (z), (time)) != 0) return route_fail((stage)); } while (0)

static char valid_a1_tag_sample(uint32_t tag_id, double curX, double curY, double curA)
{
    if (tag_id != ROBOT_A1_TAG_ID)
        return 0;
    if (fabs(curX) < 0.0001 && fabs(curY) < 0.0001 && fabs(curA) < 0.0001)
        return 0;
    if (curX < ROBOT_VISION_SAMPLE_MIN_X || curX > ROBOT_VISION_SAMPLE_MAX_X)
        return 0;
    if (curY < ROBOT_VISION_SAMPLE_MIN_Y || curY > ROBOT_VISION_SAMPLE_MAX_Y)
        return 0;
    if (curA < -360.0 || curA > 360.0)
        return 0;
    return 1;
}

static char confirm_a1_tag_seen(unsigned long timeout_ms)
{
    long start_time = GetSysTime();
    int stable_count = 0;

    SetDisplayString(7, "TAG A1", 0xFFE0, 0x0000);
    SetDisplayString(8, "WAIT A1 ID", 0xFFE0, 0x0000);
    SetWaitAICamCmd(ROBOT_AI_CAM_PORT, ROBOT_AI_CAM_MODEL);
    SetWaitForTime(ROBOT_A1_TAG_WARMUP_SEC);

    while ((GetSysTime() - start_time) <= timeout_ms)
    {
        uint32_t tag_id = GetAICamIDData(ROBOT_AI_CAM_PORT, ROBOT_A1_TAG_ID, 1);

        if (tag_id == ROBOT_A1_TAG_ID)
        {
            double curX = GetAICamIDData(ROBOT_AI_CAM_PORT, ROBOT_A1_TAG_ID, 2);
            double curY = GetAICamIDData(ROBOT_AI_CAM_PORT, ROBOT_A1_TAG_ID, 3);
            double curA = GetAICamIDData(ROBOT_AI_CAM_PORT, ROBOT_A1_TAG_ID, 5);

            SetDisplayVar(1, curX, YELLOW, BLACK);
            SetDisplayVar(3, curY, YELLOW, BLACK);
            SetDisplayVar(5, curA, YELLOW, BLACK);

            if (valid_a1_tag_sample(tag_id, curX, curY, curA))
            {
                stable_count++;
                if (stable_count >= ROBOT_A1_SEEN_CONFIRM_LOOPS)
                {
                    speed_control(0, 0);
                    SetDisplayString(8, "A1 SEEN OK", 0x07E0, 0x0000);
                    return 1;
                }
            }
            else
            {
                stable_count = 0;
                speed_control(0, 0);
            }
        }
        else
        {
            stable_count = 0;
            speed_control(0, 0);
            SetDisplayString(8, "A1 NO ID", 0xF800, 0x0000);
        }

        SetWaitForTime(0.02);
    }

    speed_control(0, 0);
    SetDisplayString(8, "TAG FAIL", 0xF800, 0x0000);
    return 0;
}

static char run_baseline_stage_1(void)
{
    SetDisplayString(7, "STAGE 1", 0xFFE0, 0x0000);
    TRY_ROUTE_GO("S1 STRAFE", -40, 53, 2);
    SetWaitForTime(0.2);

    go_times(-18, 1, 0.6);
    SetWaitForTime(0.4);

    TRY_ROUTE_GO("S1 FORWARD", 24, 1.5, 1);
    SetWaitForTime(0.2);

    if (!confirm_a1_tag_seen(12000))
        return route_fail("A1 TAG FAIL");
    SetWaitForTime(0.8);

    TRY_ROUTE_GO("A1 APPROACH", 20, 5.5, 1);
    SetWaitForTime(0.5);

    TRY_ARM_MOVE("A1 DOWN", 0, 190, -10, 1600);
    SetWaitForTime(0.4);
    paw_control(ROBOT_PAW_GRIP, 1500);
    SetWaitForTime(0.7);

    TRY_ARM_MOVE("A1 LIFT", 0, 150, 50, 1600);
    TRY_ROUTE_GO("A1 MOVE", 20, 5.0, 1);
    SetWaitForTime(0.4);

    set_back(1680, 2000, 1650);
    SetWaitForTime(0.5);

    paw_control(ROBOT_PAW_OPEN, 1200);
    SetWaitForTime(0.5);

    TRY_ARM_MOVE("A1 RESET", 0, 120, 30, 2000);
    return 1;
}

static char run_baseline_stage_2(void)
{
    SetDisplayString(7, "STAGE 2", 0xFFE0, 0x0000);
    TRY_ROUTE_GO("S2 MOVE1", -40, 68, 2);
    SetWaitForTime(0.2);
    TRY_ROUTE_GO("S2 MOVE2", 40, 39, 1);
    SetWaitForTime(0.2);
    TRY_ROUTE_GO("S2 MOVE3", 40, 48, 2);
    SetWaitForTime(0.2);
    TRY_ROUTE_GO("S2 MOVE4", -30, 6, 1);
    SetWaitForTime(0.2);

    if (!trace_go_to_tag("TAG A2", 210, 80, 358, 12000, 0))
        return route_fail("A2 TAG FAIL");
    SetWaitForTime(0.2);

    TRY_ROUTE_GO("A2 APPROACH", 30, 5, 1);
    SetWaitForTime(0.2);

    TRY_ARM_MOVE("A2 DOWN", 0, 230, -10, 1400);
    paw_control(1580, 1200);
    SetWaitForTime(0.5);

    TRY_ARM_MOVE("A2 LIFT", 0, 110, 30, 2200);
    SetWaitForTime(0.2);

    TRY_ROUTE_GO("S2 MOVE5", -40, 53, 2);
    SetWaitForTime(0.2);

    TRY_ROUTE_TURN("S2 TURN1", 9.5, -40, 40);
    SetWaitForTime(0.2);

    TRY_ROUTE_GO("S2 MOVE6", 40, 108, 1);
    SetWaitForTime(0.2);

    TRY_ROUTE_GO("S2 MOVE7", 40, 66, 2);
    SetWaitForTime(0.3);

    TRY_ROUTE_TURN("S2 TURN2", 180, -40, 40);
    SetWaitForTime(0.3);

    TRY_ROUTE_GO("S2 MOVE8", -30, 16, 2);
    SetWaitForTime(0.3);

    TRY_ROUTE_GO("S2 MOVE9", -30, 10, 1);
    SetWaitForTime(1);

    if (!trace_go_to_tag("TAG A3", 65, 60, 179, 12000, 0))
        return route_fail("A3 TAG FAIL");
    SetWaitForTime(1);
    return 1;
}

static char run_baseline_stage_3(void)
{
    SetDisplayString(7, "STAGE 3", 0xFFE0, 0x0000);
    TRY_ROUTE_GO("S3 MOVE1", 30, 7, 1);
    SetWaitForTime(0.2);

    TRY_ARM_MOVE("S3 ARM1", 0, 200, 100, 1400);
    SetWaitForTime(0.2);

    TRY_ARM_MOVE("S3 ARM2", 0, 200, 0, 1400);
    paw_control(ROBOT_PAW_OPEN, 1000);
    SetWaitForTime(0.2);

    set_back(1680, 2000, 1660);
    SetWaitForTime(0.2);
    paw_control(ROBOT_PAW_GRIP, 1500);
    SetWaitForTime(0.2);

    set_back(1680, 1200, 1400);
    SetWaitForTime(0.2);
    TRY_ARM_MOVE("S3 ARM3", 0, 200, 100, 1400);
    SetWaitForTime(0.2);

    TRY_ARM_MOVE("S3 ARM4", 0, 190, 30, 1400);
    paw_control(ROBOT_PAW_OPEN, 1200);
    SetWaitForTime(0.2);

    TRY_ARM_MOVE("S3 ARM5", 0, 180, 180, 1200);
    SetWaitForTime(0.2);
    return 1;
}

static char run_baseline_stage_4(void)
{
    SetDisplayString(7, "STAGE 4", 0xFFE0, 0x0000);
    TRY_ROUTE_GO("S4 MOVE1", 40, 2, 3);
    SetWaitForTime(0.2);
    TRY_ROUTE_GO("S4 MOVE2", 60, 80, 2);
    SetWaitForTime(0.2);
    TRY_ROUTE_GO("S4 MOVE3", 40, 16, 1);
    SetWaitForTime(0.2);
    TRY_ROUTE_GO("S4 MOVE4", 60, 30, 2);
    SetWaitForTime(0.2);
    TRY_ROUTE_GO("S4 MOVE5", 60, 11, 3);
    SetWaitForTime(1);
    TRY_ROUTE_GO("S4 MOVE6", 60, 10, 3);
    SetWaitForTime(0.2);
    TRY_ROUTE_GO("S4 MOVE7", 60, 20, 2);
    SetWaitForTime(0.2);

    TRY_ARM_MOVE("S4 ARM1", 0, 150, 50, 1200);
    trace_go_to_tag("TAG END", 180, 100, 0, 15000, 1);

    TRY_ARM_MOVE("S4 ARM2", 0, 230, -10, 1400);
    paw_control(ROBOT_PAW_GRIP, 1200);
    SetWaitForTime(1);
    TRY_ARM_MOVE("S4 ARM3", 0, 230, 270, 1200);
    TRY_ROUTE_GO("S4 MOVE8", 60, 35, 3);
    SetWaitForTime(0.3);
    TRY_ROUTE_GO("S4 MOVE9", 60, 20, 1);
    SetWaitForTime(1);
    TRY_ARM_MOVE("S4 ARM4", 0, 320, 180, 1200);
    SetWaitForTime(1);
    paw_control(ROBOT_PAW_OPEN, 1000);
    SetWaitForTime(1);
    TRY_ROUTE_GO("S4 MOVE10", -60, 15, 1);
    TRY_ARM_MOVE("S4 ARM5", 0, 150, 50, 1200);
    return 1;
}

static char run_stage(RobotStageId stage_id, const char *stage_name, RobotStageRunner runner)
{
    long start_time = GetSysTime();
    long elapsed_ms = 0;
    char ok = 0;

    g_current_stage = stage_id;
    SetDisplayString(7, stage_name, 0xFFE0, 0x0000);
    SetDisplayVar(ROBOT_STAGE_DISPLAY_LINE, (double)stage_id, YELLOW, BLACK);

    ok = runner();
    elapsed_ms = GetSysTime() - start_time;
    speed_control(0, 0);

    SetDisplayString(8, ok ? "STAGE OK" : "STAGE FAIL", ok ? 0x07E0 : 0xF800, 0x0000);
    SetDisplayVar(ROBOT_STAGE_DISPLAY_LINE, (double)elapsed_ms / 1000.0, ok ? 0x07E0 : 0xF800, BLACK);

    if (!ok)
        return 0;

    SetWaitForTime(0.2);
    return 1;
}

static void run_baseline_route(void)
{
    if (!run_stage(ROBOT_STAGE_1, "STAGE 1", run_baseline_stage_1))
        return;
    if (!run_stage(ROBOT_STAGE_2, "STAGE 2", run_baseline_stage_2))
        return;
    if (!run_stage(ROBOT_STAGE_3, "STAGE 3", run_baseline_stage_3))
        return;
    if (!run_stage(ROBOT_STAGE_4, "STAGE 4", run_baseline_stage_4))
        return;

    g_current_stage = ROBOT_STAGE_DONE;
    SetDisplayString(8, "ROUTE DONE", 0x07E0, 0x0000);
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
