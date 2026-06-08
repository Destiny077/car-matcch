/*
 * Encoder-based chassis motion control.
 *
 * v6 keeps the original four direction modes, but fixes the M1 encoder read,
 * clamps unsafe inputs, adds a simple trapezoid-like speed ramp, and keeps
 * forward encoder balancing active throughout the move.
 */

#ifndef _GO_BMP_
#define _GO_BMP_

#include <stm32h7xx_hal.h>
#include <stm32f4xx_hal.h>

#include "HardwareInfo.c"
#include <SetMotorCode.h>
#include <GetMotorCode.h>
#include <GetSysTime.h>
#include <SetMotorConstSpeed.h>
#include <SetWaitForTime.h>
#include "hardware_config.c"
#include "speed_control.c"

static long encoder_abs_long(long value)
{
    return (value < 0) ? -value : value;
}

static int route_abs_int(int value)
{
    return (value < 0) ? -value : value;
}

static int clamp_int(int value, int min_value, int max_value)
{
    if (value > max_value)
        return max_value;
    if (value < min_value)
        return min_value;
    return value;
}

static void apply_direction_speed(int sp, int dir)
{
    int sp_reverse = -sp;

    if (dir == 1)
    {
        SetMotorConstSpeed(_M1_, sp);
        SetMotorConstSpeed(_M2_, sp);
        SetMotorConstSpeed(_M3_, sp);
        SetMotorConstSpeed(_M4_, sp);
    }
    else if (dir == 2)
    {
        SetMotorConstSpeed(_M1_, sp_reverse);
        SetMotorConstSpeed(_M2_, sp);
        SetMotorConstSpeed(_M3_, sp);
        SetMotorConstSpeed(_M4_, sp_reverse);
    }
    else if (dir == 3)
    {
        SetMotorConstSpeed(_M1_, 0);
        SetMotorConstSpeed(_M2_, sp);
        SetMotorConstSpeed(_M3_, sp);
        SetMotorConstSpeed(_M4_, 0);
    }
    else if (dir == 4)
    {
        SetMotorConstSpeed(_M1_, sp);
        SetMotorConstSpeed(_M2_, 0);
        SetMotorConstSpeed(_M3_, 0);
        SetMotorConstSpeed(_M4_, sp);
    }
}

static void balance_forward_speed(int sp, long left_code, long right_code)
{
    long error = left_code - right_code;
    int adjust = (int)((double)error * ROBOT_ROUTE_ENCODER_BALANCE_GAIN);
    int signed_adjust = 0;

    adjust = clamp_int(adjust, -ROBOT_ROUTE_ENCODER_BALANCE_LIMIT, ROBOT_ROUTE_ENCODER_BALANCE_LIMIT);
    signed_adjust = (sp >= 0) ? adjust : -adjust;

    SetMotorConstSpeed(_M1_, sp - signed_adjust);
    SetMotorConstSpeed(_M2_, sp + signed_adjust);
    SetMotorConstSpeed(_M3_, sp - signed_adjust);
    SetMotorConstSpeed(_M4_, sp + signed_adjust);
}

static int balanced_motor_command(int sp, int direction_sign, long progress, long target_progress)
{
    int base = direction_sign * sp;
    int magnitude = route_abs_int(base);
    int adjust = (int)((double)(target_progress - progress) * ROBOT_ROUTE_ENCODER_BALANCE_GAIN);
    int max_speed = (int)ROBOT_ROUTE_MAX_SPEED;

    if (direction_sign == 0 || sp == 0)
        return 0;

    adjust = clamp_int(adjust, -ROBOT_ROUTE_ENCODER_BALANCE_LIMIT, ROBOT_ROUTE_ENCODER_BALANCE_LIMIT);
    magnitude = clamp_int(magnitude + adjust, 0, max_speed);

    return (base < 0) ? -magnitude : magnitude;
}

static void balance_direction_speed(int sp, int dir, long d1, long d2, long d3, long d4)
{
    int s1 = 0;
    int s2 = 0;
    int s3 = 0;
    int s4 = 0;
    long sum = 0;
    int active = 0;
    long target = 0;

    if (dir == 1)
    {
        s1 = 1;
        s2 = 1;
        s3 = 1;
        s4 = 1;
    }
    else if (dir == 2)
    {
        s1 = -1;
        s2 = 1;
        s3 = 1;
        s4 = -1;
    }
    else if (dir == 3)
    {
        s2 = 1;
        s3 = 1;
    }
    else if (dir == 4)
    {
        s1 = 1;
        s4 = 1;
    }

    if (s1 != 0)
    {
        sum += d1;
        active++;
    }
    if (s2 != 0)
    {
        sum += d2;
        active++;
    }
    if (s3 != 0)
    {
        sum += d3;
        active++;
    }
    if (s4 != 0)
    {
        sum += d4;
        active++;
    }

    if (active == 0)
    {
        speed_control(0, 0);
        return;
    }

    target = sum / active;
    SetMotorConstSpeed(_M1_, balanced_motor_command(sp, s1, d1, target));
    SetMotorConstSpeed(_M2_, balanced_motor_command(sp, s2, d2, target));
    SetMotorConstSpeed(_M3_, balanced_motor_command(sp, s3, d3, target));
    SetMotorConstSpeed(_M4_, balanced_motor_command(sp, s4, d4, target));
}

static double route_moved_cm(int dir, long d1, long d2, long d3, long d4, double encoder_rate)
{
    if (dir == 1)
        return (double)(d1 + d2 + d3 + d4) * encoder_rate / 4.0;
    if (dir == 2)
        return (double)(d1 + d2 + d3 + d4) * encoder_rate / 4.0 * 0.888;
    if (dir == 3)
        return (double)(d2 + d3) * encoder_rate / 2.0;
    if (dir == 4)
        return (double)(d1 + d4) * encoder_rate / 2.0;
    return 0.0;
}

static int route_profile_speed(int target_sp, double moved_cm, double target_cm)
{
    int target_abs = route_abs_int(target_sp);
    int min_abs = ROBOT_ROUTE_RAMP_MIN_SPEED;
    int profiled_abs = target_abs;
    double remain_cm = target_cm - moved_cm;
    double scale = 1.0;

    if (target_abs <= min_abs || target_cm <= 0.001 || ROBOT_ROUTE_RAMP_CM <= 0.001)
        return target_sp;

    if (moved_cm < ROBOT_ROUTE_RAMP_CM)
        scale = moved_cm / ROBOT_ROUTE_RAMP_CM;
    if (remain_cm < ROBOT_ROUTE_RAMP_CM && (remain_cm / ROBOT_ROUTE_RAMP_CM) < scale)
        scale = remain_cm / ROBOT_ROUTE_RAMP_CM;

    if (scale < 0.0)
        scale = 0.0;
    if (scale > 1.0)
        scale = 1.0;

    profiled_abs = min_abs + (int)((double)(target_abs - min_abs) * scale);
    profiled_abs = clamp_int(profiled_abs, min_abs, target_abs);

    return (target_sp < 0) ? -profiled_abs : profiled_abs;
}

char go_bmp(int sp, double bmp, int dir)
{
    extern double g_wheel2R;
    extern double g_motorencoders;
    extern double rate_encoder;

    long vl = 0;
    long vr = 0;
    long vll = 0;
    long vrr = 0;
    long code1 = 0;
    long code2 = 0;
    long code3 = 0;
    long code4 = 0;
    long start_time = 0;
    long timeout_ms = ROBOT_MOVE_TIMEOUT_BASE_MS;
    int max_speed = (int)ROBOT_ROUTE_MAX_SPEED;
    char reached = 0;

    if (dir < 1 || dir > 4)
        return 0;
    if (bmp < 0)
        bmp = -bmp;
    if (bmp <= 0.001)
        return 1;
    if (g_wheel2R <= 0.0 || g_motorencoders <= 0.0)
        return 0;

    sp = clamp_int(sp, -max_speed, max_speed);
    if (sp == 0)
        return 0;

    timeout_ms += (long)(bmp * ROBOT_MOVE_TIMEOUT_PER_CM_MS);
    rate_encoder = (3.1415926) * g_wheel2R / g_motorencoders;

    SetMotorCode(_M1_);
    SetMotorCode(_M2_);
    SetMotorCode(_M3_);
    SetMotorCode(_M4_);

    code1 = GetMotorCode(_M1_);
    code2 = GetMotorCode(_M2_);
    code3 = GetMotorCode(_M3_);
    code4 = GetMotorCode(_M4_);
    start_time = GetSysTime();

    apply_direction_speed(route_profile_speed(sp, 0.0, bmp), dir);

    while (1)
    {
        long d1 = 0;
        long d2 = 0;
        long d3 = 0;
        long d4 = 0;
        double moved_cm = 0.0;
        int profiled_sp = 0;

        if ((GetSysTime() - start_time) > timeout_ms)
            break;

        vl = GetMotorCode(_M1_);
        vr = GetMotorCode(_M2_);
        vll = GetMotorCode(_M3_);
        vrr = GetMotorCode(_M4_);

        d1 = encoder_abs_long(vl - code1);
        d2 = encoder_abs_long(vr - code2);
        d3 = encoder_abs_long(vll - code3);
        d4 = encoder_abs_long(vrr - code4);
        moved_cm = route_moved_cm(dir, d1, d2, d3, d4, rate_encoder);

        if (moved_cm >= bmp)
        {
            reached = 1;
            break;
        }

        profiled_sp = route_profile_speed(sp, moved_cm, bmp);
        balance_direction_speed(profiled_sp, dir, d1, d2, d3, d4);

        SetWaitForTime(ROBOT_MOVE_CONTROL_PERIOD_SEC);
    }

    speed_control(0, 0);

    SetMotorCode(_M1_);
    SetMotorCode(_M2_);
    SetMotorCode(_M3_);
    SetMotorCode(_M4_);

    return reached;
}

#endif
