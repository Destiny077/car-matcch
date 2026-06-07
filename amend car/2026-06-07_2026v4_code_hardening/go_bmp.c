/*
 * ========================================
 * 文件名: go_bmp.c
 * 描述: 基于编码器的直线/转向运动控制模块
 * 功能: 使用电机编码器反馈实现精确的距离控制运动
 * ========================================
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
#include "hardware_config.c"
#include "speed_control.c"

static long encoder_abs_long(long value)
{
    return (value < 0) ? -value : value;
}

/*
 * 函数名: go_bmp
 * 参数:
 *   sp - 电机速度 (范围: -100~100)
 *   bmp - 目标距离 (单位: cm)
 *   dir - 运动方向:
 *        1 = 直线前进或后退
 *        2 = 左平移或右平移
 *        3 = 左前平移或右后平移
 *        4 = 右前平移或左后平移
 * 描述:
 *   使用四个电机的编码器反馈进行精确的距离控制运动。
 *   通过计算每个电机的编码增量，转换为实际位移距离，
 *   当累计位移达到目标距离时，停止电机。
 */
char go_bmp(int sp, double bmp, int dir)
{
    // ========== 获取外部全局变量 ==========
    extern double g_wheel2R;        // 轮胎直径
    extern double g_carD;           // 轮距
    extern double g_motorencoders;  // 马达编码值
    extern double rate_encoder;     // 编码值转换系数

    // ========== 局部变量声明 ==========
    long vl = 0;        // M1电机(前左)的实时编码值
    long vr = 0;        // M2电机(前右)的实时编码值
    long vll = 0;       // M3电机(后左)的实时编码值
    long vrr = 0;       // M4电机(后右)的实时编码值
    long sp23 = 0;      // 速度参数的反向值（用于某些方向）
    long code1 = 0;     // M1电机的初始编码值
    long code2 = 0;     // M2电机的初始编码值
    long code3 = 0;     // M3电机的初始编码值
    long code4 = 0;     // M4电机的初始编码值
    long start_time = GetSysTime();
    long timeout_ms = ROBOT_MOVE_TIMEOUT_BASE_MS;
    char reached = 0;
    
    // ========== 初始化编码器并保存初始值 ==========
    SetMotorCode(_M1_);             // 重置M1编码器
    SetMotorCode(_M2_);             // 重置M2编码器
    SetMotorCode(_M3_);             // 重置M3编码器
    SetMotorCode(_M4_);             // 重置M4编码器
    
    code1 = GetMotorCode(_M1_);     // 保存M1初始编码值
    code2 = GetMotorCode(_M2_);     // 保存M2初始编码值
    code3 = GetMotorCode(_M3_);     // 保存M3初始编码值
    code4 = GetMotorCode(_M4_);     // 保存M4初始编码值
    
    // ========== 参数处理 ==========
    if (bmp < 0)
        bmp = -bmp;                 // 取目标距离的绝对值
    timeout_ms += (long)(bmp * ROBOT_MOVE_TIMEOUT_PER_CM_MS);
    sp23 = -sp;                     // 计算反向速度
    
    // ========== 根据方向设置电机速度 ==========
    if ( dir==1 )
    {
        // 方向1: 直线前进(四驱全向)
        // 四个电机同速正转
        SetMotorConstSpeed(_M1_, sp);
        SetMotorConstSpeed(_M2_, sp);
        SetMotorConstSpeed(_M3_, sp);
        SetMotorConstSpeed(_M4_, sp);
    }
    else
    {
        if ( dir == 2 )
        {
            // 方向2: 平移运动(左移或右转向)
            // 左侧电机反转，右侧电机正转
            SetMotorConstSpeed(_M1_, sp23);
            SetMotorConstSpeed(_M2_, sp);
            SetMotorConstSpeed(_M3_, sp);
            SetMotorConstSpeed(_M4_, sp23);
        }
        else
        {
            if ( dir == 3 )
            {
                // 方向3: 右转（右移）
                // 前后左电机停止，前后右电机工作
                SetMotorConstSpeed(_M1_, 0);
                SetMotorConstSpeed(_M2_, sp);
                SetMotorConstSpeed(_M3_, sp);
                SetMotorConstSpeed(_M4_, 0);
            }
            else
            {
                if ( dir == 4 )
                {
                    // 方向4: 左转（左移）
                    // 前后右电机停止，前后左电机工作
                    SetMotorConstSpeed(_M1_, sp);
                    SetMotorConstSpeed(_M2_, 0);
                    SetMotorConstSpeed(_M3_, 0);
                    SetMotorConstSpeed(_M4_, sp);
                }
            }
        }
    }
    
    // ========== 编码器反馈控制循环 ==========
    while (1)
    {
        if ((GetSysTime() - start_time) > timeout_ms)
        {
            break;
        }

        // 获取当前编码值
        vl = GetMotorCode(_M1_);
        vr = GetMotorCode(_M2_);
        vll = GetMotorCode(_M3_);
        vrr = GetMotorCode(_M4_);
        
        // 计算编码值到距离的转换系数
        // 公式: (π * 轮胎直径) / 编码值
        rate_encoder = (3.1415926) * g_wheel2R / g_motorencoders;
        
        // 根据不同的方向计算累计距离
        if ( dir == 1 )
        {
            long distance_code = encoder_abs_long((vl - code1) + (vll - code3) + (vr - code2) + (vrr - code4));
            if ( (double)distance_code * rate_encoder / 4.0 >= bmp )
            {
                reached = 1;
                break;      // 达到目标距离，退出循环
            }
        }
        else if ( dir == 2 )
        {
            long distance_code = encoder_abs_long((code1 - vl) + (code4 - vrr) + (vr - code2) + (vll - code3));
            if ( (double)distance_code * rate_encoder / 4.0 * 0.888 >= bmp )
            {
                reached = 1;
                break;  // 达到目标距离，退出循环
            }
        }
        else if ( dir == 3 )
        {
            long distance_code = encoder_abs_long((vr - code2) + (vll - code3));
            if ( (double)distance_code * rate_encoder / 2.0 >= bmp )
            {
                reached = 1;
                break;  // 达到目标距离，退出循环
            }
        }
        else if ( dir == 4 )
        {
            long distance_code = encoder_abs_long((vl - code1) + (vrr - code4));
            if ( (double)distance_code * rate_encoder / 2.0 >= bmp )
            {
                reached = 1;
                break;  // 达到目标距离，退出循环
            }
        }
    }
    
    // ========== 停止所有电机 ==========
    speed_control(0, 0);
    
    // ========== 重置编码器 ==========
    SetMotorCode(_M1_);
    SetMotorCode(_M2_);
    SetMotorCode(_M3_);
    SetMotorCode(_M4_);

    return reached;
}
#endif

