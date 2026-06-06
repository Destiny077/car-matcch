/*
 * ========================================
 * 文件名: turn_o.c
 * 描述: 原地旋转控制模块
 * 功能: 使用编码器反馈实现精确的原地旋转控制
 * ========================================
 */

#ifndef _TURN_O_
#define _TURN_O_
#include <stm32h7xx_hal.h>

#include "HardwareInfo.c"
#include <SetMotorCode.h>
#include <GetMotorCode.h>
#include <SetMotorConstSpeed.h>
#include "speed_control.c"

/*
 * 函数名: turn_o
 * 参数:
 *   angle - 旋转角度 (单位: 度, 范围: 0~360)
 *   spl - 左侧电机速度 (范围: -100~100，负值表示反转)
 *   spr - 右侧电机速度 (范围: -100~100，正值表示正转)
 * 描述:
 *   通过差分驱动原理（左右轮速度相反）实现原地旋转。
 *   使用编码器反馈进行精确位置控制，直到旋转角度达到目标值。
 *   通过轮距和编码器数据计算实际旋转角度。
 */
void turn_o(double angle, int spl, int spr)
{
    // ========== 获取外部全局变量 ==========
    extern double g_carD;               // 轮距（两轮中心之间的距离）
    extern double g_wheel2R;            // 轮胎直径
    extern double g_motorencoders;      // 马达一圈的编码值

    long var0 = 0;      // M1当前编码值
    long var1 = 0;      // M2当前编码值
    long code1 = 0;     // M1初始编码值
    long code2 = 0;     // M2初始编码值
    double rate_encoder = 0;    // 编码值转换系数
    long anlge_z_error = 0;     // 旋转角度误差
    long code3 = 0;     // M3初始编码值
    long code4 = 0;     // M4初始编码值
    long var2 = 0;      // M3当前编码值
    long var3 = 0;      // M4当前编码值
    
    // ========== 重置所有电机编码器 ==========
    SetMotorCode(_M1_);
    SetMotorCode(_M2_);
    SetMotorCode(_M3_);
    SetMotorCode(_M4_);
    
    // ========== 保存初始编码值 ==========
    angle = abs(angle);         // 取绝对值
    code1 = GetMotorCode(_M1_);
    code2 = GetMotorCode(_M2_);
    code3 = GetMotorCode(_M3_);
    code4 = GetMotorCode(_M4_);
    
    // ========== 设置电机速度(差分驱动实现旋转) ==========
    SetMotorConstSpeed(_M1_, spl);
    SetMotorConstSpeed(_M2_, spr);
    SetMotorConstSpeed(_M3_, spl);
    SetMotorConstSpeed(_M4_, spr);
    
    angle = abs(angle);
    
    // ========== 编码器反馈控制循环 ==========
    while (1)
    {
        // 获取当前编码值
        var0 = GetMotorCode(_M1_);
        var1 = GetMotorCode(_M2_);
        var2 = GetMotorCode(_M3_);
        var3 = GetMotorCode(_M4_);
        
        // 计算编码值增量
        long inc_encoder_L = (var0 + var2) - (code1 + code3);   // 左轮总编码增量
        long inc_encoder_R = (var1 + var3) - (code2 + code4);   // 右轮总编码增量
        
        // 计算编码值到距离的转换系数
        // 公式: 周长 * 编码值 / 马达一圈的编码值
        rate_encoder = (3.1415926) * g_wheel2R / g_motorencoders;
        
        // 计算左右轮的位移差（米）
        double lenth_error = (double)(inc_encoder_L - inc_encoder_R) * rate_encoder;
        
        // 根据轮距和位移差计算旋转角度
        // 公式: 旋转角度 = (位移差 / 轮距) * (180 / π)
        anlge_z_error = (lenth_error / g_carD) * 51 / 3.1415926;
        
        // 检查是否达到目标旋转角度
        if ( fabs(anlge_z_error) >= angle )
        {
            break;      // 达到目标角度，退出循环
        }
    }
    
    // ========== 停止所有电机 ==========
    speed_control(0, 0);
    
    // 重置编码器
    SetMotorCode(_M1_);
    SetMotorCode(_M2_);
    SetMotorCode(_M3_);
    SetMotorCode(_M4_);
}
#endif

