/*
 * ========================================
 * 文件名: settingRobot.c
 * 描述: 机器人参数配置模块
 * 功能: 初始化机器人的物理参数和电机控制参数
 * ========================================
 */

#ifndef _SETTINGROBOT_
#define _SETTINGROBOT_
#include <stm32h7xx_hal.h>
#include <stm32f4xx_hal.h>

#include "HardwareInfo.c"
#include "hardware_config.c"
#include <SetMotorConstSpeedValue.h>

/*
 * 函数名: settingRobot
 * 参数:
 *   wheel2R - 轮胎直径 (单位: cm)
 *   robotD - 轮距，即两个轮子中心之间的距离 (单位: cm)
 *   sum_encoders - 电机转一圈的编码值总数
 * 描述:
 *   设置机器人的物理参数，用于后续的运动学计算和编码器距离转换。
 *   同时为四个电机设置恒速控制参数（PID参数）。
 */
void settingRobot(double wheel2R, double robotD, double sum_encoders)
{
    // ========== 获取全局变量引用 ==========
    extern double g_carD;           // 轮距
    extern double g_wheel2R;        // 轮胎直径
    extern double g_motorencoders;  // 马达编码值

    // ========== 保存参数到全局变量 ==========
    g_wheel2R = wheel2R;            // 保存轮胎直径
    g_carD = robotD;                // 保存轮距
    g_motorencoders = sum_encoders; // 保存编码值

    // ========== 设置四个电机的恒速控制参数 ==========
    // 参数说明:
    // 第1参数: 电机ID
    // 第2参数: 编码值（用于速度计算）
    // 第3参数: P系数（比例）
    // 第4参数: I系数（积分）
    // 第5参数: D系数（微分）
    
    // 配置M1电机 (前左)
    SetMotorConstSpeedValue(_M1_, g_motorencoders, ROBOT_MOTOR_PID_P, ROBOT_MOTOR_PID_I, ROBOT_MOTOR_PID_D);
    
    // 配置M2电机 (前右)
    SetMotorConstSpeedValue(_M2_, g_motorencoders, ROBOT_MOTOR_PID_P, ROBOT_MOTOR_PID_I, ROBOT_MOTOR_PID_D);
    
    // 配置M3电机 (后左)
    SetMotorConstSpeedValue(_M3_, g_motorencoders, ROBOT_MOTOR_PID_P, ROBOT_MOTOR_PID_I, ROBOT_MOTOR_PID_D);
    
    // 配置M4电机 (后右)
    SetMotorConstSpeedValue(_M4_, g_motorencoders, ROBOT_MOTOR_PID_P, ROBOT_MOTOR_PID_I, ROBOT_MOTOR_PID_D);
}
#endif

