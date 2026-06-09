/*
 * ========================================
 * 文件名: speed_control.c
 * 描述: 电机速度控制模块
 * 功能: 设置四个电机(M1-M4)的转速，支持前进和后退方向控制
 * ========================================
 */

#ifndef _SPEED_CONTROL_
#define _SPEED_CONTROL_
#include <stm32h743xx.h>

#include "HardwareInfo.c"
#include <SetMotorConstSpeed.h>

/*
 * 函数名: speed_control
 * 参数:
 *   spl - 左侧电机速度 (范围: -100 ~ 100)
 *   spr - 右侧电机速度 (范围: -100 ~ 100)
 * 描述: 
 *   设置四个电机的恒速转动。其中:
 *   - M1和M3为左侧电机对（前后各一个）
 *   - M2和M4为右侧电机对（前后各一个）
 *   - 正值表示正转，负值表示反转
 *   - 速度值会被限制在±100范围内
 */
void speed_control(int spl, int spr)
{
    // ===== 对左侧速度进行限制 =====
    if ( spl>=100 )
    {
        spl=100;        // 最大正速度100
    }
    if ( spl<=-100 )
    {
        spl=-100;       // 最大负速度-100
    }
    
    // ===== 对右侧速度进行限制 =====
    if ( spr>=100 )
    {
        spr=100;        // 最大正速度100
    }
    if ( spr<=-100 )
    {
        spr=-100;       // 最大负速度-100
    }
    
    // ===== 设置所有电机转速 =====
    SetMotorConstSpeed(_M1_, spl);     // 前左电机
    SetMotorConstSpeed(_M2_, spr);     // 前右电机
    SetMotorConstSpeed(_M3_, spl);     // 后左电机
    SetMotorConstSpeed(_M4_, spr);     // 后右电机
}
#endif

