/*
 * ========================================
 * 文件名: go_times.c
 * 描述: 基于时间的电机控制模块
 * 功能: 按照指定的方向和速度控制电机运转指定时间
 * ========================================
 */

#ifndef _GO_TIMES_
#define _GO_TIMES_
#include <stm32h743xx.h>

#include "HardwareInfo.c"
#include <SetMotorConstSpeed.h>
#include <SetWaitForTime.h>
#include "speed_control.c"

/*
 * 函数名: go_times
 * 参数:
 *   sp - 电机速度 (范围: -100~100)
 *   dir - 运动方向:
 *        1 = 直线前进或后退
 *        2 = 左平移或右平移
 *        3 = 左前平移或右后平移
 *        4 = 右前平移或左后平移
 *   time - 运动时间 (单位: 秒)
 * 描述:
 *   根据指定的方向和速度控制四个电机在指定时间内工作，然后停止。
 *   每种方向对应不同的电机组合速度设置。
 */
void go_times(int sp, int dir, double time)
{
    long sp23 = 0;  // 反向速度值（用于左转等动作）
    
    // 计算反向速度
    sp23 = -sp;
    
    // 根据方向设置电机速度
    if ( dir==1 )
    {
        // 方向1: 直线前进(全向)
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
            // 方向2: 平移(Strafe)左移/右转向
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
                // 方向3: 右转运动
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
                    // 方向4: 左转运动
                    // 前后右电机停止，前后左电机工作
                    SetMotorConstSpeed(_M1_, sp);
                    SetMotorConstSpeed(_M2_, 0);
                    SetMotorConstSpeed(_M3_, 0);
                    SetMotorConstSpeed(_M4_, sp);
                }
            }
        }
    }
    
    // 等待指定的时间
    SetWaitForTime(time);
    
    // 时间到达后停止所有电机
    speed_control(0, 0);
}
#endif

