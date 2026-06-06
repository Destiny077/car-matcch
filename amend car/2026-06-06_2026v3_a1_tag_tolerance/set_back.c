/*
 * ========================================
 * 文件名: set_back.c
 * 描述: 舵机组合控制模块（回程动作）
 * 功能: 协调多个舵机执行复杂的动作序列（如放置物体或回程）
 * ========================================
 */

#ifndef _SET_BACK_
#define _SET_BACK_
#include <stm32h7xx_hal.h>
#include "fun.c"

#include "HardwareInfo.c"

/*
 * 函数名: set_back
 * 参数:
 *   s1_data - 舵机0的目标角度数据 (范围: 500~2500)
 *   time - 总执行时间 (单位: 毫秒)
 *   servo1 - 舵机2的目标角度数据 (范围: 500~2500)
 * 描述:
 *   协调控制多个舵机(0、1、2、3)执行一个复杂的动作序列，
 *   通常用于在拾取物体后执行回程或放置动作。
 *   舵机控制顺序:
 *   - 舵机0: 前半段时间执行
 *   - 舵机1、2、3: 完整时间段内执行
 */
void set_back(int s1_data, int time, int servo1)
{
    // 舵机0执行前半段时间
    // 参数: 舵机ID(0), 目标角度(s1_data), 执行时间(time/2)
    servo_control(0, s1_data, time/2);
    
    // 舵机1执行完整时间
    // 参数: 舵机ID(1), 目标角度(servo1), 执行时间(time)
    servo_control(1, servo1, time);
    
    // 舵机2执行固定位置
    // 参数: 舵机ID(2), 目标角度(1150), 执行时间(time)
    servo_control(2, 1150, time);
    
    // 舵机3执行固定位置
    // 参数: 舵机ID(3), 目标角度(2200), 执行时间(time)
    servo_control(3, 2200, time);
    
    // 等待所有舵机执行完成（转换毫秒为秒）
    SetWaitForTime((float)time/1000);
}
#endif

