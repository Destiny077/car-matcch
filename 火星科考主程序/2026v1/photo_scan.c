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
 * 函数名: photo_scan
 * 参数:
 *   s0_data - 舵机0的目标角度数据 (范围: 500~2500)
 *   s1_data - 舵机1的目标角度数据 (范围: 500~2500)
 *   s2_data - 舵机2的目标角度数据 (范围: 500~2500)
 *   s3_data - 舵机3的目标角度数据 (范围: 500~2500)
 *   s4_data - 舵机4的目标角度数据 (范围: 500~2500)
 *   s5_data - 舵机5的目标角度数据 (范围: 500~2500)
 *   time - 总执行时间 (单位: 毫秒)
 * 描述:
 *   控制多个舵机(0、1、2、3、4、5)自定义角度，
 *   通常用于设计自定义动作。
 */
void photo_scan(int s0_data, int s1_data, int s2_data, int s3_data, int s4_data, int s5_data, int time)
{
    // 舵机0执行前半段时间
    // 参数: 舵机ID(0), 目标角度(s1_data), 执行时间(time/2)
    servo_control(0, s0_data, time);

    // 舵机1执行完整时间
    // 参数: 舵机ID(1), 目标角度(servo1), 执行时间(time)
    servo_control(1, s1_data, time);

    // 舵机2执行固定位置
    // 参数: 舵机ID(2), 目标角度(1150), 执行时间(time)
    servo_control(2, s2_data, time);

    // 舵机3执行固定位置
    // 参数: 舵机ID(3), 目标角度(2200), 执行时间(time)
    servo_control(3, s3_data, time);

    // 舵机2执行固定位置
    // 参数: 舵机ID(4), 目标角度(1150), 执行时间(time)
    servo_control(4, s4_data, time);

    // 舵机3执行固定位置
    // 参数: 舵机ID(5), 目标角度(2200), 执行时间(time)
    servo_control(5, s5_data, time);

    // 等待所有舵机执行完成（转换毫秒为秒）
    SetWaitForTime((float)time / 1000);
}

#endif