/*
 * ========================================
 * 文件名: paw_control.c
 * 描述: 爪子(夹爪)控制模块
 * 功能: 通过舵机控制爪子的开闭，支持自定义角度和执行时间
 * ========================================
 */

#ifndef _PAW_CONTROL_
#define _PAW_CONTROL_
#include <stm32h7xx_hal.h>
#include "fun.c"

#include "HardwareInfo.c"

/*
 * 函数名: paw_control
 * 参数:
 *   angle_data - 舵机目标角度数据 (范围: 500~2500)
 *   time - 舵机执行时间 (单位: 毫秒)
 * 描述:
 *   控制舵机ID为5的舵机，实现爪子的开闭功能:
 *   - angle_data = 750 时爪子处于打开状态
 *   - angle_data = 1700+ 时爪子处于闭合状态
 *   函数会自动等待舵机执行完成
 */
void paw_control(int angle_data, int time)
{
    // 调用舵机通用控制函数
    // 参数: 舵机ID(5), 目标角度, 执行时间
    servo_control(5, angle_data, time);
    
    // 等待舵机执行完成（将毫秒转换为秒）
    SetWaitForTime((float)time/1000);
}
#endif

