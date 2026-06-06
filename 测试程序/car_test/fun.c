/*
 * ========================================
 * 文件名: fun.c
 * 描述: 舵机控制和机械臂驱动基础函数模块
 * 功能: 提供舵机通信协议、舵机控制、机械臂驱动等基础功能
 * ========================================
 */

#ifndef _FUN_
#define _FUN_
#include <stm32h7xx_hal.h>
#include <GetMotorCode.h>
#include <SetMotorCode.h>
#include "speed_control.c"                // M1-M4电机速度控制
#include <SetInBeep.h>
#include <GetSysTime.h>
#include <GetRightButton.h>
#include <GetLeftButton.h>
#include <SetDisplayVar.h>
#include <SetWaitForTime.h>
#include <SetMotor.h>
#include <GetSysTime.h>
#include <SetMotorServo.h>
#include <SetMagneticServoDegreeTime.h>

#include <SetAICam.h>
#include <GetAICam.h>
#include "string.h"
#include "init.c"
#include "kinematic.c"
#include <SetDisplayVar.h>

// ========== 外部函数和变量声明 ==========
uint8_t kinematics_move(float x, float y, float z, int time);

extern uint8_t RX_buff[256];            // UART接收缓冲
extern uint8_t RX_byte, RX_Frame_flag;  // UART接收字节和帧标志
extern float L0, L1, L2, L3;            // 机械臂连杆长度
extern int servo_pwm[6];                // 舵机PWM值数组
extern float servo_angle[6];            // 舵机角度数组

/*
 * 函数名: servo_control
 * 参数:
 *   servo_id - 舵机ID (范围: 0~255)
 *   angle_data - 舵机目标角度数据 (范围: 500~2500，对应舵机脉宽)
 *   finish_time - 舵机旋转完成时间 (单位: 毫秒)
 * 描述:
 *   使用U形舵机通信协议控制舵机。协议格式: #XXXPYYYYTZZZZ!
 *   其中:
 *   - XXX: 3位舵机ID
 *   - YYYY: 4位脉宽值(500-2500)
 *   - ZZZZ: 4位执行时间(毫秒)
 *   
 *   示例: "#005P1500T1000!" 表示舵机5运行到1500脉宽，用时1000ms
 */
void servo_control(uint8_t servo_id, uint32_t angle_data, uint32_t finish_time)
{
    char m_str[20];     // 临时字符串缓冲
    // 舵机通信协议模板: #XXXPYYYYTZZZZ!
    uint8_t send_servo_buf[] = "#000P0000T0000!";
    
    // ========== 处理舵机ID，转换为3位字符串 ==========
    sprintf(m_str, "%d", servo_id);
    if(servo_id >= 100 && servo_id < 256)
    {
        // 3位ID: 直接复制
        send_servo_buf[1] = m_str[0];
        send_servo_buf[2] = m_str[1];
        send_servo_buf[3] = m_str[2];
    }
    else if(servo_id >= 10 && servo_id < 100)
    {
        // 2位ID: 前面补0
        send_servo_buf[1] = '0';
        send_servo_buf[2] = m_str[0];
        send_servo_buf[3] = m_str[1];
    }
    else if(servo_id >= 0 && servo_id < 10)
    {
        // 1位ID: 前面补00
        send_servo_buf[1] = '0';
        send_servo_buf[2] = '0';
        send_servo_buf[3] = m_str[0];
    }
    memset(m_str, 0, 20);   // 清空临时缓冲
    
    // ========== 处理脉宽值，转换为4位字符串 ==========
    sprintf(m_str, "%d", angle_data);
    if(angle_data >= 1000 && angle_data < 2501)
    {
        // 4位数字: 直接复制
        send_servo_buf[5] = m_str[0];
        send_servo_buf[6] = m_str[1];
        send_servo_buf[7] = m_str[2];
        send_servo_buf[8] = m_str[3];
    }
    else if(angle_data >= 500 && angle_data < 1000)
    {
        // 3位数字: 前面补0
        send_servo_buf[5] = '0';
        send_servo_buf[6] = m_str[0];
        send_servo_buf[7] = m_str[1];
        send_servo_buf[8] = m_str[2];
    }
    memset(m_str, 0, 20);   // 清空临时缓冲
    
    // ========== 处理执行时间，转换为4位字符串 ==========
    sprintf(m_str, "%d", finish_time);
    if(finish_time >= 1000 && finish_time < 10000)
    {
        // 4位时间: 直接复制
        send_servo_buf[10] = m_str[0];
        send_servo_buf[11] = m_str[1];
        send_servo_buf[12] = m_str[2];
        send_servo_buf[13] = m_str[3];
    }
    else if(finish_time >= 100 && finish_time < 1000)
    {
        // 3位时间: 前面补0
        send_servo_buf[10] = '0';
        send_servo_buf[11] = m_str[0];
        send_servo_buf[12] = m_str[1];
        send_servo_buf[13] = m_str[2];
    }
    else if(finish_time >= 10 && finish_time < 100)
    {
        // 2位时间: 前面补00
        send_servo_buf[10] = '0';
        send_servo_buf[11] = '0';
        send_servo_buf[12] = m_str[0];
        send_servo_buf[13] = m_str[1];
    }
    else if(finish_time >= 0 && finish_time < 10)
    {
        // 1位时间: 前面补000
        send_servo_buf[10] = '0';
        send_servo_buf[11] = '0';
        send_servo_buf[12] = '0';
        send_servo_buf[13] = m_str[0];
    }
    memset(m_str, 0, 20);   // 清空临时缓冲
    
    // ========== 通过UART发送舵机命令 ==========
    P8_uart_send(send_servo_buf, strlen((char*)send_servo_buf));
}

/*
 * 注释掉的函数: get_id
 * 功能: 查询舵机ID
 * 注: 舵机与主机是单向通讯，主机无法接收舵机反馈数据
 */

/*
 * 函数名: arm_drive
 * 参数:
 *   finish_time - 舵机执行时间 (单位: 毫秒)
 * 描述:
 *   驱动机械臂的4个舵机(0~3)执行由kinematics_analysis计算好的位置。
 *   这4个舵机分别控制:
 *   - 舵机0: 底座旋转(绕Z轴)
 *   - 舵机1: 第一段关节
 *   - 舵机2: 第二段关节
 *   - 舵机3: 第三段关节
 */
void arm_drive(uint32_t finish_time)
{
    // 依次控制4个舵机，使用之前计算好的PWM值
    servo_control(0, servo_pwm[0], finish_time);
    servo_control(1, servo_pwm[1], finish_time);
    servo_control(2, servo_pwm[2], finish_time);
    servo_control(3, servo_pwm[3], finish_time);
    
    // 等待舵机执行完成
    SetWaitForTime((float)finish_time / 1000);
}

/*
 * 函数名: arm_init
 * 描述:
 *   初始化机械臂所有舵机到中位(1500脉宽)，
 *   然后执行一个2秒的平缓运动，确保舵机启动正常。
 */
void arm_init(void)
{
    int count_i = 0;
    
    // 设置所有舵机(0~5)的PWM值为中位1500
    for(count_i = 0; count_i < 6; count_i++)
    {
        servo_pwm[count_i] = 1500;
    }
    
    SetWaitForTime(0.1);        // 等待100ms
    arm_drive(2000);            // 执行2秒的初始化运动
    SetWaitForTime(2);          // 等待运动完成
}

#include "HardwareInfo.c"

/*
 * 函数名: fun
 * 描述: 占位函数（当前为空）
 */
void fun()
{
    // 该函数为空
}
#endif

