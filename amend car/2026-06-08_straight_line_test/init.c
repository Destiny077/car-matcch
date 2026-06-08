/*
 * ========================================
 * 文件名: init.c
 * 描述: 系统初始化模块
 * 功能: 初始化GPIO、UART、机械臂参数和控制接口
 * ========================================
 */

#ifndef _INIT_
#define _INIT_
#include <stm32h7xx_hal.h>
#include <SetData.h>
#include <GetData.h>
#include <GetMotorCode.h>
#include <SetMotorCode.h>
#include <GetAHRS.h>
#include <SetAHRS.h>
#include <SetUartData.h>
#include <GetUartData.h>
#include "speed_control.c"
#include <SetInBeep.h>
#include <GetSysTime.h>
#include <SetLCDClear.h>
#include <GetRightButton.h>
#include <GetLeftButton.h>
#include <SetDisplayPicture.h>
#include <SetDisplayString.h>
#include <SetDisplayVar.h>
#include <SetWaitForTime.h>
#include "fun.c"
#include "view_all.c"
#include "JMLib.c"
#include "hardware_config.c"
#include <jmkernel.h>
#include <math.h>

// ========== 机械臂运动学函数声明 ==========
uint8_t kinematics_move(float x, float y, float z, int time);
char touch1(int x1, int y1, int x2, int y2);
void control_interface(void);
char interface_touch(void);

// ========== 机械臂参数 ==========
float L0, L1, L2, L3;           // 机械臂各段连杆长度
int arm_x, arm_y, arm_z;        // 机械臂当前位置坐标

// ========== UART通讯相关变量 ==========
uint8_t RX_buff[256] = {0};     // UART接收缓冲，最多256字节
uint8_t RX_byte, RX_Frame_flag = 0;  // 接收字节和帧完成标志
int RX_num = 0;                 // 接收数据计数

// ========== 系统状态变量 ==========
char drive_status = 0;          // 驱动状态
char touch_state = 0;           // 触摸状态

/*
 * 函数名: setup_kinematics
 * 参数:
 *   l0 - 基座高度
 *   l1 - 第一段连杆长度
 *   l2 - 第二段连杆长度
 *   l3 - 第三段连杆长度
 * 描述:
 *   初始化机械臂的运动学参数，设置GPIO用于控制舵机通讯。
 *   同时将机械臂的初始位置设置为(0, 100, 100)。
 */
void setup_kinematics(float l0, float l1, float l2, float l3) 
{
    // ========== 保存机械臂参数 ==========
    L0 = l0;    // 基座高度
    L1 = l1;    // 第一段连杆
    L2 = l2;    // 第二段连杆
    L3 = l3;    // 第三段连杆
    
    // ========== 初始化机械臂位置 ==========
    arm_x = ROBOT_ARM_INIT_X;
    arm_y = ROBOT_ARM_INIT_Y;
    arm_z = ROBOT_ARM_INIT_Z;
    
    // ========== 初始化GPIO(用于舵机通讯) ==========
    GPIO_InitTypeDef GPIO_PortInitStruct;
    
    // 使能GPIOE时钟
    __HAL_RCC_GPIOE_CLK_ENABLE();

    // 配置PE15作为输出引脚（用于舵机通讯控制）
    GPIO_PortInitStruct.Pin = GPIO_PIN_15;
    GPIO_PortInitStruct.Mode = GPIO_MODE_OUTPUT_PP;    // 推挽输出
    GPIO_PortInitStruct.Pull = GPIO_PULLUP;             // 上拉
    GPIO_PortInitStruct.Speed = GPIO_SPEED_FREQ_LOW;    // 低速
    HAL_GPIO_Init(GPIOE, &GPIO_PortInitStruct);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);  // 初始设置为高
}

/*
 * 函数名: P8_uart_send
 * 参数:
 *   buffer - 发送数据缓冲指针
 *   length - 发送数据长度
 * 描述:
 *   通过UART6发送数据到舵机（包括RS485收发控制）。
 *   PE15引脚用于控制RS485收发转换芯片。
 */
void P8_uart_send(uint8_t* buffer, uint16_t length)
{
    int i = 0;
    
    SetWaitForTime(0.002);    //延时
    // ========== 设置RS485为发送模式 ==========
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
    
    // ========== 发送数据 ==========
    for (i = 0; i < length; i++) 
    {
        SetUartData(buffer[i], 6);  // 通过UART6发送单字节
    }
    SetWaitForTime(0.002);
    
    // ========== 设置RS485为接收模式 ==========
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
}

/*
 * 函数名: P8_RecvFrame
 * 描述:
 *   阻塞式接收UART数据帧。帧格式为: #...!
 *   以#开始，以!结束。接收完整一帧后设置RX_Frame_flag标志。
 */
void P8_RecvFrame(void)
{	
    while(1)
    {
        // 获取单个字节
        RX_byte = GetUartData(6);
        
        if(RX_byte != 0 && RX_Frame_flag == 0)
        {
            // 根据接收字节类型进行处理
            switch(RX_byte)
            {
                // ========== 帧开始标志 ==========
                case '#':
                    RX_buff[RX_num] = RX_byte;
                    RX_num++;
                    break;
                
                // ========== 帧结束标志 ==========
                case '!':
                    RX_buff[RX_num] = RX_byte;
                    RX_num++;
                    RX_Frame_flag = 1;      // 标记帧接收完成
                    RX_num = 0;              // 重置计数器
                    break;
                
                // ========== 默认情况 - 数据字节 ==========
                default:
                    if(RX_num != 0)
                    {
                        RX_buff[RX_num] = RX_byte;
                        RX_num++;
                    }
                    break;
            }
        }
        // ========== 帧接收完成，退出循环 ==========
        else if(RX_Frame_flag == 1)
        {
            break;
        }
    }
}

/*
 * 函数名: Uart_ProcessFrame
 * 描述:
 *   处理接收到的UART数据帧。
 *   如果帧接收完成，将其原样回显发送，然后清空缓冲。
 */
void Uart_ProcessFrame(void)
{
    if (RX_Frame_flag == 1)
    {
        // ========== 回显发送接收到的数据 ==========
        P8_uart_send(RX_buff, strlen((char*)RX_buff));
        
        // ========== 清空缓冲 ==========
        memset(RX_buff, 0, sizeof(RX_buff));
        
        // ========== 重置帧标志 ==========
        RX_Frame_flag = 0;
    }
}

/*
 * 函数名: button_detect
 * 返回值:
 *   1 = 检测到右按键被按下
 *   0 = 未检测到
 * 描述:
 *   检测右按键的按下事件，包括防抖处理。
 *   按键按下时等待按键释放后才返回。
 */
char button_detect(void)
{
    if(GetRightButton() == 1)
    {
        SetWaitForTime(0.01);   // 防抖延时
        if(GetRightButton() == 1)
        {
            // 等待按键释放
            while(GetRightButton() == 1);
            return 1;           // 按键已按下并释放
        }
        else 
            return 0;           // 误检
    }
    else 
        return 0;               // 无按键
}

#include "HardwareInfo.c"

/*
 * 函数名: init
 * 描述:
 *   系统初始化函数。包括:
 *   1. 初始化机械臂运动学参数(连杆长度)
 *   2. 驱动机械臂到初始位置
 *   3. 初始化舵机(4号和5号)
 *   4. 显示触摸控制界面
 *   5. 等待用户按右键开始主程序
 */
void init()
{
    // ========== 初始化机械臂参数 ==========
    // 参数: 基座高度(100), 连杆L1(106), L2(85), L3(160)
    setup_kinematics(ROBOT_ARM_L0, ROBOT_ARM_L1, ROBOT_ARM_L2, ROBOT_ARM_L3);
    
    // ========== 驱动机械臂到初始位置 ==========
    // 将机械臂运动学逆解算结果发送给舵机
    if (kinematics_move(arm_x, arm_y, arm_z, 2000) != 0)
        SetDisplayString(7, "ARM INIT FAIL", 0xF800, 0x0000);
    
    // ========== 初始化其他舵机位置 ==========
    servo_control(ROBOT_SERVO_AUX_ID, ROBOT_SERVO_AUX_INIT, 1000);   // 舵机4: 中位
    servo_control(ROBOT_SERVO_PAW_ID, ROBOT_SERVO_PAW_INIT, 1000);   // 舵机5: 角度调整
    
    // ========== 显示触摸控制界面 ==========
    control_interface();
    
    // ========== 触摸控制循环 ==========
    // 等待用户操作或右键开始信号
    while (1)
    {
        // 获取触摸事件
        touch_state = interface_touch();
        
        // ========== 右键按下：开始主程序 ==========
        if(touch_state == 3)
        {
            // 清屏，退出初始化
            SetLCDClear(BLACK);
            break;
        }
        // ========== 更新按钮：执行当前位置 ==========
        else if(touch_state == 2)
        {
            // 根据当前坐标驱动机械臂
            if (kinematics_move(arm_x, arm_y, arm_z, 1000) == 0)
                SetWaitForTime(1);
            else
                SetDisplayString(7, "ARM MOVE FAIL", 0xF800, 0x0000);
        }
    }  
}
#endif

