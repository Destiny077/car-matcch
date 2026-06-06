/*
 * ========================================
 * 文件名: 2026v1.c
 * 描述: 机器人主程序 - 负责整个系统的初始化和主任务流程控制
 * 功能: 初始化系统参数、电机方向、获取PID参数、执行运动任务序列
 * ========================================
 */

#include <stm32h7xx_hal.h>
#include <SetMotor.h>
#include <SetDisplayVar.h>
#include <GetData.h>
#include "fun.c"

// PID控制器结构体定义
typedef struct
{
    double kp;         // 比例系数 (Proportional)
    double ki;         // 积分系数 (Integral)
    double kd;         // 微分系数 (Derivative)
    double prev_error; // 上一次误差
    double integral;   // 积分累加项
} PID_Controller;

// 定义三个方向的PID控制器
PID_Controller pidX = {0, 0, 0.0, 0, 0}; // X轴PID控制器
PID_Controller pidY = {0, 0, 0.0, 0, 0}; // Y轴PID控制器
PID_Controller pidW = {0, 0, 0.0, 0, 0}; // 旋转角度PID控制器

// 包含所有模块
#include "go_to_tag.c"         // 图像导航模块
#include "HardwareInfo.c"      // 硬件信息模块
#include "JMLib.c"             // 库文件
#include <SetMotorDirection.h> // 电机方向控制
#include "settingRobot.c"      // 机器人参数设置模块
#include "speed_control.c"     // 电机速度控制模块
#include <SetFontSize.h>       // 字体大小设置
#include "init.c"              // 系统初始化模块
#include <SetWaitAICamCmd.h>   // AI摄像头命令
#include "kinematic.c"         // 运动学模块
#include "paw_control.c"       // 爪子控制模块
#include "go_bmp.c"            // 基于编码值的直线运动模块
#include <SetWaitForTime.h>    // 延时函数
#include <SetDisplayString.h>  // 显示字符串
#include "go_times.c"          // 按时间的电机控制模块
#include "set_back.c"          // 舵机组合控制模块
#include "turn_o.c"            // 原地旋转模块

// ========== 全局变量定义 ==========
double Kp = 0;              // PID比例系数
double Ki = 0;              // PID积分系数
double Kd = 0;              // PID微分系数
double g_carD = 0;          // 轮距（两轮中心之间的距离）单位: cm
double g_wheel2R = 0;       // 轮胎的直径 单位: cm
double g_motorencoders = 0; // 马达转动一圈的编码值
double rate_encoder = 0;    // 编码值转换单位（cm）

/*
 * 函数名: main
 * 描述: 系统主程序入口
 * 功能流程:
 *   1. 系统初始化
 *   2. 设置电机旋转方向
 *   3. 配置机器人参数(轮径26.5cm, 轮距6.0cm, 编码器计数2048)
 *   4. 获取PID参数并设置
 *   5. 执行运动任务序列
 */
int main(void)
{
    E7RCU_Init();    // 初始化E7系统
    long QRcode = 0; // 二维码识别结果
    long count = 0;  // 计数器
    QRcode = 0;
    count = 0;

    // 显示程序运行状态
    SetDisplayString(1, "程序运行中", 0xFFE0, 0x0000);

    // ========== 设置所有电机旋转方向 ==========
    SetMotorDirection(_M1_, 1); // M1电机正向
    SetMotorDirection(_M2_, 0); // M2电机反向
    SetMotorDirection(_M3_, 1); // M3电机正向
    SetMotorDirection(_M4_, 0); // M4电机反向

    // ========== 初始化机器人参数 ==========
    // 参数: 轮胎直径(6.0), 轮距(26.5cm), 编码器计数(2048)
    settingRobot(6.0, 26.5, 2048);

    // 停止所有电机
    speed_control(0, 0);

    SetFontSize(1); // 设置字体大小

    // 初始化系统（包括UART、GPIO等）
    init();

    // 初始化AI摄像头，设置为Apr标签识别模式
    SetWaitAICamCmd(7, "zm_hxkk_26");
    SetWaitForTime(1);

    // ========== 获取并设置PID参数 ==========
    // 从数据存储器读取PID参数（由于缩放因子100）
    pidW.kp = (double)GetData(9) / 100.0;  // 从地址9读取旋转角度PID-P
    pidW.ki = (double)GetData(10) / 100.0; // 从地址10读取旋转角度PID-I
    pidW.kd = (double)GetData(11) / 100.0; // 从地址11读取旋转角度PID-D

    // ========== 任务执行序列 ==========

    // 第一步: 初始化位置，执行运动学计算(关节角度设置)
    // 参数: 速度(0), 关节2位置(120), 关节3位置(30), 执行时间(1000ms)
    kinematic(0, 120, 30, 1000);

    // 打开爪子
    paw_control(1100, 1000);

    while (1)
    {
        go_bmp(30, 10, 1);      //前进
        SetWaitForTime(0.5);    //延时
        go_bmp(-30, 10, 1);     //后退
        SetWaitForTime(0.5);    //延时

        go_bmp(30, 10, 2);      //左平移
        SetWaitForTime(0.5);    //延时
        go_bmp(-30, 10, 2);     //右平移
        SetWaitForTime(0.5);    //延时

        go_bmp(30, 10, 3);      //左前斜向平移
        SetWaitForTime(0.5);    //延时
        go_bmp(-30, 10, 3);     //右后斜向平移
        SetWaitForTime(0.5);    //延时

        go_bmp(30, 10, 4);      //右前斜向平移
        SetWaitForTime(0.5);    //延时
        go_bmp(-30 , 10, 4);    //左后斜向平移
        SetWaitForTime(0.5);    //延时

        turn_o(45, 30, -30);    //原地顺时针旋转45°
        SetWaitForTime(0.5);    //延时
        turn_o(45, -30, 30);    //原地逆时针旋转45°
        SetWaitForTime(0.5);    // 延时
    }

    while (1)
        ;
}
