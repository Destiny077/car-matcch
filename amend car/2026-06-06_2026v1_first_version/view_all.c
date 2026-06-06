/*
 * ========================================
 * 文件名: view_all.c
 * 描述: 触摸屏UI控制和调试界面模块
 * 功能: 实现机械臂的手动控制界面和实时参数显示
 * ========================================
 */

#ifndef _VIEW_ALL_
#define _VIEW_ALL_
#include <stm32h7xx_hal.h>
#include <SetLCDRectangle.h>
#include <SetLCDBack.h>
#include <SetLCDSolidCircle.h>
#include <GetTouchScreenX.h>
#include <GetTouchScreenY.h>
#include <GetTouchScreen.h>
#include <SetLCDFilledRectangle.h>
#include <SetData.h>
#include <GetData.h>
#include <SetDisplayString.h>
#include <GetRightButton.h>
#include <GetLeftButton.h>
#include <SetWaitForTime.h>
#include <SetInBeep.h>
#include <GetSysTime.h>
#include "init.c"
#include "fun.c"
#include "JMLib.c"
#include "hardware_config.c"
#include <jmkernel.h>

// ========== 外部变量声明 ==========
extern int arm_x, arm_y, arm_z;     // 机械臂当前位置(X, Y, Z坐标)

/*
 * 函数名: SetBeep
 * 描述: 产生一次短鸣音，用于触摸反馈
 */
void SetBeep()
{
    SetInBeep(ON);
    SetWaitForTime(0.05);       // 蜂鸣50ms
    SetInBeep(OFF);
    SetWaitForTime(0.05);       // 停止50ms
}

/*
 * 函数名: touch1
 * 参数:
 *   x1, x2 - X坐标范围[x1, x2]
 *   y1, y2 - Y坐标范围[y1, y2]
 * 返回值: 
 *   1 = 触摸点在指定区域内
 *   0 = 触摸点不在指定区域内
 * 描述:
 *   检测触摸点是否落在指定的矩形区域内。
 *   使用防抖处理，确保检测的稳定性。
 */
char touch1(int x1, int y1, int x2, int y2)
{
    int vx = 0, vy = 0;
    
    // 获取触摸点坐标
    vx = GetTouchScreenX();
    vy = GetTouchScreenY();
    
    // 检查触摸点是否在矩形区域内
    if(((x1 < vx) && (vx < x2)) && ((y1 < vy) && (vy < y2)))
    {
        // 防抖：等待10ms后再确认
        SetWaitForTime(0.01);
        if(GetTouchScreen() == 1)
        {
            return 1;   // 确认触摸
        }
        else 
            return 0;   // 触摸已释放
    }
    else
        return 0;       // 不在区域内
}

/*
 * 函数名: control_interface
 * 描述:
 *   绘制机械臂手动控制界面。
 *   包括:
 *   - 方向控制按钮(前、后、左、右)
 *   - 垂直控制按钮(上、下)
 *   - 更新显示按钮
 *   - 实时显示机械臂的X、Y、Z坐标
 */
void control_interface(void)
{
    // ========== 绘制方向控制按钮（黄色区域） ==========
    SetLCDFilledRectangle(50, 50, 100, 100, YELLOW);      // 前
    SetLCDFilledRectangle(0, 100, 50, 150, YELLOW);       // 左
    SetLCDFilledRectangle(100, 100, 150, 150, YELLOW);    // 右
    SetLCDFilledRectangle(50, 150, 100, 200, YELLOW);     // 后
    
    // ========== 绘制垂直控制按钮（绿色区域） ==========
    SetLCDFilledRectangle(180, 50, 230, 100, GREEN);      // 上
    SetLCDFilledRectangle(180, 150, 230, 200, GREEN);     // 下
    
    // ========== 绘制更新按钮（蓝色区域） ==========
    SetLCDFilledRectangle(20, 240, 100, 300, BLUE);       // 更新按钮
    
    // ========== 标注方向按钮 ==========
    LCD_String(50 + 17, 50 + 17, "前", BLACK, YELLOW, 0);
    LCD_String(50 + 17, 150 + 17, "后", BLACK, YELLOW, 0);
    LCD_String(0 + 17, 100 + 17, "左", BLACK, YELLOW, 0);
    LCD_String(100 + 17, 100 + 17, "右", BLACK, YELLOW, 0);
    
    // ========== 标注垂直按钮 ==========
    LCD_String(180 + 17, 50 + 17, "上", BLACK, GREEN, 0);
    LCD_String(180 + 17, 150 + 17, "下", BLACK, GREEN, 0);
    
    // ========== 标注其他按钮 ==========
    LCD_String(20 + 24, 240 + 22, "更新", WHITE, BLUE, 0);
    LCD_String(176, 300, "右键开始", WHITE, BLACK, 0);
    
    // ========== 显示坐标标签 ==========
    LCD_String(0, 20, "X:", WHITE, BLACK, 0);
    LCD_String(80, 20, "Y:", WHITE, BLACK, 0);
    LCD_String(160, 20, "Z:", WHITE, BLACK, 0);
    
    // ========== 显示实时坐标值 ==========
    LCD_DisplayFloatNum(16, 20, arm_x, WHITE, BLACK, 0X0401);
    LCD_DisplayFloatNum(96, 20, arm_y, WHITE, BLACK, 0X0401);
    LCD_DisplayFloatNum(176, 20, arm_z, WHITE, BLACK, 0X0401);
}

/*
 * 函数名: interface_touch
 * 返回值: 1 = 有按键被触摸，0 = 无
 * 描述:
 *   处理触摸屏按键按下事件，实现机械臂的手动控制。
 *   根据触摸位置调整机械臂的X、Y、Z坐标，增量为10。
 */
char interface_touch(void)
{
    // ========== 前进 ==========
    if(touch1(50, 50, 100, 100))
    {
        arm_y += 10;                            // Y增加
        LCD_DisplayFloatNum(96, 20, arm_y, WHITE, BLACK, 0X0401);
        SetBeep();                              // 触摸反馈音
        return 1;
    }
    // ========== 后退 ==========
    else if(touch1(50, 150, 100, 200))
    {
        arm_y -= 10;                            // Y减少
        LCD_DisplayFloatNum(96, 20, arm_y, WHITE, BLACK, 0X0401);
        SetBeep();
        return 1;
    }
    // ========== 左移 ==========
    else if(touch1(0, 100, 50, 150))
    {
        arm_x -= 10;                            // X减少
        LCD_DisplayFloatNum(16, 20, arm_x, WHITE, BLACK, 0X0401);
        SetBeep();
        return 1;
    }
    // ========== 右移 ==========
    else if(touch1(100, 100, 150, 150))
    {
        arm_x += 10;                            // X增加
        LCD_DisplayFloatNum(16, 20, arm_x, WHITE, BLACK, 0X0401);
        SetBeep();
        return 1;
    }
    // ========== 上升 ==========
    else if(touch1(180, 50, 230, 100))
    {
        arm_z += 10;                            // Z增加
        LCD_DisplayFloatNum(176, 20, arm_z, WHITE, BLACK, 0X0401);
        SetBeep();
        return 1;
    }
    // ========== 下降 ==========
    else if(touch1(180, 150, 230, 200))
    {
        arm_z -= 10;                            // Z减少
        LCD_DisplayFloatNum(176, 20, arm_z, WHITE, BLACK, 0X0401);
        SetBeep();
        return 1;
    }
    // ========== 更新按钮 ==========
    else if(touch1(20, 240, 100, 300))
    {
        SetBeep();
        return 2;   // 返回2表示更新
    }
    // ========== 右键按钮 - 复位 ==========
    else if(GetRightButton() == 1)
    {
        // 重置机械臂到初始位置
        arm_x = ROBOT_ARM_INIT_X;
        arm_y = ROBOT_ARM_INIT_Y;
        arm_z = ROBOT_ARM_INIT_Z;
        
        // 更新显示
        LCD_DisplayFloatNum(16, 20, arm_x, WHITE, BLACK, 0X0401);
        LCD_DisplayFloatNum(96, 20, arm_y, WHITE, BLACK, 0X0401);
        LCD_DisplayFloatNum(176, 20, arm_z, WHITE, BLACK, 0X0401);
        
        SetBeep();
        return 3;   // 返回3表示复位
    }
    // ========== 无触摸 ==========
    else 
        return 0;
}

#include "HardwareInfo.c"

/*
 * 函数名: view_all
 * 描述: 占位函数（当前为空）
 */
void view_all()
{
    // 该函数为空
}
#endif

