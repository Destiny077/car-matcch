/*
 * ========================================
 * 文件名: go_to_tag.c
 * 描述: 基于视觉的Apr标签导航模块
 * 功能: 使用AI摄像头检测Apr标签，通过PID控制实现自动导航到目标标签
 * ========================================
 */

#ifndef _GO_TO_TAG_
#define _GO_TO_TAG_
#include <stm32h7xx_hal.h>
#include <GetSysTime.h>
#include <GetAICam.h>
#include "speed_control.c"
#include <SetMotorConstSpeed.h>
#include <SetMotor.h>
#include "hardware_config.c"

// ========== 宏定义 ==========
// 数值限幅宏: 限制value在[min, max]范围内
#define CLAMP(x, min, max) ((x) > (max) ? (max) : ((x) < (min) ? (min) : (x)))

// ========== PID积分限幅值 ==========
double max_I = 100;
/*
 * 函数名: UpdatePID
 * 参数:
 *   pid - PID控制器结构体指针
 *   error - 当前误差值
 * 返回值: 计算得到的PID输出值
 * 描述:
 *   执行PID计算，包括：
 *   - 比例项(P): kp * error
 *   - 积分项(I): ki * integral（带限幅防止积分饱和）
 *   - 微分项(D): kd * (error - prev_error)
 *   输出 = P + I + D
 */
double UpdatePID(PID_Controller *pid, double error)
{
    // 积分累加
    pid->integral += error;

    // 积分限幅，防止积分过大导致超调
    if (pid->integral > max_I)
        pid->integral = max_I;
    if (pid->integral < -max_I)
        pid->integral = -max_I;

    // 微分项 = 误差变化量
    double derivative = error - pid->prev_error;

    // PID输出计算
    double output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);

    // 保存本次误差供下次计算使用
    pid->prev_error = error;

    return output;
}

#include "HardwareInfo.c"

/*
 * 函数名: go_to_tag
 * 参数:
 *   targetX - 目标Apr标签的X坐标
 *   targetY - 目标Apr标签的Y坐标
 *   targetA - 目标方向角度 (单位: 度)
 *   timeout_ms - 超时时间 (单位: 毫秒)
 * 描述:
 *   使用AI摄像头获取Apr标签位置，通过PID控制器驱动电机
 *   实现自动导航到目标标签，并对齐方向。
 *
 *   控制策略:
 *   - X轴(左右): 两段速度控制（远距离快，近距离慢）
 *   - Y轴(前后): 两段速度控制（远距离快，近距离慢）
 *   - W轴(旋转): PID闭环控制，精确对齐方向
 */
char go_to_tag(double targetX, double targetY, double targetA, unsigned long timeout_ms, char catch_type)
{
    // ========== 获取外部PID控制器 ==========
    extern PID_Controller pidX, pidY, pidW;

    // ========== 获取开始时间用于超时计时 ==========
    long startTime = GetSysTime();
    char reached = 0;

    // ========== 控制周期常数 ==========
    const int LOOP_PERIOD = 20; // 设定 20ms 的控制周期
    long last_loop_time = GetSysTime();

    pidW.prev_error = 0;
    pidW.integral = 0;

    // ========== 导航控制循环 ==========
    while (1)
    {
        if ((GetSysTime() - startTime) > timeout_ms)
            break;

        // ========== 1. 获取当前传感器数据（从AI摄像头) ==========
        if (GetAICam(ROBOT_AI_CAM_PORT, 1, 1) == 1 || catch_type == 1)
        {
            double curX = GetAICam(ROBOT_AI_CAM_PORT, 1, 2); // 获取X坐标
            double curY = GetAICam(ROBOT_AI_CAM_PORT, 1, 3); // 获取Y坐标
            double curA = GetAICam(ROBOT_AI_CAM_PORT, 1, 5); // 获取当前方向角度

            // ========== 检查标签是否丢失 ==========
            // 如果X、Y、A都为0，表示未检测到标签，保持小速度运动
            if (fabs(curX) < 0.0001 && fabs(curY) < 0.0001 && fabs(curA) < 0.0001)
            {
                // 保持极小的前进速度，防止停滞
                SetMotorConstSpeed(_M1_, ROBOT_TAG_LOST_CRAWL_SPEED);
                SetMotorConstSpeed(_M2_, ROBOT_TAG_LOST_CRAWL_SPEED);
                SetMotorConstSpeed(_M3_, ROBOT_TAG_LOST_CRAWL_SPEED);
                SetMotorConstSpeed(_M4_, ROBOT_TAG_LOST_CRAWL_SPEED);
                continue; // 跳到下一次循环
            }

            // ========== 2. 计算误差值 ==========
            double errX = -(targetX - curX); // X轴误差，调整极性
            double errY = -(targetY - curY); // Y轴误差，调整极性

            // ========== 3. 计算角度误差（特殊处理0/360跳转) ==========
            // 确保角度差在[-180, 180]范围内
            double errA = targetA - curA;
            while (errA > 180)
                errA -= 360;
            while (errA < -180)
                errA += 360;

            // ========== 5. 旋转(W)轴: PID控制 ==========
            // 使用PID使机器人对齐目标方向
            double vw = 0;

            // ========== 6. X轴(左右): 两段速度策略 ==========
            const double DIST_THRESHOLD = 30.0; // 远近切换阈值
            const double SPEED_HIGH = 20.0;     // 远距离快移速度
            const double SPEED_LOW = 3.0;       // 近距离微调速度

            // 逻辑计算：X轴两段速
            double vx = 0;
            if (fabs(errX) < 2.0)
            {
                vx = 0; // 到达目标
            }
            else if (fabs(errX) > DIST_THRESHOLD)
            {
                vx = (errX > 0) ? SPEED_HIGH : -SPEED_HIGH; // 远距离快跑
            }
            else
            {
                vx = (errX > 0) ? SPEED_LOW : -SPEED_LOW; // 近距离慢磨
            }

            // ========== 7. Y轴(前后): 两段速度策略 ==========
            double vy = 0;
            if (fabs(errY) < 2.0)
            {
                vy = 0;
            }
            else if (fabs(errY) > DIST_THRESHOLD)
            {
                // 注意: Y轴方向是反向的(负Y表示前进)
                vy = (errY > 0) ? -SPEED_HIGH : SPEED_HIGH;
            }
            else
            {
                vy = (errY > 0) ? -SPEED_LOW : SPEED_LOW;
            }

            // ========== 8. 重新计算旋转速度（微调） ==========
            // 用PID精确控制旋转方向
            vw = UpdatePID(&pidW, errA);
            vw = CLAMP(vw, -20, 20); // 旋转速度限幅

            if (catch_type == 1)
                vw = 0;

            // ========== 9. 麦克纳姆轮分解(全向轮) ==========
            // 将目标速度(vx, vy, vw)分解为4个麦克纳姆轮的速度
            // 麦克纳姆轮运动学:
            //   M1(前左) = vx + vy + vw
            //   M2(前右) = vx - vy - vw
            //   M3(后左) = vx - vy + vw
            //   M4(后右) = vx + vy - vw
            double m1_sp = vy + vx + vw;
            double m2_sp = vy - vx - vw;
            double m3_sp = vy - vx + vw;
            double m4_sp = vy + vx - vw;

            // 保存到数组便于后续处理
            double motors[4] = {m1_sp, m2_sp, m3_sp, m4_sp};

            // ========== 10. 实时显示当前位置和角度 ==========
            // 用于调试和监控导航过程
            SetDisplayVar(1, curX, YELLOW, BLACK);
            SetDisplayVar(3, curY, YELLOW, BLACK);
            SetDisplayVar(5, curA, YELLOW, BLACK);

            // ========== 11. 发送指令到电机 ==========
            // 将分解后的速度值发送到四个电机
            SetMotorConstSpeed(_M1_, (int)motors[0]);
            SetMotorConstSpeed(_M2_, (int)motors[1]);
            SetMotorConstSpeed(_M3_, (int)motors[2]);
            SetMotorConstSpeed(_M4_, (int)motors[3]);

            // ========== 12. 到达判定 ==========
            // 当X、Y误差都小于3，角度误差小于1°时，认为已到达
            if (fabs(errX) <= 3 && fabs(errY) <= 3 && fabs(errA) < 1)
            {
                reached = 1;
                break;
            }
        }

        // ========== 13. 控制周期管理 ==========
        // 保持一定的控制频率，确保稳定性
        while ((GetSysTime() - last_loop_time) < LOOP_PERIOD)
        {
            // 等待直到达到控制周期时间
        }
        last_loop_time = GetSysTime();
    }
    // ========== 14. 导航完成，停止所有电机 ==========
    speed_control(0, 0);
    return reached;
}
#endif
