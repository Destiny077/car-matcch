/*
 * ========================================
 * 文件名: kinematic.c
 * 描述: 机械臂运动学计算和驱动控制模块
 * 功能: 实现机械臂的逆向运动学计算和舵机驱动
 * 机械臂结构: 4个舵机驱动，L0(基座高度), L1、L2、L3(连杆长度)
 * ========================================
 */

#ifndef _KINEMATIC_
#define _KINEMATIC_
#include <stm32h7xx_hal.h>
#include <math.h>
#include "JMLib.c"

#define pi 3.1415926    // 圆周率

// 舵机PWM值和角度数组
int servo_pwm[6];       // 6个舵机的PWM值
float servo_angle[6];   // 6个舵机的角度值

// 外部机械臂参数
extern float L0, L1, L2, L3;    // 机械臂连杆长度

uint8_t kinematics_move(float x, float y, float z, int time);

/*
 * 函数名: kinematics_analysis
 * 参数:
 *   x, y, z - 目标点在空间中的坐标
 *   Alpha - 连杆L3与水平面的夹角(单位:度)
 * 返回值:
 *   0 = 计算成功
 *   1 = Z坐标超出范围
 *   2 = 目标点超出工作范围
 *   3 = 余弦值超出范围
 *   4 = theta4超出范围
 *   5 = 中间计算超出范围
 *   6 = theta5超出范围
 *   7 = theta3超出范围
 * 描述:
 *   计算机械臂逆向运动学，求解各关节的角度，使机械臂末端到达指定位置。
 *   采用几何方法进行计算。
 */
uint8_t kinematics_analysis(float x, float y, float z, float Alpha)
{
    // ========== 局部变量声明 ==========
    float theta2, theta3, theta4, theta5;    // 各关节角度
    float l0, l1, l2, l3, l12;               // 连杆长度和合成长度
    float l1_2, l2_2, l12_2;                 // 长度的平方
    float aaa, bbb, ccc, zf_flag;            // 中间计算变量
    float x_2, y_2, z_2;                     // 坐标的平方
    float Alpha_rad;                         // 角度转弧度
    
    float m_a, m_b;
    
    // ========== 角度制转弧度制 ==========
    Alpha_rad = Alpha * pi / 180.0;
    
    // ========== 预计算坐标的平方值 ==========
    x_2 = x * x;
    y_2 = y * y;
    z_2 = z * z;
    
    // ========== 获取机械臂参数 ==========
    l0 = L0;    // 基座高度
    l1 = L1;    // 第一段连杆长度
    l2 = L2;    // 第二段连杆长度
    l3 = L3;    // 第三段连杆长度
    
    l1_2 = l1 * l1;
    l2_2 = l2 * l2;
    
    // ========== 计算舵机0的角度(绕Z轴旋转角度) ==========
    theta2 = atan2(x, y) * 180.0 / pi;
    
    // ========== 计算S3关节在YZ平面的坐标 ==========
    // 先计算XY平面的距离
    y = sqrt(x_2 + y_2);
    // 从目标点减去L3在Y轴上的投影（考虑Alpha角）
    y = y - l3 * cos(Alpha_rad);
    // 计算Z坐标（从基座开始，需要减去基座高度L0和L3在Z轴上的投影）
    z = z - l0 - l3 * sin(Alpha_rad);
    
    y_2 = y * y;
    z_2 = z * z;
    
    // ========== 计算S3关节到目标的直线距离 ==========
    l12 = sqrt(y_2 + z_2);
    l12_2 = l12 * l12;
    
    // ========== 边界检查1: Z坐标下限 ==========
    if(z < -l0)
    {
        return 1;   // Z坐标超出下限
    }
    
    // ========== 边界检查2: 工作范围 ==========
    // S3关节到目标的距离不能超过(L1+L2)
    if(l12 > (l1 + l2))
    {
        return 2;   // 目标点超出工作范围
    }
    
    // ========== 逆向运动学计算 - 使用余弦定理 ==========
    // 计算角度ccc: S3关节到目标点在YZ平面的角度
    ccc = acos(y / l12);
    
    // 计算角度bbb的余弦值: 三角形(L1, L12, L2)中L1与L12之间的角度
    bbb = (l12_2 + l1_2 - l2_2) / (2 * l1 * l12);
    
    // ========== 数值检查: 余弦值范围检查 ==========
    if(bbb > 1 || bbb < -1)
    {
        return 5;   // 无解，目标点无法到达
    }
    
    // ========== 确定Z方向标志(上下) ==========
    if (z < 0)
    {
        zf_flag = -1;   // Z坐标为负，标记为下方
    } else
    {
        zf_flag = 1;    // Z坐标为正，标记为上方
    }
    
    // ========== 计算舵机2(theta5)的角度 ==========
    theta5 = ccc * zf_flag + acos(bbb);
    theta5 = theta5 * 180.0 / pi;    // 转换为度数
    
    // ========== 边界检查3: theta5范围 ==========
    if(theta5 > 180.0 || theta5 < 0.0)
    {
        return 6;   // theta5超出范围
    }
    
    // ========== 计算舵机3(theta4)的角度 ==========
    // 使用余弦定理计算L2与L12之间的夹角
    aaa = -(l12_2 - l1_2 - l2_2) / (2 * l1 * l2);
    
    // ========== 数值检查: 余弦值范围检查 ==========
    if (aaa > 1 || aaa < -1)
    {
        return 3;   // 无解
    }
    
    theta4 = acos(aaa);
    theta4 = 180.0 - theta4 * 180.0 / pi;    // 转换并调整
    
    // ========== 边界检查4: theta4范围 ==========
    if (theta4 > 135.0 || theta4 < -135.0)
    {
        return 4;   // theta4超出范围
    }
    
    // ========== 计算舵机1(theta3)的角度 ==========
    theta3 = Alpha - theta5 + theta4;
    
    // ========== 边界检查5: theta3范围 ==========
    if(theta3 > 90.0 || theta3 < -90.0)
    {
        return 7;   // theta3超出范围
    }
    
    // ========== 保存计算结果到数组 ==========
    servo_angle[0] = theta2;           // 舵机0: 绕Z轴旋转角度
    servo_angle[1] = theta5 - 90;      // 舵机1: 调整后的theta5
    servo_angle[2] = theta4;           // 舵机2: theta4
    servo_angle[3] = theta3;           // 舵机3: theta3
    
    // ========== 将角度转换为舵机PWM值 ==========
    // 舵机通常接收1500μs为中位，±270度对应±2000μs范围
    servo_pwm[0] = (int)(1500 - 2000.0 * servo_angle[0] / 270.0);
    servo_pwm[1] = (int)(1500 + 2000.0 * servo_angle[1] / 270.0);
    servo_pwm[2] = (int)(1500 + 2000.0 * servo_angle[2] / 270.0);
    servo_pwm[3] = (int)(1500 + 2000.0 * servo_angle[3] / 270.0);
    
    return 0;   // 计算成功
}

/*
 * 函数名: kinematics_move
 * 参数:
 *   x, y, z - 目标点在空间中的坐标
 *   time - 舵机执行时间 (单位: 毫秒)
 * 返回值:
 *   0 = 移动成功
 *   2 = 目标点无可行解
 * 描述:
 *   通过遍历L3的不同夹角(Alpha从0°到-130°)，
 *   寻找能到达目标点的所有可行解。
 *   选择使L3与水平夹角最大的解作为最优解（增加稳定性），
 *   然后驱动舵机执行运动。
 */
uint8_t kinematics_move(float x, float y, float z, int time)
{
    int i, j, min = 0, flag = 0;
    char status;

    // 遍历L3夹角，寻找最佳解
    flag = 0;
    // 从0°开始，每次递减2°，直到-130°
    for (i = 0; i >= -130; i -= 2)
    {
        // 计算这个夹角下是否有可行解
        status = kinematics_analysis(x, y, z, i);
        
        if (status == 0)  // 如果计算成功
        {
            if (i < min)
                min = i;  // 记录最小(最大负值)的可行角度
            flag = 1;     // 标记找到了至少一个可行解
        }
    }

    // ========== 执行最优解 ==========
    if (flag)
    {
        // 用最大负值(L3与水平最大夹角)的角度作为最优解
        kinematics_analysis(x, y, z, min);
        
        // 驱动舵机执行运动
        arm_drive(time);
        
        return 0;   // 成功
    }

    return 2;   // 找不到可行解
}

#include "HardwareInfo.c"

/*
 * 函数名: kinematic
 * 参数:
 *   x - 目标点X坐标
 *   y - 目标点Y坐标
 *   z - 目标点Z坐标
 *   time - 执行时间 (单位: 毫秒)
 * 描述:
 *   上层接口函数，调用运动学计算并等待执行完成。
 */
void kinematic(double x, double y, double z, int time)
{
    // 调用运动学计算和舵机驱动函数
    kinematics_move(x, y, z, time);
    
    // 等待运动完成（转换毫秒为秒）
    SetWaitForTime((float)time / 1000);
}
#endif

