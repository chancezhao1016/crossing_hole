#include "pid_cal.h"
#include "struct_typedef.h"
#include "pid.h"
#include "CAN_receive.h"
#include "math.h"
#include "power_control.h"


/* 底盘控制初始化：增益 / 模糊参数，可在调试时修改 */
Chassis_Ctrl_t chassis_ctrl = {
    .wheel_base   = {3.5f, 0.0f, 0.0f},  // wheel loop Kp Ki Kd
    .angle_base   = {0.0f, 0.0f, 0.0f},  // 角度环 Kp Ki Kd
    .vel_base     = {0.3f, 0.0f, 0.0f},  // velocity loop Kp Ki Kd
    .spin_base    = {1.0f, 0.0f, 0.0f},  // spin loop Kp Ki Kd
    .fuzzy_ratio  = {0.3f, 0.01f, 0.1f}, // 模糊 Kp Ki Kd 比例
    .fuzzy_estep  = 30.0f,               // 模糊误差步长（度）
    .fuzzy_ecstep = 100.0f,              // 模糊误差变化率步长（度/秒）
};

extern PidTypeDef buffer_pid;//超电缓冲能量 PID
extern fp32 BufferPID_parameters[3];

//麦轮速度环 ×4 PID
void M3508_Travel_init(void)
{
    PID_init(&chassis_ctrl.wheel_speed[0], 0, chassis_ctrl.wheel_base, WHEEL_SPEED_MAX_OUT, WHEEL_SPEED_MAX_IOUT);
    PID_init(&chassis_ctrl.wheel_speed[1], 0, chassis_ctrl.wheel_base, WHEEL_SPEED_MAX_OUT, WHEEL_SPEED_MAX_IOUT);
    PID_init(&chassis_ctrl.wheel_speed[2], 0, chassis_ctrl.wheel_base, WHEEL_SPEED_MAX_OUT, WHEEL_SPEED_MAX_IOUT);
    PID_init(&chassis_ctrl.wheel_speed[3], 0, chassis_ctrl.wheel_base, WHEEL_SPEED_MAX_OUT, WHEEL_SPEED_MAX_IOUT);
}


//超电缓冲能量 PID
void Chassis_BufferPower_init(void)
{
    PID_init(&buffer_pid, 0, BufferPID_parameters, 60, 0);
}


//跟随双环（角度环 + 速度环）
void Chassis_Follow_init(void)
{
    PID_init(&chassis_ctrl.vw_angle, 0, chassis_ctrl.angle_base, 20, 20);//角度环
    PID_init(&chassis_ctrl.vw_vel, 0, chassis_ctrl.vel_base, 5000, 800);//速度环
}



//小陀螺pid结构体

//小陀螺结构体初始化
void Chassis_Spin_init(void)
{
    PID_init(&chassis_ctrl.spin, 0, chassis_ctrl.spin_base, 16000, 2000);
}


//下电函数（其实没用过当作工具库存起来）
void CHASSIS_ZEFR_PID(void)
{
    CAN_cmd_chassis_travel(0, 0, 0, 0);
}
