#include "pid_cal.h"
#include "struct_typedef.h"
#include "pid.h"
#include "CAN_receive.h"
#include "math.h"
#include "power_control.h"



PidTypeDef M3508_speedPID[4];//麦轮速度环 ×4
PidTypeDef Pid_Vw_A;//角度环
PidTypeDef Pid_Vw_V;//速度环
extern PidTypeDef buffer_pid;//超电缓冲能量 PID
//*********** pid parameters *****************//
static const fp32 M3508_speedPID_parameters[3]   = {3.5, 0, 0};
static const fp32 Chassis_Follow_A_PID_parameters[3] = {0.2, 0, 0};
static const fp32 Chassis_Follow_V_PID_parameters[3] = {1, 0, 0};//空的
static const fp32 Chassis_Spin_V_PID_parameters[3]   = {1, 0, 0};
extern fp32 BufferPID_parameters[3];
//*************************************//

//麦轮速度环 ×4 PID
void M3508_Travel_init(void)
{
    PID_init(&M3508_speedPID[0], 0, M3508_speedPID_parameters, WHEEL_SPEED_MAX_OUT, WHEEL_SPEED_MAX_IOUT);
    PID_init(&M3508_speedPID[1], 0, M3508_speedPID_parameters, WHEEL_SPEED_MAX_OUT, WHEEL_SPEED_MAX_IOUT);
    PID_init(&M3508_speedPID[2], 0, M3508_speedPID_parameters, WHEEL_SPEED_MAX_OUT, WHEEL_SPEED_MAX_IOUT);
    PID_init(&M3508_speedPID[3], 0, M3508_speedPID_parameters, WHEEL_SPEED_MAX_OUT, WHEEL_SPEED_MAX_IOUT);
}


//超电缓冲能量 PID
void Chassis_BufferPower_init(void)
{
    PID_init(&buffer_pid, 0, BufferPID_parameters, 60, 0);
}


//跟随双环（角度环 + 速度环）
void Chassis_Follow_init(void)
{
    PID_init(&Pid_Vw_A, 0, Chassis_Follow_A_PID_parameters, 20, 20);//角度环
    PID_init(&Pid_Vw_V, 0, Chassis_Follow_V_PID_parameters, 5000, 800);//速度环
}



//小陀螺pid结构体
PidTypeDef Pid_Spin_V;

//小陀螺结构体初始化
void Chassis_Spin_init(void)
{
    PID_init(&Pid_Spin_V, 0, Chassis_Spin_V_PID_parameters, 16000, 2000);
}


//下电函数（其实没用过当作工具库存起来）
void CHASSIS_ZEFR_PID(void)
{
    CAN_cmd_chassis_travel(0, 0, 0, 0);
}
