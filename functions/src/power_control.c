#include "power_control.h"

/* 功率分配系数（CHASSIS_TASK.c 小陀螺模式使用） */
fp32 k3508 = 0.55f;
fp32 k6020 = 0.45f;

/* 缓冲功率 PID（Chassis_BufferPower_init 使用） */
PidTypeDef buffer_pid;
fp32 BufferPID_parameters[3] = {3.5, 0, 1};
