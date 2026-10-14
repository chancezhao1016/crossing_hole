#ifndef _POWER_CONTROL_H
#define _POWER_CONTROL_H

#include "struct_typedef.h"
#include "pid.h"

/* 功率分配系数（CHASSIS_TASK.c 小陀螺模式使用） */
extern fp32 k3508;
extern fp32 k6020;

/* 缓冲功率 PID（Chassis_BufferPower_init 使用） */
extern PidTypeDef buffer_pid;
extern fp32 BufferPID_parameters[3];

#endif
