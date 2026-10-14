#ifndef _PID_CAL_H
#define _PID_CAL_H


#include "pid.h"
#include "CAN_receive.h"

//*************** PID 参数定义 *************//
//底盘速度环参数
#define WHEEL_SPEED_KP              1.91f
#define WHEEL_SPEED_KI              0.014f
#define WHEEL_SPEED1_KI             0.0f
#define WHEEL_SPEED_KD              2.0f
#define WHEEL_SPEED_MAX_OUT         16000.0f
#define WHEEL_SPEED_MAX_IOUT        2000.0f
//底盘电流环参数
#define WHEEL_GURRENT_KP            0.155f
#define WHEEL_GURRENT_KI            0.1f
#define WHEEL_GURRENT_KD            0.038555f
#define WHEEL_GURRENT_MAX_OUT       16000.0f
#define WHEEL_GURRENT_MAX_IOUT      4000.0f
//底盘跟随参数
#define WHEEL_FOLLOW_KP             0.3f
#define WHEEL_FOLLOW_KI             0.01f
#define WHEEL_FOLLOW_KD             0.0f
#define WHEEL_FOLLOW_MAX_OUT        4000.0f
#define WHEEL_FOLLOW_MAX_IOUT       500.0f



//***********************************//
void M3508_Travel_init(void);//pid初始化
void CHASSIS_ZEFR_PID(void);//底盘无力
void CHASSIS_LIMIT_PID(void);//底盘锁死
void CHASSIS_NORM_PID(void);//底盘控制
void Chassis_BufferPower_init(void);//pid初始化
void Chassis_Follow_init(void);
void Chassis_Spin_init(void);
//*************** PID 结构体定义 *************//
//pid	chassis

extern 	PidTypeDef M3508_speedPID[4];
#endif
