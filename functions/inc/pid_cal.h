#ifndef _PID_CAL_H
#define _PID_CAL_H


#include "pid.h"
#include "CAN_receive.h"
#include "fuzzy_pid.h"

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

typedef struct
{
    /* ===== 跟随状态 ===== */
    float yaw_angle;            // 云台 yaw 角 0~360
    float error_angle;          // 底盘/云台夹角
    float yaw_v;                // 云台下发的 yaw 角速度 rad/s

    /* ===== 各环基础增益（debug 里可改）===== */
    fp32 wheel_base[3];         // 轮速度环 Kp Ki Kd
    fp32 angle_base[3];         // 角度环 Kp Ki Kd
    fp32 vel_base[3];           // 速度环（阻尼）Kp Ki Kd
    fp32 spin_base[3];          // 小陀螺速度环 Kp Ki Kd
    fp32 fuzzy_ratio[3];        // 模糊PID Kp/Ki/Kd 增量缩放
    fp32 fuzzy_estep;           // 模糊误差步长(度)
    fp32 fuzzy_ecstep;          // 模糊误差率步长(度/s)

    /* ===== PID 环 ===== */
    PidTypeDef wheel_speed[4];  // 轮速度环 ×4（3508）
    PidTypeDef vw_angle;        // 跟随角度环
    PidTypeDef vw_vel;          // 跟随速度环（阻尼）
    PidTypeDef spin;            // 小陀螺速度环

    /* ===== 模糊PID ===== */
    FuzzyRule_t fuzzy;
} Chassis_Ctrl_t;

extern Chassis_Ctrl_t chassis_ctrl;
#endif
