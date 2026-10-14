#ifndef MECANUM_H
#define MECANUM_H

/* 麦轮编号（对应 3508 CAN ID 0x201 ~ 0x204，与旧项目 mecanum.c 一致） */
#define LEFT_UP_MECANUM_WHEEL    0
#define RIGHT_UP_MECANUM_WHEEL   1
#define LEFT_DOWN_MECANUM_WHEEL  2
#define RIGHT_DOWN_MECANUM_WHEEL 3

#define PI 3.1415926535f

/* 麦轮几何尺寸（沿用旧项目 robot_config.h，硬件一致） */
#define MECANUM_WIDTH         0.15f /* 麦轮横向半轴距 m */
#define MECANUM_LENGTH        0.20f /* 麦轮纵向半轴距 m */
#define MECANUM_WHEEL_RADIUS  0.06f /* 麦轮半径 m */

/* 速度单位换算（沿用旧项目 mecanum.h） */
#define MECANUM_SPEED_TO_DEGEREE_S (180.0f / MECANUM_WHEEL_RADIUS / PI) /* m/s -> 轮 deg/s */
#define MECANUM_DEGEREE_S_TO_MS    (PI * MECANUM_WHEEL_RADIUS / 180.0f) /* 轮 deg/s -> m/s */

/* 3508 电机反馈换算（沿用旧项目 M3508.h） */
#define M3508_SPEED_RATIO        6.0f  /* rpm -> deg/s */
#define M3508_REDUCTION_RATIO    15.0f /* 轮减速比 */
#define M3508_RPM_TO_WHEEL_DEG_S (M3508_SPEED_RATIO / M3508_REDUCTION_RATIO) /* rpm -> 轮 deg/s (=0.4) */

/* 逆运动学：底盘目标速度(vx[m/s], vy[m/s], vw[rad/s]) -> 4 轮目标转速(deg/s) */
void mecanum_inv_kinematics(float vx, float vy, float vw, float wheel_deg_s[4]);

/* 正运动学：4 轮转速(deg/s) -> 底盘速度(vx,vy,vw) */
void mecanum_pos_kinematics(const float wheel_deg_s[4], float *vx, float *vy, float *vw);

#endif
