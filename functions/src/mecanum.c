#include "mecanum.h"

#define SQRT_2        1.41421356f
#define DIVIDE_SQRT_2 0.70710678f

/**
 * @brief 逆运动学：底盘目标速度 -> 4 轮目标转速
 * @note  与旧项目 mecanum.c 一致：vx/vy[m/s]、vw[rad/s] -> 轮转速[deg/s]
 */
void mecanum_inv_kinematics(float vx, float vy, float vw, float wheel_deg_s[4])
{
    float wl = MECANUM_WIDTH + MECANUM_LENGTH;

    wheel_deg_s[LEFT_UP_MECANUM_WHEEL]    = ( vx * SQRT_2 + vy * SQRT_2 - vw * wl) * MECANUM_SPEED_TO_DEGEREE_S;
    wheel_deg_s[RIGHT_UP_MECANUM_WHEEL]   = ( vx * SQRT_2 - vy * SQRT_2 - vw * wl) * MECANUM_SPEED_TO_DEGEREE_S;
    wheel_deg_s[LEFT_DOWN_MECANUM_WHEEL]  = (-vx * SQRT_2 + vy * SQRT_2 - vw * wl) * MECANUM_SPEED_TO_DEGEREE_S;
    wheel_deg_s[RIGHT_DOWN_MECANUM_WHEEL] = (-vx * SQRT_2 - vy * SQRT_2 - vw * wl) * MECANUM_SPEED_TO_DEGEREE_S;
}

/**
 * @brief 正运动学：4 轮转速 -> 底盘速度
 * @note  与旧项目 mecanum.c 一致：轮转速[deg/s] -> vx/vy[m/s]、vw[rad/s]
 */
void mecanum_pos_kinematics(const float wheel_deg_s[4], float *vx, float *vy, float *vw)
{
    float wl = MECANUM_WIDTH + MECANUM_LENGTH;

    *vx = ( wheel_deg_s[LEFT_UP_MECANUM_WHEEL] + wheel_deg_s[RIGHT_UP_MECANUM_WHEEL]
           - wheel_deg_s[LEFT_DOWN_MECANUM_WHEEL] - wheel_deg_s[RIGHT_DOWN_MECANUM_WHEEL])
          / 4.0f * DIVIDE_SQRT_2 * MECANUM_DEGEREE_S_TO_MS;

    *vy = ( wheel_deg_s[LEFT_UP_MECANUM_WHEEL] - wheel_deg_s[RIGHT_UP_MECANUM_WHEEL]
           + wheel_deg_s[LEFT_DOWN_MECANUM_WHEEL] - wheel_deg_s[RIGHT_DOWN_MECANUM_WHEEL])
          / 4.0f * DIVIDE_SQRT_2 * MECANUM_DEGEREE_S_TO_MS;

    *vw = (-wheel_deg_s[LEFT_UP_MECANUM_WHEEL] - wheel_deg_s[RIGHT_UP_MECANUM_WHEEL]
           - wheel_deg_s[LEFT_DOWN_MECANUM_WHEEL] - wheel_deg_s[RIGHT_DOWN_MECANUM_WHEEL])
          / 4.0f / wl * MECANUM_DEGEREE_S_TO_MS;
}
