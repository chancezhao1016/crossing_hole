#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H


#include "struct_typedef.h"
#include "pid.h"

#define DegreesToRadians 0.01745329251f
#define RadiansToDegrees 57.295779513f


#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

typedef enum
{
  LEFT,
	RIGHT,
} CHASSISMOTOR_ENUM;

/* 底盘模式（原 COMMON_TASK.h，COMMON_TASK 已删除，此处保留类型供 CHASSIS_TASK 使用） */
typedef enum
{
    Chassis_Tune,
    Chassis_Follow,
    Chassis_Spin,
    Chassis_OFF,
    Chassis_Mode_Num
} Chassis_Mode_Enum;

/* 云台跟随零点 / 云台电机方向 */
#define GIMBAL_FOLLOW_ZERO 193.809814f
#define GIMBAL_MOTOR_SIGN -1

/* 速度上限（沿用旧工程 robot_config.h 速度-功率拟合低点） */
#define CHASSIS_TRANSLATE_SPEED_MAX 0.7f  /* 平移 m/s */
#define CHASSIS_YAW_SPEED_MAX       5.0f  /* 自旋 rad/s */

typedef struct
{
    Chassis_Mode_Enum Chassis_Mode;
} Comm_Typedef_Info;

/* 遥控器数据（原 remote_control.h，DBUS 驱动已删除，仅保留类型/按键定义供编译或后续移植时新车底盘调试） */
#define KEY_PRESSED_OFFSET_W     ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S     ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A     ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D     ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL  ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q     ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E     ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R     ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F     ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G     ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z     ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X     ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C     ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V     ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B     ((uint16_t)1 << 15)

typedef struct
{
    struct
    {
        int16_t ch[5];
        char s[2];
    } rc;
    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;
    struct
    {
        uint16_t v;
    } key;
} RC_ctrl_t;

//底盘跟随模式参数
#define WHEEL_FOLLOW1_KP 0.3f
#define WHEEL_FOLLOW1_KI 0.0f
#define WHEEL_FOLLOW1_KD 0.0f
#define WHEEL_FOLLOW1_MAX_OUT 3000.0f
#define WHEEL_FOLLOW1_MAX_IOUT 0.0f

/*
  ┌─────────────────────┬──────────────┐
  │        保留         │     角色     │
  ├─────────────────────┼──────────────┤
  │ Last_Mode           │ 模式切换检测 │
  ├─────────────────────┼──────────────┤
  │ Raw.Vx / Raw.Vy     │ 云台输入平移 │
  ├─────────────────────┼──────────────┤
  │ Target.Vx / Vy / Vw │ 旋转后目标   │
  ├─────────────────────┼──────────────┤
  │ Target.Vw_spin      │ 小陀螺中间量 │
  ├─────────────────────┼──────────────┤
  │ Target.Wheeldeg[4]  │ 四轮目标转速 │
  ├─────────────────────┼──────────────┤
  │ Transform.COS / SIN │ 坐标系旋转   │
  └─────────────────────┴──────────────┘
*/

typedef struct
  {
      uint8_t Last_Mode;

      struct
      {
          float Vx;
          float Vy;
      } Raw;

      struct
      {
          float Vx;
          float Vy;
          float Vw;
          float Vw_spin;
          float Wheeldeg[4];
      } Target;

      struct
      {
          float COS;
          float SIN;
      } Transform;
  } Chassis_Info_Typedef;


void Chassis_Init(void);
void Chassis_Taget_Update(void);
void Chassis_Cmd(void);
void TURN_ON(fp32);
extern void CHASSIS_TASK(void const * argument);

#endif

