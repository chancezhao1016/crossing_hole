#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H
#include "struct_typedef.h"

/* ============ CAN ID 配置（参照旧工程 config/can_config.h） ============ */
/* 轮电机 3508 (CAN1) */
#define DJI_3508_MOTORS_1 0x201
#define DJI_3508_MOTORS_2 0x202
#define DJI_3508_MOTORS_3 0x203
#define DJI_3508_MOTORS_4 0x204

/* 云台通信 (CAN2) */
#define GIMBAL_COMM_CAN_ID_1 0x150 /* 云台 -> 底盘 接收 */
#define GIMBAL_COMM_CAN_ID_2 0x151
#define SEND_TO_GIMBAL_CAN_ID_1 0x160 /* 底盘 -> 云台 发送 */
#define SEND_TO_GIMBAL_CAN_ID_2 0x161

/* 超级电容 (CAN1) */
#define SUPER_POWER_CAN_ID 0x051
#define SEND_TO_SUPER_POWER_CAN_ID 0x050

/* 无线充电 (CAN1) */
#define WIRELESS_CAN_ID 0x052

/* CAN 接收 FIFO 过滤 ID（参照旧工程 can_config.h） */
#define CAN1_FIFO0_ID0 DJI_3508_MOTORS_1
#define CAN1_FIFO0_ID1 DJI_3508_MOTORS_2
#define CAN1_FIFO0_ID2 DJI_3508_MOTORS_3
#define CAN1_FIFO0_ID3 DJI_3508_MOTORS_4

#define CAN1_FIFO1_ID0 SUPER_POWER_CAN_ID
#define CAN1_FIFO1_ID1 WIRELESS_CAN_ID
#define CAN1_FIFO1_ID2 0x001
#define CAN1_FIFO1_ID3 0x002

#define CAN2_FIFO0_ID0 GIMBAL_COMM_CAN_ID_1
#define CAN2_FIFO0_ID1 GIMBAL_COMM_CAN_ID_2
#define CAN2_FIFO0_ID2 0x002
#define CAN2_FIFO0_ID3 0x001

#define CAN2_FIFO1_ID0 0x002
#define CAN2_FIFO1_ID1 0x003
#define CAN2_FIFO1_ID2 0x000
#define CAN2_FIFO1_ID3 0x001

/* ============ 云台通信数据结构（参照旧工程 GimbalReceive.h / GimbalSend.h） ============ */
#pragma pack(push, 1)
typedef struct
{
    uint16_t robot_state : 1;       /* 机器人状态 */
    uint16_t control_type : 1;      /* 控制方式 */
    uint16_t chassis_mode_action : 3; /* 底盘模式 */
    uint16_t gimbal_mode : 3;       /* 云台模式 */
    uint16_t is_pc_on : 1;
    uint16_t aim_mode : 3;          /* 自瞄模式 */
    uint16_t gimbal_position : 2;   /* 头部模式 */
    uint16_t super_power : 2;       /* 超级电容状态 */

    uint16_t yaw_pose;              /* 云台 yaw 轴电机机械角度 */

    int8_t robot_speed_x;           /* *10，x 方向为云台正方向 */
    int8_t robot_speed_y;
    int8_t robot_speed_w;

    uint8_t set_friction_speed;
} GimbalReceivePack1;

typedef struct
{
    int16_t gimbal_pitch;           /* 云台 pitch 角度（角度制） */
    int16_t turn_yaw_speed;         /* 云台 yaw 目标速度，deg/s * 100 */
    uint8_t reverse[4];
} GimbalReceivePack2;

typedef struct
{
    uint8_t alive_flag : 1;         /* 存活标志位 */
    uint8_t robot_color : 1;        /* 机器人颜色：红 1 蓝 0 */
    uint8_t robot_level : 6;        /* 机器人等级 */

    float shoot_speed;              /* 发射初速度 25 上限 */
    int8_t shoot_avaiable;          /* 剩余发弹量 */
    uint8_t heat_cooling;           /* 热量恢复冷却 */

    uint8_t reverse[1];
} GimbalSendPack_1;

typedef struct
{
    uint8_t reverse[8];
} GimbalSendPack_2;
#pragma pack(pop)

/* ============ 超级电容数据结构（参照旧工程 NingCap.h） ============ */
#pragma pack(push, 1)
typedef struct
{
    uint16_t cap_vol;       /* 0.01 V */
    int16_t chassis_power;  /* 0.01 W，超电输出到底盘轮电机侧功率 */
    uint16_t referee_power; /* 0.01 W，裁判系统输入到超电侧功率 */
    uint16_t reverse;
} SuperCapRecvData;

typedef struct
{
    uint16_t buffer_energy;
    uint16_t P_ref;         /* 0.01 W */
    uint16_t reverse;
    uint16_t wireless_start;
} SuperCapSendData;
#pragma pack(pop)

/* ============ 电机反馈数据结构 ============ */
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
    uint16_t offset_ecd;
    int32_t round_cnt;
    int32_t total_ecd;
    int32_t pingdong_ecd;
} motor_measure_t;

/* ============ 掉线检测（参照旧工程 debug.h/debug.c） ============ */
typedef enum { DET_OFF = 0, DET_TIMEOUT, DET_ON } LossState_e;

typedef struct
{
    uint32_t last_tick;   /* 上次收到消息的 tick(ms) */
    uint32_t loss_time;   /* 丢帧阈值(ms) */
    LossState_e state;    /* 掉线/超时/在线 */
} Loss_Debugger;

/* 丢帧阈值(ms)（参照旧工程 can_config.h，原为秒） */
#define DJI_WHEELS_LOSS_TIME   5u
#define GIMBAL_LOSS_TIME       6u
#define SUPER_POWER_LOSS_TIME  6u

extern Loss_Debugger wheels_comm_detector[4];  /* 3508 轮电机 */
extern Loss_Debugger gimbal_comm_detector[2];  /* 云台通信 */
extern Loss_Debugger super_power_detector;     /* 超电 */

void LossUpdate(Loss_Debugger *det, uint32_t loss_time);
void LossDetect(Loss_Debugger *det);
void LossDetectAll(void);

/* ============ 函数声明 ============ */
void CAN_cmd_chassis_travel(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void CAN_send_supercap(uint16_t buffer_energy, uint16_t P_ref);
void CAN_send_gimbal(const uint8_t *data1, const uint8_t *data2);

extern motor_measure_t motor_chassis_3508[4];
extern GimbalReceivePack1 gimbal_receiver_pack1;
extern GimbalReceivePack2 gimbal_receiver_pack2;
extern GimbalSendPack_1 gimbal_pack_send_1;
extern GimbalSendPack_2 gimbal_pack_send_2;
extern SuperCapRecvData cap_recv_data;
extern SuperCapSendData cap_send_data;

#endif
