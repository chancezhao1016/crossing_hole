#include "GimbalTask.h"
#include "cmsis_os.h"
#include "CAN_receive.h"
#include "string.h"

static uint8_t send_to_gimbal_data_1[8];
static uint8_t send_to_gimbal_data_2[8];

/**
 * @brief 组装云台发送包（参照旧工程 GimbalSend.c / GimbalTask.c）
 *        暂无裁判系统/热量数据，先发存活标志 + 默认值，后续可接入裁判数据。
 */
static void SendToGimbalPack(void)
{
    gimbal_pack_send_1.alive_flag = 1;    /* 存活标志 */
    gimbal_pack_send_1.robot_color = 0;   /* 0=蓝 1=红 */
    gimbal_pack_send_1.robot_level = 0;
    gimbal_pack_send_1.shoot_speed = 0;
    gimbal_pack_send_1.shoot_avaiable = 0;
    gimbal_pack_send_1.heat_cooling = 0;

    memcpy(send_to_gimbal_data_1, &gimbal_pack_send_1, 8);
    memcpy(send_to_gimbal_data_2, &gimbal_pack_send_2, 8);
}

/**
 * @brief 云台通信任务：周期发送底盘状态给云台（0x160/0x161，CAN2）
 */
void Gimbal_Task(void const *argument)
{
    osDelay(1000);
    while (1)
    {
        SendToGimbalPack();
        CAN_send_gimbal(send_to_gimbal_data_1, send_to_gimbal_data_2);
        osDelay(4);
    }
}
