#include "CAN_receive.h"
#include "main.h"
#include "string.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/* ============ 接收数据结构实例 ============ */
motor_measure_t motor_chassis_3508[4];
GimbalReceivePack1 gimbal_receiver_pack1;
GimbalReceivePack2 gimbal_receiver_pack2;
GimbalSendPack_1 gimbal_pack_send_1;
GimbalSendPack_2 gimbal_pack_send_2;
SuperCapRecvData cap_recv_data;
SuperCapSendData cap_send_data;

/* ============ 掉线检测 ============ */
Loss_Debugger wheels_comm_detector[4];
Loss_Debugger gimbal_comm_detector[2];
Loss_Debugger super_power_detector;

void LossUpdate(Loss_Debugger *det, uint32_t loss_time)
{
    det->last_tick = HAL_GetTick();
    det->loss_time = loss_time;
    det->state = DET_ON;
}

void LossDetect(Loss_Debugger *det)
{
    if (HAL_GetTick() - det->last_tick > det->loss_time)
        det->state = DET_OFF;
}

void LossDetectAll(void)
{
    for (int i = 0; i < 4; i++) LossDetect(&wheels_comm_detector[i]);
    for (int i = 0; i < 2; i++) LossDetect(&gimbal_comm_detector[i]);
    LossDetect(&super_power_detector);
}

/* ============ 3508 电机解析 ============ */
#define get_motor_measure(ptr, data)                                    \
{                                                                       \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
        if((ptr)->ecd - (ptr)->last_ecd > 4096)                         \
            (ptr)->round_cnt--;                                         \
        else if ((ptr)->ecd - (ptr)->last_ecd < -4096)                  \
            (ptr)->round_cnt++;                                         \
        (ptr)->total_ecd = (ptr)->round_cnt * 8192 + (ptr)->ecd - (ptr)->offset_ecd; \
        (ptr)->pingdong_ecd = (ptr)->round_cnt * 8192 + (ptr)->ecd;     \
}

/* ============ 接收分发（参照旧工程 CanReceiveAll） ============ */
static void CanReceiveAll(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *rx_header, uint8_t *data)
{
    if (hcan == &hcan1)
    {
        switch (rx_header->StdId)
        {
        case DJI_3508_MOTORS_1:
            get_motor_measure(&motor_chassis_3508[0], data);
            LossUpdate(&wheels_comm_detector[0], DJI_WHEELS_LOSS_TIME);
            break;
        case DJI_3508_MOTORS_2:
            get_motor_measure(&motor_chassis_3508[1], data);
            LossUpdate(&wheels_comm_detector[1], DJI_WHEELS_LOSS_TIME);
            break;
        case DJI_3508_MOTORS_3:
            get_motor_measure(&motor_chassis_3508[2], data);
            LossUpdate(&wheels_comm_detector[2], DJI_WHEELS_LOSS_TIME);
            break;
        case DJI_3508_MOTORS_4:
            get_motor_measure(&motor_chassis_3508[3], data);
            LossUpdate(&wheels_comm_detector[3], DJI_WHEELS_LOSS_TIME);
            break;
        case SUPER_POWER_CAN_ID:
            memcpy(&cap_recv_data, data, sizeof(SuperCapRecvData));
            LossUpdate(&super_power_detector, SUPER_POWER_LOSS_TIME);
            break;
        case WIRELESS_CAN_ID:
            /* 无线充电：暂不处理 */
            break;
        default:
            break;
        }
    }
    else if (hcan == &hcan2)
    {
        switch (rx_header->StdId)
        {
        case GIMBAL_COMM_CAN_ID_1:
            memcpy(&gimbal_receiver_pack1, data, 8);
            LossUpdate(&gimbal_comm_detector[0], GIMBAL_LOSS_TIME);
            break;
        case GIMBAL_COMM_CAN_ID_2:
            memcpy(&gimbal_receiver_pack2, data, 8);
            LossUpdate(&gimbal_comm_detector[1], GIMBAL_LOSS_TIME);
            break;
        default:
            break;
        }
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    CanReceiveAll(hcan, &rx_header, rx_data);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data);
    CanReceiveAll(hcan, &rx_header, rx_data);
}

/* ============ 发送：3508 四轮（0x200 广播，CAN1） ============ */
void CAN_cmd_chassis_travel(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    CAN_TxHeaderTypeDef tx_message;
    uint8_t tx_data[8];
    uint32_t send_mail_box;

    tx_message.StdId = 0x200;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;
    tx_data[0] = motor1 >> 8;
    tx_data[1] = motor1;
    tx_data[2] = motor2 >> 8;
    tx_data[3] = motor2;
    tx_data[4] = motor3 >> 8;
    tx_data[5] = motor3;
    tx_data[6] = motor4 >> 8;
    tx_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan1, &tx_message, tx_data, &send_mail_box);
}

/* ============ 发送：超级电容（0x050，CAN1） ============ */
void CAN_send_supercap(uint16_t buffer_energy, uint16_t P_ref)
{
    CAN_TxHeaderTypeDef tx_message;
    uint8_t tx_data[8];
    uint32_t send_mail_box;

    cap_send_data.buffer_energy = buffer_energy;
    cap_send_data.P_ref = P_ref;
    cap_send_data.reverse = 0;
    cap_send_data.wireless_start = 0;

    tx_message.StdId = SEND_TO_SUPER_POWER_CAN_ID;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;
    memcpy(tx_data, &cap_send_data, 8);

    HAL_CAN_AddTxMessage(&hcan1, &tx_message, tx_data, &send_mail_box);
}

/* ============ 发送：云台（0x160/0x161，CAN2） ============ */
void CAN_send_gimbal(const uint8_t *data1, const uint8_t *data2)
{
    CAN_TxHeaderTypeDef tx_message;
    uint32_t send_mail_box;

    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;

    tx_message.StdId = SEND_TO_GIMBAL_CAN_ID_1;
    HAL_CAN_AddTxMessage(&hcan2, &tx_message, (uint8_t *)data1, &send_mail_box);

    tx_message.StdId = SEND_TO_GIMBAL_CAN_ID_2;
    HAL_CAN_AddTxMessage(&hcan2, &tx_message, (uint8_t *)data2, &send_mail_box);
}
