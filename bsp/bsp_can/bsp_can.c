#include "bsp_can.h"
#include "main.h"
#include "CAN_receive.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/**
 * @brief 配置 CAN 接收过滤器和中断（参照旧工程 boards/src/bsp_can.c）
 *        CAN1 FIFO0: 3508 轮电机 0x201~0x204
 *        CAN1 FIFO1: 超电 0x051 / 无线 0x052
 *        CAN2 FIFO0: 云台 0x150 / 0x151
 */
void can_filter_init(void)
{
    CAN_FilterTypeDef can_filter_st;

    /* CAN1 FIFO0：3508 轮电机 */
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDLIST;
    can_filter_st.FilterScale = CAN_FILTERSCALE_16BIT;
    can_filter_st.FilterIdHigh = CAN1_FIFO0_ID0 << 5;
    can_filter_st.FilterIdLow = CAN1_FIFO0_ID1 << 5;
    can_filter_st.FilterMaskIdHigh = CAN1_FIFO0_ID2 << 5;
    can_filter_st.FilterMaskIdLow = CAN1_FIFO0_ID3 << 5;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter_st.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    /* CAN1 FIFO1：超电 / 无线充电 */
    can_filter_st.FilterBank = 2;
    can_filter_st.FilterIdHigh = CAN1_FIFO1_ID0 << 5;
    can_filter_st.FilterIdLow = CAN1_FIFO1_ID1 << 5;
    can_filter_st.FilterMaskIdHigh = CAN1_FIFO1_ID2 << 5;
    can_filter_st.FilterMaskIdLow = CAN1_FIFO1_ID3 << 5;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);

    /* CAN2 FIFO0：云台通信 */
    can_filter_st.FilterBank = 15;
    can_filter_st.FilterIdHigh = CAN2_FIFO0_ID0 << 5;
    can_filter_st.FilterIdLow = CAN2_FIFO0_ID1 << 5;
    can_filter_st.FilterMaskIdHigh = CAN2_FIFO0_ID2 << 5;
    can_filter_st.FilterMaskIdLow = CAN2_FIFO0_ID3 << 5;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

    /* CAN2 FIFO1：保留 */
    can_filter_st.FilterBank = 16;
    can_filter_st.FilterIdHigh = CAN2_FIFO1_ID0 << 5;
    can_filter_st.FilterIdLow = CAN2_FIFO1_ID1 << 5;
    can_filter_st.FilterMaskIdHigh = CAN2_FIFO1_ID2 << 5;
    can_filter_st.FilterMaskIdLow = CAN2_FIFO1_ID3 << 5;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

    /* 发送邮箱空中断 */
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_TX_MAILBOX_EMPTY);
}
