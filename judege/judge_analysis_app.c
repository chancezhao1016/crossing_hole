#define __DRIVER_GLOBALS

#include "judge_analysis_app.h"

#include "string.h"
#include "crc_check.h"
#include "usart.h"
uint8_t judge_re[400];

extern DMA_HandleTypeDef hdma_usart6_rx;
void judgeCalculate(uint8_t JudgeDataBuffer[JudgeBufferLength])//裁判系统解算
{	
	static uint16_t start_pos=0,next_start_pos=0;
	while(1)
	{
		memcpy(&JudgementData.frameHeader, &JudgeDataBuffer[start_pos],FrameHeader_Len);
		/*先校验头帧0xA5 然后crc8校验帧头 再crc16位校验整包*/
		if((JudgementData.frameHeader.SOF==(uint16_t)JudgeFrameHeader)\
    &&(1==Verify_CRC8_Check_Sum(&JudgeDataBuffer[start_pos],FrameHeader_Len))\
    &&(1==Verify_CRC16_Check_Sum(&JudgeDataBuffer[start_pos], JudgementData.frameHeader.DataLenth+FrameHeader_Len+4)))//数据位长度+帧头长度+命令码长度+校验码长度
		{
			memcpy(&JudgementData.rxCmdId, (&JudgeDataBuffer[start_pos]+5), sizeof(JudgementData.rxCmdId));
			JudgeDataBuffer[start_pos]++;//每处理完一次就在帧头加一防止再次处理这帧数据
			next_start_pos=start_pos+9+JudgementData.frameHeader.DataLenth;//9为 5位帧头 2位数据长度 2校验位
			
			switch(JudgementData.rxCmdId)
			{
				case ID_game_state://读取比赛状态数据 0x0001
				{
					memcpy(&JudgementData.GameState,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去
				}
				break;
				case ID_game_result://读取比赛结果 0x0002
				{
					memcpy(&JudgementData.GameResult,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去
				}
				break;
				case ID_game_robot_HP://读取机器人血量数0x0003
				{
					memcpy(&JudgementData.GameRobotHP,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去
				}
				break;
				case ID_event_data://读取场地事件 0x0101
				{
					memcpy(&JudgementData.EventData,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去
				}
				break;
				case ID_supply_projectile_action://读取补给站数据 0x0102
				{
					memcpy(&JudgementData.SupplyProjectileAction,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去
				}
				break;
				case ID_referee_warning://0x0104,//裁判警告
				{
					memcpy(&JudgementData.RefereeWarn,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去
				}
				break;				
				case ID_dart_remaining_time://0x0105,//飞镖发射相关数据
				{
					memcpy(&JudgementData.DartRemain,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去
				}
				break;
				
				case Robot_Status_ID://0x0201,//机器人状态  等级
				{
					memcpy(&JudgementData.robot_status_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去
				}
				break;
				case power_heat_data_ID://0x0202,//枪口热量 底盘功率
				{	
					memcpy(&JudgementData.power_heat_data_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去
			  }					
				case robot_pos_ID://0x0203,//机器人位置数据
				{
					memcpy(&JudgementData.robot_pos_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去
				}
				break;
				case robot_buff_ID: //0x0204,//buff
        {
					memcpy(&JudgementData.buff_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去			
				}
				break;
				case robot_hurt_ID://=0x0206,//伤害类型
				{
					memcpy(&JudgementData.hurt_data_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去			
				}		
				break;
				case shoot_data_ID://0x0207,//射频射速
				{
					memcpy(&JudgementData.shoot_data_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去			

				}		
				break;
				case projectile_allowance_ID: //0x0208,//允许发弹量	
        {
					memcpy(&JudgementData.projectile_allowance_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去			
				}
				break;
				case rfid_status_ID: //=0x0209,//机器人 RFID 模块状态
        {
					memcpy(&JudgementData.rfid_status_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去			
				}
				break;
				case dart_client_cmd_ID:// =0x020A,//飞镖选手端指令数据
				{
					memcpy(&JudgementData.dart_client_cmd_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去			
				}		
				break;
				case ground_robot_position_ID: //	=0x020B,//地面机器人位置数据
        {
					memcpy(&JudgementData.ground_robot_position_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去			
				}
				break;
				case radar_mark_data_ID://=0x020C,//雷达标记进度数据
				{
					memcpy(&JudgementData.radar_mark_data_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去			
				}		
				break;
				case sentry_info_ID://  =0x020D,//哨兵自主决策信息同步
				{
					memcpy(&JudgementData.sentry_info_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去			
				}		
				break;				
				case radar_info_t_ID://=0x020E,//雷达自主决策信息同步
				{
					memcpy(&JudgementData.radar_info_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去			
				}		
				break;						
				
				case student_interactive_header_ID:
        {
					memcpy(&JudgementData.userinfo,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去			
				}
				break;
//				case controller_header_ID: //自定义控制器
//        {
//					memcpy(&JudgementData.controller_data_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去			
//				}
//				break;
				case key_inpict_ID: //图传串口传键鼠
        {
				memcpy(&JudgementData.robot_key_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去			
				}
				break;
				
			}
			start_pos=next_start_pos;
		}
		else
		{
			start_pos=0;
			break;				
		}		
		/**如果头指针越界了退出循环**/
		if(start_pos>JudgeBufferLength)
		{
			start_pos=0;
			break;
		}
	}
}

uint8_t JudgeUARTtemp;
//Judge
//void USART6_IRQHandler(void)
//{
//    JudgeUARTtemp = USART6->DR;
//    JudgeUARTtemp = USART6->SR;
//    
//    DMA_Cmd(DMA2_Stream1, DISABLE);
//			
//	  judgeCalculate(JudgeDataBuffer);
//		
//    //重启DMA
//    DMA_ClearFlag(DMA2_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);
//    while(DMA_GetCmdStatus(DMA2_Stream1) != DISABLE);
//    DMA_SetCurrDataCounter(DMA2_Stream1, JudgeBufferLength);
//    DMA_Cmd(DMA2_Stream1, ENABLE);
//}

//void USART3_SendBuff (uint8_t *buf,uint16_t len)
//{
//  int i;
//	for(i=0; i<len;i++)
//	{
//    USART_SendData(USART6,*((u8*)&buf+i));
//	}
//while( USART_GetFlagStatus(USART6 ,USART_FLAG_TC) == RESET );
//        
//}

//tFrame Frame = {.FrameHeader.SOF = 0xA5};
//uint8_t tx_buf[200];
//void Send_FrameData(uint16_t cmdid, uint8_t * pchMessage,uint16_t dwLength)
//{	
//	uint16_t frame_length = HEADER_LEN + CMD_LEN + dwLength + CRC_LEN;
//	
//	tFrameHeader *p_header = (tFrameHeader*)tx_buf;
//  
//  p_header->SOF          = 0xA5;
//  p_header->DataLenth    = dwLength;
//  p_header->Seq          = 0;
//	 
//	memcpy(&tx_buf[HEADER_LEN], (uint8_t*)&cmdid, CMD_LEN); 
//  Append_CRC8_Check_Sum(tx_buf, HEADER_LEN);  
//  memcpy(&tx_buf[HEADER_LEN + CMD_LEN], pchMessage, dwLength);
//  Append_CRC16_Check_Sum(tx_buf, frame_length);
//	
//  USART3_SendBuff((uint8_t *)&tx_buf,frame_length);               //帧发送	
//	
//} 

//extern uint8_t client_custom_Send[28]={0xA5,0x13,0,0,0x6D,  0x01,0x03,  0x80,0xD1,   0x00,0x00,  0x00,0x00,  0,0,0,0, 0,0,0,0, 0,0,0,0, 0,    0,0};
//extern uint8_t robot_custom_Send[28]={0xA5,0x13,0,0,0x6D,  0x01,0x03,  0x00,0x02,   0x00,0x00,  0x00,0x02,  0,0,0,0, 0,0,0,0, 0,0,0,0, 0,    0,0};
//	
//void client_send(uint8_t * data)
//{
//		JudgementData.userinfo.id.send_id=JudgementData.robot_status_t.robot_id;
//	
//		if(JudgementData.robot_status_t.robot_id>10)
//			JudgementData.userinfo.id.receive_id=0x0106+JudgementData.userinfo.id.send_id;
//		else
//			JudgementData.userinfo.id.receive_id=0x0100+JudgementData.userinfo.id.send_id;


//		JudgementData.userinfo.data3=8;

//		memcpy(data,&JudgementData.userinfo,28);
//		Append_CRC16_Check_Sum(data,28);
//		USART3_SendBuff(data,28);
//}

//void robot_send(uint8_t * data)
//{
//		JudgementData.robot_data_t.id.send_id=JudgementData.robot_status_t.robot_id;
//		if(JudgementData.robot_status_t.robot_id<10)
//			JudgementData.robot_data_t.id.receive_id=3;
//		else 
//			JudgementData.robot_data_t.id.receive_id=17;		
//		memcpy(data,&JudgementData.robot_data_t,28);
//		Append_CRC16_Check_Sum(data,28);
//		USART3_SendBuff(data,28);
//}

//void client_init(uint8_t * data1,uint8_t * data2)
//{
//		memcpy(&JudgementData.userinfo,data1,28);
//		memcpy(&JudgementData.robot_data_t,data2,28);	
//}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{
  if(huart==&huart6)
    {
			judgeCalculate(judge_re);
			memset(judge_re,0,400);
      HAL_UARTEx_ReceiveToIdle_DMA(&huart6,judge_re, sizeof(judge_re));
      __HAL_DMA_DISABLE_IT(&hdma_usart6_rx,DMA_IT_HT);
    }
}
