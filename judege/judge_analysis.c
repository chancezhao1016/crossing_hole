 #define __DRIVER_GLOBALS

#include "judge_analysis.h"
#include "string.h"
#include "crc_check.h"
#include "usart.h"

uint8_t judge_re[512];
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
void judgeCalculate(uint8_t JudgeDataBuffer[JudgeBufferLength])//裁判系统解算
{	
	//uint8_t JudgeDataBuffer[JudgeBufferLength];
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
				case ID_game_state://读取比赛状态数据 001
				{
					memcpy(&JudgementData.GameState,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去
				}
				break;
				case ID_game_result://读取比赛结果 002
				{
					memcpy(&JudgementData.GameResult,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去
				}
				break;
				case ID_game_robot_HP://读取机器人血量数据003
				{
					memcpy(&JudgementData.GameRobotHP,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去
				}
				break;
				case ID_event_data://读取场地事件 101
				{
					memcpy(&JudgementData.EventData,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去
				}
				break;
				case ID_supply_projectile_action://读取补给站数据 102
				{
					memcpy(&JudgementData.SupplyProjectileAction,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去
				}
				break;
				case Robot_Status_ID://读取机器人等级血量等 201
				{
					memcpy(&JudgementData.game_robot_state_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去
				}
				break;
				case power_heat_data_ID://读取机器人枪口热量 202
				{	
					memcpy(&JudgementData.power_heat_data_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去
			  }					
				break;
				case shoot_data_ID://读取机器人射频射速 207
				{
					memcpy(&JudgementData.shoot_data_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去			
				}		
				break;
				case robot_hurt_ID://读取机器人伤害 206
				{
					memcpy(&JudgementData.robot_hurt_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去			
				}		
				break;
				case robot_buff_ID: //增益 204
        {
					memcpy(&JudgementData.robot_buff_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去			
				}
				break;
				case student_interactive_header_ID:
        {
					memcpy(&JudgementData.userinfo,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去			
				}
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


//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//{
//	if(huart == &huart6)
//	{
////	// 停止DMA传输
////HAL_DMA_Abort(&hdma_usart6_rx); // 假设hdma_uart_rx是你的DMA句柄

//		 // judgeCalculate(JudgeDataBuffer);
////// 重启DMA传输
////HAL_DMA_DeInit(&hdma_usart6_rx); // 假设hdma_uart_rx是你的DMA句柄
////HAL_DMA_Init(&hdma_usart6_rx); // 假设hdma_uart_rx是你的DMA句柄
////// 重启后重新配置DMA
////HAL_DMA_Start(&hdma_usart6_rx, (uint8_t)&JudgeDataBuffer, (uint32_t)&(USART6->DR), JudgeBufferLength); // 假设UART_RX_DMA_Buffer是你的DMA缓冲区

//	}
//}

//裁判串口读取
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{
  if(huart==&huart1)
    {
			judgeCalculate(judge_re);
			memset(judge_re,0,400);
      HAL_UARTEx_ReceiveToIdle_DMA(&huart1,judge_re, sizeof(judge_re));
      __HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);
    }
}

//void USART6_IRQHandler(void)
//{
////    JudgeUARTtemp = USART6->DR;
////    JudgeUARTtemp = USART6->SR;
//    
//	
//	
//	

//	
//	
//	
////    DMA_Cmd(DMA1_Stream1, DISABLE);//hal库关闭dma
////			
////	  judgeCalculate(JudgeDataBuffer);
////		
////    //重启DMA        //hal库重启DMA 
////    DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);
////    while(DMA_GetCmdStatus(DMA1_Stream1) != DISABLE);
////    DMA_SetCurrDataCounter(DMA1_Stream1, JudgeBufferLength);
////    DMA_Cmd(DMA1_Stream1, ENABLE);
//}

//目前没用*********************************************************************************
/*
void USART3_SendBuff (uint8_t *buf,uint16_t len)
{
  int i;
	for(i=0; i<len;i++)
	{
    USART_SendData(USART3,*((u8*)&buf+i));
	}
while( USART_GetFlagStatus(USART3 ,USART_FLAG_TC) == RESET );
        
}

tFrame Frame = {.FrameHeader.SOF = 0xA5};
uint8_t tx_buf[200];
void Send_FrameData(uint16_t cmdid, uint8_t * pchMessage,uint16_t dwLength)
{	
	uint16_t frame_length = HEADER_LEN + CMD_LEN + dwLength + CRC_LEN;
	
	FrameHeader_t *p_header = (FrameHeader_t*)tx_buf;
  
  p_header->SOF          = 0xA5;
  p_header->DataLength   = dwLength;
  p_header->Seq          = 0;
	 
	memcpy(&tx_buf[HEADER_LEN], (uint8_t*)&cmdid, CMD_LEN); 
  Append_CRC8_Check_Sum(tx_buf, HEADER_LEN);  
  memcpy(&tx_buf[HEADER_LEN + CMD_LEN], pchMessage, dwLength);
  Append_CRC16_Check_Sum(tx_buf, frame_length);
	
  USART3_SendBuff((uint8_t *)&tx_buf,frame_length);               //帧发送	
	
} 

extern uint8_t client_custom_Send[28]={0xA5,0x13,0,0,0x6D,  0x01,0x03,  0x80,0xD1,   0x00,0x00,  0x00,0x00,  0,0,0,0, 0,0,0,0, 0,0,0,0, 0,    0,0};
extern uint8_t robot_custom_Send[28]={0xA5,0x13,0,0,0x6D,  0x01,0x03,  0x00,0x02,   0x00,0x00,  0x00,0x02,  0,0,0,0, 0,0,0,0, 0,0,0,0, 0,    0,0};
	
	
void client_send(uint8_t * data)
{
		JudgementData.userinfo.id.send_id=JudgementData.game_robot_state_t.robot_id;
	
		if(JudgementData.game_robot_state_t.robot_id>10)
			JudgementData.userinfo.id.receive_id=0x0106+JudgementData.userinfo.id.send_id;
		else
			JudgementData.userinfo.id.receive_id=0x0100+JudgementData.userinfo.id.send_id;


		JudgementData.userinfo.data3=8;

		memcpy(data,&JudgementData.userinfo,28);
		Append_CRC16_Check_Sum(data,28);
		USART3_SendBuff(data,28);
}

void robot_send(uint8_t * data)
{
		JudgementData.robot_data_t.id.send_id=JudgementData.game_robot_state_t.robot_id;
		if(JudgementData.game_robot_state_t.robot_id<10)
			JudgementData.robot_data_t.id.receive_id=3;
		else 
			JudgementData.robot_data_t.id.receive_id=17;		
		memcpy(data,&JudgementData.robot_data_t,28);
		Append_CRC16_Check_Sum(data,28);
		USART3_SendBuff(data,28);
}

void client_init(uint8_t * data1,uint8_t * data2)
{
		memcpy(&JudgementData.userinfo,data1,28);
		memcpy(&JudgementData.robot_data_t,data2,28);	
}


void get_chassis_power_and_buffer(float *power, float *buffer, uint16_t *powmax)
{
    *power = JudgementData.power_heat_data_t.chassis_power;
    *buffer = JudgementData.power_heat_data_t.chassis_power_buffer;
		*powmax = JudgementData.game_robot_state_t.chassis_power_limit;

}
*/

//**************************************************************************************************
