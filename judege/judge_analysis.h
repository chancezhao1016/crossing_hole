#ifndef JUDGE_ANALYSIS_H
#define JUDGE_ANALYSIS_H

#include "stm32f4xx.h"
/************************ 2017年的裁判系统协议结构体 Start ***********************/
//#define JudgeBufferLength       150
//#define JudgeFrameLength_1      44
//#define JudgeFrameLength_2      11
//#define JudgeFrameLength_3      24

//#define JudgeFrameHeader        0xA5        //帧头 

#ifdef  __DRIVER_GLOBALS
#define __DRIVER_EXT
#else
#define __DRIVER_EXT extern
#endif

//小符状态枚举
typedef enum
{
    BUFF_TYPE_NONE, //无效
    BUFF_TYPE_ARMOR = 0x01, //防御符
    BUFF_TYPE_SUPPLY = 0x04, //加血符
    BUFF_TYPE_BULLFTS= 0x08, //加弹符
}LBuffType_Enum;

//位置状态结构体
typedef __packed struct
{
    uint8_t flag; //0 无效， 1 有效
    uint32_t x;
    uint32_t y;
    uint32_t z;
    uint32_t compass;
}GpsData_Struct;

//比赛进程信息结构体
typedef __packed struct
{
    uint32_t remainTime;
    uint16_t remainLifeValue;
    float realChassisOutV;
    float realChassisOutA;
    uint8_t runeStatus[4];
    uint8_t bigRune0Status;
    uint8_t bigRune1status;
    uint8_t conveyorBelts0:2;
    uint8_t conveyorBelts1:2;
    uint8_t parkingApron0:1;
    uint8_t parkingApron1:1;
    uint8_t parkingApron2:1;
    uint8_t parkingApron3:1;
    GpsData_Struct gpsData;
}GameInfo_Struct;

//实时血量变化信息结构体
typedef __packed struct
{
    uint8_t weakId:4;
    uint8_t way:4;
    uint16_t value;
}RealBloodChangedData_Struct;

//实时射击信息结构体
typedef __packed struct
{
    float realBulletShootSpeed;
    float realBulletShootFreq;
    float realGolfShootSpeed;
    float realGolfShootFreq;
}RealShootData_Struct;

//裁判系统结构体
typedef struct
{
    float RealVoltage;                  //实时电压
    float RealCurrent;                  //实时电流
    int16_t LastBlood;                  //剩余血量
    uint8_t LastHartID;                 //上次收到伤害的装甲板ID号
    //portTickType LastHartTick;          //上次受伤害时间 
    float LastShotSpeed;                //上次射击速度
    //·portTickType LastShotTick;          //上次射击时间
#if INFANTRY == 7
    uint16_t ShootNum;                  //已发射子弹数
    uint8_t BulletUseUp;                //1 基地子弹射完          0 基地子弹未射完
    uint16_t ShootFail;                 //发射失败时间 
#endif
}InfantryJudge_Struct;

//裁判系统数据缓存
//__DRIVER_EXT uint8_t JudgeDataBuffer[JudgeBufferLength];
//实时电压
//__DRIVER_EXT InfantryJudge_Struct InfantryJudge;
////帧率计数器
//__DRIVER_EXT float JudgeFrameCounter;
////帧率
//__DRIVER_EXT float JudgeFrameRate;

/************************ 2017年的裁判系统协议结构体 End ***********************/

/************************ 2018年的裁判系统协议结构体 Start ***********************/
#define JudgeBufferLength       512

#define JudgeBufferLength_PJ        29 //0X0004
#define JudgeBufferLength_Position  25 //0X0008
#define JudgeFrameLength_PJPosi_SH      54 //之所以是54是因为大疆将04 与 08两段数据安排到一帧里发送，但是校验却是其中一个包的校验
#define JudgeFrameLength_1_SH      54 

#define JudgeFrameLength_2_SH      11
#define JudgeFrameLength_3_SH      24

//比赛机器人状态 (0x0001)
typedef __packed struct
{
	uint16_t stageRemianTime;
	uint8_t gameProgress;
	uint8_t robotLevel;
	uint16_t remainHP;
	uint16_t maxHP;
}extGameRobotState_t;

//实施射击信息 (0x0003)
typedef __packed struct
{
	uint8_t bulletType;
	uint8_t bulletFreq;
	float bulletSpeed;
	int16_t bulletcount;
	uint8_t bulletMAXspeed;
}extShootData_t;

//实时功率热量数据 (0x0004)
typedef __packed struct
{
	float chassisVolt;
	float chassisCurrent;
	float chassisPower;
	float chassisPowerBuffer;
	uint16_t shooterHeat0;
	uint16_t shooterHeat1;
	uint16_t shooterHeat0MAX;
	uint16_t shooterHeat1MAX;
	uint16_t shooterHeatCooling0;
	uint16_t shooterHeatCooling1;
}extPowerHeatData_t;

//机器人位置朝向信息 (0x0008)
typedef __packed struct
{
	float x;
	float y;
	float z;
	float yaw;
}extGameRobotPos_t;

//裁判系统数据缓存
//__DRIVER_EXT uint8_t JudgeDataBuffer[JudgeBufferLength];

//__DRIVER_EXT extGameRobotState_t extGameRobotState;

//__DRIVER_EXT extGameRobotPos_t extGameRobotPos;

//__DRIVER_EXT extPowerHeatData_t extPowerHeatData;

//__DRIVER_EXT extShootData_t extShootData;

/************************ 2018年的裁判系统协议结构体 End ***********************/

///************************ 2021年的裁判系统协议结构体 Start ***********************/

#define JudgeFrameHeader        0xA5        //帧头
#define FrameHeader_Len         5 

#define CMD_LEN      2    //cmdid bytes
#define CRC_LEN      2    //crc16 bytes
#define HEADER_LEN   sizeof(FrameHeader_t)  //frame length

typedef __packed struct
{
  uint8_t   SOF;//0xA5
  uint16_t  DataLenth;
	uint8_t   Seq;
  uint8_t   CRC8;
}tFrameHeader;

typedef enum 
{
	GameInfo = 0x0001,     
	RealBloodChangedData,  
	RealShootData,            
	SelfDefinedData =0x0100, 
	Wrong = 0x1301
}tCmdID; 

typedef __packed struct
{
	uint8_t SOF;               //数据起始字节，固定为0xA5          
	uint16_t DataLength;       //数据长度
	uint8_t Seq;               //包序号
	uint8_t CRC8;              //帧头CRC校验
}FrameHeader_t;//帧头

/***************命令码ID********************/

/* 

	ID: 0x0001  Byte:  3    比赛状态数据       			发送频率 1Hz      
	ID: 0x0002  Byte:  1    比赛结果数据         		比赛结束后发送      
	ID: 0x0003  Byte:  2    比赛机器人存活数据   		1Hz发送  
	ID: 0x0101  Byte:  4    场地事件数据   				事件改变后发送
	ID: 0x0102  Byte:  3    场地补给站动作标识数据    	动作改变后发送 
	ID: 0X0103  Byte:  2    场地补给站预约子弹数据      参赛队发送，10Hz 
	ID: 0X0201  Byte: 15    机器人状态数据        		10Hz
	ID: 0X0202  Byte: 14    实时功率热量数据   			50Hz       
	ID: 0x0203  Byte: 16    机器人位置数据           	10Hz
	ID: 0x0204  Byte:  1    机器人增益数据           	增益状态改变后发送
	ID: 0x0205  Byte:  3    空中机器人能量状态数据      10Hz
	ID: 0x0206  Byte:  1    伤害状态数据           		伤害发生后发送
	ID: 0x0207  Byte:  6    实时射击数据           		子弹发射后发送
	ID: 0x0301  Byte:  n    机器人间交互数据           	发送方触发发送,10Hz
	
*/

typedef enum
{
	ID_game_state       					= 0x0001,//比赛状态数据
	ID_game_result 	   						= 0x0002,//比赛结果数据
	ID_game_robot_HP       				= 0x0003,//比赛机器人存活数据
	ID_dart_state									= 0x0004,
	
	ID_event_data  								= 0x0101,//场地事件数据 
	ID_supply_projectile_action   = 0x0102,//场地补给站动作标识数据
	ID_referee_warning						=	0x0104,
	ID_dart_remaining_time				= 0x0105,
	
	Robot_Status_ID               =0x0201,//机器人状态  等级
	power_heat_data_ID   	        =0x0202,//枪口热量 底盘功率
	robot_hurt_ID                 =0x0206,//伤害类型
	shoot_data_ID	      					=0x0207,//射频射速
	robot_buff_ID                 =0x0204,//增益

	student_interactive_header_ID     =0x0301,
} Judege_Cmd_ID;
//帧头

typedef __packed struct
{
  uint8_t   SOF;//0xA5
  uint16_t  DataLenth;//数据位长度
	uint8_t   Seq;//包序号
  uint8_t   CRC8;//crc8位校验
}tFrameHeader_Judge;

//比赛状态数据 命令码ID 0x0001  发送频率：1Hz
typedef __packed struct
{
	uint8_t game_type:4;     //0-3bit:比赛类型 1：大师赛 2:单项赛 3：人工智能挑战赛 4；3V3 5:1V1
	uint8_t game_progress:4; //4-7bit:比赛阶段 0：未开始比赛 1：准备阶段 2：自检阶段 3:5s倒计时 4：对战中 5：比赛结算中
	uint16_t stage_remain_time; //当前阶段剩余时间 /s
	
	uint64_t SyncTimeStamp;
}ext_game_status_t;

//比赛结果数据 命令码ID 0x0002  发送频率：比赛结束后发送
typedef __packed struct
{
	uint8_t winner;  //0:平局 1:红方胜利 2：蓝方胜利
}ext_game_result_t;

//机器人血量数据 命令码ID 0x0003  发送频率：1Hz
typedef __packed struct
{
	uint16_t red_1_robot_HP; //红1英雄机器人血量  未上场以及罚下血量为0
	uint16_t red_2_robot_HP; //红2工程机器人血量 
	uint16_t red_3_robot_HP; //红3步兵机器人血量 
	uint16_t red_4_robot_HP; //红4步兵机器人血量 
	uint16_t red_5_robot_HP; //红5步兵机器人血量 
	uint16_t red_7_robot_HP; //红5哨兵机器人血量 
	
	uint16_t red_outpost_HP; //红方前哨站
	uint16_t red_base_HP; //红方基地
	
	uint16_t blue_1_robot_HP; //蓝1英雄机器人血量  未上场以及罚下血量为0
	uint16_t blue_2_robot_HP; //蓝2工程机器人血量 
	uint16_t blue_3_robot_HP; //蓝3步兵机器人血量 
	uint16_t blue_4_robot_HP; //蓝4步兵机器人血量 
	uint16_t blue_5_robot_HP; //蓝5步兵机器人血量 
	uint16_t blue_7_robot_HP; //蓝5哨兵机器人血量 
	
	uint16_t blue_outpost_HP; //蓝方前哨站
	uint16_t blue_base_HP; //蓝方基地
}ext_game_robot_HP_t;

//飞镖发射状态 命令码ID 0x0004  发送频率：飞镖发射后发送  发送范围：所有机器人
typedef __packed struct
{
	uint8_t dart_belong; //发射飞镖的队伍: 1：红方飞镖  2：蓝方飞镖
	uint8_t stage_remaining_time; //发射时剩余的比赛时间  /s
}ext_dart_status;

//场地事件数据 命令码ID 0x0101  发送频率：事件改变后发送
/*
bit0: 己方补给站 1号补血点占领状态 1为已占领
bit1: 己方补给站 2号补血点占领状态 1为已占领
bit2: 己方补给站 3号补血点占领状态 1为已占领
bit3-5：己方能量机关状态：
	bit3：打击点占领状态 1为已占领
	bit4：小能量机关激活状态 1为已激活
	bit5：大能量机关激活状态 1为已激活
bit6：己方R2环形高地占领状态 1为已占领
bit7：己方R3梯形高地占领状态 1为已占领
bit8：己方R4梯形高地占领状态 1为已占领 
bit9：己方基地护盾状态：
			1为基地有虚拟护盾血量
			0为基地无虚拟护盾血量
bit10：己方前哨战状态：
			1为前哨战存活
			0为前哨战被击毁
bit10-31：保留

*/
typedef __packed struct
{
	uint32_t event_type;
}ext_event_data_t;

//补给站动作标识 命令码ID 0x0102  发送频率：动作改变后发送 发送范围：己方机器人
typedef __packed struct
{
	uint8_t supply_projectile_id; //补给口ID 1:1号补给口  2：2号补给口
	uint8_t supply_robot_id;   //补弹机器人|D：0为当前无机器人补弹，1为红方英雄机器人补弹，2为红方工程机器人补弹，3/4/5为红方步兵机器人补弹，
	                           //101为蓝方英雄机器人补弹，102为蓝方工程机器人补弹，103/104/105为蓝方步兵机器人补弹
	
	uint8_t supply_projectile_step;  //出弹口开闭状态：0为关闭，1为子弹准备中，2为子弹下落
	 
	uint8_t supply_projectile_num;  //补弹数量50: 50颗子弹 100:100颗子弹 150:150颗子弹 200:200颗子弹。
}ext_supply_projectile_action_t;

//裁判警告信息 命令码ID 0x0104  发送频率：发生警告后发送 
typedef __packed struct
{
	uint8_t level;  //警告等级 1：黄牌  2：红牌  3：判负
	uint8_t foul_robot_id;  //犯规机器人ID 判负时，机器人ID为0  黄牌、红牌时，机器人ID为犯规机器人ID
}ext_referee_warning_t;


//飞镖发射口倒计时 命令码ID 0x0105  发送频率：1Hz  发送范围：己方机器人
typedef __packed struct
{
	uint8_t dart_remaining_time;  //15s倒计时
}ext_dart_remaining_time_t;

/*伤害类型*/
typedef __packed struct
{
uint8_t armor_id : 4;
uint8_t hurt_type : 4;
} ext_robot_hurt_t;
/*比赛机器人状态*/
typedef __packed struct
{
//	uint8_t robot_id;//机器人ID
//	uint8_t robot_level;//机器人等级
//	uint16_t remain_HP;//机器人剩余HP
//	uint16_t max_HP;//机器人上限血量
//	uint16_t shooter_heat0_cooling_rate;//机器人17mm枪口每秒冷却值
//	uint16_t shooter_heat0_cooling_limit;//枪口热量上限
//	uint16_t shooter_heat1_cooling_rate;//42mm枪口每秒冷却值
//	uint16_t shooter_heat1_cooling_limit;//枪口热量上限
//	uint8_t mains_power_gimbal_output : 1;
//	uint8_t mains_power_chassis_output : 1;
//	uint8_t mains_power_shooter_output : 1;
	
	

	
//	uint8_t robot_id;
//	uint8_t robot_level; //机器人等级 1：一级；2：二级：3：三级。
//	uint16_t remain_HP;  //机器人剩余血量
//	uint16_t max_HP;     //机器人上限血量
//	
//	
//	uint16_t shooter_id1_17mm_cooling_rate; //机器人1号17mm枪口每秒冷却值
//	uint16_t shooter_id1_17mm_cooling_limit;//机器人1号17mm枪口热量上限
//	uint16_t shooter_id1_17mm_speed_limit;  //机器人1号17mm枪口上限速度 m/s
//	
//	uint16_t shooter_id2_17mm_cooling_rate; //机器人2号17mm枪口每秒冷却值
//	uint16_t shooter_id2_17mm_cooling_limit;//机器人2号17mm枪口每秒冷却值
//	uint16_t shooter_id2_17mm_speed_limit;  //机器人2号17mm枪口上限速度 m/s

//	uint16_t shooter_id2_42mm_cooling_rate;//机器人42mm枪口每秒冷却值
//	uint16_t shooter_id2_42mm_cooling_limit;//机器人42mm枪口热量上限
//	uint16_t shooter_id2_42mm_speed_limit;//机器人42mm枪口上限速度 m/s 
//	
//	uint16_t chassis_power_limit;  //机器人底盘功率限制
//	uint8_t mains_power_gimbal_output:1;  //0bt:gmba口输出：1为有24V输出，0为无24v输出：                                     
//	uint8_t mains_power_chassis_output:1; // 1bit:chassis口输出：1为有24V输出，0为无24v输出：
//	uint8_t mains_power_shooter_output:1; //2bit:shooter口输出：1为有24V输出，0为无24v输出：

		uint8_t robot_id;//机器人ID
	uint8_t robot_level;//机器人等级
	uint16_t remain_HP;//机器人剩余HP
	uint16_t max_HP;//机器人上限血量

	uint16_t shooter_barrel_cooling_value;//机器人枪口热量每秒冷却值
uint16_t shooter_barrel_heat_limit;//枪口热量上限
uint16_t chassis_power_limit;//底盘功率上限
	uint8_t mains_power_gimbal_output : 1;//gimbal 口输出： 0 为无输出， 1 为 24V 输出
	uint8_t mains_power_chassis_output : 1;//chassis 口输出： 0 为无输出， 1 为 24V 输出
	uint8_t mains_power_shooter_output : 1;//shooter 口输出： 0 为无输出， 1 为 24V 输出


} ext_game_robot_state_t;

/*底盘功率及枪口热量*/
typedef __packed struct
{
	uint16_t chassis_volt;//底盘输出电压  单位 毫伏
	uint16_t chassis_current;//底盘输出电流  单位毫安
	float chassis_power;//底盘输出功率  单位 W
uint16_t buffer_energy;//缓冲能量（单位： J）
uint16_t shooter_17mm_1_barrel_heat;//第 1 个 17mm 发射机构的枪口热量
uint16_t shooter_17mm_2_barrel_heat;//第 2 个 17mm 发射机构的枪口热量
uint16_t shooter_42mm_barrel_heat;//42mm 发射机构的枪口热量
} ext_power_heat_data_t;

/*子弹信息*/
typedef __packed struct
{
	uint8_t bullet_type;//子弹类型
	uint8_t shooter_number;//发射机构 ID
	uint8_t bullet_freq;//子弹射频
	float bullet_speed;//子弹射速
} ext_shoot_data_t;

//机器人增益 命令码ID 0x0204  发送频率：1Hz 
typedef __packed struct
{
uint8_t recovery_buff;//机器人回血增益（百分比，值为 10 表示每秒恢复血量上限的 10%）
uint8_t cooling_buff;//机器人枪口冷却倍率（直接值，值为 5 表示 5 倍冷却）
uint8_t defence_buff;//机器人防御增益（百分比，值为 50 表示 50%防御增益）
uint8_t vulnerability_buff;//机器人负防御增益（百分比，值为 30 表示-30%防御增益）
uint16_t attack_buff;//机器人攻击增益（百分比，值为 50 表示 50%攻击增益）
}ext_buff_t;




/*用户自定义部分显示在屏幕上*/
/*0-1 数据内容ID 0xD180
	2-3 发送者的ID
*/

typedef __packed struct
{
	uint16_t rxCmdId;
	uint16_t data_id;
	uint16_t send_id;
	uint16_t receive_id;
} id_data_t;

typedef __packed struct
{
	tFrameHeader Header;
	id_data_t    id;
	float data1;
	float data2;
	float data3;
	uint8_t masks;
	uint16_t crc_16;
} client_custom_data_t;

//机器人之间相互通信
typedef __packed struct
{
	uint8_t data[112];
} robot_interactive_data_t;

//typedef __packed struct
//{
//	uint16_t data_cmd_id;
//	uint16_t send_ID;
//	uint16_t receiver_ID;
//}ext_student_interactive_header_data_t;



//己方通讯


typedef  struct
{
  tFrameHeader          frameHeader;
  uint16_t              rxCmdId;
	
	ext_game_status_t     GameState;				        	//0x0001
	ext_game_result_t     GameResult;			        	//0x0002
	ext_game_robot_HP_t   GameRobotHP;			          //0x0003
	ext_dart_status       GameDart;                  //0x0004
	
	ext_event_data_t        			   EventData;					        //0x0101		
	ext_supply_projectile_action_t	 SupplyProjectileAction;		//0x0102
	//ext_supply_projectile_booking_t	 SupplyProjectileBooking;	  //0x0103
	ext_referee_warning_t            RefereeWarn;               //0x0104
	ext_dart_remaining_time_t        DartRemain;                //0x0105

  ext_power_heat_data_t power_heat_data_t;
  ext_shoot_data_t		  shoot_data_t;
	ext_robot_hurt_t      robot_hurt_t;
	ext_buff_t            robot_buff_t;
	ext_game_robot_state_t game_robot_state_t;
//  robot_interactive_data_t robot_data_t;
	client_custom_data_t robot_data_t;
	client_custom_data_t userinfo;
}JudgementDataTypedef;

typedef __packed struct
{
	float data1;
	float data2;
	float data3;
	uint8_t mask;
}tSelfDefineInfo;  

typedef  __packed struct
{
	float data1;
	float data2;
	float data3;
	uint8_t mask;
}SelfDefineInfo_t;   //学生上传自定义数据 (0x0005)  

typedef  struct
{
  tFrameHeader_Judge          frameHeader;
  uint16_t              rxCmdId;
		__packed union 
	{
  ext_power_heat_data_t power_heat_data_t;
  ext_shoot_data_t		  shoot_data_t;
	ext_robot_hurt_t      robot_hurt_t;
	ext_game_robot_state_t game_robot_state_t;
  robot_interactive_data_t robot_data_t;
	client_custom_data_t userinfo;
		}Data;
	uint16_t        CRC16;
	uint16_t        CRC16_2 ;
}FRAME;

typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	tSelfDefineInfo SelfDefineInfo; 
	uint16_t        CRC16;         
}tFrame;  

__DRIVER_EXT uint8_t JudgeDataBuffer[JudgeBufferLength];
__DRIVER_EXT SelfDefineInfo_t SelfDefineInfo;
__DRIVER_EXT JudgementDataTypedef JudgementData;

void Send_FrameData(uint16_t cmdid, uint8_t * pchMessage,uint16_t dwLength);
void client_send(uint8_t * data);
void client_init(uint8_t * data1,uint8_t * data2);
void get_chassis_power_and_buffer(float *power, float *buffer, uint16_t *powmax);

/************************ 2019年的裁判系统协议结构体 End ***********************/


#endif
