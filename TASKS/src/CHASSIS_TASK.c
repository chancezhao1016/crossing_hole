/*
         _____                    _____                    _____                    _____                    _____                    _____                    _____
        /\    \                  /\    \                  /\    \                  /\    \                  /\    \                  /\    \                  /\    \
       /::\    \                /::\____\                /::\    \                /::\    \                /::\    \                /::\    \                /::\    \
      /::::\    \              /:::/    /               /::::\    \              /::::\    \              /::::\    \               \:::\    \              /::::\    \
     /::::::\    \            /:::/    /               /::::::\    \            /::::::\    \            /::::::\    \               \:::\    \            /::::::\    \
    /:::/\:::\    \          /:::/    /               /:::/\:::\    \          /:::/\:::\    \          /:::/\:::\    \               \:::\    \          /:::/\:::\    \
   /:::/  \:::\    \        /:::/____/               /:::/__\:::\    \        /:::/__\:::\    \        /:::/__\:::\    \               \:::\    \        /:::/__\:::\    \
  /:::/    \:::\    \      /::::\    \              /::::\   \:::\    \       \:::\   \:::\    \       \:::\   \:::\    \              /::::\    \       \:::\   \:::\    \
 /:::/    / \:::\    \    /::::::\    \   _____    /::::::\   \:::\    \    ___\:::\   \:::\    \    ___\:::\   \:::\    \    ____    /::::::\    \    ___\:::\   \:::\    \
/:::/    /   \:::\    \  /:::/\:::\    \ /\    \  /:::/\:::\   \:::\    \  /\   \:::\   \:::\    \  /\   \:::\   \:::\    \  /\   \  /:::/\:::\    \  /\   \:::\   \:::\    \
/:::/____/     \:::\____\/:::/  \:::\    /::\____\/:::/  \:::\   \:::\____\/::\   \:::\   \:::\____\/::\   \:::\   \:::\____\/::\   \/:::/  \:::\____\/::\   \:::\   \:::\____\
\:::\    \      \::/    /\::/    \:::\  /:::/    /\::/    \:::\  /:::/    /\:::\   \:::\   \::/    /\:::\   \:::\   \::/    /\:::\  /:::/    \::/    /\:::\   \:::\   \::/    /
 \:::\    \      \/____/  \/____/ \:::\/:::/    /  \/____/ \:::\/:::/    /  \:::\   \:::\   \/____/  \:::\   \:::\   \/____/  \:::\/:::/    / \/____/  \:::\   \:::\   \/____/
  \:::\    \                       \::::::/    /            \::::::/    /    \:::\   \:::\    \       \:::\   \:::\    \       \::::::/    /            \:::\   \:::\    \
   \:::\    \                       \::::/    /              \::::/    /      \:::\   \:::\____\       \:::\   \:::\____\       \::::/____/              \:::\   \:::\____\
    \:::\    \                      /:::/    /               /:::/    /        \:::\  /:::/    /        \:::\  /:::/    /        \:::\    \               \:::\  /:::/    /
     \:::\    \                    /:::/    /               /:::/    /          \:::\/:::/    /          \:::\/:::/    /          \:::\    \               \:::\/:::/    /
      \:::\    \                  /:::/    /               /:::/    /            \::::::/    /            \::::::/    /            \:::\    \               \::::::/    /
       \:::\____\                /:::/    /               /:::/    /              \::::/    /              \::::/    /              \:::\____\               \::::/    /
        \::/    /                \::/    /                \::/    /                \::/    /                \::/    /                \::/    /                \::/    /
         \/____/                  \/____/                  \/____/                  \/____/                  \/____/                  \/____/                  \/____/
*/

#include "CHASSIS_TASK.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "CAN_receive.h"
#include "struct_typedef.h"
#include "pid.h"
#include "math.h"
#include "pid_cal.h"
#include "ramp.h"
#include "arm_math.h"
#include "mecanum.h"

// ==========================================================
// 全局变量与静态变量
// ==========================================================
Chassis_Info_Typedef Chassis_Info;
fp32 vx_set, vy_set, vw_set, vx_s, vy_s;
extern fp32 k3508;
extern fp32 k6020;

RC_ctrl_t rc_ctrl;                 /* DBUS 驱动已删除，遥控器数据恒为 0 */
Comm_Typedef_Info Comm_Info;        /* 底盘模式（原 COMMON_TASK.c 定义） */
uint16_t pwr_t;

/* 云台下发的数据（gimbal_receiver_pack1/pack2，来自 CAN2 0x150/0x151） */
static float gimbal_yaw_angle = 0.0f;   /* 云台 yaw 角度制 0~360 */
static float gimbal_error_angle = 0.0f; /* 底盘/云台夹角 */
static float follow_yaw_v = 0.0f;       /* 云台下发的 yaw 角速度 rad/s */

extern PidTypeDef M3508_speedPID[4];
extern PidTypeDef Pid_Vw_A;
extern PidTypeDef Pid_Vw_V;
extern PidTypeDef Pid_Spin_V;

// ==========================================================
// 内部函数声明
// ==========================================================
void Chassis_Init(void);
static void Clear_Chassis_PID(void);
void Chassis_Taget_Update(void);
void Chassis_PidCal(void);
void Chassis_Cmd(void);
void Chassis_KeyMouse_Ctrl(void); // <--- 新增的键鼠控制函数声明（DBUS 已删，当前为死代码）

/* 角度归一化到 [-180, 180] */
static float limit_angle(float in)
{
    float angle = fmodf(in, 360.0f);
    if (angle < -180.0f) angle += 360.0f;
    if (angle > 180.0f) angle -= 360.0f;
    return angle;
}

/* 四面对称跟随：取前/后/左/右里绝对值最小的夹角 */
static float angle_z_err_get(float target_ang, float zeros_angle)
{
    float AngErr_front = limit_angle(zeros_angle - target_ang);
    float AngErr_back  = limit_angle(AngErr_front + 180.0f);
    float AngErr_left  = limit_angle(AngErr_front + GIMBAL_MOTOR_SIGN * 90.0f);
    float AngErr_right = limit_angle(AngErr_front - GIMBAL_MOTOR_SIGN * 90.0f);

    float minAngle = fminf(fabsf(AngErr_front),
                  fminf(fabsf(AngErr_back),
                  fminf(fabsf(AngErr_left), fabsf(AngErr_right))));

    if (fabsf(fabsf(AngErr_front) - minAngle) < 1e-6f) return AngErr_front;
    if (fabsf(fabsf(AngErr_back) - minAngle) < 1e-6f)  return AngErr_back;
    if (fabsf(fabsf(AngErr_left) - minAngle) < 1e-6f)  return AngErr_left;
    return AngErr_right;
}

// ==========================================================
// 1. 底盘任务主循环
// ==========================================================
void CHASSIS_TASK(void const * argument)
{
    Chassis_Init();//底盘初始化
    while(1)
    {
        Chassis_Taget_Update();//动学解算模块
        Chassis_PidCal();
        LossDetectAll();     /* 掉线检测 */
        Chassis_Cmd();
        osDelay(1);
    }
}

// ==========================================================
// 底盘初始化
// ==========================================================
void Chassis_Init()
{
    osDelay(1000);

    M3508_Travel_init();//麦轮速度环 ×4 PID
	  Chassis_BufferPower_init();//超电缓冲能量 PID
    Chassis_Follow_init();//跟随双环（角度环 + 速度环）
    Chassis_Spin_init();//小陀螺结构体初始化

    Chassis_Info.Last_Mode = 0xFF;//底盘失能
}

static void Clear_Chassis_PID(void)
{
    PID_clear(&M3508_speedPID[0]);
    PID_clear(&M3508_speedPID[1]);
    PID_clear(&M3508_speedPID[2]);
    PID_clear(&M3508_speedPID[3]);
    PID_clear(&Pid_Vw_A);
    PID_clear(&Pid_Vw_V);
    PID_clear(&Pid_Spin_V);
}

// ==========================================================
// 全局变量定义 (需要在头文件或别的.c文件里用 extern 声明以便调用)
// ==========================================================
uint8_t Auto_shoot = 0;
uint8_t Pitch_state = 0; // 新增：0为低头(正常)，1为抬头
uint8_t FIRE = 0;        // 原 COMMON_TASK.c 定义

// ==========================================================
// 【新增】操作手键鼠模式控制函数 (状态机解耦)
// 包含：WASD平移、F/C/B键状态切换、Shift加速/Ctrl减速、按键冲突急停、右键检测、V键摩擦轮
// 注：DBUS 驱动已删除，rc_ctrl 恒为 0，本函数当前不会被调用（死代码）
// ==========================================================
void Chassis_KeyMouse_Ctrl(void)
{
    static uint8_t f_key_last = 0;
    static uint8_t c_key_last = 0;
    static uint8_t b_key_last = 0;
    static uint8_t v_key_last = 0;

    // 内部状态机：0:调试模式, 1:底盘跟随, 2:小陀螺 (上电默认为0)
    static uint8_t pc_chassis_state = 0;

    uint8_t f_key_now = (rc_ctrl.key.v & KEY_PRESSED_OFFSET_F) ? 1 : 0;  //底盘跟随和小陀螺切换
    uint8_t c_key_now = (rc_ctrl.key.v & KEY_PRESSED_OFFSET_C) ? 1 : 0;  //底盘切调试模式，无旋转速度
    uint8_t b_key_now = (rc_ctrl.key.v & KEY_PRESSED_OFFSET_B) ? 1 : 0;  //一键抬头低头
    uint8_t v_key_now = (rc_ctrl.key.v & KEY_PRESSED_OFFSET_V) ? 1 : 0;  //获取V键当前状态

   // ---- 1. 模式切换状态机 (严格优先级路由) ----

    // 【最高优先级】：B 键 (强制一键跟随，配合云台抬头/低头)
    if (b_key_now == 1 && b_key_last == 0)
    {
        pc_chassis_state = 1; // 强制底盘切跟随

        // 翻转 Pitch_state：0变1，1变0
        if (Pitch_state == 0) Pitch_state = 1;
        else                  Pitch_state = 0;
    }
    // 【次高优先级】：C 键 (一键切回调试/脱离陀螺急停)
    else if (c_key_now == 1 && c_key_last == 0)
    {
        pc_chassis_state = 0;
    }
    // 【最低优先级】：F 键 (常规跟随与小陀螺互相切换)
    else if (f_key_now == 1 && f_key_last == 0)
    {
        if (pc_chassis_state == 1) pc_chassis_state = 2;
        else pc_chassis_state = 1;
    }

    // 更新底盘模式相关按键历史状态
    b_key_last = b_key_now;
    c_key_last = c_key_now;
    f_key_last = f_key_now;

    // 将内部状态映射到底盘全局模式
    if (pc_chassis_state == 0) Comm_Info.Chassis_Mode = Chassis_Tune;
    else if (pc_chassis_state == 1) Comm_Info.Chassis_Mode = Chassis_Follow;
    else if (pc_chassis_state == 2) Comm_Info.Chassis_Mode = Chassis_Spin;

    // ---- 2. V键控制摩擦轮 (边沿触发状态翻转) ----
    if (v_key_now == 1 && v_key_last == 0)
    {
        if (FIRE == 0) FIRE = 1;
        else           FIRE = 0;
    }
    v_key_last = v_key_now; // 更新V键历史状态

    // ---- 3. WASD 键盘速度映射 (带冲突急停与加减速逻辑) ----
    float key_vx = 0.0f;
    float key_vy = 0.0f;
    float key_speed = 2000.0f; // 键盘基准平移速度，后期可微调

    // Ctrl 减速优先级最高 (防超容/防飞车)，Shift 加速其次
    if (rc_ctrl.key.v & KEY_PRESSED_OFFSET_CTRL) {
        key_speed = 800.0f;  // Ctrl静步减速，省电容
    } else if (rc_ctrl.key.v & KEY_PRESSED_OFFSET_SHIFT) {  // Shift开超电加速
        key_speed = 4000.0f; // 平移速度增加
                             // 旋转速度增加(请在其他解算逻辑中配置目标旋转速度)
    }

    // 提取按键状态 (按下为 1，松开为 0)
    int8_t key_w = (rc_ctrl.key.v & KEY_PRESSED_OFFSET_W) ? 1 : 0;
    int8_t key_s = (rc_ctrl.key.v & KEY_PRESSED_OFFSET_S) ? 1 : 0;
    int8_t key_a = (rc_ctrl.key.v & KEY_PRESSED_OFFSET_A) ? 1 : 0;
    int8_t key_d = (rc_ctrl.key.v & KEY_PRESSED_OFFSET_D) ? 1 : 0;

    // 逻辑相减，计算最终方向因子 (-1, 0, 1)，完美抵消按键冲突
    key_vx = (key_w - key_s) * key_speed;
    key_vy = (key_a - key_d) * key_speed;

    // ---- 4. 赋值给底盘统一输入接口 ----
    Chassis_Info.Raw.Vx = key_vx;
    Chassis_Info.Raw.Vy = key_vy;

    // ---- 5. 鼠标右键检测 (更新Auto_shoot标志位) ----
    // press_r 为 1 表示按下，为 0 表示松开
    if (rc_ctrl.mouse.press_r != 0) {
        Auto_shoot = 1;
    } else {
        Auto_shoot = 0;
    }
}
// ==========================================================
// 2. 运动学解算模块
// 专门负责将输入指令转化为底盘的物理目标速度与目标角度
// ==========================================================
void Chassis_Taget_Update()
{
    // --------------------------------------------------------
    // (1) 云台数据解析（gimbal_receiver_pack1/pack2 来自 CAN2 0x150/0x151）
    // --------------------------------------------------------
	
		//角度制云台角度
    gimbal_yaw_angle = gimbal_receiver_pack1.yaw_pose / 65536.0f * 360.0f;
		//弧度制云台速度
    follow_yaw_v = gimbal_receiver_pack2.turn_yaw_speed * 0.01f * DegreesToRadians;

	
		//状态检测及运动刷新
    if (gimbal_receiver_pack1.robot_state == 1 && gimbal_comm_detector[0].state == DET_ON) /*（云台状态） 在线且控制模式 */
    {
				//坐标系转换
        Chassis_Info.Raw.Vx = gimbal_receiver_pack1.robot_speed_y / 100.0f * CHASSIS_TRANSLATE_SPEED_MAX; /* 向右 m/s */
        Chassis_Info.Raw.Vy = gimbal_receiver_pack1.robot_speed_x / 100.0f * CHASSIS_TRANSLATE_SPEED_MAX; /* 向前 m/s */

				//底盘状态刷新
        switch (gimbal_receiver_pack1.chassis_mode_action)
        {
        case 1: Comm_Info.Chassis_Mode = Chassis_Tune;   break; /* 仅底盘 */
        case 2: Comm_Info.Chassis_Mode = Chassis_Follow; break; /* 云台跟随 */
        case 3: Comm_Info.Chassis_Mode = Chassis_Spin;   break; /* 恒速旋转 */
        default: Comm_Info.Chassis_Mode = Chassis_OFF;   break;
        }
    }
    else//下电保护
    {
        Chassis_Info.Raw.Vx = 0;
        Chassis_Info.Raw.Vy = 0;
        Comm_Info.Chassis_Mode = Chassis_OFF;
    }

    // --------------------------------------------------------
    // (2) 底盘/云台夹角（四面对称取最小）
		//计算云台当前角度相对于机械零位的偏差，并限制在 [-180°, 180°] 范围内
    // 四面对称：底盘有对称结构，偏差 90° 和 270° 等价（取最小角度）
    // --------------------------------------------------------
    gimbal_error_angle = angle_z_err_get(gimbal_yaw_angle, GIMBAL_FOLLOW_ZERO);

    // --------------------------------------------------------
    // (3) 坐标系旋转（云台系 -> 底盘系）
    // --------------------------------------------------------
		//前方误差角度
    float angle_err_front = limit_angle(GIMBAL_MOTOR_SIGN * (gimbal_yaw_angle - GIMBAL_FOLLOW_ZERO));
		//此函数就是输入角度制，ai报错不用管
		arm_sin_cos_f32(angle_err_front, &Chassis_Info.Transform.SIN, &Chassis_Info.Transform.COS);
		
		//目标速度解算
    Chassis_Info.Target.Vx = (Chassis_Info.Raw.Vx * Chassis_Info.Transform.COS - Chassis_Info.Raw.Vy * Chassis_Info.Transform.SIN);
    Chassis_Info.Target.Vy = (Chassis_Info.Raw.Vy * Chassis_Info.Transform.COS + Chassis_Info.Raw.Vx * Chassis_Info.Transform.SIN);

    // --------------------------------------------------------
    // (4) 目标自旋速度 (Vw) 计算
    // --------------------------------------------------------
		//状态跳变清pid
    if (Comm_Info.Chassis_Mode != Chassis_Info.Last_Mode)
    {
        Clear_Chassis_PID();
        Chassis_Info.Last_Mode = Comm_Info.Chassis_Mode;
    }

    switch (Comm_Info.Chassis_Mode)
    {
        case Chassis_Follow:
        {
            float follow_feedback = PID_calc(&Pid_Vw_A, gimbal_error_angle, 0);
            float abs_err = fabsf(gimbal_error_angle);
            if (abs_err < 0.2f) follow_feedback = 0.0f;
            else if (abs_err < 1.0f) follow_feedback *= (abs_err - 0.2f) / (1.0f - 0.2f);
            Chassis_Info.Target.Vw = GIMBAL_MOTOR_SIGN * follow_feedback + follow_yaw_v;
            /* 限幅（同旧工程 LIMIT ±speed_yaw_max） */
            if (Chassis_Info.Target.Vw > CHASSIS_YAW_SPEED_MAX)        Chassis_Info.Target.Vw = CHASSIS_YAW_SPEED_MAX;
            else if (Chassis_Info.Target.Vw < -CHASSIS_YAW_SPEED_MAX)  Chassis_Info.Target.Vw = -CHASSIS_YAW_SPEED_MAX;
            break;
        }

        case Chassis_Spin:
            /* spin: robot_speed_w -> rad/s (open-loop, same as old CV_ROTATE) */
            Chassis_Info.Target.Vw = gimbal_receiver_pack1.robot_speed_w / 100.0f * CHASSIS_YAW_SPEED_MAX;
            break;

        case Chassis_Tune:
            /* 仅底盘：平移已在上方坐标旋转算出，这里只关自旋 */
            Chassis_Info.Target.Vw = 0;
            break;

        case Chassis_OFF:
        default:
            /* 停机：平移+自旋全清零（同旧工程 chassis_powerdown_control） */
            Chassis_Info.Target.Vx = 0;
            Chassis_Info.Target.Vy = 0;
            Chassis_Info.Target.Vw = 0;
            break;
    }

    // --------------------------------------------------------
    // (5) 麦轮逆运动学：底盘目标速度 -> 4 轮目标转速(deg/s)
    // --------------------------------------------------------
    mecanum_inv_kinematics(Chassis_Info.Target.Vx,
                           Chassis_Info.Target.Vy,
                           Chassis_Info.Target.Vw,
                           Chassis_Info.Target.Wheeldeg);
}

// ==========================================================
// 3. 闭环控制模块
// 包含角度误差计算、滞回死区逻辑以及各环 PID 运算（已去除余弦衰减）
// ==========================================================
void Chassis_PidCal(void)
{
    /* 4 个 3508 轮电机速度环：目标转速 = 逆运动学输出，反馈 = 电机实际转速 */
    for (int i = 0; i < 4; i++)
    {
        PID_calc(&M3508_speedPID[i], motor_chassis_3508[i].speed_rpm * M3508_RPM_TO_WHEEL_DEG_S, Chassis_Info.Target.Wheeldeg[i]);
    }
}

// ==========================================================
// 4. 指令下发模块
// 集中安全校验以及 CAN 报文发送
// ==========================================================
void Chassis_Cmd(void)
{
    // 判断底盘是否处于允许运动的状态
    if(gimbal_receiver_pack1.robot_state == 1)
    {
        CAN_cmd_chassis_travel(M3508_speedPID[0].out, M3508_speedPID[1].out,
                               M3508_speedPID[2].out, M3508_speedPID[3].out);
    }
    else
    {
        CAN_cmd_chassis_travel(0, 0, 0, 0);
    }
}
