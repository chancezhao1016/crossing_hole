/**
  ******************************************************************************
  * @file	 pid.h
  * @author  Hongxi Wong
  * @version V1.0.6
  * @date    2019/12/17
  * @brief   
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#ifndef _PID_H
#define _PID_H

#include "stdint.h"

#define ABS(x) ((x > 0) ? x : -x)

typedef enum pid_Improvement_e
{
    NONE = 0X00,                        //0000 0000
    Integral_Limit = 0x01,              //0000 0001
    Derivative_On_Measurement = 0x02,   //0000 0010
    Trapezoid_Intergral = 0x04,         //0000 0100
    Proportional_On_Measurement = 0x08, //0000 1000
    OutputFilter = 0x10,                //0001 0000
    ChangingIntegralRate = 0x20,        //0010 0000
    DerivativeFilter = 0x40,            //0100 0000
    ErrorHandle = 0x80,                 //1000 0000
} PID_Improvement_e;

typedef enum errorType_e
{
    PID_ERROR_NONE = 0x00U,
    Motor_Blocked = 0x01U
} ErrorType_e;

typedef struct
{
    uint64_t ERRORCount;
    ErrorType_e ERRORType;
} PID_ErrorHandler_t;

typedef struct _PID_TypeDef_Imp
{
    float Target;              //目标值
    float LastNoneZeroTarget;  //上一次非零目标值
    float Kp;                  //PID三个参数
    float Ki;                  
    float Kd;

    float Measure;             //当前测量值
    float Last_Measure;        //上次测量值
    float Err;                 //当前误差
    float Last_Err;            //上次误差
                  
    float Pout;                //PID三个分量输出
    float Iout;                
    float Dout;                
    float ITerm;               //积分累计项ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B

    float Output;              //当前总输出
    float Last_Output;         //上次总输出
    float Last_Dout;           //
 
    float MaxOut;              //输出最大值限制
    float IntegralLimit;       //积分限幅值
    float DeadBand;            //死区范围
    float ControlPeriod;       //控制周期
    float MaxErr;              //最大误差限制
    float ScalarA;             //积分分离参数For Changing Integral
    float ScalarB;             //ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
    float Output_Filtering_Coefficient;      //输出滤波系数
    float Derivative_Filtering_Coefficient;  //微分滤波系数

    uint32_t thistime;
    uint32_t lasttime;
    uint8_t dtime;

    uint8_t Improve;

    PID_ErrorHandler_t ERRORHandler;

    void (*PID_param_init_Imp)(
        struct _PID_TypeDef_Imp *pid,
        uint16_t maxOut,
        uint16_t integralLimit,
        float deadband,
        float Kp,
        float ki,
        float kd,
        float A,
        float B,
        float output_filtering_coefficient,
        float derivative_filtering_coefficient,
        uint8_t improve);

    void (*PID_reset)(
        struct _PID_TypeDef_Imp *pid,
        float Kp,
        float ki,
        float kd);
} PID_TypeDef_Imp;

static void f_Trapezoid_Intergral(PID_TypeDef_Imp *pid);
static void f_Integral_Limit(PID_TypeDef_Imp *pid);
static void f_Derivative_On_Measurement(PID_TypeDef_Imp *pid);
static void f_Changing_Integral_Rate(PID_TypeDef_Imp *pid);
static void f_Output_Filter(PID_TypeDef_Imp *pid);
static void f_Derivative_Filter(PID_TypeDef_Imp *pid);
static void f_Output_Limit(PID_TypeDef_Imp *pid);
static void f_Proportion_Limit(PID_TypeDef_Imp *pid);
static void f_PID_ErrorHandle(PID_TypeDef_Imp *pid);

void PID_Init_Imp(
    PID_TypeDef_Imp *pid,
    uint16_t max_out,
    uint16_t intergral_limit,
    float deadband,

    float kp,
    float ki,
    float kd,

    float A,
    float B,

    float output_filtering_coefficient,
    float derivative_filtering_coefficient,

    uint8_t improve);
float PID_Calculate_Imp(PID_TypeDef_Imp *pid, float measure, float target);
		
////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////NORM-PID//////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
#define ecd_range 8191
#define half_ecd_range 4096

#define PI 3.1415926535f

#include "main.h"
enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef float fp32;

typedef struct
{
    uint8_t mode;
    uint8_t enable;  // 开关：1=使能 0=禁用
    //PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;
    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 Ref;      //参考值（目标值）
    fp32 Measure;  //实际值（测量值/反馈）

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次

} PidTypeDef;
extern void pid_init(PidTypeDef *pid, fp32 kp, fp32 ki, fp32 kd, fp32 max_out, fp32 max_iout);
extern void PID_init(PidTypeDef *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);
extern fp32 PID_calc(PidTypeDef *pid, fp32 ref, fp32 set);

extern fp32 PID_Calc(PidTypeDef *pid, fp32 ref, fp32 set);
extern fp32 PID_Calc_Ecd(PidTypeDef *pid, fp32 ref, fp32 set);
extern fp32 PID_Calc_Angle(PidTypeDef *pid, fp32 ref, fp32 set);
extern void PID_clear(PidTypeDef *pid);
fp32 PID_Calc_Ang(PidTypeDef *pid, fp32 ref, fp32 set);
void Ecd_zero(float *set,float *ref);
static fp32 ecd_zero(uint16_t ecd, uint16_t offset_ecd);
fp32 angle_zero(fp32 angle, fp32 offset_angle);		
extern fp32 pid_calc(PidTypeDef *pid, fp32 ref, fp32 set);		
		
		
		
		
		
#endif
