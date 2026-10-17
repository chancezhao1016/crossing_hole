/**
  ******************************************************************************
  * @file    pid.c
  * @author  Hongxi Wong
  * @version V1.0.6
  * @date    2019/12/17
  * @brief   对每一个pid结构体都要先进行函数的连接，再进行初始化
  ******************************************************************************
  * @attention 
  *
	梯形积分（Trapezoid_Intergral）：使用梯形积分代替矩形积分，提高积分项的精度。

  变积分（ChangingIntegralRate）：根据误差大小动态调整积分系数，减少超调。

  积分限幅（Integral_Limit）：防止积分项过大，避免积分饱和。

  微分先行（Derivative_On_Measurement）：对测量值进行微分，而不是对误差微分，可以减少设定值变化引起的微分冲击。

  微分滤波（DerivativeFilter）：对微分项进行低通滤波，减少高频噪声的影响。

  输出滤波（OutputFilter）：对输出进行低通滤波，使输出更平滑。

  误差处理（ErrorHandle）：可以检测电机堵转等异常情况。
  ******************************************************************************
	
  */
#include "pid.h"

/***************************PID param initialize******************************/
static void PID_param_init_Imp(
    PID_TypeDef_Imp *pid,
    uint16_t max_out,
    uint16_t intergral_limit,
    float deadband,

    float kp,
    float Ki,
    float Kd,

    float Changing_Integral_A,  
    float Changing_Integral_B,  

    float output_filtering_coefficient,
    float derivative_filtering_coefficient,

    uint8_t improve)
{
    pid->DeadBand = deadband;
    pid->IntegralLimit = intergral_limit;
    pid->MaxOut = max_out;
    pid->MaxErr = max_out * 2;
    pid->Target = 0;

    pid->Kp = kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->ITerm = 0;

    pid->ScalarA = Changing_Integral_A;
    pid->ScalarB = Changing_Integral_B;

    pid->Output_Filtering_Coefficient = output_filtering_coefficient;

    pid->Derivative_Filtering_Coefficient = derivative_filtering_coefficient;

    pid->Improve = improve;

    pid->ERRORHandler.ERRORCount = 0;
    pid->ERRORHandler.ERRORType = PID_ERROR_NONE;

    pid->Output = 0;
}

/**************************PID param reset*********************************/
static void f_PID_reset_Imp(PID_TypeDef_Imp *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    if (pid->Ki == 0)
        pid->Iout = 0;
}

/***************************PID calculate**********************************/
float PID_Calculate_Imp(PID_TypeDef_Imp *pid, float measure, float target)
{
    if (pid->Improve & ErrorHandle) //ErrorHandle
    {
        f_PID_ErrorHandle(pid); // last_xxx = xxx
        if (pid->ERRORHandler.ERRORType != PID_ERROR_NONE)
        {
            pid->Output = 0;
            return 0; //Catch ERROR
        }
    }

    pid->Measure = measure;
    pid->Target = target;
    pid->Err = pid->Target - pid->Measure;

    if (ABS(pid->Err) > pid->DeadBand)
    {
        pid->Pout = pid->Kp * pid->Err;
        pid->ITerm = pid->Ki * pid->Err;
        pid->Dout = pid->Kd * (pid->Err - pid->Last_Err);

        //Trapezoid Intergral
        if (pid->Improve & Trapezoid_Intergral)
            f_Trapezoid_Intergral(pid);
        //Changing Integral Rate
        if (pid->Improve & ChangingIntegralRate)
            f_Changing_Integral_Rate(pid);
        //Integral limit
        if (pid->Improve & Integral_Limit)
            f_Integral_Limit(pid);
        //Derivative On Measurement
        if (pid->Improve & Derivative_On_Measurement)
            f_Derivative_On_Measurement(pid);
        //Derivative filter
        if (pid->Improve & DerivativeFilter)
            f_Derivative_Filter(pid);

        pid->Iout += pid->ITerm;

        pid->Output = pid->Pout + pid->Iout + pid->Dout;

        //Output Filter
        if (pid->Improve & OutputFilter)
            f_Output_Filter(pid);

        //Output limit
        f_Output_Limit(pid);

        //Proportional limit
        f_Proportion_Limit(pid);
    }
    pid->Last_Measure = pid->Measure;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;
    pid->Last_Err = pid->Err;

    return pid->Output;
}

/*****************PID Improvement Function*********************/
static void f_Trapezoid_Intergral(PID_TypeDef_Imp *pid)
{
    pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2);
}

static void f_Changing_Integral_Rate(PID_TypeDef_Imp *pid)
{
    if (pid->Err * pid->Iout > 0)
    {
        //Integral still increasing
        if (ABS(pid->Err) <= pid->ScalarB)
            return; //Full integral
        if (ABS(pid->Err) <= (pid->ScalarA + pid->ScalarB))
            pid->ITerm *= (pid->ScalarA - ABS(pid->Err) + pid->ScalarB) / pid->ScalarA;
        else
            pid->ITerm = 0;
    }
}

static void f_Integral_Limit(PID_TypeDef_Imp *pid)
{
    float temp_Output, temp_Iout;
    temp_Iout = pid->Iout + pid->ITerm;
    temp_Output = pid->Pout + pid->Iout + pid->Dout;
    if (ABS(temp_Output) > pid->MaxOut)
    {
        if (pid->Err * pid->Iout > 0)
        {
            //Integral still increasing
            pid->ITerm = 0;
        }
    }

    if (temp_Iout > pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = pid->IntegralLimit;
    }
    if (temp_Iout < -pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = -pid->IntegralLimit;
    }
}

static void f_Derivative_On_Measurement(PID_TypeDef_Imp *pid)
{
    pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure);
}

static void f_Derivative_Filter(PID_TypeDef_Imp *pid)
{
    pid->Dout = pid->Dout * pid->Derivative_Filtering_Coefficient +
                pid->Last_Dout * (1 - pid->Derivative_Filtering_Coefficient);
}

static void f_Output_Filter(PID_TypeDef_Imp *pid)
{
    pid->Output = pid->Output * pid->Output_Filtering_Coefficient +
                  pid->Last_Output * (1 - pid->Output_Filtering_Coefficient);
}

static void f_Output_Limit(PID_TypeDef_Imp *pid)
{
    if (pid->Output > pid->MaxOut)
    {
        pid->Output = pid->MaxOut;
    }
    if (pid->Output < -(pid->MaxOut))
    {
        pid->Output = -(pid->MaxOut);
    }
}

static void f_Proportion_Limit(PID_TypeDef_Imp *pid)
{
    //Proportion limit is insignificant in control process
    //but it makes variable chart look better
    if (pid->Pout > pid->MaxOut)
    {
        pid->Pout = pid->MaxOut;
    }
    if (pid->Pout < -(pid->MaxOut))
    {
        pid->Pout = -(pid->MaxOut);
    }
}

/*****************PID ERRORHandle Function*********************/
static void f_PID_ErrorHandle(PID_TypeDef_Imp *pid)
{
    /*Motor Blocked Handle*/
    if (pid->Output < pid->MaxOut * 0.01f)
        return;

    if ((ABS(pid->Target - pid->Measure) / pid->Target) > 0.9f)
    {
        //Motor blocked counting
        pid->ERRORHandler.ERRORCount++;
    }
    else
    {
        pid->ERRORHandler.ERRORCount = 0;
    }

    if (pid->ERRORHandler.ERRORCount > 1000)
    {
        //Motor blocked over 1000times
        pid->ERRORHandler.ERRORType = Motor_Blocked;
    }
}

/*****************PID structure initialize*********************/
void PID_Init_Imp(
    PID_TypeDef_Imp *pid,
    uint16_t max_out,
    uint16_t intergral_limit,
    float deadband,

    float kp,
    float Ki,
    float Kd,

    float A,
    float B,

    float output_filtering_coefficient,
    float derivative_filtering_coefficient,
    uint8_t improve)
{
    pid->PID_param_init_Imp = PID_param_init_Imp;
    pid->PID_reset = f_PID_reset_Imp;
    pid->PID_param_init_Imp(pid, max_out, intergral_limit, deadband,
                        kp, Ki, Kd, A, B, output_filtering_coefficient, derivative_filtering_coefficient, improve);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////NORM-PID//////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }
void PID_init(PidTypeDef *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->enable = 1;  // 默认使能
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}
fp32 PID_calc(PidTypeDef *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->Ref = set;
    pid->Measure = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

void pid_init(PidTypeDef *pid, fp32 kp, fp32 ki, fp32 kd, fp32 max_out, fp32 max_iout)
{
    if (pid == NULL)
    {
        return;
    }
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}
fp32 pid_calc(PidTypeDef *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->Ref = set;
    pid->Measure = ref;
    pid->error[0] = set - ref;
    pid->Pout = pid->Kp * pid->error[0];
    pid->Iout += pid->Ki * pid->error[0];
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
    pid->Dout = pid->Kd * pid->Dbuf[0];
    LimitMax(pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    LimitMax(pid->out, pid->max_out);
    return pid->out;
}

void Handle_Over_Zero(float *set, float *ref, float T)
{
    if (*set - *ref > (T / 2)) // 4096 ：半圈机械角度
    {
        *ref += T;
    }
    else if (*set - *ref < -(T / 2))
    {
        *ref = *ref - T;
    }
}

fp32 PID_Calc_over_zero(PidTypeDef *pid, fp32 ref, fp32 set, float T)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
    Handle_Over_Zero(&set, &ref, T);
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->Ref = set;
    pid->error[0] = set - ref;
    pid->Pout = pid->Kp * pid->error[0];
    pid->Iout += pid->Ki * pid->error[0];
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
    pid->Dout = pid->Kd * pid->Dbuf[0];
    LimitMax(pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    LimitMax(pid->out, pid->max_out);
    
    return pid->out;
}

void pid_clear(PidTypeDef *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->Measure = pid->Ref = 0.0f;
}
//
fp32 PID_Calc(PidTypeDef *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
		
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->Ref = set;
    pid->Measure = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

fp32 PID_Calc_Ecd(PidTypeDef *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->Ref = set;
    pid->Measure = ref;
		//过零处理
    pid->error[0] = ecd_zero(set, ref);
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

fp32 last_angle, this_angle, angle_time;
fp32 PID_Calc_Angle(PidTypeDef *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->Ref = set;
    pid->Measure = ref;
		
		//过零处理
    pid->error[0] = angle_zero(set, ref);
		
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}
fp32 PID_Calc_Ang(PidTypeDef *pid, fp32 ref, fp32 set)
{

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->Ref = set;
    pid->Measure = ref;
		
		//过零处理
    pid->error[0] = set - ref;
		if(pid->error[0]>180)
		{
			 pid->error[0] -= 360;
		}
		else if(pid->error[0] <-180)
		{
			 pid->error[0] += 360;
		}
			pid->Pout = pid->Kp * pid->error[0];
			pid->Iout += pid->Ki * pid->error[0];
			pid->Dbuf[2] = pid->Dbuf[1];
			pid->Dbuf[1] = pid->Dbuf[0];
			pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
			pid->Dout = pid->Kd * pid->Dbuf[0];
			LimitMax(pid->Iout, pid->max_iout);
			pid->out = pid->Pout + pid->Iout + pid->Dout;
			LimitMax(pid->out, pid->max_out);
   
    return pid->out;
}

void PID_clear(PidTypeDef *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->Measure = pid->Ref = 0.0f;
}

//过零函数
static fp32 ecd_zero(uint16_t ecd, uint16_t offset_ecd)
{
		int32_t relative_ecd = ecd - offset_ecd;

		if(relative_ecd > half_ecd_range)
		{
					relative_ecd -= ecd_range;
		}
		if(relative_ecd < - half_ecd_range)
		{
					relative_ecd += ecd_range;
		}

		return relative_ecd;
}

fp32 angle_zero(fp32 angle, fp32 offset_angle)
{
		fp32 relative_angle = angle - offset_angle;

		if(relative_angle >  1.25f * PI)
		{
					relative_angle -= 2*PI;
		}
		else if(relative_angle < - 1.25f * PI)
		{
					relative_angle += 2*PI;		
		}

		return relative_angle;
}

