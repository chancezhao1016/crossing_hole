
/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : curve.c
 * @brief          : curve functions
 * @author         : none
 * @date           : none
 * @version        : none
 ******************************************************************************
 * @attention      : 
 ******************************************************************************
 用于生成平滑S型曲线的算法库，基于五阶多项式实现，适用于需要平滑过渡的运动控制、信号生成、动画效果等场景。通过该库可以生成无冲击、加速度连续的运动轨迹。
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "curve.h"

#define VAL_LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))


void Curve_Init(Curve_Info_Typedef *Curve_Info){

   memset(&Curve_Info->New,0,sizeof(Curve_Info->New));
   memset(&Curve_Info->Old,0,sizeof(Curve_Info->Old));
   
	 Curve_Info->dT = 0.001f;
	 Curve_Info->Last_Set_Value = 0;
}

float FivePower(float Now_Time,float Use_Time){

   float Time = 0;
	 Time = Now_Time/Use_Time;
	 
	 return 10.f*powf(Time,3.f) - 15.f*powf(Time,4.f) + 6*powf(Time,5.f); 

}

void Curve_Calculate(Curve_Info_Typedef *Curve_Info,float Set_Value,float *Now_Value,float Value_Width){

   if(Set_Value != Curve_Info->Last_Set_Value){
			
		  if(Curve_Info->New.Now_Time == Curve_Info->New.Use_Time){
			
          if(Curve_Info->Old.Now_Time < Curve_Info->Old.Use_Time){
				     	
						 Curve_Info->New.Now_Time = 0;
						 Curve_Info->New.Err_Value = Set_Value -  Curve_Info->Old.Err_Value;
						 Curve_Info->New.Start_Value =0;
					   Curve_Info->Last_Set_Value = Set_Value;
						 Curve_Info->New.Use_Time = Value_Width;
					}else{
					   memset(&Curve_Info->New,0,sizeof(Curve_Info->New));
             Curve_Info->Old.Now_Time = 0; 
					   Curve_Info->Old.Err_Value = Set_Value - *Now_Value;
					   Curve_Info->Old.Start_Value = *Now_Value;
					   Curve_Info->Last_Set_Value = Set_Value;
						 Curve_Info->Old.Use_Time = Value_Width;
					}   
			}
	 }
	 
	 if(Curve_Info->Old.Use_Time != 0){
	    
		  Curve_Info->Old.Now_Time += Curve_Info->dT;
	    Curve_Info->Old.Now_Time = VAL_LIMIT(Curve_Info->Old.Now_Time,0.f,Curve_Info->Old.Use_Time);
	    Curve_Info->Old.Now_Value = Curve_Info->Old.Start_Value + Curve_Info->Old.Err_Value*FivePower(Curve_Info->Old.Now_Time,Curve_Info->Old.Use_Time);
	 }
   
	  if(Curve_Info->New.Use_Time != 0){
	    
		  Curve_Info->New.Now_Time += Curve_Info->dT;
	    Curve_Info->New.Now_Time = VAL_LIMIT(Curve_Info->New.Now_Time,0.f,Curve_Info->New.Use_Time);
	    Curve_Info->New.Now_Value = Curve_Info->New.Start_Value + Curve_Info->New.Err_Value*FivePower(Curve_Info->New.Now_Time,Curve_Info->New.Use_Time);
	 }
		
	 
    *Now_Value = Curve_Info->New.Now_Value + Curve_Info->Old.Now_Value;

   
}







