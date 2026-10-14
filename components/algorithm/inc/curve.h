#ifndef CURVE_H
#define CURVE_H

#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"



typedef struct{

   float Now_Value;
   float Start_Value;
   float Now_Time;
	 float Use_Time;
   float Err_Value;

}Curve_Value_Info_Typedef;

typedef struct{
  
	Curve_Value_Info_Typedef  New;
  Curve_Value_Info_Typedef  Old;
  float dT;
	float Last_Set_Value;
	
}Curve_Info_Typedef;


extern void Curve_Init(Curve_Info_Typedef *Curve_Info);
extern void Curve_Calculate(Curve_Info_Typedef *Curve_Info,float Set_Value,float *Now_Value,float Value_Width);
#endif
