#ifndef __SYSTEM_H_
#define __SYSTEM_H_

typedef struct
{
	int Encoder_Value;   // 编码器值
	float Current_Speed; // 当前速度值
	float Target_Speed;  // 目标速度值
	
	float L_Error;			 // 上次误差值
	float LL_Error;			 // 上上次误差
	
	float PWM_OUT;  		 // 输出的PWM值
}_Moto;

extern _Moto Moto1,Moto2,Moto3,Moto4;

#endif
