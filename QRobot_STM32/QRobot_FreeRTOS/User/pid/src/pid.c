#include "pid.h"


// 增量式PID
// pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)
// 只使用PI控制

float KP = 800;
float KI = 200;
float KD = 0;


float PID_Incremental(_Moto *MOTOR)
{
	float Error = 0; 		// 当前误差
	float P_Error = 0; 	// P项的误差
	float I_Error = 0; 	// I项的误差
	float D_Error = 0; 	// D项的误差
	
	Error = MOTOR->Target_Speed - MOTOR->Current_Speed; // 目标速度-当前速度
	
	// 计算PID的误差
	P_Error = Error - MOTOR->L_Error; // 当前误差 - 上次误差
	I_Error = Error; // 当前误差
	D_Error = Error - 2*MOTOR->L_Error + MOTOR->LL_Error; // 当前误差-2*上次误差+上上次误差
	
	// 计算最终输出的PWM值
	MOTOR->PWM_OUT += (KP * P_Error + KI * I_Error + KD * D_Error);
	
	// 更新误差
	MOTOR->LL_Error = MOTOR->L_Error;
	MOTOR->L_Error = Error;
	
	// 限幅
	if(MOTOR->PWM_OUT > PWM_OUT_LIMT)  MOTOR->PWM_OUT = PWM_OUT_LIMT;
	if(MOTOR->PWM_OUT < -PWM_OUT_LIMT) MOTOR->PWM_OUT = -PWM_OUT_LIMT;
	
	return MOTOR->PWM_OUT;
}





















