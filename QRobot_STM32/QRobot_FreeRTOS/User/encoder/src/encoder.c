#include "encoder.h"
#include "tim.h"
#include "system.h"



// ¶ÁÈ¡±àÂëÆ÷Âö³å²îÖµ
int16_t getTIMx_DetaCnt(TIM_HandleTypeDef *htim)
{	
	int16_t cnt;
	cnt = __HAL_TIM_GET_COUNTER(htim);
	__HAL_TIM_SetCounter(htim,0);
	return cnt;
}

// ¶ÁÈ¡±àÂëÆ÷Öµ
void Get_Velocity(void)
{
	Moto1.Encoder_Value = -getTIMx_DetaCnt(&htim1);	
	Moto2.Encoder_Value = -getTIMx_DetaCnt(&htim2);	
	Moto3.Encoder_Value = -getTIMx_DetaCnt(&htim3);	
	Moto4.Encoder_Value = getTIMx_DetaCnt(&htim4);	
	
	Moto1.Current_Speed = CURRENT_MOTO_SPEED(Moto1.Encoder_Value);
	Moto2.Current_Speed = CURRENT_MOTO_SPEED(Moto2.Encoder_Value);
	Moto3.Current_Speed = CURRENT_MOTO_SPEED(Moto3.Encoder_Value);
	Moto4.Current_Speed = CURRENT_MOTO_SPEED(Moto4.Encoder_Value);
}


