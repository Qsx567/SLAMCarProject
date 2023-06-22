#include "motor.h"
#include "tim.h"

// 满占空比是7199,限幅7100
void Motor_Control(int Motor1, int Motor2, int Motor3, int Motor4)
{
	// MOTOR1
	if (Motor1 < 0 ) {
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 7199 + Motor1);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 7199);
	}
	else {
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 7199);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 7199 - Motor1);
	}
	
	// MOTOR2
	if (Motor2 < 0 ) {
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, 7199 + Motor2);
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 7199);
	}
	else {
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, 7199);
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 7199 - Motor2);
	}
	
	// MOTOR3
	if (Motor3 < 0 ) {
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 7199 + Motor3);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, 7199);
	}
	else {
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 7199);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, 7199 - Motor3);
	}
	
	// MOTOR4
	if (Motor4 < 0 ) {
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 7199 + Motor4);
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 7199);
	}
	else {
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 7199);
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 7199 - Motor4);
	}
}

void Motor_Stop()
{
	// MOTOR1
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 7200);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 7200);

	// MOTOR2
	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, 7200);
	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 7200);

	// MOTOR3
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 7200);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, 7200);

	// MOTOR4
	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 7200);
	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 7200);
}
