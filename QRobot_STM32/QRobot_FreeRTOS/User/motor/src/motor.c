#include "motor.h"
#include "tim.h"


void Motor_Control(float Motor1, float Motor2, float Motor3, float Motor4)
{
	// MOTOR1
	if (Motor1 > 0 ) {
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 3600 + ((int)(Motor1/2)) );
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
	}
	if (Motor1 < 0 ) {
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 3600 - ((int)(Motor1/2)));
	}
	
	// MOTOR2
	if (Motor2 > 0 ) {
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, 3600 + ((int)(Motor2/2)) );
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 0);
	}
	if (Motor2 < 0 ) {
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, 0);
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 3600 - ((int)(Motor2/2)));
	}
	
	// MOTOR3
	if (Motor3 > 0 ) {
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 3600 + ((int)(Motor3/2)) );
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, 0);
	}
	if (Motor3 < 0 ) {
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 0);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, 3600 - ((int)(Motor3/2)));
	}
	
	// MOTOR4
	if (Motor4 > 0 ) {
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 3600 + ((int)(Motor4/2)) );
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 0);
	}
	if (Motor4 < 0 ) {
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 0);
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 3600 - ((int)(Motor4/2)));
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
