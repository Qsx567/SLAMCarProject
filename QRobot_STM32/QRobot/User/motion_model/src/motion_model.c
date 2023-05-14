#include "motion_model.h"
#include "system.h"

//机器人运动模型


//------- 四轮模型 ------- //

// 运动学逆解
void FourWheel_car_Motion_Inverse(float vx, float w)
{
	Moto1.Target_Speed = vx - w * (ROBOT_WHEEL_WIDTH + ROBOT_WHEEL_HIGHT) * 0.5;
	Moto2.Target_Speed = vx + w * (ROBOT_WHEEL_WIDTH + ROBOT_WHEEL_HIGHT) * 0.5;
	Moto3.Target_Speed = vx - w * (ROBOT_WHEEL_WIDTH + ROBOT_WHEEL_HIGHT) * 0.5;
	Moto4.Target_Speed = vx + w * (ROBOT_WHEEL_WIDTH + ROBOT_WHEEL_HIGHT) * 0.5;
}

