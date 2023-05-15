#include "motion_model.h"
#include "system.h"
#include "serial.h"

//机器人运动模型


//------- 四轮模型 ------- //

// 运动学逆解 -- > M1 M2 M3 M4
void FourWheel_car_Motion_Inverse(float vx, float w)
{
	Moto1.Target_Speed = vx - w * (ROBOT_WHEEL_WIDTH + ROBOT_WHEEL_HIGHT) * 0.5;
	Moto2.Target_Speed = vx + w * (ROBOT_WHEEL_WIDTH + ROBOT_WHEEL_HIGHT) * 0.5;
	Moto3.Target_Speed = vx - w * (ROBOT_WHEEL_WIDTH + ROBOT_WHEEL_HIGHT) * 0.5;
	Moto4.Target_Speed = vx + w * (ROBOT_WHEEL_WIDTH + ROBOT_WHEEL_HIGHT) * 0.5;
}

// 运动学正解 -- > Vx,w
void FourWheel_car_Motion_Positive(float v1, float v2, float v3, float v4)
{
	Send_Data.Sensor_str.X_speed = (v1 + v2 + v3 + v4) / 4;
	Send_Data.Sensor_str.Z_speed = (-v1 + v2 - v3 + v4) / (4.0f*(ROBOT_WHEEL_HIGHT+ROBOT_WHEEL_WIDTH)*0.5f);
}

