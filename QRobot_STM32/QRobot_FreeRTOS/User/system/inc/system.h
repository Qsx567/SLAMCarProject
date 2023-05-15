#ifndef __SYSTEM_H_
#define __SYSTEM_H_

#define ROBOT_WHEEL_WIDTH		0.243		// 主动轮轮距,单位：m
#define ROBOT_WHEEL_HIGHT		0.163		// 小车前后轴的轴距,单位：m

#define PWM_OUT_LIMT 				7200		// PWM最大值

typedef struct
{
	int Encoder_Value;   // 编码器值
	float Current_Speed; // 当前速度值
	float Target_Speed;  // 目标速度值
	
	float L_Error;			 // 上次误差值
	float LL_Error;			 // 上上次误差
	
	float PWM_OUT;  		 // 输出的PWM值
}_Moto;

typedef struct
{
	short X_data;
	short Y_data;
	short Z_data;
}_MPU6050;

typedef struct
{
	float X_speed;
	float Z_speed;
}_PS2;




extern _Moto Moto1,Moto2,Moto3,Moto4;
extern _MPU6050 acc,gyro;
extern _PS2 ps2;

#endif
