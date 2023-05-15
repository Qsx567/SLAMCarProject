#ifndef __SYSTEM_H_
#define __SYSTEM_H_

#define ROBOT_WHEEL_WIDTH		0.243		// �������־�,��λ��m
#define ROBOT_WHEEL_HIGHT		0.163		// С��ǰ��������,��λ��m

#define PWM_OUT_LIMT 				7200		// PWM���ֵ

typedef struct
{
	int Encoder_Value;   // ������ֵ
	float Current_Speed; // ��ǰ�ٶ�ֵ
	float Target_Speed;  // Ŀ���ٶ�ֵ
	
	float L_Error;			 // �ϴ����ֵ
	float LL_Error;			 // ���ϴ����
	
	float PWM_OUT;  		 // �����PWMֵ
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
