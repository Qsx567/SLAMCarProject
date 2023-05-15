#ifndef __SERIAL_H_
#define __SERIAL_H_
#include "main.h"

#define PROTOCOL_HEADER 0x5BB5 // 协议包头
#define PROTOCOL_END    0x5AA5 // 协议包尾

#define PROTOCOL_DATA_SIZE 68  //数据包大小

#pragma pack(1) // 设置结构体的边界对齐为1个字节

// 电机当前速度和目标速度结构体
typedef struct _Moto_str
{
	float Moto_CurrentSpeed;
	float Moto_TargetSpeed;
}Moto_str;

// MPU6050结构体
typedef struct _MPU6050_Str
{
	short X_data;
	short Y_data;
	short Z_data;
}MPU6050_Str;

//ROS和STM32串口通信的数据包
typedef union _Upload_Data
{
	uint8_t buffer[PROTOCOL_DATA_SIZE]; // 数据包buffer
	struct _Sensor_str
	{
		// 数据包头
		uint16_t Header; 				// 5B B5
		
		// 前向运动学Vx,w
		float X_speed;	 				// 00 00 00 00 
		float Z_speed;   				// 00 00 00 00 
		
		// 加速度值和陀螺仪值
		MPU6050_Str Accelerometer; // C4 06 F0 FD 6C 38
		MPU6050_Str Gyroscope;     // D8 FF 03 00 EB FF
		
		// 电机的当前速度值和目标速度值
		Moto_str MotoStr[4]; 		//M1 00 00 00 00  00 00 00 00
														//M2 00 00 00 00  00 00 00 00
														//M3 00 00 00 00  00 00 00 00 
														//M4 00 00 00 00  00 00 00 00
		
		// PID参数值
		float PID_Param[3];			//P 00 00 00 00
														//I 00 00 00 00
														//D 00 00 00 00
		
		// 数据包尾
		uint16_t End;						// 5A A5
	}Sensor_str;
}Upload_Data;

#pragma pack(4)

extern Upload_Data Send_Data, Recive_Data;
extern uint8_t uart5_rxbuff;
void STM32_TO_ROS(void);

#endif
