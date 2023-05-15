#ifndef __SERIAL_H_
#define __SERIAL_H_
#include "main.h"

#define PROTOCOL_HEADER 0x5BB5 // Э���ͷ
#define PROTOCOL_END    0x5AA5 // Э���β

#define PROTOCOL_DATA_SIZE 68  //���ݰ���С

#pragma pack(1) // ���ýṹ��ı߽����Ϊ1���ֽ�

// �����ǰ�ٶȺ�Ŀ���ٶȽṹ��
typedef struct _Moto_str
{
	float Moto_CurrentSpeed;
	float Moto_TargetSpeed;
}Moto_str;

// MPU6050�ṹ��
typedef struct _MPU6050_Str
{
	short X_data;
	short Y_data;
	short Z_data;
}MPU6050_Str;

//ROS��STM32����ͨ�ŵ����ݰ�
typedef union _Upload_Data
{
	uint8_t buffer[PROTOCOL_DATA_SIZE]; // ���ݰ�buffer
	struct _Sensor_str
	{
		// ���ݰ�ͷ
		uint16_t Header; 				// 5B B5
		
		// ǰ���˶�ѧVx,w
		float X_speed;	 				// 00 00 00 00 
		float Z_speed;   				// 00 00 00 00 
		
		// ���ٶ�ֵ��������ֵ
		MPU6050_Str Accelerometer; // C4 06 F0 FD 6C 38
		MPU6050_Str Gyroscope;     // D8 FF 03 00 EB FF
		
		// ����ĵ�ǰ�ٶ�ֵ��Ŀ���ٶ�ֵ
		Moto_str MotoStr[4]; 		//M1 00 00 00 00  00 00 00 00
														//M2 00 00 00 00  00 00 00 00
														//M3 00 00 00 00  00 00 00 00 
														//M4 00 00 00 00  00 00 00 00
		
		// PID����ֵ
		float PID_Param[3];			//P 00 00 00 00
														//I 00 00 00 00
														//D 00 00 00 00
		
		// ���ݰ�β
		uint16_t End;						// 5A A5
	}Sensor_str;
}Upload_Data;

#pragma pack(4)

extern Upload_Data Send_Data, Recive_Data;
extern uint8_t uart5_rxbuff;
void STM32_TO_ROS(void);

#endif
