#ifndef __SYSTEM_H_
#define __SYSTEM_H_

typedef struct
{
	int Encoder_Value;   // ������ֵ
	float Current_Speed; // ��ǰ�ٶ�ֵ
	float Target_Speed;  // Ŀ���ٶ�ֵ
	
	float L_Error;			 // �ϴ����ֵ
	float LL_Error;			 // ���ϴ����
	
	float PWM_OUT;  		 // �����PWMֵ
}_Moto;

extern _Moto Moto1,Moto2,Moto3,Moto4;

#endif
