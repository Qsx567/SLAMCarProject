#ifndef __ENCODER_H_
#define __ENCODER_H_

#define PI_v 3.1415926f 							// PI
#define ROBOT_WHEEL_DIAMETER 0.13f			// ����ֱ��(m) 130mm
#define COUNT_FREQUENCY      100.0f      // 100Hz --> 10ms
//#define ENCODER_COUNT_VALUE  1320.0f		// ����ÿȦ�������� 11*30*4 �ֱ���11*���ٱ�30*��Ƶϵ��4   --->520���������������
#define ENCODER_COUNT_VALUE  60000.0f		// ����ÿȦ�������� 500*30*4 �ֱ���500*���ٱ�30*��Ƶϵ��4     --->MG513�����GMR������

#define CURRENT_MOTO_SPEED(x)		(PI_v * ROBOT_WHEEL_DIAMETER * COUNT_FREQUENCY * x / ENCODER_COUNT_VALUE)  



void Get_Velocity(void);

#endif
