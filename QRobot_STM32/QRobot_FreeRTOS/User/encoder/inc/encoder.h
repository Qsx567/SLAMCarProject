#ifndef __ENCODER_H_
#define __ENCODER_H_

#define PI_v 3.1415926f 							// PI
#define ROBOT_WHEEL_DIAMETER 0.13f			// 轮子直径(m) 130mm
#define COUNT_FREQUENCY      100.0f      // 100Hz --> 10ms
//#define ENCODER_COUNT_VALUE  1320.0f		// 轮子每圈总脉冲数 11*30*4 分辨率11*减速比30*分频系数4   --->520电机带霍尔编码器
#define ENCODER_COUNT_VALUE  60000.0f		// 轮子每圈总脉冲数 500*30*4 分辨率500*减速比30*分频系数4     --->MG513电机带GMR编码器

#define CURRENT_MOTO_SPEED(x)		(PI_v * ROBOT_WHEEL_DIAMETER * COUNT_FREQUENCY * x / ENCODER_COUNT_VALUE)  



void Get_Velocity(void);

#endif
