#ifndef __ENCODER_H_
#define __ENCODER_H_

#define PI_v 3.1415926f 							// PI
#define ROBOT_WHEEL_DIAMETER 0.13f			// 轮子直径(m)
#define COUNT_FREQUENCY      100.0f      // 100Hz --> 10ms
#define ENCODER_COUNT_VALUE  1320.0f		// 轮子每圈总脉冲数 11*30*4 分辨率11*减速比30*分频系数4

#define CURRENT_MOTO_SPEED(x)		(PI_v * ROBOT_WHEEL_DIAMETER * COUNT_FREQUENCY * x / ENCODER_COUNT_VALUE)  



void Get_Velocity(void);

#endif
