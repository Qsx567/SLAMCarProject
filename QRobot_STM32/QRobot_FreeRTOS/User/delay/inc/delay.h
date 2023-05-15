#ifndef __DELAY_H__
#define __DELAY_H__
#include "main.h"

void HAL_Delay_us_init(uint8_t SYSCLK);


void delay_ms(uint32_t ms);
void HAL_Delay_us(uint32_t nus);


#endif
