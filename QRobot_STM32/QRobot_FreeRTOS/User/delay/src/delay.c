#include "delay.h"


extern TIM_HandleTypeDef htim6;

void delay_ms(uint32_t ms)
{
  __HAL_TIM_ENABLE(&htim6);
  HAL_Delay(ms);
  __HAL_TIM_DISABLE(&htim6);
    
}

void HAL_Delay_us(uint32_t nus)
{
  uint32_t cnt=0;
	
	__HAL_TIM_ENABLE(&htim6);
  __HAL_TIM_SetCounter(&htim6,0);
	
	while( cnt < nus){   
     cnt=__HAL_TIM_GetCounter(&htim6); 
  } 
	__HAL_TIM_DISABLE(&htim6);
    
}

//uint32_t fac_us;

//void HAL_Delay_us_init(uint8_t SYSCLK)
//{
//	fac_us = SYSCLK;
//}
//	

//void HAL_Delay_us(uint32_t nus)
//{
//    uint32_t ticks;
//    uint32_t told,tnow,tcnt=0;
//    uint32_t reload=SysTick->LOAD;
//    ticks=nus*fac_us; 
//    told=SysTick->VAL; 
//    while(1)
//    {
//        tnow=SysTick->VAL;
//        if(tnow!=told)
//        {
//            if(tnow<told)tcnt+=told-tnow;
//            else tcnt+=reload-tnow+told;
//            told=tnow;
//            if(tcnt>=ticks)break; 
//        }
//    };
//}


//void RCCdelay_us(uint32_t udelay)
//{
//  __IO uint32_t Delay = udelay * 72 / 8;       //72MÖ÷Æµ
//  do
//  {
//    __NOP();                                   //¿ÕÓï¾ä
//  }
//  while (Delay --);
//}


