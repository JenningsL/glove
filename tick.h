#ifndef _TICK_h_
#define _TICK_h_
#include "stm32l1xx_nucleo.h"
/***************¶¨Ê±********************************/
void TimingDelay_Decrement(void);
void Delay(__IO uint32_t nTime);
void initTick(void);

uint32_t getRunTime(void);
#endif