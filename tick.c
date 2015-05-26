#include"tick.h"
RCC_ClocksTypeDef RCC_Clocks;
static __IO uint32_t TimingDelay;
unsigned int runTime=0;
void initTick(void){
    /* SysTick end of count event each 1ms */
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);
}
void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;
  
  while(TimingDelay != 0);
}
void TimingDelay_Decrement(void)
{
  runTime++;
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}
//返回程序运行总时间，单位ms
uint32_t getRunTime(void){
  return runTime;
}