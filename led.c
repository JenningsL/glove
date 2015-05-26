#include "led.h"
#include "stm32l1xx.h"

void ledConfig(void){
  //PB4
  GPIO_InitTypeDef GPIO_InitStructure;
   /* Enable the GPIO Clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

  /* Configure Button pin as input */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}
void turnOnLed(void){
  GPIO_ResetBits(GPIOB,GPIO_Pin_4);
}
void turnOffLed(void){
  GPIO_SetBits(GPIOB,GPIO_Pin_4);
}