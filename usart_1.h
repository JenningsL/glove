#include "stm32l1xx.h"
#include <stdio.h>


//user function
void usart1_delay(uint32_t);
void usart1_sendbyte(unsigned char * byte, unsigned char len);

//config function
void usart1_rcc_config(void);
void usart1_gpio_config(void);
void usart1_usart_config(void);
void usart1_init(void);
void USART1_NVIC_Config(void);
void USART1_printf(USART_TypeDef* USARTx, uint8_t *Data,...);
//int fputc(int ch, FILE *f);