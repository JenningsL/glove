#include "stm32l1xx.h"
#include "usart_1.h"
#include <stdarg.h>

void USART1_NVIC_Config(void)
{
    
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
    NVIC_InitStructure.NVIC_IRQChannel  = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        
    NVIC_Init(&NVIC_InitStructure);//USART2 configuration 
    

    
}
void 
usart1_init(void)
{
        USART1_NVIC_Config();
	usart1_rcc_config();
	usart1_gpio_config();
	usart1_usart_config();
}

//gpio config
void
usart1_gpio_config(void)
{
	GPIO_InitTypeDef usart1_gpio_initstructure;
	
	//connect PA.9 to usart1's tx
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	//connect PA.10 to usart1's rx
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	
	/* Configure USART Tx as alternate function push-pull */
  usart1_gpio_initstructure.GPIO_Pin = GPIO_Pin_9;
  usart1_gpio_initstructure.GPIO_Mode = GPIO_Mode_AF;
  usart1_gpio_initstructure.GPIO_Speed = GPIO_Speed_40MHz;
  usart1_gpio_initstructure.GPIO_OType = GPIO_OType_PP;
  usart1_gpio_initstructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &usart1_gpio_initstructure);
    
  /* Configure USART Rx as alternate function push-pull */
  usart1_gpio_initstructure.GPIO_Pin =GPIO_Pin_10;
  GPIO_Init(GPIOA,&usart1_gpio_initstructure);
}

//rcc config
void
usart1_rcc_config(void)
{
	//enable gpio clock
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	
	//enable usart1 clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
}

//usart config
void
usart1_usart_config(void)
{
	USART_InitTypeDef	usart1_usart_initstructure;
	
	usart1_usart_initstructure.USART_BaudRate = 115200;
  usart1_usart_initstructure.USART_WordLength = USART_WordLength_8b;
  usart1_usart_initstructure.USART_StopBits = USART_StopBits_1;
  usart1_usart_initstructure.USART_Parity = USART_Parity_No;
  usart1_usart_initstructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  usart1_usart_initstructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
	//configuration
	USART_Init(USART1, &usart1_usart_initstructure);
	
	//enable
	USART_Cmd(USART1, ENABLE);
         USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
}

void 
usart1_delay(uint32_t nTime)
{
  for(; nTime != 0; nTime--);
}

void
usart1_sendbyte(unsigned char * byte, unsigned char len)
{
	unsigned char i = 0;
	for(i = 0; i < len; i++)
	{
		USART_SendData(USART1, byte[i]);
		usart1_delay(2000);
	}
}
/*
 * ��������fputc
 * ����  ���ض���c�⺯��printf��USART1
 * ����  ����
 * ���  ����
 * ����  ����printf����
 */
int fputc(int ch, FILE *f)
{
	/* ��Printf���ݷ������� */
	USART_SendData(USART1, (unsigned char) ch);
//	while (!(USART1->SR & USART_FLAG_TXE));
	while( USART_GetFlagStatus(USART1,USART_FLAG_TC)!= SET);	
	return (ch);
}
/*
 * ��������itoa
 * ����  ������������ת�����ַ���
 * ����  ��-radix =10 ��ʾ10���ƣ��������Ϊ0
 *         -value Ҫת����������
 *         -buf ת������ַ���
 *         -radix = 10
 * ���  ����
 * ����  ����
 * ����  ����USART1_printf()����
 */
static char *itoa(int value, char *string, int radix)
{
	int     i, d;
	int     flag = 0;
	char    *ptr = string;
	
	/* This implementation only works for decimal numbers. */
	if (radix != 10)
	{
	    *ptr = 0;
	    return string;
	}
	
	if (!value)
	{
	    *ptr++ = 0x30;
	    *ptr = 0;
	    return string;
	}
	
	/* if this is a negative value insert the minus sign. */
	if (value < 0)
	{
	    *ptr++ = '-';
	
	    /* Make the value positive. */
	    value *= -1;
	}
	
	for (i = 10000; i > 0; i /= 10)
	{
	    d = value / i;
	
	    if (d || flag)
	    {
	        *ptr++ = (char)(d + 0x30);
	        value -= (d * i);
	        flag = 1;
	    }
	}
	
	/* Null terminate the string. */
	*ptr = 0;
	
	return string;

} /* NCL_Itoa */

/*
 * ��������USART1_printf
 * ����  ����ʽ�������������C���е�printf��������û���õ�C��
 * ����  ��-USARTx ����ͨ��������ֻ�õ��˴���1����USART1
 *		     -Data   Ҫ���͵����ڵ����ݵ�ָ��
 *			   -...    ��������
 * ���  ����
 * ����  ���� 
 * ����  ���ⲿ����
 *         ����Ӧ��USART1_printf( USART1, "\r\n this is a demo \r\n" );
 *            		 USART1_printf( USART1, "\r\n %d \r\n", i );
 *            		 USART1_printf( USART1, "\r\n %s \r\n", j );
 */
void USART1_printf(USART_TypeDef* USARTx, uint8_t *Data,...)
{
	const char *s;
	int d;   
	char buf[16];
	
	va_list ap;
	va_start(ap, Data);
	
	while ( *Data != 0)     // �ж��Ƿ񵽴��ַ���������
	{				                          
		if ( *Data == 0x5c )  //'\'
	{									  
	switch ( *++Data )
	{
		case 'r':							          //�س���
			USART_SendData(USARTx, 0x0d);
			Data ++;
		break;
		
		case 'n':							          //���з�
			USART_SendData(USARTx, 0x0a);	
			Data ++;
		break;
		
		default:
			Data ++;
		break;
	}			 
	}
	else if ( *Data == '%')
	{									  //
	switch ( *++Data )
	{				
		case 's':										  //�ַ���
			s = va_arg(ap, const char *);
	for ( ; *s; s++) 
	{
		USART_SendData(USARTx,*s);
		while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
	}
		Data++;
		break;
	
	case 'd':										//ʮ����
	d = va_arg(ap, int);
	itoa(d, buf, 10);
	for (s = buf; *s; s++) 
	{
		USART_SendData(USARTx,*s);
		while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
	}
	Data++;
	break;
		 default:
				Data++;
		    break;
	}		 
	} /* end of else if */
	else USART_SendData(USARTx, *Data++);
	while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
	}
}

