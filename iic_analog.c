#include "stm32l1xx_nucleo.h"
#include "iic_analog.h"

#define u32 uint32_t

/************************************************************/
/*ģ��IIC���ų�ʼ������*/
/************************************************************/
void I2C_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	//ʹ��ʱ��
         RCC_AHBPeriphClockCmd (RCC_AHBPeriph_GPIOB, ENABLE);

	//�������� PB6->SDA PB7->SCL
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6| GPIO_Pin_7;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
        GPIO_Init(GPIOB, &GPIO_InitStructure);  

	//��ʼ��ICC��ģʽ
	SET_SDA;
	SET_SCL;  
}


/************************************************************/
/************************************************************/
void IIC_Delay(void)
{
	u32 i = 10;
	while( i-- );
}

/*******************************************************************
TWI_START
������������
*******************************************************************/
u8 IIC_Start(void)
{
	SET_SDA;
	IIC_DELAY;

	SET_SCL;
	IIC_DELAY;

	if( IIC_SDA_STATE == RESET )
	{
		return IIC_BUS_BUSY;
	}

	RESET_SDA;
	IIC_DELAY;

	RESET_SCL;
	IIC_DELAY;

	if( IIC_SDA_STATE == SET )
	{
		return IIC_BUS_ERROR;
	}

	return IIC_BUS_READY;
}

/*******************************************************************
TWI_STOP
����ֹͣ����
*******************************************************************/
void IIC_Stop(void)
{
	RESET_SDA;
	IIC_DELAY;

	SET_SCL;
	IIC_DELAY;

	SET_SDA;
	IIC_DELAY;
}

/*******************************************************************************
* ��������:TWI_SendNACK                                                                     
* ��    ��:�յ�����,����NACK                                                                                                                                       
 *******************************************************************************/
void IIC_SendNACK(void)
{
	RESET_SDA;
	IIC_DELAY;
	SET_SCL;
	IIC_DELAY;
	RESET_SCL; 
	IIC_DELAY; 
}

/*******************************************************************************
* ��������:TWI_SendACK                                                                     
* ��    ��:�յ�����,����ACK                                                                                                                                        
*******************************************************************************/
void IIC_SendACK(void)
{
	SET_SDA;
	IIC_DELAY;
	SET_SCL;
	IIC_DELAY;
	RESET_SCL; 
	IIC_DELAY;
}

/*******************************************************************************
 * ��������:TWI_SendByte                                                                     
 * ��    ��:����һ���ֽ�                                                                                                                                      
 *******************************************************************************/
u8 IIC_SendByte(u8 Data)
{
	 u8 i;
	 RESET_SCL;
	 for(i=0;i<8;i++)
	 {  
		//---------���ݽ���----------
		if(Data&0x80)
		{
			SET_SDA;
		}
		else
		{
			RESET_SDA;
		} 
		Data<<=1;
		IIC_DELAY;
		//---���ݽ�������һ����ʱ----
		//----����һ��������[������] 
		SET_SCL;
		IIC_DELAY;
		RESET_SCL;
		IIC_DELAY;//��ʱ,��ֹSCL��û��ɵ�ʱ�ı�SDA,�Ӷ�����START/STOP�ź�
		//---------------------------   
	 }
	 //���մӻ���Ӧ�� 
	 SET_SDA; 
	 IIC_DELAY;
	 SET_SCL;
	 IIC_DELAY;   
	 if(IIC_SDA_STATE)
	 {
		RESET_SCL;
		return IIC_NACK;
	 }
	 else
	 {
		RESET_SCL;
		return IIC_ACK;  
	 }    
}

/*******************************************************************************
 * ��������:TWI_ReceiveByte                                                                     
 * ��    ��:����һ���ֽ�                                                                                                                                       
 *******************************************************************************/
u8 IIC_RecvByte(void)
{
	 u8 i,Dat = 0;
	 SET_SDA;
	 RESET_SCL; 
	 Dat=0;
	 for(i=0;i<8;i++)
	 {
		SET_SCL;//����ʱ��������[������],�ôӻ�׼�������� 
		IIC_DELAY; 
		Dat<<=1;
		if(IIC_SDA_STATE) //������״̬
		{
			Dat|=0x01; 
		}   
		RESET_SCL;//׼�����ٴν�������  
		IIC_DELAY;//�ȴ�����׼����         
	 }
	 return Dat;
}

/******���ֽ�д��*******************************************/
void Single_Write_IIC1(u8 SlaveAddress,u8 REG_Address,u8 REG_data)
{
    IIC_Start();                  //��ʼ�ź�
    IIC_SendByte(SlaveAddress);   //�����豸��ַ+д�ź�
    IIC_SendByte(REG_Address);    //�ڲ��Ĵ�����ַ�� //��ο�����pdf22ҳ 
    IIC_SendByte(REG_data);       //�ڲ��Ĵ������ݣ� //��ο�����pdf22ҳ 
    IIC_Stop();                   //����ֹͣ�ź�
}

/********���ֽڶ�ȡ*****************************************/
u8 Single_Read_IIC1(u8 SlaveAddress, u8 REG_Address)
{  
	u8 REG_data;
    IIC_Start();                          //��ʼ�ź�
    IIC_SendByte(SlaveAddress);           //�����豸��ַ+д�ź�
    IIC_SendByte(REG_Address);            //���ʹ洢��Ԫ��ַ��//��0��ʼ	
    IIC_Start();                          //��ʼ�ź�
    IIC_SendByte(SlaveAddress+1);         //�����豸��ַ+���ź�
    REG_data = IIC_RecvByte();              //�����Ĵ�������
	IIC_SendACK();   
	IIC_Stop();                           //ֹͣ�ź�
    return REG_data; 
}

