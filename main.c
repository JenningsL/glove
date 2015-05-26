
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart_1.h"
#include "tick.h"
#include "MPU6050.h"
#include "iic_analog.h"
#include "quaternion.h"
#include "led.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "spi_flash.h"
#include "motion.h"
/* define ------------------------------------------------------------------*/
#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t
/**************** 压力传感器电路输出电压 *******************************************/
__IO uint16_t ADC_Value;
#define ADC1_DR_ADDRESS    ((uint32_t)0x40012458)
float vout=0;
void DMA_Configuration(void);
void ADC_Configuration(void);
void smoothPressVal(void);//滑动窗口滤波平滑输入电压
void fillPressSilde(void);
#define WINDOWSIZE_PRESS 10
float slideWindow[WINDOWSIZE_PRESS];
int slidePressCount=0;
/*************陀螺仪*************************/
float ACC_X,ACC_Y,ACC_Z,GYRO_X,GYRO_Y,GYRO_Z;//传入四元数函数计算的六个数据
float MOT_X,MOT_Y,MOT_Z;
float gravity[3];//假设的重力加速度
float ACC[3],MOT[3];
float totalAcc=0; //滤除掉重力加速度后的总体加速度
float speed[3]={0,0,0};//加速度积分得到速度
void gravityFilter(void);
int pitch,roll,yaw;
char buffer[100];//原来1w, 过大要发送需要改回来
#define MAX_COLLECT_SIZE 800
float collectedData[MAX_COLLECT_SIZE];//存放收集到的数据 ,最多存放200个数据点

float standardData[600];//经过标准化的数据,100个数据点
int DataCount=-1;
void EXTI9_5_IRQHandler(void);
/******************训练*********************/
void collectAndSend(void);
void buttonConfig(void);
extern float theta[];
extern float uj[];
extern int sj[];

/*****************浮点转字符串*************/
char *F2S(float d, char* str,int precise);
char *F2S(float d, char* str,int precise)
{
  char str1[40];
  int j=0,k,i;
  i = (int)d;  //浮点数的整数部分
  //d = d-(int)d;
  while(i>0)
  {
    str1[j++] = i%10+'0';
    i /= 10;
  }
  for(k=0;k<j;k++)
    str[k] = str1[j-1-k]; //

  str[j++] = '.';
  d -= (int)d;
  for(i=0;i<precise;i++)
  {
    d *= 10;
    str[j++] = (int)d+'0';
    d -= (int)d;
  }
  while(str[--j]=='0');
    str[++j] = '\0';
  return str;
}
/***********心率传感器相关变量************/
uint32_t lastHeartTick=0;
int heartRate=70;
uint32_t lastSentTime=0;//上一次发送的时间点
void sendHearRatePerSec(void);
/***********主流程相关变量*************/
uint32_t timeCollapse=0;
char command=' ';//从手机接收到的命令字符
int collect=0;//代表是否采集数据
int actionFound=0;//测试，检测到多少个数据
void detectAction(void);
void detectActionWithExpand(void);
uint8_t eventNow=1;//当前检测的是哪个动作
#define ACTION_SEARCH_WAITING 1
#define ACTION_SEARCH_SENT 2//动作搜索状态
#define ACTION_COUNTING 3//动作计数状态
uint8_t appState=ACTION_COUNTING;//默认计数状态
int main(void)
{  
  initTick();//开启时钟计时
  I2C_Configuration();//I2C2初始化
  IIC_Stop();
  MPU6050_Inital();//MPU6050初始化
  //adjustOffset();//校正偏移量
  fill_ACC_BUF();//填充滑动窗口数组
  ledConfig();
  turnOnLed();
  //turnOffLed();

  //SPI初始化
  SPI_FLASH_Init();
  //SPI_FLASH_ChipErase();

  //setParamById(1);
  getParamById(1);
  
  
  //按钮中断
  buttonConfig();
  usart1_init();
  DMA_Configuration();
  ADC_Configuration();
  
  /* 启动ADC */
  ADC_SoftwareStartConv(ADC1);
  fillPressSilde();//填充滑动窗口数组
  vout = (float) ADC_Value /4096*3.3*2;
  
  
   //初始化重力滤除
   MPU6050_Read();//读取原始数据
   //MPU6050_Dataanl();//减去偏移量
   Prepare_AccData();//平滑加速度数据
   IMU_getValues();//将四个参数转化为float
   ACC[0]=ACC_X/1672;
   gravity[0]=ACC_X/1672;
   ACC[1]=ACC_Y/1672;
   gravity[1]=ACC_Y/1672;
   ACC[2]=ACC_Z/1672;
   gravity[2]=ACC_Z/1672;
   
   //初始化心率数据发送时间点
   lastSentTime=getRunTime();
  while(1){
      
   smoothPressVal();//读取压力数据并平滑 

   gravityFilter();
   totalAcc=sqrt(MOT[0]*MOT[0]+MOT[1]*MOT[1]+MOT[2]*MOT[2]);
   //speed[0]=speed[0]+MOT[0]*0.02;
   //speed[1]=speed[1]+MOT[1]*0.02;
   //speed[2]=speed[2]+MOT[2]*0.02;
   
   //Delay(20);
   //IMUupdate(GYRO_X*Gyro_Gr,GYRO_Y*Gyro_Gr,GYRO_Z*Gyro_Gr,ACC_X,ACC_Y,ACC_Z);//计算姿态角
   //pitch= (int)Q_ANGLE.Pitch;
   //roll= (int)Q_ANGLE.Roll;
   //yaw= (int)Q_ANGLE.Yaw;
   
   //sendHearRatePerSec();
   if(appState==ACTION_SEARCH_WAITING||appState==ACTION_SEARCH_SENT){
     if(appState==ACTION_SEARCH_SENT){
       collectAndSend();
       appState=ACTION_SEARCH_WAITING;//重新进入等待状态
     }
   }else{
      switch(eventNow){
   case 0:
     if(collect||MOT[2]<-1.0){

     detectActionWithExpand();
     collect=0;
     }//举哑铃
     break;
   case 1:
     if(collect||MOT[1]<-1.0){
     DataCount=0;
     detectAction();
     collect=0;
     }//提拉哑铃
     break;
    }
   }
  
   

    

   ///////////////////////////////
     //压力数据
   /*if(DataCount>=0&&DataCount<100){
     char bufferTemp[20];
     //sprintf(bufferTemp,"")
     strcat(buffer,F2S(vout,bufferTemp,4));
     strcat(buffer," ");
     DataCount+=1;
     //Delay(20);
   }else if(DataCount==100){
     strcat(buffer,"&");
     len=strlen(buffer);
     printf(buffer);
     //free(buffer);
     buffer[0]='\0';//清空字符串数组
     DataCount=-1;
   }*/
   //////////////////////////////
   
  }

  
}
//每秒发送一次心率数据
void sendHearRatePerSec(void){
  uint32_t thisTime,timeCollapse;
  thisTime=getRunTime();
  timeCollapse=thisTime-lastSentTime;
  if(timeCollapse>=1000){
    char buffer[20];
    sprintf(buffer,"%d h",heartRate);
    printf(buffer);//发送心率数据
    lastSentTime=thisTime;
  }
  
}
//低通滤波器滤除重力加速度分量
void gravityFilter(void){
   //陀螺仪相关操作,读取
   MPU6050_Read();//读取原始数据
  // MPU6050_Dataanl();//减去偏移量
   Prepare_AccData();//平滑加速度数据
   IMU_getValues();//将四个参数转化为float
   
   ACC[0]=ACC_X/1672;
   ACC[1]=ACC_Y/1672;
   ACC[2]=ACC_Z/1672;
   for(int i=0;i<3;i+=1)
   {
     gravity[i]=0.1*ACC[i]+0.9*gravity[i];
     MOT[i]=ACC[i]-gravity[i];
   }
}
//无数据放缩版

void detectAction(void){
   DataCount=0;
   //加速度、角速度数据训练用
   while(DataCount<600){
     gravityFilter();
     //sendHearRatePerSec();
     collectedData[DataCount]=ACC_X;
     collectedData[DataCount+1]=ACC_Y;
     collectedData[DataCount+2]=ACC_Z;
     collectedData[DataCount+3]=GYRO_X;
     collectedData[DataCount+4]=GYRO_Y;
     collectedData[DataCount+5]=GYRO_Z;
     DataCount+=6;
     Delay(13);
   }
     float result;
     //unsigned int begintime=getRunTime();
     result=test(collectedData);
   if(result>0.5){
     printf("yes#");
     actionFound+=1;
   }else{
     printf("no#");
   }
   
     //timeCollapse=getRunTime()-begintime;
     
     DataCount=-1;
     lastSentTime=getRunTime()-800;//心率数据延迟发送
}


//有数据放缩版
void detectActionWithExpand(void){
  DataCount=0;
  while(totalAcc>0.2&&DataCount<MAX_COLLECT_SIZE){
       gravityFilter();
       //sendHearRatePerSec();
       totalAcc=sqrt(MOT[0]*MOT[0]+MOT[1]*MOT[1]+MOT[2]*MOT[2]);
       collectedData[DataCount]=ACC_X;
       collectedData[DataCount+1]=ACC_Y;
       collectedData[DataCount+2]=ACC_Z;
       collectedData[DataCount+3]=GYRO_X;
       collectedData[DataCount+4]=GYRO_Y;
       collectedData[DataCount+5]=GYRO_Z;
       DataCount+=6;
       Delay(13);
  }
  if(DataCount<300){
    DataCount=-1;
    return;//丢弃过小的数据
  }
  float result;
  if(DataCount<600){
    expand(collectedData,standardData,(int)DataCount/6);//对数据进行延展
    result=test(standardData);
  }else if(DataCount>600){
    //compress(collectedData,standardData,(int)DataCount/6);//对数据进行压缩，不确定是否可行
    //result=test(standardData);
    result=test(collectedData);
  }else{
    result=test(collectedData);
  }
  if(result>0.5){
     printf("yes#");
     actionFound+=1;
   }else{
     printf("no#");
   }
  DataCount=-1;
  lastSentTime=getRunTime()-800;//心率数据延迟200ms发送，避免冲突
}
//将数据发送到手机
void collectAndSend(void){
   DataCount=0;
   while(DataCount<600){
     gravityFilter();
     //sendHearRatePerSec();
     collectedData[DataCount]=ACC_X;
     collectedData[DataCount+1]=ACC_Y;
     collectedData[DataCount+2]=ACC_Z;
     collectedData[DataCount+3]=GYRO_X;
     collectedData[DataCount+4]=GYRO_Y;
     collectedData[DataCount+5]=GYRO_Z;
     DataCount+=6;
     Delay(13);
   }
     
     for(int x=0;x<600;x+=6){//1791
       char bufferTemp[200];
       //训练用
       //sprintf(bufferTemp,"%d %d %d %d %d %d;",collectedData[x],collectedData[x+1],collectedData[x+2],collectedData[x+3],collectedData[x+4],collectedData[x+5]);
       //给手机用的
       sprintf(bufferTemp,"%d %d %d %d %d %d ",collectedData[x],collectedData[x+1],collectedData[x+2],collectedData[x+3],collectedData[x+4],collectedData[x+5]);
       printf(bufferTemp);
     }
     printf("&");
     DataCount=-1;
}
void fillPressSilde(void){
  for(;slidePressCount<WINDOWSIZE_PRESS-1;slidePressCount+=1){
        if(ADC_GetFlagStatus(ADC1, ADC_FLAG_OVR))
    {
      ADC_ClearFlag(ADC1, ADC_FLAG_OVR);
    }
    slideWindow[slidePressCount]=(float) ADC_Value /4096*3.3;
  }
}
void smoothPressVal(void){
    //读取压力传感器
    if(ADC_GetFlagStatus(ADC1, ADC_FLAG_OVR))
    {
      ADC_ClearFlag(ADC1, ADC_FLAG_OVR);
    }
  slideWindow[slidePressCount]=(float) ADC_Value /4096*3.3;
  float temp=0;
  
  for(int i=0;i<WINDOWSIZE_PRESS;i+=1){
    temp+=slideWindow[i];
  }
  vout=temp/WINDOWSIZE_PRESS;
 
  if(slidePressCount==WINDOWSIZE_PRESS-1){
    slidePressCount=0;
  }else{
    slidePressCount++;
  }
   
}
//按钮配置,PA5
void buttonConfig(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the GPIO Clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  /* Configure Button pin as input */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_Init(GPIOA, &GPIO_InitStructure);


    /* Connect Button EXTI Line to Button GPIO Pin */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource5);

    /* Configure Button EXTI line */
    EXTI_InitStructure.EXTI_Line = EXTI_Line5;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set Button EXTI Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure); 
  
}

void EXTI9_5_IRQHandler(void)
{
  if ((EXTI_GetITStatus(EXTI_Line5) != RESET))
  {
    if(DataCount == -1)
    {
      DataCount = 0;
    }//开始采集数据
    
    //读取心率
    if(!lastHeartTick){
      lastHeartTick=getRunTime();//初始化
    }else{
      uint32_t thisTime=getRunTime();
      uint32_t timeCollapse=thisTime-lastHeartTick;
      uint32_t temp=(int) 60000/timeCollapse;
      if(temp<300&&temp>0){
        heartRate=(int) 60000/timeCollapse;
        lastHeartTick=thisTime;
      }//过滤掉不合理的数据
    }

    /* Clear the EXTI line pending bit */
    EXTI_ClearITPendingBit(EXTI_Line5);
  }	
}


/*****************************************************************************
功能：DMA配置
输入：none
输出：none
*****************************************************************************/
void DMA_Configuration()
{
  DMA_InitTypeDef DMA_InitStructure;
  
  /* ADC DMA配置，用于实时更新adc采集的电压值 *******************************/
  /* 使能DMA */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  /* DMA1 channel1 配置 */
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_ADDRESS;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC_Value;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  /* 使能 DMA1 channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);
  
}
/***************************************************************
功能：ADC配置函数，用于监控电池电压
输入：none
输出：none
***************************************************************/
void ADC_Configuration()
{
  ADC_InitTypeDef ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;  
  
  /* ADC配置 *****************************************************************/
  /* ADC使用的是HSI时钟，在芯片上电时已经使能 */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  /* GPIOB14 ADC通道20 IO口配置为模拟 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  /* 使能ADC时钟 */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  /* ADC配置 */
  /* 选择8MHz时钟，时钟不能太快，否侧会出错 */
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_Init(ADC1, &ADC_InitStructure);
  ADC_CommonInit(&ADC_CommonInitStructure);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_16Cycles);
  
  /* 使能DMA */
  ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
  /* 使能 ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);
  /* 使能 ADC1 */
  ADC_Cmd(ADC1, ENABLE);
  /* 等待配置完成 */
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_ADONS) == RESET)
  {
    
  }
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
