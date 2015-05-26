#include "stm32l1xx_nucleo.h"
#include "MPU6050.h"
#define RtA 		57.324841f			//???����????��	
#define AtR    		0.0174533f			//?����????��	
#define Acc_G 		0.0011963f				//?��?��?����?3��G
#define Gyro_G 		0.0610351f		//???��?����?3��?����?��?2?��y??��|����?Y2000?��????	
#define Gyro_Gr		0.0010653f  //???��?����?3��???����?��?2?��y??��|����?Y2000?��????			
#define FILTER_NUM 	20

static short ACC_X_INT,ACC_Y_INT,ACC_Z_INT;

static short ACC_X_BUF[FILTER_NUM],ACC_Y_BUF[FILTER_NUM],ACC_Z_BUF[FILTER_NUM];//������������

typedef struct 
{
  float Pitch;
  float Roll;
  float Yaw;
}FLOAT_ANGLE;

extern FLOAT_ANGLE  Q_ANGLE;//������Ԫ������֮��õ�����̬��
void fill_ACC_BUF(void);
void Prepare_AccData(void);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
void IMU_getValues(void);
//extern struct MPU6050_RAW_DATA MPU6050_LAST;