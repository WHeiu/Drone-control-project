#include "MPU9250_F.h"
#include "mp9250.h"
#include "MPU9250_#DEFINE.h"
#include "MahonyAHRS.h"
#include "usart.h"
#include "string.h"
#include "mpuu9250.h"

#define Gravity (4.0/65536.0)	    // G单位
#define Factor (2000.0/65536.0*3.14159/180.0)     	//弧度秒
#define mag_scale 	  (9600.0/65536.0/100.0)							//高斯单位   0.15μT/LSB typ

extern short accel[3],gyro[3],mag[3];
extern 	uint8_t response[3];
int16_t Acc[3] = {0}, GYO[3] = {0},Mag[3] = {0};
int32_t Gyo_Offset[3] = {0};
uint8_t Mag_Offset[3] = {0};
void SPI_ReadBytes(uint8_t Register,uint8_t Length,uint8_t* Array,enum LMSB Word)
{
	uint8_t i = 0;
	for(i = 0;i < Length;i++)
	{
		if(Word == MSB)
		{
			Array[Length - i - 1] = MPU9250_SPI_Read(Register + i);
		}
		else
		{
			Array[i] = MPU9250_SPI_Read(Register + i);
		}
	}
}
void SPI_MAG_ReadBytes(uint8_t Register,uint8_t Length,uint8_t* Array,enum LMSB Word)
{
	uint8_t i = 0;
	uint8_t res = 0;
	MPU9250_AK8963_SPI_Read(MPU9250_AK8963_I2C_ADDR,0x00,&res);
	if(res == 0x48)  //等待读取磁力计ID
	{
	  printff(USART3, "Read AK8963 ID OK!!!\r\n",strlen("Read AK8963 ID OK!!!\r\n"));	   
    MPU9250_AK8963_SPI_Write(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL2_REG, AK8963_CNTL2_SRST);// 软件复位 AK8963 ， 所有寄存器被重新初始化
  	LL_mDelay(1);
		MPU9250_AK8963_SPI_Read(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL2_REG,&res);
		if( res == AK8963_CNTL2_SRST)
		 printff(USART3, "磁力计复位成功!!!\r\n",strlen("磁力计复位成功!!!\r\n"));	
		else
		 printff(USART3, "磁力计复位失败!!!\r\n",strlen("磁力计复位失败!!!\r\n"));	;
   MPU9250_AK8963_SPI_Write(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL1_REG, 0x11);		 // 16位输出， 单次测量模式
			LL_mDelay(1);
		MPU9250_AK8963_SPI_Read(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL1_REG,&res);
		if( res == 0x11)
		 printff(USART3, "16位输出， 单次测量模式!!!\r\n",strlen("16位输出， 单次测量模式!!!\r\n"));	
		else
		 printff(USART3, "配置16位输出， 单次测量模式失败!!!\r\n",strlen("配置16位输出， 单次测量模式失败!!!\r\n"));
	 while(!(i2c_Mag_read(0x02) & 0x01));   //等待数据准备好
	};
	for(i = 0;i < Length;i++)
	{
		if(Word == MSB)
		{
//			Array[Length - i - 1] = i2c_Mag_read(Register + i);
			MPU9250_AK8963_SPI_Read(MPU9250_AK8963_I2C_ADDR,Register + i,&Array[Length - i - 1]);
		}
		else
		{
	//		Array[i] = i2c_Mag_read(Register + i);
			MPU9250_AK8963_SPI_Read(MPU9250_AK8963_I2C_ADDR,Register + i,&Array[ i ]);
		}
	}
}
void MPU9250_Read()   //注意字节序的问题
{
	uint8_t i = 0x00;
	char TEST1[20] = {0};
	char TEST2[20] = {0};
	char TEST3[20] = {0};
	char TEST4[20] = {0};
	char TEST5[20] = {0};
	char TEST6[20] = {0};
	char TEST7[20] = {0};
	char TEST8[20] = {0};
	char TEST9[20] = {0};	
	for(i = 0;i<3;i++)
	{
//	I2C_ReadBytes(I2C1,GYRO_ADDRESS,ACCEL_XOUT_H + 2*i,2,(uint8_t*)(Acc + i),MSB);//Get The X Y Z Acc
	  SPI_ReadBytes(ACCEL_XOUT_H + 2*i ,2,(uint8_t*)(Acc + i), MSB);  
//	I2C_ReadBytes(I2C1,GYRO_ADDRESS,GYRO_XOUT_H + 2*i,2,(uint8_t*)(GYO + i),MSB);//Get The X Y Z Gyo
    SPI_ReadBytes(GYRO_XOUT_H + 2*i ,2,(uint8_t*)(GYO + i), MSB);  
//	I2C_ReadBytes(I2C1,MAG_ADDRESS,MAG_XOUT_L + 2 * i,2,(uint8_t*)(Mag + i),LSB);//Get The X Y Z Mag
	  SPI_MAG_ReadBytes(MAG_XOUT_L + 2*i ,2,(uint8_t*)(Mag + i), LSB);
		Mag[i] = Mag[i] * (((uint16_t)Mag_Offset[i] - 128)/256 + 1);//scale
		GYO[i] -= Gyo_Offset[i];//Calibrate
	}
		sprintf( TEST1, "%d", Acc[0]); 
		sprintf( TEST2, "%d", Acc[1]); 
		sprintf( TEST3, "%d", Acc[2]); 
		sprintf( TEST4, "%d", GYO[0]); 
		sprintf( TEST5, "%d", GYO[1]); 
		sprintf( TEST6, "%d", GYO[2]); 
		sprintf( TEST7, "%d", Mag[0]); 
		sprintf( TEST8, "%d", Mag[1]); 
		sprintf( TEST9, "%d", Mag[2]); 
	  printff(USART3, "\r\nACC:\r\n",strlen("\r\nACC:\r\n"));
		printff(USART3, TEST1,strlen(TEST1));   printff(USART3, "\t",strlen("\t"));
		printff(USART3, TEST2,strlen(TEST2));   printff(USART3, "\t",strlen("\t"));
		printff(USART3, TEST3,strlen(TEST3));   
  	printff(USART3, "\r\nGYO:\r\n",strlen("\r\nGYO:\r\n"));
		printff(USART3, TEST4,strlen(TEST4));   printff(USART3, "\t",strlen("\t"));
		printff(USART3, TEST5,strlen(TEST5));   printff(USART3, "\t",strlen("\t"));
		printff(USART3, TEST6,strlen(TEST6));   
  	printff(USART3, "\r\nMAG:\r\n",strlen("\r\nMAG:\r\n"));
		printff(USART3, TEST7,strlen(TEST7));   printff(USART3, "\t",strlen("\t"));
		printff(USART3, TEST8,strlen(TEST8));   printff(USART3, "\t",strlen("\t"));
		printff(USART3, TEST9,strlen(TEST9));   
		printff(USART3, "\r\n",strlen("\r\n"));
}
void MPU9250_Calibration(void)
{
	uint8_t i , k;
	for(k = 0;k < 10;k++)
	{
		for(i = 0;i<3;i++)
		{
	//		I2C_ReadBytes(I2C1,GYRO_ADDRESS,GYRO_XOUT_H + 2*i,2,(uint8_t*)(GYO + i),MSB);
			SPI_ReadBytes(GYRO_XOUT_H + 2*i,2,(uint8_t*)(GYO + i), MSB);
			Gyo_Offset[i] += GYO[i];
		}
	}
	for(i = 0;i<3;i++)
	{
		Gyo_Offset[i] /= 10;
//		I2C_ReadBytes(I2C1,MAG_ADDRESS,0x10 + i,1,Mag_Offset + i,MSB);   //最后一个参数判断大小端字节序
//		Mag_Offset[i] = i2c_Mag_read(0x10+i);
//		MPU9250_AK8963_SPI_Read(MPU9250_AK8963_I2C_ADDR,0x10 + i,&Mag_Offset[i] );
	}
}
void Init_MPU9250()
{
	uint8_t Value = 0;
	uint8_t Reg = 0x00;
	uint8_t res = 0;
//	I2C_WriteRegBytes(I2C1,GYRO_ADDRESS,PWR_MGMT_1,&Value,1);
	MPU9250_SPI_Write(PWR_MGMT_1,Value);
	Value = 0x07;
//	I2C_WriteRegBytes(I2C1,GYRO_ADDRESS,SMPLRT_DIV,&Value,1);
	MPU9250_SPI_Write(SMPLRT_DIV,Value);
	Value = 0x01;
//	I2C_WriteRegBytes(I2C1,GYRO_ADDRESS,CONFIG,&Value,1);
  MPU9250_SPI_Write(CONFIG,Value); 
	Value = 0x10;//1000 degree
//	I2C_WriteRegBytes(I2C1,GYRO_ADDRESS,GYRO_CONFIG,&Value,1);
  MPU9250_SPI_Write(GYRO_CONFIG,Value); 
	Value = 0x08;//4g 0.63HZ Fliter
//	I2C_WriteRegBytes(I2C1,GYRO_ADDRESS,ACCEL_CONFIG,&Value,1);
  MPU9250_SPI_Write(ACCEL_CONFIG,Value); 
	Value = 0x04;//1.94 ms
//	I2C_WriteRegBytes(I2C1,GYRO_ADDRESS,ACCEL_CONFIG_2,&Value,1);
	MPU9250_SPI_Write(ACCEL_CONFIG_2,Value); 
	Reg = 0x02;
//	I2C_WriteRegBytes(I2C1,GYRO_ADDRESS,0x37,&Reg,1);
	MPU9250_SPI_Write(0x37,Reg); 
  Reg = 0x0F;//Fuse Mode
//	I2C_WriteRegBytes(I2C1,MAG_ADDRESS,0x0A,&Reg,1);
  MPU9250_AK8963_SPI_Read(MPU9250_AK8963_I2C_ADDR,0x0A,&Reg);
if (res == MPU9250_AK8963_CONTINUOUS_MEASUREMENT)
	printff(USART3, "write ofofofof\r\n",strlen("write ofofofof\r\n")); 
else 
	printff(USART3, "write nonoonno\r\n",strlen("write nonoonno\r\n")); 
 MPU9250_Calibration();
}
void MPU9250_Read_Pre()
{
	uint8_t Reg = 0x11;//Single time 16 bits
	uint8_t res = 0;
//	MPU9250_SPI_Write(MPU9250_I2C_SLV0_CTRL, 0x88);
//	LL_mDelay(1);
//	I2C_WriteRegBytes(I2C1,MAG_ADDRESS,0x0A,&Reg,1);
  MPU9250_AK8963_SPI_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, Reg);
  LL_mDelay(10);
	MPU9250_AK8963_SPI_Read(MPU9250_AK8963_I2C_ADDR,MPU9250_AK8963_CNTL,&res);//测试MPU9250_AK8963_SPI_Read
	if(res == 0x11)  //等待读取磁力计ID
	{
	 printff(USART3, " AK8963 config OK!!!\r\n",strlen(" AK8963 config OK!!!\r\n"));	   
	}
	else 
		printff(USART3, " AK8963 config NO!!!\r\n",strlen(" AK8963 config NO!!!\r\n"));	   ;
}
void Sensor_Read(void)
{
	int i;
//	MPU9250_Read_Pre();
	//Single_Read_Temp();//2ms
	//ReadPressure_Pre();
	//LL_mDelay(2);
//	MPU9250_Read();
	for(i = 0;i<3;i++)
	{
  	Mag[i] = mag[i];
		GYO[i] = gyro[i] ;
    Acc[i] =  accel[i];
//	Mag[i] = mag[i] * (((uint16_t)response[i] - 128.0)/256.0 + 1.0);   //scale
//	GYO[i] -= Gyo_Offset[i];//Calibrate
	}
	
	Mahony_update(GYO[0]* Factor,GYO[1]* Factor,GYO[2] * Factor,Acc[0] * Gravity,Acc[1]* Gravity,Acc[2]* Gravity,Mag[0] * mag_scale,Mag[1] * mag_scale,Mag[2] * mag_scale);
	Mahony_computeAngles();
	LL_mDelay(2);
//	ReadPressure();
//	Pressure_Caculate();
}
