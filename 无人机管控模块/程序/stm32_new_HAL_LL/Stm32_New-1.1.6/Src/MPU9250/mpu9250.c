#include "mpu9250.h"
#include <stdbool.h>
#include "stm32l4xx_hal.h"
#include "usart.h"
#include "string.h"
#include "math.h"
#include <stdlib.h>
#include "Posture_Parsing.h"

bool Gyro_Offset_Flag=false;   // IMU偏移量获取标志
int16_XYZ ACC_Offset,GRY_Offset;     // 陀螺仪偏移量
float_XYZ EXP_ANGLE,EXP_RateOffset;
float YawRateGol;
extern float_XYZ acce_f,gyro_f;        //IMU 数据
float PitchOutput,RollOutput,YawOutput; 
extern bool GRY_Stable_Flag;     // 陀螺仪数据稳定标志
extern float_RPY Q_ANGLE,Q_Rad;

extern SPI_HandleTypeDef hspi1;
static short MPU9250_AK8963_ASA[3] = {0, 0, 0};
static uint8_t SPI1_ReadWriteByte(uint8_t TxData)
{	
  HAL_StatusTypeDef ret;
  uint8_t Rxdata;
  ret = HAL_SPI_TransmitReceive(&hspi1, &TxData,&Rxdata, 1, 100);

  return Rxdata;
}
//MPU9250_SPI_Write
//MPU9250的SPI写一个字节函数
//reg_addr 寄存器地址  data 要写入的数据
void MPU9250_SPI_Write(uint8_t reg_addr, uint8_t data)
{
	MPU9250_CS_L;
	SPI1_ReadWriteByte(reg_addr); //发送reg地址
	SPI1_ReadWriteByte(data);
	MPU9250_CS_H;
}

//MPU9250_SPI_Write_LEN
//MPU9250的SPI写多个字节函数
// reg_addr 寄存器地址  	len字节数		*data 要写入的数据存放的数组
void MPU9250_SPI_Write_LEN(uint8_t reg_addr, uint8_t len, uint8_t* data)
{
	uint32_t i = 0;
	MPU9250_CS_L;
	SPI1_ReadWriteByte(reg_addr);
	while(i < len)
	{
		SPI1_ReadWriteByte(data[i++]);
	}
	MPU9250_CS_H;
}
//MPU9250_SPI_Read
//MPU9250的读一个字节函数
//reg_addr 寄存器地址
uint8_t MPU9250_SPI_Read(uint8_t reg_addr)
{
	uint8_t dummy = 0;
	uint8_t data = 0;

	MPU9250_CS_L;
	SPI1_ReadWriteByte(0x80 | reg_addr);
	data = SPI1_ReadWriteByte(dummy);
	MPU9250_CS_H;
	return data;
}
//MPU9250_SPI_Read_LEN
//MPU9250的读多个字节函数
//reg_addr 寄存器地址  len	读取字节数	*data数据存放数组
void MPU9250_SPI_Read_LEN(uint8_t reg_addr, uint8_t len, uint8_t* data)
{
	uint32_t i = 0;
	uint8_t dummy = 0x00;

	MPU9250_CS_L;
	SPI1_ReadWriteByte(MPU9250_I2C_READ | reg_addr);
	while(i < len){
		data[i++] = SPI1_ReadWriteByte(dummy);
	}
	MPU9250_CS_H;
}
//以下四个函数，作用是设置磁力计
int MPU9250_AK8963_SPI_Read(uint8_t akm_addr, uint8_t reg_addr, uint8_t* data) 
{
	uint8_t status = 0;
	uint32_t timeout = 0;

	MPU9250_SPI_Write_LEN(MPU9250_I2C_SLV4_REG, 1, &reg_addr);
	LL_mDelay(1);
	reg_addr = akm_addr | MPU9250_I2C_READ;
	MPU9250_SPI_Write_LEN(MPU9250_I2C_SLV4_ADDR, 1, &reg_addr);
	LL_mDelay(1);
	reg_addr = MPU9250_I2C_SLV4_EN;
	MPU9250_SPI_Write_LEN(MPU9250_I2C_SLV4_CTRL, 1, &reg_addr);
	LL_mDelay(1);

	do
	{
		if (timeout++ > 50)
		{
			return -2;
		}
		MPU9250_SPI_Read_LEN(MPU9250_I2C_MST_STATUS, 1, &status);
		LL_mDelay(1);
	} 
	while ((status & MPU9250_I2C_SLV4_DONE) == 0);
	
	MPU9250_SPI_Read_LEN(MPU9250_I2C_SLV4_DI, 1, data);
	return 0;
}

int MPU9250_AK8963_SPI_Read_LEN(uint8_t akm_addr, uint8_t reg_addr, uint8_t len, uint8_t* data)
{
	uint8_t index = 0;
	uint8_t status = 0;
	uint32_t timeout = 0;
	uint8_t tmp = 0;

	tmp = akm_addr | MPU9250_I2C_READ;
	MPU9250_SPI_Write_LEN(MPU9250_I2C_SLV4_ADDR, 1, &tmp);
	LL_mDelay(1);
	while(index < len){
		tmp = reg_addr + index;
		MPU9250_SPI_Write_LEN(MPU9250_I2C_SLV4_REG, 1, &tmp);
		LL_mDelay(1);
		tmp = MPU9250_I2C_SLV4_EN;
		MPU9250_SPI_Write_LEN(MPU9250_I2C_SLV4_CTRL, 1, &tmp);
		LL_mDelay(1);

		do {
			if (timeout++ > 50){
				return -2;
			}
			MPU9250_SPI_Read_LEN(MPU9250_I2C_MST_STATUS, 1, &status);
			LL_mDelay(2);
		} while ((status & MPU9250_I2C_SLV4_DONE) == 0);
		MPU9250_SPI_Read_LEN( MPU9250_I2C_SLV4_DI, 1, data + index);
		LL_mDelay(1);
		index++;
	}
	return 0;
}

int MPU9250_AK8963_SPI_Write(uint8_t akm_addr, uint8_t reg_addr, uint8_t data)
{
	uint32_t timeout = 0;
	uint8_t status = 0;
	uint8_t tmp = 0;

	tmp = akm_addr;
	MPU9250_SPI_Write_LEN(MPU9250_I2C_SLV4_ADDR, 1, &tmp);
	LL_mDelay(1);
	tmp = reg_addr;
	MPU9250_SPI_Write_LEN(MPU9250_I2C_SLV4_REG, 1, &tmp);
	LL_mDelay(1);
	tmp = data;
	MPU9250_SPI_Write_LEN(MPU9250_I2C_SLV4_DO, 1, &tmp);
	LL_mDelay(1);
	tmp = MPU9250_I2C_SLV4_EN;
	MPU9250_SPI_Write_LEN(MPU9250_I2C_SLV4_CTRL, 1, &tmp);
	LL_mDelay(1);

	do {
		if (timeout++ > 50)
			return -2;

		MPU9250_SPI_Read_LEN(MPU9250_I2C_MST_STATUS, 1, &status);
		LL_mDelay(1);
	} while ((status & MPU9250_I2C_SLV4_DONE) == 0);
	if (status & MPU9250_I2C_SLV4_NACK)
		return -3;
	return 0;
}

int MPU9250_AK8963_SPI_Write_LEN(uint8_t akm_addr, uint8_t reg_addr, uint8_t len, uint8_t* data)
{
	uint32_t timeout = 0;
	uint8_t status = 0;
	uint8_t tmp = 0;
	uint8_t index = 0;

	tmp = akm_addr;
	MPU9250_SPI_Write_LEN(MPU9250_I2C_SLV4_ADDR, 1, &tmp);
	LL_mDelay(1);

	while(index < len){
		tmp = reg_addr + index;
		MPU9250_SPI_Write_LEN(MPU9250_I2C_SLV4_REG, 1, &tmp);
		LL_mDelay(1);
		MPU9250_SPI_Write_LEN(MPU9250_I2C_SLV4_DO, 1, data + index);
		LL_mDelay(1);
		tmp = MPU9250_I2C_SLV4_EN;
		MPU9250_SPI_Write_LEN(MPU9250_I2C_SLV4_CTRL, 1, &tmp);
		LL_mDelay(1);

		do {
			if (timeout++ > 50)
				return -2;
			MPU9250_SPI_Read_LEN(MPU9250_I2C_MST_STATUS, 1, &status);
			LL_mDelay(1);
		} while ((status & MPU9250_I2C_SLV4_DONE) == 0);
		if (status & MPU9250_I2C_SLV4_NACK)
			return -3;
		index++;
	}
	return 0;
}


#if 1
uint8_t InitOffset = 0;
void MPU6500_Init(void)
{
	
	//复位MPU9250（mpu6500）
	MPU9250_SPI_Write(MPU9250_PWR_MGMT_1, MPU9250_RESET);  
	LL_mDelay(100);		
	MPU9250_SPI_Write(MPU9250_SIGNAL_PATH_RESET, 0x07); 		
	LL_mDelay(1);	
	MPU9250_SPI_Write(MPU9250_PWR_MGMT_1, 0x03);   		
	LL_mDelay(1);	
	MPU9250_SPI_Write(MPU9250_SMPLRT_DIV, 0x00);   		
	LL_mDelay(1);	
	MPU9250_SPI_Write(MPU9250_CONFIG, 0x01);   		
	LL_mDelay(1);	
	MPU9250_SPI_Write(MPU9250_ACCEL_CONFIG2, 0x05);   		
	LL_mDelay(1);	
	MPU9250_SPI_Write(MPU9250_INT_PIN_CFG, 0x20);   		
	LL_mDelay(1);	
	MPU9250_SPI_Write(MPU9250_INT_ENABLE, 0x01);   		
	LL_mDelay(1);	
	MPU9250_SPI_Write(MPU9250_GYRO_CONFIG, 3<<3);   		
	LL_mDelay(1);	
	MPU9250_SPI_Write(MPU9250_ACCEL_CONFIG, 1<<3);   		
	LL_mDelay(1);	
}

int16_XYZ ACC_RealData,GRY_RealData;
uint16_t Peace, PeaceTime;  
uint16_t turbulen;

void READ_MPU6500(void)
{
	uint8_t BUF_6500_buff[14];
	int16_XYZ Old_GRY;
	
	Old_GRY.X=GRY_RealData.X;
	Old_GRY.Y=GRY_RealData.Y;
	Old_GRY.Z=GRY_RealData.Z;
	
  MPU9250_SPI_Read_LEN(MPU9250_ACCEL_XOUT_H, 14,BUF_6500_buff);
	ACC_RealData.X=(( ((int16_t)BUF_6500_buff[0]) <<8) |BUF_6500_buff[1]);
	ACC_RealData.Y=(( ((int16_t)BUF_6500_buff[2]) <<8) |BUF_6500_buff[3]);	
	ACC_RealData.Z=(( ((int16_t)BUF_6500_buff[4]) <<8) |BUF_6500_buff[5]);

	GRY_RealData.X=(( ((int16_t)BUF_6500_buff[8]) <<8) |BUF_6500_buff[9]);
	GRY_RealData.Y=(( ((int16_t)BUF_6500_buff[10]) <<8) |BUF_6500_buff[11]);	
	GRY_RealData.Z=(( ((int16_t)BUF_6500_buff[12]) <<8) |BUF_6500_buff[13]);	

	if(Gyro_Offset_Flag==true)
	{ 	
		GRY_RealData.X -= GRY_Offset.X;
		GRY_RealData.Y -= GRY_Offset.Y;
		GRY_RealData.Z -= GRY_Offset.Z;
  }
	
	turbulen=abs(Old_GRY.X-GRY_RealData.X)+abs(Old_GRY.Y-GRY_RealData.Y)+abs(Old_GRY.Z-GRY_RealData.Z);	
	
	if(turbulen < 20) Peace++;  else {Peace = 0; GRY_Stable_Flag = false;} 
	
	if(Peace > 100) GRY_Stable_Flag = true;

	if(InitOffset == 0)
	{	   
        if(Peace > 100) InitOffset = 1;
	}
	
	
}

void Get_GRYOffset(void)
{ 
	uint8_t i;
	int_fast32_t  GRY_XTemp=0,GRY_YTemp=0,GRY_ZTemp=0;
	Gyro_Offset_Flag=false;
	
  for(i=0;i<30;i++)	
	{
		READ_MPU6500();
		GRY_XTemp+=GRY_RealData.X;
		GRY_YTemp+=GRY_RealData.Y;
		GRY_ZTemp+=GRY_RealData.Z;
		LL_mDelay(5);	
 	}		
	GRY_XTemp/=30;
	GRY_YTemp/=30;
	GRY_ZTemp/=30;

 	GRY_Offset.X=GRY_XTemp;
 	GRY_Offset.Y=GRY_YTemp;
 	GRY_Offset.Z=GRY_ZTemp;	
	Gyro_Offset_Flag=true;
}

float_XYZ ACC_Est;
float ACC_EstZsum, ACC_EstZCount;


//==================================================================================
//转换成地理坐标系（加速度计）
//
//==================================================================================
// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void rotateV(float_XYZ *v,float_RPY *delta)
{
    float_XYZ v_tmp = *v;	

    // This does a  "proper" matrix rotation using gyro deltas without small-angle approximation
    float mat[3][3];
    float cosx, sinx, cosy, siny, cosz, sinz;
    float coszcosx, coszcosy, sinzcosx, coszsinx, sinzsinx;

    cosx = cosf(-delta->Roll);				//rool	
    sinx = sinf(-delta->Roll);				//rool
    cosy = cosf(-delta->Pitch);				//pitch
    siny = sinf(-delta->Pitch);				//pitch
    cosz = cosf(-delta->Yaw);				//yaw
    sinz = sinf(-delta->Yaw);				//yaw	

    coszcosx = cosz * cosx;
    coszcosy = cosz * cosy;
    sinzcosx = sinz * cosx;
    coszsinx = sinx * cosz;
    sinzsinx = sinx * sinz;
	
    mat[0][0] = coszcosy;
    mat[0][1] = -cosy * sinz;
    mat[0][2] = siny;
    mat[1][0] = sinzcosx + (coszsinx * siny);
    mat[1][1] = coszcosx - (sinzsinx * siny);
    mat[1][2] = -sinx * cosy;
    mat[2][0] = (sinzsinx) - (coszcosx * siny);
    mat[2][1] = (coszsinx) + (sinzcosx * siny);
    mat[2][2] = cosy * cosx;

    v->X = v_tmp.X * mat[0][0] + v_tmp.Y * mat[1][0] + v_tmp.Z * mat[2][0];
    v->Y = v_tmp.X * mat[0][1] + v_tmp.Y * mat[1][1] + v_tmp.Z * mat[2][1];
    v->Z = v_tmp.X * mat[0][2] + v_tmp.Y * mat[1][2] + v_tmp.Z * mat[2][2];
}


void Cal_TsData(void) 
{ 	 
	acce_f.X = ACC_RealData.X * ACC_Gain;
  acce_f.Y = ACC_RealData.Y * ACC_Gain;
	acce_f.Z = ACC_RealData.Z * ACC_Gain;
    
	gyro_f.X = GRY_RealData.X * Gyr_Gain;
	gyro_f.Y = GRY_RealData.Y * Gyr_Gain;
	gyro_f.Z = GRY_RealData.Z * Gyr_Gain;
	
	
	ACC_Est.Y = acce_f.X;	
  ACC_Est.X = -acce_f.Y;	
  ACC_Est.Z = acce_f.Z;	
	
	Q_Rad.Pitch = Q_ANGLE.Pitch / 57.3f;
	Q_Rad.Roll = Q_ANGLE.Roll / 57.3f;
	Q_Rad.Yaw = Q_ANGLE.Yaw / 57.3f;
 
  rotateV(&ACC_Est,&Q_Rad);	//获得地理坐标系的 加速度值ACC_Est

	ACC_EstZCount++;
	ACC_EstZsum += ACC_Est.Z;

}

float get_acc_mod(void)
{
  float AccMod = sqrtf(acce_f.X*acce_f.X+acce_f.Y*acce_f.Y+acce_f.Z*acce_f.Z); 
  return AccMod;
}

void MPU9250_Get_Data(void)
{
	
	printff(USART3,"\r\n",strlen("\r\n"));
	printff(USART3,"\r\n",strlen("\r\n"));
	while(1)
	{	
  	printff(USART3, "-----------------------------------\r\n", strlen("-----------------------------------\r\n"));
	  printff(USART3,"**************MPU9250**************\r\n",strlen("**************MPU9250**************\r\n"));
		READ_MPU6500();    //读取IMU原始数据
		Cal_TsData();      //IMU原始数据计算转换
		attitude_quat_5ms_task();    //姿态解算
	  printff(USART3, "\r\n-----------------------------------\r\n\r\n", strlen("\r\n-----------------------------------\r\n\r\n"));
    LL_mDelay(200);
	}
}

#endif















