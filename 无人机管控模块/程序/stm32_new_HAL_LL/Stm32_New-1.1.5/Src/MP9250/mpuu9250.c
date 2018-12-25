#include "mpuu9250.h"
#include "mp9250.h"
#include "stm32l4xx_hal.h"
#include "usart.h"
#include "string.h"
#include "MPU9250_F.h"

uint8_t response[3] = {0, 0, 0};
extern uint8_t Mag_Offset[3];
extern int32_t Gyo_Offset[3];
extern short accel[3],gyro[3],mag[3];
extern float Magx ,Magy, Magz ;
extern float Mag[];
//////////////////////////////////////////////////////////////////////////
enum MPU9250_GYRO_DLPF 
{
    MPU9250_GYRO_DLPF_250HZ = 0,
    MPU9250_GYRO_DLPF_184HZ,
    MPU9250_GYRO_DLPF_92HZ,
    MPU9250_GYRO_DLPF_41HZ,
    MPU9250_GYRO_DLPF_20HZ,
    MPU9250_GYRO_DLPF_10HZ,
    MPU9250_GYRO_DLPF_5HZ,
    MPU9250_GYRO_DLPF_3600HZ,
    NUM_GYRO_DLPF
};
enum MPU9250_GYRO_FSR 
{
    MPU9250_FSR_250DPS = 0,
    MPU9250_FSR_500DPS,
    MPU9250_FSR_1000DPS,
    MPU9250_FSR_2000DPS,
    MPU9250_NUM_GYRO_FSR
};
enum MPU9250_ACCEL_DLPF 
{
    MPU9250_ACCEL_DLPF_460HZ = 0,
    MPU9250_ACCEL_DLPF_184HZ,
    MPU9250_ACCEL_DLPF_92HZ,
    MPU9250_ACCEL_DLPF_41HZ,
    MPU9250_ACCEL_DLPF_20HZ,
    MPU9250_ACCEL_DLPF_10HZ,
    MPU9250_ACCEL_DLPF_5HZ,
    MPU9250_ACCEL_DLPF_460HZ2,
    MPU9250_NUM_ACCEL_DLPF
};
enum MPU9250_ACCEL_FSR 
{
    MPU9250_FSR_2G = 0,
    MPU9250_FSR_4G,
    MPU9250_FSR_8G,
    MPU9250_FSR_16G,
    MPU9250_NUM_ACCEL_FSR
};
enum MPU9250_CLK 
{
    MPU9250_CLK_INTERNAL = 0,
    MPU9250_CLK_PLL,
    MPU9250_NUM_CLK
};
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
//MPU9250��SPIдһ���ֽں���
//reg_addr �Ĵ�����ַ  data Ҫд�������
void MPU9250_SPI_Write(uint8_t reg_addr, uint8_t data)
{
	MPU9250_CS_L;
	SPI1_ReadWriteByte(reg_addr); //����reg��ַ
	SPI1_ReadWriteByte(data);
	MPU9250_CS_H;
}

//MPU9250_SPI_Write_LEN
//MPU9250��SPIд����ֽں���
// reg_addr �Ĵ�����ַ  	len�ֽ���		*data Ҫд������ݴ�ŵ�����
void MPU9250_SPI_Write_LEN(uint8_t reg_addr, uint8_t len, uint8_t* data)
{
	u32 i = 0;
	MPU9250_CS_L;
	SPI1_ReadWriteByte(reg_addr);
	while(i < len)
	{
		SPI1_ReadWriteByte(data[i++]);
	}
	MPU9250_CS_H;
}
//MPU9250_SPI_Read
//MPU9250�Ķ�һ���ֽں���
//reg_addr �Ĵ�����ַ
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
//MPU9250�Ķ�����ֽں���
//reg_addr �Ĵ�����ַ  len	��ȡ�ֽ���	*data���ݴ������
void MPU9250_SPI_Read_LEN(uint8_t reg_addr, uint8_t len, uint8_t* data)
{
	u32 i = 0;
	uint8_t dummy = 0x00;

	MPU9250_CS_L;
	SPI1_ReadWriteByte(MPU9250_I2C_READ | reg_addr);
	while(i < len){
		data[i++] = SPI1_ReadWriteByte(dummy);
	}
	MPU9250_CS_H;
}
//�����ĸ����������������ô�����
int MPU9250_AK8963_SPI_Read(uint8_t akm_addr, uint8_t reg_addr, uint8_t* data) 
{
	uint8_t status = 0;
	u32 timeout = 0;

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
	u32 timeout = 0;
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
	u32 timeout = 0;
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
	u32 timeout = 0;
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
#if 0
// MPU9250_Get9AxisRawData
//MPU9250��ȡ��������
// *accel ���ټ����� *gyro����������	*mag ����������	���� �����СӦΪ3	x y z
void MPU9250_Get9AxisRawData(short *accel, short * gyro, short * mag)
{
	uint8_t data[22];
	MPU9250_SPI_Read_LEN(MPU9250_ACCEL_XOUT_H, 22, data);
	
	accel[0] = (data[0] << 8) | data[1];
	accel[1] = (data[2] << 8) | data[3];
	accel[2] = (data[4] << 8) | data[5];
	
	gyro[0] = (data[8] << 8)  | data[9];
	gyro[1] = (data[10] << 8) | data[11];
	gyro[2] = (data[12] << 8) | data[13];

	if (!(data[14] & MPU9250_AK8963_DATA_READY) || (data[14] & MPU9250_AK8963_DATA_OVERRUN)){
		return;
	}
	if (data[21] & MPU9250_AK8963_OVERFLOW){
		return;
	}
	mag[0] = (data[16] << 8) | data[15];
	mag[1] = (data[18] << 8) | data[17];
	mag[2] = (data[20] << 8) | data[19];

	//ned x,y,z
	mag[0] = ((long)mag[0] * MPU9250_AK8963_ASA[0]) >> 8;
	mag[1] = ((long)mag[1] * MPU9250_AK8963_ASA[1]) >> 8;
	mag[2] = ((long)mag[2] * MPU9250_AK8963_ASA[2]) >> 8;
}
#endif
#if 1
//MPU9250_Init
//MPU9250��ʼ��
void MMPU9250_Init(void)
{
	uint8_t data = 0, state = 0;
	uint8_t res = 1;
	uint8_t WhoAmI=0;
	
	char TEST10[20] = {0};
	char TEST11[20] = {0};
	char TEST12[20] = {0};
	
	//////////////////////////////////////////////////////////////////////////
	//MPU9250 Reset 
	MPU9250_SPI_Write(MPU9250_PWR_MGMT_1, MPU9250_RESET);  //��λMPU9250 		
	LL_mDelay(100);		
	//MPU9250 Set Clock Source
	MPU9250_SPI_Write(MPU9250_PWR_MGMT_1, MPU9250_CLOCK_PLLGYROZ);   //��ʼ��MPU9250ʱ��Դ
	LL_mDelay(1);	
	//MPU9250 Set Interrupt
	//Interrupt status is cleared if any read operation is performed.
	MPU9250_SPI_Write(MPU9250_INT_PIN_CFG, MPU9250_INT_ANYRD_2CLEAR);
	LL_mDelay(1);
	MPU9250_SPI_Write(MPU9250_INT_ENABLE, DISABLE);
	LL_mDelay(1);
	//MPU9250 Set Sensors
	//Enable all xyz
	MPU9250_SPI_Write(MPU9250_PWR_MGMT_2, MPU9250_XYZ_GYRO & MPU9250_XYZ_ACCEL);   //ʹ�������Ǻͼ��ٶȼ�
	LL_mDelay(1);
	//MPU9250 Set SampleRate
	//SAMPLE_RATE = Internal_Sample_Rate / (1 + SMPLRT_DIV)
	MPU9250_SPI_Write(MPU9250_SMPLRT_DIV, SMPLRT_DIV);     //���ò����� Internal_Sample_Rate/(1 + 25)  �ڲ������ʣ�����
	LL_mDelay(1);
	//MPU9250 Set Full Scale Gyro Range
	//Fchoice_b[1:0] = [00] enable DLPF
	MPU9250_SPI_Write(MPU9250_GYRO_CONFIG, (MPU9250_FSR_1000DPS << 3));
	LL_mDelay(1);
	//MPU9250 Set Full Scale Accel Range PS:2G
	MPU9250_SPI_Write(MPU9250_ACCEL_CONFIG, (MPU9250_FSR_2G << 3));    //���ü��ٶ����� �� 2g
	LL_mDelay(1);
	//MPU9250 Set Accel DLPF
	data = MPU9250_SPI_Read(MPU9250_ACCEL_CONFIG2);    //���ü��ٶ��˲���
	data |= MPU9250_ACCEL_DLPF_41HZ;
	LL_mDelay(1);
	MPU9250_SPI_Write(MPU9250_ACCEL_CONFIG2, data);
	LL_mDelay(1);
	//MPU9250 Set Gyro DLPF
	MPU9250_SPI_Write(MPU9250_CONFIG, MPU9250_GYRO_DLPF_41HZ);   // �����������˲���
	LL_mDelay(1);
	//MPU9250 Set SPI Mode
	state = MPU9250_SPI_Read(MPU9250_USER_CTRL);
	LL_mDelay(1);
	//Reset I2C Slave module and put the serial interface in SPI mode only. This bit auto clears after one clock cycle.
	MPU9250_SPI_Write(MPU9250_USER_CTRL, state | MPU9250_I2C_IF_DIS);
	LL_mDelay(1);
	state = MPU9250_SPI_Read(MPU9250_USER_CTRL);
	LL_mDelay(1);
	//Enable the I2C Master I/F module; pins ES_DA and ES_SCL are isolated from pins SDA/SDI and SCL/ SCLK.
	MPU9250_SPI_Write(MPU9250_USER_CTRL, state | MPU9250_I2C_MST_EN);
	LL_mDelay(1);
	//////////////////////////////////////////////////////////////////////////
	//AK8963 Setup
	//reset AK8963
//	AK8963_SelfTest();
	//���Ĵ�����ģʽ�ȸ�λ�������ڽ���POWER DOWNģʽ���������POWER DOWN �� ����ģʽ��Ч

	data = MPU9250_SPI_Read(MPU9250_INT_PIN_CFG);    
  LL_mDelay(1);
	MPU9250_SPI_Write(MPU9250_INT_PIN_CFG, (0xfd & data));   // DISable Bypassģʽ
	LL_mDelay(1);
	MPU9250_AK8963_SPI_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL2, MPU9250_AK8963_CNTL2_SRST);  //��λ������
	LL_mDelay(2);
	//BIT: Output bit setting 4800uT
	// "1": 16-bit output 
	MPU9250_AK8963_SPI_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_POWER_DOWN);
	LL_mDelay(1);
	//Fuse ROM access mode
	MPU9250_AK8963_SPI_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_FUSE_ROM_ACCESS);  //ÿ����������ȵ������ݴ洢��Fuse ROM��
	LL_mDelay(1);
	MPU9250_AK8963_SPI_Read(MPU9250_AK8963_I2C_ADDR,MPU9250_AK8963_CNTL,&res);//����MPU9250_AK8963_SPI_Read
	LL_mDelay(1);
//  if (res == MPU9250_AK8963_FUSE_ROM_ACCESS)
//	printff(USART3, "���ô�����FUSE ROMģʽ�ɹ�\r\n",strlen("���ô�����FUSE ROMģʽ�ɹ�\r\n"));
//	else
	printff(USART3, "�ȴ����ô�����FUSE ROMģʽ\r\n",strlen("�ȴ����ô�����FUSE ROMģʽ\r\n"));
	while(!(res == MPU9250_AK8963_FUSE_ROM_ACCESS))
	{
		MPU9250_AK8963_SPI_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL2, MPU9250_AK8963_CNTL2_SRST);  //��λ������
  	LL_mDelay(2);
		//BIT: Output bit setting 4800uT
		// "1": 16-bit output 
		MPU9250_AK8963_SPI_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_POWER_DOWN);
		LL_mDelay(1);
		//Fuse ROM access mode
		MPU9250_AK8963_SPI_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_FUSE_ROM_ACCESS);  //ÿ����������ȵ������ݴ洢��Fuse ROM��
		LL_mDelay(1);
		MPU9250_AK8963_SPI_Read(MPU9250_AK8963_I2C_ADDR,MPU9250_AK8963_CNTL,&res);//����MPU9250_AK8963_SPI_Read
		LL_mDelay(1);
	}
	printff(USART3, "���ô�����FUSE ROMģʽ�ɹ�\r\n",strlen("���ô�����FUSE ROMģʽ�ɹ�\r\n"));
	//AK8963 get calibration data
	//Magnetic sensor X-axis sensitivity adjustment value
	MPU9250_AK8963_SPI_Read_LEN(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_ASAX, 3, response);
	LL_mDelay(10);
	//AK8963_SENSITIVITY_SCALE_FACTOR
	//����У׼������     ÿ���ϵ�������У׼������������ͬ������������������������
	MPU9250_AK8963_ASA[0] = (short)(response[0]) + 128;
	MPU9250_AK8963_ASA[1] = (short)(response[1]) + 128;
	MPU9250_AK8963_ASA[2] = (short)(response[2]) + 128;
	
		sprintf( TEST10, "%u", response[0]); 
  	sprintf( TEST11, "%u", response[1]); 
	  sprintf( TEST12, "%u", response[2]); 
	  printff(USART3, "response:\r\n",strlen("response:\r\n"));
		printff(USART3, TEST10,strlen(TEST10));   printff(USART3, "\t",strlen("\t"));
		printff(USART3, TEST11,strlen(TEST11));   printff(USART3, "\t",strlen("\t"));
		printff(USART3, TEST12,strlen(TEST12));   printff(USART3, "\t",strlen("\t"));
	  printff(USART3, "\r\n",strlen("\r\n"));
	LL_mDelay(1);
	
	MPU9250_AK8963_SPI_Write(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL2_REG,AK8963_CNTL2_SRST); // Reset AK8963
  LL_mDelay(1);
	MPU9250_AK8963_SPI_Read(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL1_REG,&res);
	LL_mDelay(2);
	printff(USART3, "�ȴ������Ƹ�λ\r\n",strlen("�ȴ������Ƹ�λ\r\n"));	 
  while(!(res == 0x00))
	{
		MPU9250_AK8963_SPI_Write(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL2_REG,AK8963_CNTL2_SRST); // Reset AK8963
		LL_mDelay(2);
		MPU9250_AK8963_SPI_Read(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL1_REG,&res);
		LL_mDelay(1);
	}
	printff(USART3, "�����Ƹ�λ�ɹ�\r\n",strlen("�����Ƹ�λ�ɹ�\r\n"));	 
	MPU9250_AK8963_SPI_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_POWER_DOWN);
	LL_mDelay(1);
	MPU9250_AK8963_SPI_Read(MPU9250_AK8963_I2C_ADDR,MPU9250_AK8963_CNTL,&res);
	LL_mDelay(1);
	printff(USART3, "�ȴ�������POWER DOWN\r\n",strlen("�ȴ�������POWER DOWN\r\n"));
	while( !(res == MPU9250_AK8963_POWER_DOWN))
	{
		MPU9250_AK8963_SPI_Write(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL1_REG,MPU9250_AK8963_POWER_DOWN); 
		LL_mDelay(1);
	}
	printff(USART3, "���ô�����POWER DOWNģʽ�ɹ�\r\n",strlen("���ô�����POWER DOWNģʽ�ɹ�\r\n"));
	//there is a stop between reads.
	MPU9250_SPI_Write(MPU9250_I2C_MST_CTRL, 0x5D);//400kHz��IICͨ������	
	LL_mDelay(1);
	MPU9250_SPI_Write(MPU9250_I2C_SLV0_ADDR, MPU9250_AK8963_I2C_ADDR | MPU9250_I2C_READ);
	LL_mDelay(1);
	MPU9250_SPI_Write(MPU9250_I2C_SLV0_REG, MPU9250_AK8963_ST1);
	LL_mDelay(1);
	//Enable reading data from this slave at the sample rate and storing data at the
	//first available EXT_SENS_DATA register, which is always EXT_SENS_DATA_00 for I2C slave 0.
	MPU9250_SPI_Write(MPU9250_I2C_SLV0_CTRL, 0x88);
	LL_mDelay(1);
	MPU9250_SPI_Write(MPU9250_I2C_SLV4_CTRL, 0x09);
	LL_mDelay(1);
	//When enabled, slave 0 will only be accessed 1+I2C_MST_DLY) samples as determined by SMPLRT_DIV and DLPF_CFG
	MPU9250_SPI_Write(MPU9250_I2C_MST_DELAY_CTRL, 0x81);
	LL_mDelay(100);
	

//	//��������   
//	MPU9250_AK8963_SPI_Write(MPU9250_AK8963_I2C_ADDR,MPU9250_AK8963_CNTL, MPU9250_AK8963_CONTINUOUS_MEASUREMENT);
//	LL_mDelay(1);
//	MPU9250_AK8963_SPI_Read(MPU9250_AK8963_I2C_ADDR,MPU9250_AK8963_CNTL,&res);//����MPU9250_AK8963_SPI_Read
//	LL_mDelay(1);
//	printff(USART3, "�ȴ�������������������ģʽ2\r\n",strlen("�ȴ�������������������ģʽ2\r\n"));
//	while(!(res == MPU9250_AK8963_CONTINUOUS_MEASUREMENT))
//	{
//		MPU9250_AK8963_SPI_Write(MPU9250_AK8963_I2C_ADDR,MPU9250_AK8963_CNTL, MPU9250_AK8963_CONTINUOUS_MEASUREMENT);
//		LL_mDelay(1);
//		MPU9250_AK8963_SPI_Read(MPU9250_AK8963_I2C_ADDR,MPU9250_AK8963_CNTL,&res);//����MPU9250_AK8963_SPI_Read
//		LL_mDelay(1);
//	}
//	printff(USART3, "������������������ģʽ2�ɹ�\r\n",strlen("������������������ģʽ2�ɹ�\r\n"));
 //printf("MPU9250_AK8963_CNTL = 0x%X\r\n",res);//Ӧ����0x16
	//function is disabled for this slave
//	MPU9250_SPI_Write(MPU9250_I2C_SLV4_CTRL, 0x09);
//	LL_mDelay(1);
//	//When enabled, slave 0 will only be accessed 1+I2C_MST_DLY) samples as determined by SMPLRT_DIV and DLPF_CFG
//	MPU9250_SPI_Write(MPU9250_I2C_MST_DELAY_CTRL, 0x81);
//	LL_mDelay(100);
	
//	MPU9250_AK8963_SPI_Write(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL2_REG,AK8963_CNTL2_SRST); // Reset AK8963
//  LL_mDelay(1);
//	MPU9250_AK8963_SPI_Read(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL1_REG,&res);
//	LL_mDelay(1);
////	if (res == 0x00)
//	printff(USART3, "�ȴ������Ƹ�λ\r\n",strlen("�ȴ������Ƹ�λ\r\n"));	 
//  while(!(res == 0x00))
//	{
//		MPU9250_AK8963_SPI_Write(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL2_REG,AK8963_CNTL2_SRST); // Reset AK8963
//		LL_mDelay(1);
//		MPU9250_AK8963_SPI_Read(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL1_REG,&res);
//		LL_mDelay(1);
//	}
//	printff(USART3, "�����Ƹ�λ�ɹ�\r\n",strlen("�����Ƹ�λ�ɹ�\r\n"));	 
//  LL_mDelay(1);	
//	MPU9250_AK8963_SPI_Write(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL1_REG,MPU9250_AK8963_POWER_DOWN);   //Ҫ��������ģʽ�Ƚ����ô�����ΪPOWER DOWNģʽ�����ٵȴ�100us
//	LL_mDelay(1);
//	MPU9250_AK8963_SPI_Read(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL1_REG,&res);
//	LL_mDelay(1);
//	printff(USART3, "�ȴ�������POWER DOWN\r\n",strlen("�ȴ�������POWER DOWN\r\n"));
//	while( !(res == MPU9250_AK8963_POWER_DOWN))
//	{
//		MPU9250_AK8963_SPI_Write(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL1_REG,MPU9250_AK8963_POWER_DOWN); 
//		LL_mDelay(1);
//	}
//	printff(USART3, "���ô�����POWER DOWNģʽ�ɹ�\r\n",strlen("���ô�����POWER DOWNģʽ�ɹ�\r\n"));
//	printff(USART3, "���ô�����16λ���ģʽ2�ɹ�\r\n",strlen("���ô�����16λ���ģʽ2�ɹ�\r\n"));
//	MPU9250_AK8963_SPI_Write(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL1_REG,MPU9250_AK8963_CONTINUOUS_MEASUREMENT);       // use i2c to set AK8963 working on Continuous measurement mode1 & 16-bit output	
//  LL_mDelay(1);
//	MPU9250_AK8963_SPI_Read(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL1_REG,&res);
//	LL_mDelay(1);
////	if (res == 0x16)
////	printff(USART3, "���ô�����16λ���ģʽ2�ɹ�\r\n",strlen("���ô�����16λ���ģʽ2�ɹ�\r\n"));
////  else
//	printff(USART3, "�ȴ����ô�����16λ���ģʽ2\r\n",strlen("�ȴ����ô�����16λ���ģʽ2\r\n"));
//	while( !(res == MPU9250_AK8963_CONTINUOUS_MEASUREMENT))
//	{
//		MPU9250_AK8963_SPI_Write(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL1_REG,MPU9250_AK8963_CONTINUOUS_MEASUREMENT);       // use i2c to set AK8963 working on Continuous measurement mode1 & 16-bit output	
//		LL_mDelay(1);
//		MPU9250_AK8963_SPI_Read(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL1_REG,&res);
//		LL_mDelay(1);
//	}
//	printff(USART3, "���ô�����16λ���ģʽ2�ɹ�\r\n",strlen("���ô�����16λ���ģʽ2�ɹ�\r\n"));
	
	MPU9250_Calibration();
	
	//��֤��ʼ�����
	WhoAmI=MPU9250_SPI_Read(WHO_AM_I); 
	if (WhoAmI == 0x71)
		printff(USART3, "��ȡMPU6500 ID �ɹ�\r\n",strlen("��ȡMPU6500 ID �ɹ�\r\n"));	 		
}

void  AK8963_SelfTest(void)
{
	uint8_t res = 0;
	uint8_t byte = 0;
	uint8_t data[7];
	short mx, my, mz;
	int i = 0;
	//��λ������ Ȼ��POWER DOWN
	MPU9250_AK8963_SPI_Write(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL2_REG,AK8963_CNTL2_SRST); // Reset AK8963
  LL_mDelay(1);
	MPU9250_AK8963_SPI_Read(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL1_REG,&res);
	LL_mDelay(2);
	printff(USART3, "�ȴ������Ƹ�λ\r\n",strlen("�ȴ������Ƹ�λ\r\n"));	 
  while(!(res == 0x00))
	{
		MPU9250_AK8963_SPI_Write(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL2_REG,AK8963_CNTL2_SRST); // Reset AK8963
		LL_mDelay(2);
		MPU9250_AK8963_SPI_Read(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL1_REG,&res);
		LL_mDelay(1);
	}
	printff(USART3, "�����Ƹ�λ�ɹ�\r\n",strlen("�����Ƹ�λ�ɹ�\r\n"));	 
	MPU9250_AK8963_SPI_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_POWER_DOWN);
	LL_mDelay(1);
	MPU9250_AK8963_SPI_Read(MPU9250_AK8963_I2C_ADDR,MPU9250_AK8963_CNTL,&res);
	LL_mDelay(1);
	printff(USART3, "�ȴ�������POWER DOWN\r\n",strlen("�ȴ�������POWER DOWN\r\n"));
	while( !(res == MPU9250_AK8963_POWER_DOWN))
	{
		MPU9250_AK8963_SPI_Write(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL1_REG,MPU9250_AK8963_POWER_DOWN); 
		LL_mDelay(1);
	}
	printff(USART3, "���ô�����POWER DOWNģʽ�ɹ�\r\n",strlen("���ô�����POWER DOWNģʽ�ɹ�\r\n"));
	MPU9250_AK8963_SPI_Read(MPU9250_AK8963_I2C_ADDR,MPU9250_AK8963_ASTC,&byte);
	LL_mDelay(1);
	byte = byte | 0x40;  //ֻ�����Լ�λ������λ����
	
	MPU9250_AK8963_SPI_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_ASTC, byte);
	LL_mDelay(1);
	MPU9250_AK8963_SPI_Read(MPU9250_AK8963_I2C_ADDR,MPU9250_AK8963_ASTC,&res);
	LL_mDelay(1);
	printff(USART3, "�ȴ������������Լ�λ\r\n",strlen("�ȴ������������Լ�λ\r\n"));
	while( !(res == byte))
	{
		MPU9250_AK8963_SPI_Write(MPU9250_AK8963_I2C_ADDR,MPU9250_AK8963_ASTC,byte); 
		LL_mDelay(1);
	}
	printff(USART3, "�����������Լ�λ�ɹ�\r\n",strlen("�����������Լ�λ�ɹ�\r\n"));
	//���ô�����Ϊ�Լ�ģʽ
	MPU9250_AK8963_SPI_Write(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL1_REG,MPU9250_AK8963_SelfTest);       // use i2c to set AK8963 working on Continuous measurement mode1 & 16-bit output	
  LL_mDelay(1);
	MPU9250_AK8963_SPI_Read(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL1_REG,&res);
	LL_mDelay(1);
	printff(USART3, "�ȴ����ô�����Ϊ�Լ�ģʽ\r\n",strlen("�ȴ����ô�����Ϊ�Լ�ģʽ\r\n"));
	while( !(res == MPU9250_AK8963_SelfTest))
	{
		MPU9250_AK8963_SPI_Write(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL1_REG,MPU9250_AK8963_SelfTest);       // use i2c to set AK8963 working on Continuous measurement mode1 & 16-bit output	
		LL_mDelay(1);
		MPU9250_AK8963_SPI_Read(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL1_REG,&res);
		LL_mDelay(1);
	}
	printff(USART3, "���ô������Լ�ģʽ�ɹ�\r\n",strlen("���ô������Լ�ģʽ�ɹ�\r\n"));
	//ͨ����ST2�Ĵ��������ST1�Ĵ���  ǰ��������������ģʽ���ⲿ��������ģʽ��  ******************************************************************
//	MPU9250_AK8963_SPI_Read(MPU9250_AK8963_I2C_ADDR,AK8963_ST2_REG,&res);   
//	LL_mDelay(1);
	MPU9250_AK8963_SPI_Read(MPU9250_AK8963_I2C_ADDR,AK8963_ST1_REG,&res);  
	LL_mDelay(1);
	printff(USART3, "�ȴ�����������׼����\r\n",strlen("�ȴ�����������׼����\r\n"));
	while (!res )   //������׼���ú�ST1״̬�Ĵ�����Ϊ0
	{
		MPU9250_AK8963_SPI_Read(MPU9250_AK8963_I2C_ADDR,AK8963_ST1_REG,&res);  
		LL_mDelay(1);
	}
	printff(USART3, "�����������Ѿ�׼����\r\n",strlen("�����������Ѿ�׼����\r\n"));
	//��ȡ����������
	for(i = 0; i < 6; i++)
	{
	  MPU9250_AK8963_SPI_Read(MPU9250_AK8963_I2C_ADDR,0x03 + i,&data[i]);
	  LL_mDelay(1);
	};
	mx = (data[1] << 8) | data[0];
	my = (data[3] << 8) | data[2];
	mz = (data[5] << 8) | data[4];
	
	MPU9250_AK8963_SPI_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_POWER_DOWN);
	LL_mDelay(1);
	MPU9250_AK8963_SPI_Read(MPU9250_AK8963_I2C_ADDR,MPU9250_AK8963_CNTL,&res);
	LL_mDelay(1);
	printff(USART3, "�ȴ�������POWER DOWN\r\n",strlen("�ȴ�������POWER DOWN\r\n"));
	while( !(res == MPU9250_AK8963_POWER_DOWN))
	{
		MPU9250_AK8963_SPI_Write(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL1_REG,MPU9250_AK8963_POWER_DOWN); 
		LL_mDelay(1);
	}
	printff(USART3, "���ô�����POWER DOWNģʽ�ɹ�\r\n",strlen("���ô�����POWER DOWNģʽ�ɹ�\r\n"));
  if( (-200 < mx) && (mx < 200))
		printff(USART3, "������X���Լ�ɹ�\r\n",strlen("������X���Լ�ɹ�\r\n"));
	else
		printff(USART3, "������X���Լ�ʧ��\r\n",strlen("������X���Լ�ʧ��\r\n"));
	
  if( (-200 < my) && (my < 200))
		printff(USART3, "������Y���Լ�ɹ�\r\n",strlen("������Y���Լ�ɹ�\r\n"));
	else
		printff(USART3, "������Y���Լ�ʧ��\r\n",strlen("������Y���Լ�ʧ��\r\n"));
	
  if( (-3200 < mz) && (mz < -800))
		printff(USART3, "������Z���Լ�ɹ�\r\n",strlen("������Z���Լ�ɹ�\r\n"));
	else
		printff(USART3, "������Z���Լ�ʧ��\r\n",strlen("������Z���Լ�ʧ��\r\n"));
}

// MPU9250_Get9AxisRawData
// MPU9250��ȡ��������  �Ե��β���ģʽ
// *accel ���ټ����� *gyro����������	*mag ����������	���� �����СӦΪ3	x y z
void MPU9250_Get9AxisRawData(short *accel, short * gyro, short * mag)
{
	uint8_t data[25];
	uint8_t res;
	int i;
	char TEST1[20] = {0};
	char TEST2[20] = {0};
	char TEST3[20] = {0};
	char TEST4[20] = {0};
	char TEST5[20] = {0};
	char TEST6[20] = {0};
	char TEST7[20] = {0};
	char TEST8[20] = {0};
	char TEST9[20] = {0};
	
	//���Ĵ�����ģʽ�ȸ�λ�������ڽ���POWER DOWNģʽ���������POWER DOWN �� ����ģʽ��Ч
	MPU9250_SPI_Write(MPU9250_I2C_SLV4_CTRL, 0x09);
	LL_mDelay(1);
	//When enabled, slave 0 will only be accessed 1+I2C_MST_DLY) samples as determined by SMPLRT_DIV and DLPF_CFG
	MPU9250_SPI_Write(MPU9250_I2C_MST_DELAY_CTRL, 0x81);
	LL_mDelay(100);
	
	MPU9250_AK8963_SPI_Write(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL2_REG,AK8963_CNTL2_SRST); // Reset AK8963
  LL_mDelay(1);
	MPU9250_AK8963_SPI_Read(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL1_REG,&res);
	LL_mDelay(1); 
  while(!(res == 0x00))
	{
		MPU9250_AK8963_SPI_Write(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL2_REG,AK8963_CNTL2_SRST); // Reset AK8963
		LL_mDelay(1);
		MPU9250_AK8963_SPI_Read(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL1_REG,&res);
		LL_mDelay(1);
		printff(USART3, "�����Ƹ�λʧ��\r\n",strlen("�����Ƹ�λʧ��\r\n"));	
	} 
  LL_mDelay(1);	
	MPU9250_AK8963_SPI_Write(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL1_REG,MPU9250_AK8963_POWER_DOWN);   //Ҫ��������ģʽ�Ƚ����ô�����ΪPOWER DOWNģʽ�����ٵȴ�100us
	LL_mDelay(1);
	MPU9250_AK8963_SPI_Read(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL1_REG,&res);
	LL_mDelay(1);
	while( !(res == MPU9250_AK8963_POWER_DOWN))
	{
		MPU9250_AK8963_SPI_Write(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL1_REG,MPU9250_AK8963_POWER_DOWN); 
		LL_mDelay(1);
		printff(USART3, "������POWER DOWNʧ��\r\n",strlen("������POWER DOWNʧ��\r\n"));
	}
	MPU9250_AK8963_SPI_Write(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL1_REG,MPU9250_AK8963_SINGLE_MEASUREMENT);       // use i2c to set AK8963 working on Continuous measurement mode1 & 16-bit output	
  LL_mDelay(1);
	MPU9250_AK8963_SPI_Read(MPU9250_AK8963_I2C_ADDR,AK8963_ST1_REG,&res);  
	LL_mDelay(1);
	printff(USART3, "�ȴ�����������׼����\r\n",strlen("�ȴ�����������׼����\r\n"));
	while (!res )   //������׼���ú�ST1״̬�Ĵ�����Ϊ0
	{
		MPU9250_AK8963_SPI_Read(MPU9250_AK8963_I2C_ADDR,AK8963_ST1_REG,&res);  
		LL_mDelay(1);
	}
	printff(USART3, "�����������Ѿ�׼����\r\n",strlen("�����������Ѿ�׼����\r\n"));
	
	
#if 0
//	char TEST1[20] = {0};
//	char TEST2[20] = {0};
//	char TEST3[20] = {0};
//	char TEST4[20] = {0};
//	char TEST5[20] = {0};
//	char TEST6[20] = {0};
//	char TEST7[20] = {0};
//	char TEST8[20] = {0};
//	char TEST9[20] = {0};	
	
//	MPU9250_AK8963_SPI_Write(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL2_REG,AK8963_CNTL2_SRST); // Reset AK8963
//  LL_mDelay(1);
//	MPU9250_AK8963_SPI_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_POWER_DOWN);
//	LL_mDelay(1);
//	MPU9250_AK8963_SPI_Read(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL1_REG,&res);
//	if (res == 0x00)
//	printff(USART3, "�����Ƹ�λ�ɹ�\r\n",strlen("�����Ƹ�λ�ɹ�\r\n"));	 	
//  LL_mDelay(2);	
//	MPU9250_AK8963_SPI_Write(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL1_REG,0x11);       // use i2c to set AK8963 working on Continuous measurement mode1 & 16-bit output	
//  LL_mDelay(1);
//	MPU9250_AK8963_SPI_Read(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL1_REG,&res);
//	if (res == 0x11)
//	printff(USART3, "���ô�����16λ������β���ģʽ�ɹ�\r\n",strlen("���ô�����16λ������β���ģʽ�ɹ�\r\n"));
//  else
//	printff(USART3, "���ô�����16λ������β���ģʽʧ��\r\n",strlen("���ô�����16λ������β���ģʽʧ��\r\n"));
#endif
	
	MPU9250_SPI_Read_LEN(MPU9250_ACCEL_XOUT_H, 22, data);
	
	accel[0] = (data[0] << 8) | data[1];
	accel[1] = (data[2] << 8) | data[3];
	accel[2] = (data[4] << 8) | data[5];
	
	gyro[0] = (data[8] << 8)  | data[9];
	gyro[1] = (data[10] << 8) | data[11];
	gyro[2] = (data[12] << 8) | data[13];
 
  for(i = 0; i < 3; i++)
   gyro[i] -= Gyo_Offset[i]; 
	 
	 
	for(i = 0; i < 6; i++)
	{
	MPU9250_AK8963_SPI_Read(MPU9250_AK8963_I2C_ADDR,0x03 + i,&data[15 + i]);
	LL_mDelay(1);
	};
   
	mag[0] = (data[16] << 8) | data[15];
	mag[1] = (data[18] << 8) | data[17];
	mag[2] = (data[20] << 8) | data[19];

    printff(USART3, "\r\n�ӼĴ������ȡ������:\r\n",strlen("\r\n�ӼĴ������ȡ������:\r\n"));
	  sprintf( TEST1, "%d", accel[0]); 
		sprintf( TEST2, "%d", accel[1]); 
		sprintf( TEST3, "%d", accel[2]); 
		sprintf( TEST4, "%d", gyro[0]); 
		sprintf( TEST5, "%d", gyro[1]); 
		sprintf( TEST6, "%d", gyro[2]); 
		sprintf( TEST7, "%d", mag[0]); 
		sprintf( TEST8, "%d", mag[1]); 
		sprintf( TEST9, "%d", mag[2]); 
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
		printff(USART3, "\r\n",strlen("\r\n"));
	
	
#if 0
//		sprintf( TEST1, "%d", accel[0]); 
//		sprintf( TEST2, "%d", accel[1]); 
//		sprintf( TEST3, "%d", accel[2]); 
//		sprintf( TEST4, "%d", gyro[0]); 
//		sprintf( TEST5, "%d", gyro[1]); 
//		sprintf( TEST6, "%d", gyro[2]); 
//		sprintf( TEST7, "%d", mag[0]); 
//		sprintf( TEST8, "%d", mag[1]); 
//		sprintf( TEST9, "%d", mag[2]); 
//	  printff(USART3, "\r\nACC:\r\n",strlen("\r\nACC:\r\n"));
//		printff(USART3, TEST1,strlen(TEST1));   printff(USART3, "\t",strlen("\t"));
//		printff(USART3, TEST2,strlen(TEST2));   printff(USART3, "\t",strlen("\t"));
//		printff(USART3, TEST3,strlen(TEST3));   
//  	printff(USART3, "\r\nGYO:\r\n",strlen("\r\nGYO:\r\n"));
//		printff(USART3, TEST4,strlen(TEST4));   printff(USART3, "\t",strlen("\t"));
//		printff(USART3, TEST5,strlen(TEST5));   printff(USART3, "\t",strlen("\t"));
//		printff(USART3, TEST6,strlen(TEST6));   
//  	printff(USART3, "\r\nMAG:\r\n",strlen("\r\nMAG:\r\n"));
//		printff(USART3, TEST7,strlen(TEST7));   printff(USART3, "\t",strlen("\t"));
//		printff(USART3, TEST8,strlen(TEST8));   printff(USART3, "\t",strlen("\t"));
//		printff(USART3, TEST9,strlen(TEST9));   
//		printff(USART3, "\r\n",strlen("\r\n"));
//		printff(USART3, "\r\n",strlen("\r\n"));
#endif 
	
	//ned x,y,z
	Magy = (float)(((long)mag[0] * MPU9250_AK8963_ASA[0]) >> 8);
	Magx = (float)(((long)mag[1] * MPU9250_AK8963_ASA[1]) >> 8);
	Magz = (float)(((long)mag[2] * MPU9250_AK8963_ASA[2]) >> 8);
	Magz = - Magz;
	
	
	//У׼������
	Mag[0] = (Magx - 49.181265) * 0.985359;
	Mag[1] = (Magy - 119.677108) * 1.002879;
	Mag[2] = (Magz + 335.111033) * 1.011761;
	
}





#endif


#if 0
//////////////////////////////////////////////////////////////////////////////////////////
//****************��ʼ��MPU9250��������Ҫ��ο�pdf�����޸�************************
void Init_MPU9250(void)
{	
	uint8_t WhoAmI=0;
	uint8_t res = 0;
	MPU9250_Write_Reg(PWR_MGMT_1, 0x00);	    //�������״̬
	MPU9250_Write_Reg(CONFIG, 0x07);            //��ͨ�˲�Ƶ�ʣ�����ֵ��0x07(3600Hz)�˼Ĵ����ھ���Internal_Sample_Rate==8K
	
/**********************Init SLV0 i2c**********************************/	
//Use SPI-bus read slave0
	MPU9250_Write_Reg(INT_PIN_CFG ,0x30);       // INT Pin / Bypass Enable Configuration  
	MPU9250_Write_Reg(I2C_MST_CTRL,0x4d);       //I2C MAster mode and Speed 400 kHz
	MPU9250_Write_Reg(USER_CTRL ,0x20);         // I2C_MST_EN 
	MPU9250_Write_Reg(I2C_MST_DELAY_CTRL ,0x01);//��ʱʹ��I2C_SLV0 _DLY_ enable 	
	MPU9250_Write_Reg(I2C_SLV0_CTRL ,0x81);     //enable IIC	and EXT_SENS_DATA==1 Byte
	
/*******************Init GYRO and ACCEL******************************/	
	MPU9250_Write_Reg(SMPLRT_DIV, 0x07);        //�����ǲ����ʣ�����ֵ��0x07(1kHz) (SAMPLE_RATE= Internal_Sample_Rate / (1 + SMPLRT_DIV) )
	MPU9250_Write_Reg(GYRO_CONFIG, 0x18);       //�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
	//MPU9250_Write_Reg(ACCEL_CONFIG, 0x18);    //���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x18(���Լ죬16G)
	MPU9250_Write_Reg(ACCEL_CONFIG, 0x01);      //���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G)
	MPU9250_Write_Reg(ACCEL_CONFIG_2, 0x08);    //���ټƸ�ͨ�˲�Ƶ�� ����ֵ ��0x08  ��1.13kHz��	
		
/**********************Init MAG **********************************/
	MPU9250_AK8963_SPI_Write(MPU9250_AK8963_I2C_ADDR,AK8963_CNTL2_REG,AK8963_CNTL2_SRST); // Reset AK8963
	res = MPU9250_AK8963_SPI_Read(AK8963_CNTL2_REG);
	if (res == AK8963_CNTL2_SRST)
		printf("�����������ɹ� \n");
	MPU9250_AK8963_SPI_Write(AK8963_CNTL1_REG,0x12);       // use i2c to set AK8963 working on Continuous measurement mode1 & 16-bit output	


	//��֤��ʼ�����
	WhoAmI=MPU9250_Read_Reg(WHO_AM_I); 
	printf("WhoAmI:0x%X  \n",WhoAmI);

}
#endif