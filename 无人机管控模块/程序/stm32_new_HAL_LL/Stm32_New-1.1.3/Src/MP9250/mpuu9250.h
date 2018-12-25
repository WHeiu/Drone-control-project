#ifndef __MMP9250_
#define __MMP9250_
#include "mp9250.h"
//MPU9250的陀螺仪单位转换系数：
//#define MPU9250_Gyro_FS1000 32.8	 
//MPU9250的加速度计单位转换系数：
//#define Acc_FS2g  1671.84  //to m/ss
//MPU9250磁力计的单位转换系数：
//#define Mega_FS16BIT_Ga  6.83		/*1uT-10mGass*/
//static s16 MPU9250_AK8963_ASA[3] = {0, 0, 0};//1T=10000GASS 1uT=0.01GASS
// 定义MPU9250内部地址
//////////////////////////////////////////////////////////////////////////
//Register Map for Gyroscope and Accelerometer
#define MPU9250_SELF_TEST_X_GYRO        0x00
#define MPU9250_SELF_TEST_Y_GYRO        0x01
#define MPU9250_SELF_TEST_Z_GYRO        0x02

#define MPU9250_SELF_TEST_X_ACCEL       0x0D
#define MPU9250_SELF_TEST_Y_ACCEL       0x0E
#define MPU9250_SELF_TEST_Z_ACCEL       0x0F

#define MPU9250_XG_OFFSET_H             0x13
#define MPU9250_XG_OFFSET_L             0x14
#define MPU9250_YG_OFFSET_H             0x15
#define MPU9250_YG_OFFSET_L             0x16
#define MPU9250_ZG_OFFSET_H             0x17
#define MPU9250_ZG_OFFSET_L             0x18
#define MPU9250_SMPLRT_DIV              0x19	//陀螺仪采样率，典型值：0x07(125Hz) 
#define MPU9250_CONFIG                  0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define MPU9250_GYRO_CONFIG             0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define MPU9250_ACCEL_CONFIG            0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define MPU9250_ACCEL_CONFIG2           0x1D
#define MPU9250_LP_ACCEL_ODR            0x1E
#define MPU9250_WOM_THR                 0x1F

#define MPU9250_FIFO_EN                 0x23
#define MPU9250_I2C_MST_CTRL            0x24
#define MPU9250_I2C_SLV0_ADDR           0x25
#define MPU9250_I2C_SLV0_REG            0x26
#define MPU9250_I2C_SLV0_CTRL           0x27
#define MPU9250_I2C_SLV1_ADDR           0x28
#define MPU9250_I2C_SLV1_REG            0x29
#define MPU9250_I2C_SLV1_CTRL           0x2A
#define MPU9250_I2C_SLV2_ADDR           0x2B
#define MPU9250_I2C_SLV2_REG            0x2C
#define MPU9250_I2C_SLV2_CTRL           0x2D
#define MPU9250_I2C_SLV3_ADDR           0x2E
#define MPU9250_I2C_SLV3_REG            0x2F
#define MPU9250_I2C_SLV3_CTRL           0x30
#define MPU9250_I2C_SLV4_ADDR           0x31
#define MPU9250_I2C_SLV4_REG            0x32
#define MPU9250_I2C_SLV4_DO             0x33
#define MPU9250_I2C_SLV4_CTRL           0x34
#define MPU9250_I2C_SLV4_DI             0x35
#define MPU9250_I2C_MST_STATUS          0x36
#define MPU9250_INT_PIN_CFG             0x37
#define MPU9250_INT_ENABLE              0x38

#define MPU9250_INT_STATUS              0x3A
#define MPU9250_ACCEL_XOUT_H            0x3B
#define MPU9250_ACCEL_XOUT_L            0x3C
#define MPU9250_ACCEL_YOUT_H            0x3D
#define MPU9250_ACCEL_YOUT_L            0x3E
#define MPU9250_ACCEL_ZOUT_H            0x3F
#define MPU9250_ACCEL_ZOUT_L            0x40
#define MPU9250_TEMP_OUT_H              0x41
#define MPU9250_TEMP_OUT_L              0x42
#define MPU9250_GYRO_XOUT_H             0x43
#define MPU9250_GYRO_XOUT_L             0x44
#define MPU9250_GYRO_YOUT_H             0x45
#define MPU9250_GYRO_YOUT_L             0x46
#define MPU9250_GYRO_ZOUT_H             0x47
#define MPU9250_GYRO_ZOUT_L             0x48
#define MPU9250_EXT_SENS_DATA_00        0x49
#define MPU9250_EXT_SENS_DATA_01        0x4A
#define MPU9250_EXT_SENS_DATA_02        0x4B
#define MPU9250_EXT_SENS_DATA_03        0x4C
#define MPU9250_EXT_SENS_DATA_04        0x4D
#define MPU9250_EXT_SENS_DATA_05        0x4E
#define MPU9250_EXT_SENS_DATA_06        0x4F
#define MPU9250_EXT_SENS_DATA_07        0x50
#define MPU9250_EXT_SENS_DATA_08        0x51
#define MPU9250_EXT_SENS_DATA_09        0x52
#define MPU9250_EXT_SENS_DATA_10        0x53
#define MPU9250_EXT_SENS_DATA_11        0x54
#define MPU9250_EXT_SENS_DATA_12        0x55
#define MPU9250_EXT_SENS_DATA_13        0x56
#define MPU9250_EXT_SENS_DATA_14        0x57
#define MPU9250_EXT_SENS_DATA_15        0x58
#define MPU9250_EXT_SENS_DATA_16        0x59
#define MPU9250_EXT_SENS_DATA_17        0x5A
#define MPU9250_EXT_SENS_DATA_18        0x5B
#define MPU9250_EXT_SENS_DATA_19        0x5C
#define MPU9250_EXT_SENS_DATA_20        0x5D
#define MPU9250_EXT_SENS_DATA_21        0x5E
#define MPU9250_EXT_SENS_DATA_22        0x5F
#define MPU9250_EXT_SENS_DATA_23        0x60

#define MPU9250_I2C_SLV0_DO             0x63
#define MPU9250_I2C_SLV1_DO             0x64
#define MPU9250_I2C_SLV2_DO             0x65
#define MPU9250_I2C_SLV3_DO             0x66
#define MPU9250_I2C_MST_DELAY_CTRL      0x67
#define MPU9250_SIGNAL_PATH_RESET       0x68
#define MPU9250_MOT_DETECT_CTRL         0x69
#define MPU9250_USER_CTRL               0x6A
#define MPU9250_PWR_MGMT_1              0x6B	//电源管理，典型值：0x00(正常启用)
#define MPU9250_PWR_MGMT_2              0x6C

#define MPU9250_FIFO_COUNTH             0x72
#define MPU9250_FIFO_COUNTL             0x73
#define MPU9250_FIFO_R_W                0x74
#define MPU9250_WHO_AM_I                0x75	//ID寄存器(默认数值0x71，只读)
#define MPU9250_XA_OFFSET_H             0x77
#define MPU9250_XA_OFFSET_L             0x78

#define MPU9250_YA_OFFSET_H             0x7A
#define MPU9250_YA_OFFSET_L             0x7B

#define MPU9250_ZA_OFFSET_H             0x7D
#define MPU9250_ZA_OFFSET_L             0x7E
//
#define MPU9250_I2C_READ 0x80

//Magnetometer register maps
#define MPU9250_AK8963_WIA                 0x00
#define MPU9250_AK8963_INFO                0x01
#define MPU9250_AK8963_ST1                 0x02
#define MPU9250_AK8963_XOUT_L              0x03
#define MPU9250_AK8963_XOUT_H              0x04
#define MPU9250_AK8963_YOUT_L              0x05
#define MPU9250_AK8963_YOUT_H              0x06
#define MPU9250_AK8963_ZOUT_L              0x07
#define MPU9250_AK8963_ZOUT_H              0x08
#define MPU9250_AK8963_ST2                 0x09
#define MPU9250_AK8963_CNTL                0x0A
#define MPU9250_AK8963_CNTL2               0x0B
#define MPU9250_AK8963_RSV                 0x0B //DO NOT ACCESS <MPU9250_AK8963_CNTL2>
#define MPU9250_AK8963_ASTC                0x0C
#define MPU9250_AK8963_TS1                 0x0D //DO NOT ACCESS
#define MPU9250_AK8963_TS2                 0x0E //DO NOT ACCESS
#define MPU9250_AK8963_I2CDIS              0x0F
#define MPU9250_AK8963_ASAX                0x10
#define MPU9250_AK8963_ASAY                0x11
#define MPU9250_AK8963_ASAZ                0x12

#define MPU9250_AK8963_I2C_ADDR 		   0x0C
#define MPU9250_AK8963_POWER_DOWN 		   0x10
#define MPU9250_AK8963_FUSE_ROM_ACCESS 	   0x1F
#define MPU9250_AK8963_SINGLE_MEASUREMENT  0x11
#define MPU9250_AK8963_CONTINUOUS_MEASUREMENT 0x16 //MODE2刷新速率 100Hz
#define MPU9250_AK8963_SelfTest               0x14
#define MPU9250_AK8963_DATA_READY      (0x01)
#define MPU9250_AK8963_DATA_OVERRUN    (0x02)
#define MPU9250_AK8963_OVERFLOW        (0x80)
#define MPU9250_AK8963_DATA_ERROR      (0x40)
#define MPU9250_AK8963_CNTL2_SRST 0x01

//
#define MPU9250_I2C_SLV4_EN   0x80
#define MPU9250_I2C_SLV4_DONE 0x40
#define MPU9250_I2C_SLV4_NACK 0x10
//
#define MPU9250_I2C_IF_DIS   (0x10)
#define MPU9250_I2C_MST_EN   (0x20)
#define MPU9250_FIFO_RST     (0x04)
#define MPU9250_FIFO_ENABLE  (0x40)
//
#define MPU9250_RESET          0x80
#define MPU9250_CLOCK_MASK     0xF8
#define MPU9250_CLOCK_INTERNAL 0x00
#define MPU9250_CLOCK_PLL      0x01
#define MPU9250_CLOCK_PLLGYROZ 0x03
#define MPU9250_FS_SEL_MASK    0xE7
#define MPU9250_SLEEP_MASK     0x40
//
#define MPU9250_XYZ_GYRO       0xC7
#define MPU9250_XYZ_ACCEL      0xF8
//
#define MPU9250_RAW_RDY_EN       (0x01)
#define MPU9250_RAW_DATA_RDY_INT (0x01)
#define MPU9250_FIFO_OVERFLOW    (0x10)
//
#define MPU9250_INT_ANYRD_2CLEAR (0x10)
#define MPU9250_LATCH_INT_EN     (0x20)
//
#define MPU9250_MAX_FIFO        (1024)
#define MPU9250_FIFO_SIZE_1024  (0x40)
#define MPU9250_FIFO_SIZE_2048  (0x80)
#define MPU9250_FIFO_SIZE_4096  (0xC0)

#define MPU9250_TEMP_OUT  (0x80)
#define MPU9250_GYRO_XOUT (0x40)
#define MPU9250_GYRO_YOUT (0x20)
#define MPU9250_GYRO_ZOUT (0x10)
#define MPU9250_ACCEL     (0x08)
//////////////////////////////////////////////////////////////////////////
//#define SMPLRT_DIV 0
#define MPU9250_SPIx_ADDR 0x00
/******************************************************************************/
#define MPU9250_SPI_ReadWriteByte(x)	SPI1_ReadWriteByte(x)
//#define MPU9250_CS_L 					GPIO_ResetBits(GPIOA,GPIO_Pin_4)			
//#define MPU9250_CS_H 					GPIO_SetBits(GPIOA,GPIO_Pin_4)

//#define MPU_9250_DISENABLE  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);//片选
//#define MPU_9250_ENABLE     HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);

#define MPU9250_CS_H     HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET)//片选
#define MPU9250_CS_L     HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET)
/******************************************************************************/
static uint8_t SPI1_ReadWriteByte(uint8_t TxData);
void MPU9250_SPI_Write(uint8_t reg_addr, uint8_t data);
void MPU9250_SPI_Write_LEN(uint8_t reg_addr, uint8_t len, uint8_t* data);
void MPU9250_SPI_Write_LEN(uint8_t reg_addr, uint8_t len, uint8_t* data);
uint8_t MPU9250_SPI_Read(uint8_t reg_addr);
void MPU9250_SPI_Read_LEN(uint8_t reg_addr, uint8_t len, uint8_t* data);
int MPU9250_AK8963_SPI_Read(uint8_t akm_addr, uint8_t reg_addr, uint8_t* data);
int MPU9250_AK8963_SPI_Read_LEN(uint8_t akm_addr, uint8_t reg_addr, uint8_t len, uint8_t* data);
int MPU9250_AK8963_SPI_Write(uint8_t akm_addr, uint8_t reg_addr, uint8_t data);
int MPU9250_AK8963_SPI_Write_LEN(uint8_t akm_addr, uint8_t reg_addr, uint8_t len, uint8_t* data);
void MMPU9250_Init(void);
void MPU9250_Get9AxisRawData(short *accel, short * gyro, short * mag);
void  AK8963_SelfTest(void);
#endif