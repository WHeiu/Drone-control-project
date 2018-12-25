#ifndef MPU9250_F
#define MPU9250_F
#include "stm32l4xx_hal.h"
enum LMSB {MSB,LSB};

//void Init_MPU9250(void);
void Sensor_Read(void);
void MPU9250_Read(void);
void MPU9250_Read_Pre(void);
void SPI_ReadBytes(uint8_t Register,uint8_t Length,uint8_t* Array,enum LMSB Word);
void SPI_MAG_ReadBytes(uint8_t Register,uint8_t Length,uint8_t* Array,enum LMSB Word);
#endif
