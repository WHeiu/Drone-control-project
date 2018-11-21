#ifndef __BMP280_SUPPORT_H__
#define __BMP280_SUPPORT_H__
#include "delay.h"
#include "stdio.h"
//#include "myiic.h"
//#include "myspi.h"
#include "bmp280.h"
#include "math.h"
/*支持BMP280库 SPI通信的函数*/
unsigned char SPI_READ_WRITE_STRING\
(unsigned char *addr, unsigned char *data, unsigned char cnt);
unsigned char SPI_WRITE_STRING(unsigned char *data,unsigned char cnt);

/*支持BMP280库 IIC通信的函数*/
unsigned char I2C_WRITE_STRING(unsigned char dev_addr, unsigned char *data, unsigned char len);
unsigned char I2C_WRITE_READ_STRING\
(unsigned char dev_addr, unsigned char *reg_addr, \
unsigned char *buf, unsigned char ack, unsigned char cnt);

s32 bmp280_data_readout_template(/*void*/ u32 *temp,u32 *press);
float bmp280PressureToAltitude(float* pressure);

#endif

