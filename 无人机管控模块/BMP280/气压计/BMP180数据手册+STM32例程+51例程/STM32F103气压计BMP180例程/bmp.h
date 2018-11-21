#ifndef __BMP180_H
#define __BMP180_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////
//本程序只供学习使用 
//////////////////////////////////////////////////////////////////////////////////
 
//IO方向设置
#define SDA_IN()  {
GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=8<<12;
}
#define SDA_OUT() {
GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=3<<12;
}
#define oss 3 //精度  范围：0~3
//IO操作函数 
#define IIC_SCL     PBout(10) //SCL
#define IIC_SDA    PBout(11) //SDA
#define READ_SDA   PBin(11)  //输入SDA
#define BMP180_WR_ADDR 0xEE
#define BMP180_RD_ADDR 0xEF
 
 
//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口
void IIC_Start(void);//发送IIC开始信号
void IIC_Stop(void);  //发送IIC停止信号
void IIC_Send_Byte(u8 txd);//IIC发送一个字节
u8 IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 IIC_Wait_Ack(void); //IIC等待ACK信号
void IIC_Ack(void);//IIC发送ACK信号
void IIC_NAck(void);//IIC不发送ACK信号
 
void BMP180_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 BMP180_Read_One_Byte(u8 daddr,u8 addr);
u8 BMP180_ReadOneByte(u8 ReadAddr);
short BMP180_CRC_Read(u8 addr);
u16 BMP085_Get_UT(void);
long BMP085_Get_UP(void);
//int BMP180_ReadTowByte(u8 add);
long  BMP_UP_Read(void); 
#endif