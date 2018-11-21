#ifndef __MYIIC_H__
#define __MYIIC_H__
#include "sys.h"

//IO方向设置
#define	SDA_OUT() { GPIO_InitTypeDef  GPIO_InitStructure;GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;GPIO_InitStructure.GPIO_Mode =GPIO_Mode_Out_PP;GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; GPIO_Init(GPIOB, &GPIO_InitStructure);}
#define	SDA_IN() { GPIO_InitTypeDef  GPIO_InitStructure;GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IPU;GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; GPIO_Init(GPIOB, &GPIO_InitStructure);}              

//IO操作函数	 
#define IIC_SCL    PBout(6) //SCL
#define IIC_SDA    PBout(7) //SDA	 
#define READ_SDA   PBin(7)  //输入SDA 

//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口				 
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8 IIC_Read_Byte(u8 ack);//IIC读取一个字节
u8 IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	 
u8 I2C_Readkey(u8 I2C_Addr);

u8 I2C_ReadOneByte(u8 I2C_Addr,u8 addr);
u8 IICwriteByte(u8 dev, u8 reg, u8 data);
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data);
u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data);
u8 IICwriteBit(u8 dev,u8 reg,u8 bitNum,u8 data);
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data);

#endif

//------------------End of File----------------------------














