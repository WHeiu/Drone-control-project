#ifndef __BMP180_H
#define __BMP180_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////
//������ֻ��ѧϰʹ�� 
//////////////////////////////////////////////////////////////////////////////////
 
//IO��������
#define SDA_IN()  {
GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=8<<12;
}
#define SDA_OUT() {
GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=3<<12;
}
#define oss 3 //����  ��Χ��0~3
//IO�������� 
#define IIC_SCL     PBout(10) //SCL
#define IIC_SDA    PBout(11) //SDA
#define READ_SDA   PBin(11)  //����SDA
#define BMP180_WR_ADDR 0xEE
#define BMP180_RD_ADDR 0xEF
 
 
//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��
void IIC_Start(void);//����IIC��ʼ�ź�
void IIC_Stop(void);  //����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); //IIC�ȴ�ACK�ź�
void IIC_Ack(void);//IIC����ACK�ź�
void IIC_NAck(void);//IIC������ACK�ź�
 
void BMP180_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 BMP180_Read_One_Byte(u8 daddr,u8 addr);
u8 BMP180_ReadOneByte(u8 ReadAddr);
short BMP180_CRC_Read(u8 addr);
u16 BMP085_Get_UT(void);
long BMP085_Get_UP(void);
//int BMP180_ReadTowByte(u8 add);
long  BMP_UP_Read(void); 
#endif