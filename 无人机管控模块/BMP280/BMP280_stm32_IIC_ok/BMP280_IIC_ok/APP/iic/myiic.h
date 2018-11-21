#ifndef __MYIIC_H
#define __MYIIC_H
#include "SysTick.h"
//////////////////////////////////////////////////////////////////////////////////	 

//IO��������	 
#define IIC_SCL    PBout(10) //SCL
#define IIC_SDA    PBout(11) //SDA	 
#define READ_SDA   PBin(11)  //����SDA 

//IIC���в�������
void SDA_OUT(void);
void SDA_IN(void);
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	  
u8 iicDevReadByte(u8 devaddr,u8 addr);
void iicDevRead(u8 devaddr,u8 addr,u8 len,u8 *rbuf);
void iicDevWriteByte(u8 devaddr,u8 addr,u8 data);
void iicDevWrite(u8 devaddr,u8 addr,u8 len,u8 *wbuf);

#endif
















