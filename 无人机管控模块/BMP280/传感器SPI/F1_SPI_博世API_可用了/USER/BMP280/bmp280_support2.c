#include "bmp280_support.h"
#include "myspi.h"
/*
addr������ַ
data���洢�����׵�ַ
cnt������
*/
unsigned char SPI_READ_WRITE_STRING(unsigned char *addr, unsigned char *data, unsigned char cnt)
{
    int i;
    CE_L;
    for(i=0;i<cnt;i++)
    {
         data[i]=spixfer(addr[i]);
    }
    CE_H;
    return 0;//alway success
}
/*
data��������д������ݵ�ַ
cnt������
*/
unsigned char SPI_WRITE_STRING(unsigned char *data,unsigned char cnt)
{
    int i;
    CE_L;
    for(i=0;i<cnt;i++)
    {
       spixfer(data[i]);
    }
    CE_H;
    return 0;//alway success
}
/**********************************************/

///*
//���ܣ���һ������ַ������д������
//dev_addr:�豸��ַ
//data:Ҫд�����ݵĵ�ַ��data0������Ҫд�ĵ�һ��λ��
//len:data�Ĵ�С
//*/
//unsigned char I2C_WRITE_STRING(unsigned char dev_addr, unsigned char *data, unsigned char len)
//{
//    int i=0;
//    IIC_Start();     
//	IIC_Send_Byte((dev_addr<<1)|0); //����������ַ+д����	
//	IIC_Wait_Ack();	
//	for(i=1; i<len; i++)
//	{
//        IIC_Send_Byte(data[0]++); 
//        IIC_Wait_Ack();	
//        IIC_Send_Byte(data[i]);  
//		IIC_Wait_Ack();	
//	}
//    IIC_Stop( );	
//    return 0;//alway success
//}

///*
//dev_addr:�豸��ַ
//reg_addr:�Ĵ�����ַ
//buf���������ݵĻ�������
//ack����1��0Ӧ��
//cnt�����ĸ���
//*/
//unsigned char I2C_WRITE_READ_STRING\
//(unsigned char dev_addr, unsigned char *reg_addr, \
//unsigned char *buf, unsigned char ack, unsigned char cnt)
//{
//    IIC_Start();
//    IIC_Send_Byte((dev_addr<<1)|0); //����������ַ+д����
//    if(IIC_Wait_Ack())          //�ȴ�Ӧ��
//    {
//        IIC_Stop();
//        return 1;
//    }
//    IIC_Send_Byte(*reg_addr);         //д�Ĵ�����ַ
//    IIC_Wait_Ack();             //�ȴ�Ӧ��
//	  IIC_Start();                
//    IIC_Send_Byte((dev_addr<<1)|1); //����������ַ+������
//    IIC_Wait_Ack();             //�ȴ�Ӧ��
//    while(cnt)
//    { 
//        if(cnt==1)*buf=IIC_Read_Byte(0);//������,����nACK 
//        else *buf=IIC_Read_Byte(1);		//������,����ACK  
//        cnt--;
//        buf++;  
//    }
//    IIC_Stop();                 //����һ��ֹͣ����
//    return 0;//alway success
//}


#define CONST_PF 0.1902630958	                                               //(1/5.25588f) Pressure factor
#define FIX_TEMP 25				                                               // Fixed Temperature. ASL is a function of pressure and temperature, but as the temperature changes so much (blow a little towards the flie and watch it drop 5 degrees) it corrupts the ASL estimates.
								                                               // TLDR: Adjusting for temp changes does more harm than good.
/*
 * Converts pressure to altitude above sea level (ASL) in meters
*/
float bmp280PressureToAltitude(float* pressure/*, float* groundPressure, float* groundTemp*/)
{
    if (*pressure>0)
    {
        return((pow((1015.7f/ *pressure),CONST_PF)-1.0f)*(FIX_TEMP+273.15f))/0.0065f;
    }
    else
    {
        return 0;
    }
}





