#include "bmp280_support.h"
#include "myspi.h"
/*
addr：读地址
data：存储数据首地址
cnt：个数
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
data：保存了写入和数据地址
cnt：个数
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
//功能：在一连续地址中依次写入数据
//dev_addr:设备地址
//data:要写的数据的地址，data0是数据要写的第一个位置
//len:data的大小
//*/
//unsigned char I2C_WRITE_STRING(unsigned char dev_addr, unsigned char *data, unsigned char len)
//{
//    int i=0;
//    IIC_Start();     
//	IIC_Send_Byte((dev_addr<<1)|0); //发送器件地址+写命令	
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
//dev_addr:设备地址
//reg_addr:寄存器地址
//buf：读回数据的缓冲数组
//ack：是1否0应答
//cnt：读的个数
//*/
//unsigned char I2C_WRITE_READ_STRING\
//(unsigned char dev_addr, unsigned char *reg_addr, \
//unsigned char *buf, unsigned char ack, unsigned char cnt)
//{
//    IIC_Start();
//    IIC_Send_Byte((dev_addr<<1)|0); //发送器件地址+写命令
//    if(IIC_Wait_Ack())          //等待应答
//    {
//        IIC_Stop();
//        return 1;
//    }
//    IIC_Send_Byte(*reg_addr);         //写寄存器地址
//    IIC_Wait_Ack();             //等待应答
//	  IIC_Start();                
//    IIC_Send_Byte((dev_addr<<1)|1); //发送器件地址+读命令
//    IIC_Wait_Ack();             //等待应答
//    while(cnt)
//    { 
//        if(cnt==1)*buf=IIC_Read_Byte(0);//读数据,发送nACK 
//        else *buf=IIC_Read_Byte(1);		//读数据,发送ACK  
//        cnt--;
//        buf++;  
//    }
//    IIC_Stop();                 //产生一个停止条件
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





