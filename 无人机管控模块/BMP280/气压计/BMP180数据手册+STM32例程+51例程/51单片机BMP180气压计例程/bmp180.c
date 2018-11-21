#include  <REG52.H>
#include "bmp180.h"
#include "delay.h"
//#include "bmpi2c.h"
#include "LQ12864.h"
 
#include  <math.h>    //Keil library 
#include  <stdlib.h>  //Keil library 
#include  <stdio.h>   //Keil library
#include  <INTRINS.H> //Keil library
 
#define   uchar unsigned char
#define   uint unsigned int
 
 
#defineBMP085_SlaveAddress   0xee  //定义器件在IIC总线中的从地址                              
#define OSS 0// Oversampling Setting (note: code is not set up to use other OSS values)
 
 
typedef unsigned char  BYTE;
typedef unsigned short WORD;
 
long  temperature;//温度值
long  pressure;//压力值
long  height;//相对海拔高度值
 
//uchar ge,shi,bai,qian,wan,shiwan;           //显示变量
int  dis_data;                              //变量
 
short ac1;
short ac2;
short ac3;
unsigned short ac4;
unsigned short ac5;
unsigned short ac6;
short b1;
short b2;
short mb;
short mc;
short md;
 
/**************************************
延时5毫秒(STC90C52RC@12M)
不同的工作环境,需要调整此函数
当改用1T的MCU时,请调整此延时函数
**************************************/
void Delay5ms()
{
 
    WORD n = 560;
 
    while (n--);
 
}
 
 
/**************************************
起始信号
**************************************/
void BMP085_Start()
{
 
    SDA = 1;                    //拉高数据线
    SCL = 1;                    //拉高时钟线
    Delay5us();                 //延时
    SDA = 0;                    //产生下降沿
    Delay5us();                 //延时
    SCL = 0;                    //拉低时钟线
 
}
 
/**************************************
停止信号
**************************************/
void BMP085_Stop()
{
 
    SDA = 0;                    //拉低数据线
    SCL = 1;                    //拉高时钟线
    Delay5us();                 //延时
    SDA = 1;                    //产生上升沿
    Delay5us();                 //延时
 
}
 
/**************************************
发送应答信号
入口参数:ack (0:ACK 1:NAK)
**************************************/
void BMP085_SendACK(bit ack)
{
 
    SDA = ack;                  //写应答信号
    SCL = 1;                    //拉高时钟线
    Delay5us();                 //延时
    SCL = 0;                    //拉低时钟线
    Delay5us();                 //延时
 
}
 
/**************************************
接收应答信号
**************************************/
bit BMP085_RecvACK()
{
 
    SCL = 1;                    //拉高时钟线
    Delay5us();                 //延时
    CY = SDA;                   //读应答信号
    SCL = 0;                    //拉低时钟线
    Delay5us();                 //延时
 
    return CY;
 
}
 
/**************************************
向IIC总线发送一个字节数据
**************************************/
void BMP085_SendByte(BYTE dat)
{
 
    BYTE i;
 
    for (i=0; i<8; i++)         //8位计数器
    {
 
        dat <<= 1;              //移出数据的最高位
        SDA = CY;               //送数据口
        SCL = 1;                //拉高时钟线
        Delay5us();             //延时
        SCL = 0;                //拉低时钟线
        Delay5us();             //延时
     
}
    BMP085_RecvACK();
 
}
 
/**************************************
从IIC总线接收一个字节数据
**************************************/
BYTE BMP085_RecvByte()
{
 
    BYTE i;
    BYTE dat = 0;
 
    SDA = 1;                    //使能内部上拉,准备读取数据,
    for (i=0; i<8; i++)         //8位计数器
    {
 
        dat <<= 1;
        SCL = 1;                //拉高时钟线
        Delay5us();             //延时
        dat |= SDA;             //读数据              
        SCL = 0;                //拉低时钟线
        Delay5us();             //延时
     
}
    return dat;
 
}
 
 
//*********************************************************
//读出BMP085内部数据,连续两个
//*********************************************************
short Multiple_read(uchar ST_Address)
{
    
uchar msb, lsb;
short _data;
    BMP085_Start();                          //起始信号
    BMP085_SendByte(BMP085_SlaveAddress);    //发送设备地址+写信号
    BMP085_SendByte(ST_Address);             //发送存储单元地址
    BMP085_Start();                          //起始信号
    BMP085_SendByte(BMP085_SlaveAddress+1);         //发送设备地址+读信号
 
    msb = BMP085_RecvByte();                 //BUF[0]存储
    BMP085_SendACK(0);                       //回应ACK
    lsb = BMP085_RecvByte();    
BMP085_SendACK(1);                       //最后一个数据需要回NOACK
 
    BMP085_Stop();                           //停止信号
    Delay5ms();
    _data = msb << 8;
_data |= lsb;
return _data;
 
}
//********************************************************************
long bmp085ReadTemp(void)
{
 
 
    BMP085_Start();                  //起始信号
    BMP085_SendByte(BMP085_SlaveAddress);   //发送设备地址+写信号
    BMP085_SendByte(0xF4);          // write register address
    BMP085_SendByte(0x2E);       // write register data for temp
    BMP085_Stop();                   //发送停止信号
Delay_1ms(10);// max time is 4.5ms
 
return (long) Multiple_read(0xF6);
 
 
}
//*************************************************************
long bmp085ReadPressure(void)
{
 
long pressure = 0;
 
    BMP085_Start();                   //起始信号
    BMP085_SendByte(BMP085_SlaveAddress);   //发送设备地址+写信号
    BMP085_SendByte(0xF4);          // write register address
    BMP085_SendByte(0x34);         // write register data for pressure
    BMP085_Stop();                    //发送停止信号
Delay_1ms(10);                      // max time is 4.5ms
 
pressure = Multiple_read(0xF6);
pressure &= 0x0000FFFF;
 
return pressure;
 
}
 
//void conversion(long temp_data) 
//{
   
//   
//    shiwan=temp_data/100000+0x30 ;
//    temp_data=temp_data%100000;   //取余运算
//    wan=temp_data/10000+0x30 ;
//    temp_data=temp_data%10000;   //取余运算
//qian=temp_data/1000+0x30 ;
//    temp_data=temp_data%1000;    //取余运算
//    bai=temp_data/100+0x30   ;
//    temp_data=temp_data%100;     //取余运算
//    shi=temp_data/10+0x30    ;
//    temp_data=temp_data%10;      //取余运算
//    ge=temp_data+0x30;
//
}
 
///**************************************
//延时5微秒(STC90C52RC@12M)
//不同的工作环境,需要调整此函数，注意时钟过快时需要修改
//当改用1T的MCU时,请调整此延时函数
//**************************************/
//void Delay5us()
//{
 
//    _nop_();_nop_();_nop_();_nop_();
//    _nop_();_nop_();_nop_();_nop_();
//_nop_();_nop_();_nop_();_nop_();
//_nop_();_nop_();_nop_();_nop_();
//_nop_();_nop_();_nop_();_nop_();
//_nop_();_nop_();_nop_();_nop_();
//
}
 
 
 
 
//**************************************************************
 
//初始化BMP085，根据需要请参考pdf进行修改**************
void Init_BMP085()
{
 
ac1 = Multiple_read(0xAA);
ac2 = Multiple_read(0xAC);
ac3 = Multiple_read(0xAE);
ac4 = Multiple_read(0xB0);
ac5 = Multiple_read(0xB2);
ac6 = Multiple_read(0xB4);
b1 =  Multiple_read(0xB6);
b2 =  Multiple_read(0xB8);
mb =  Multiple_read(0xBA);
mc =  Multiple_read(0xBC);
md =  Multiple_read(0xBE);
 
}
//***********************************************************************
void bmp085Convert()
{
 
unsigned int ut;
unsigned long up;
long x1, x2, b5, b6, x3, b3, p;
unsigned long b4, b7;
 
 
ut = bmp085ReadTemp();   // 读取温度
//ut = bmp085ReadTemp();   // 读取温度
up = bmp085ReadPressure();  // 读取压强
//up = bmp085ReadPressure();  // 读取压强
x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
x2 = ((long) mc << 11) / (x1 + md);
b5 = x1 + x2;
 temperature = ((b5 + 8) >> 4);
// conversion(temperature);
// DisplayOneChar(4,0,'T');       //温度显示
//     DisplayOneChar(5,0,':');
//     DisplayOneChar(7,0,bai);      
//     DisplayOneChar(8,0,shi);
//     DisplayOneChar(9,0,'.');
// DisplayOneChar(10,0,ge);
// DisplayOneChar(11,0,0XDF);     //温度单位
// DisplayOneChar(12,0,'C');
 
  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;
   
  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;
   
  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;
     
  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
 pressure = p+((x1 + x2 + 3791)>>4);
 
height=(101325-pressure)*9;
 
 
 
// conversion(pressure);
//     DisplayOneChar(4,1,'P');    //显示压强
//     DisplayOneChar(5,1,':');
// DisplayOneChar(6,1,shiwan);
// DisplayOneChar(7,1,wan);   
//     DisplayOneChar(8,1,qian);
//     DisplayOneChar(9,1,'.');
//     DisplayOneChar(10,1,bai);
//     DisplayOneChar(11,1,shi);
// DisplayOneChar(12,1,'K');   //气压单位
// DisplayOneChar(13,1,'p');
// DisplayOneChar(14,1,'a');
 
}
 
void OLED_BMP180()
{
 
Init_BMP085(); 
bmp085Convert();
LCD_P8x16Str(20,3,"H:");
OLCD_P8x16(36,3,height/10000);
OLCD_P8x16(44,3,height%10000/1000);
OLCD_P8x16(52,3,height%1000/100);
//OLCD_P8x16(60,3,height%100/10);
LCD_P8x16Str(60,3,"M");
 
LCD_P8x16Str(20,5,"P:");
OLCD_P8x16(36,5,pressure/100000);
OLCD_P8x16(44,5,pressure%100000/10000);
OLCD_P8x16(52,5,pressure%10000/1000);
LCD_P8x16Str(60,5,".");
OLCD_P8x16(68,5,pressure%1000/100);
LCD_P8x16Str(76,5,"Kpa");
 
LCD_P8x16Str(20,1,"T:");
OLCD_P8x16(36,1,temperature/100);
OLCD_P8x16(44,1,temperature%100/10);
LCD_P8x16Str(52,1,".");
OLCD_P8x16(60,1,temperature%10);
LCD_P8x16Str(68,1,"'C");
 
}