#ifndef _bmp180_h_
#define _bmp180_h_
 
sbit  SCL=P1^4;      //IIC时钟引脚定义
sbit   SDA=P1^5;      //IIC数据引脚定义
externlong  temperature;
extern long  pressure;
extern  long  height;
 
void Init_BMP085();
void bmp085Convert();
//----------------------------------------
void OLED_BMP180();//only LQ12864 use
#endif
/*eg:------------------------------------------
L1602_init();
L1602_char(1,1,'T');//温度部分
L1602_char(1,2,':');
L1602_char(1,5,'.');
L1602_char(1,7,0XDF);
L1602_char(1,8,'C');
L1602_char(1,10,'H');//海拔部分
L1602_char(1,14,'.');
L1602_char(1,16,'m');
L1602_char(2,1,'P');//气压部分
L1602_char(2,2,':');
L1602_char(2,6,'.');
L1602_char(2,8,'K');
L1602_char(2,9,'p');
 
while(1)
{
 
Init_BMP085();
bmp085Convert();
L1602_char(1,3,temperature/100+48);
L1602_char(1,4,temperature%100/10+48);
L1602_char(1,6,temperature%10+48);
 
L1602_char(1,11,height/10000+48);
L1602_char(1,12,height%10000/1000+48);
L1602_char(1,13,height%1000/100+48);
L1602_char(1,15,height%100/10+48);
 
L1602_char(2,3,pressure/100000+48);
L1602_char(2,4,pressure%100000/10000+48);
L1602_char(2,5,pressure%10000/1000+48);
L1602_char(2,7,pressure%1000/100+48);
 
}
----------------------------------------------*/