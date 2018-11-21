#include "BMP180.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////
//��ѹ������BMP180��STM32������������ֻ��ѧϰʹ�� 
//////////////////////////////////////////////////////////////////////////////////
  
//��ʼ��IIC
void IIC_Init(void)
{
      
GPIO_InitTypeDef GPIO_InitStructure;
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE );
    
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //�������
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOB, &GPIO_InitStructure);
GPIO_SetBits(GPIOB,GPIO_Pin_10|GPIO_Pin_11); //PB10,PB11 �����
 
}
//����IIC��ʼ�ź�
void IIC_Start(void)
{
 
SDA_OUT();     //sda�����
IIC_SDA=1;   
IIC_SCL=1;
delay_us(4);
 IIC_SDA=0;//START:when CLK is high,DATA change form high to low
delay_us(4);
IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ��������
 
} 
//����IICֹͣ�ź�
void IIC_Stop(void)
{
 
SDA_OUT();//sda�����
IIC_SCL=0;
IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 delay_us(4);
IIC_SCL=1;
IIC_SDA=1;//����I2C���߽����ź�
delay_us(4);  
 
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Wait_Ack(void)
{
 
u8 ucErrTime=0;
SDA_IN();      //SDA����Ϊ���� 
IIC_SDA=1;delay_us(1);  
IIC_SCL=1;delay_us(1);
while(READ_SDA)
{
 
ucErrTime++;
if(ucErrTime>250)
{
 
IIC_Stop();
return 1;
 
}
 
}
IIC_SCL=0;//ʱ�����0   
return 0; 
 
}
//����ACKӦ��
void IIC_Ack(void)
{
 
IIC_SCL=0;
SDA_OUT();
IIC_SDA=0;
delay_us(2);
IIC_SCL=1;
delay_us(2);
IIC_SCL=0;
 
}
//������ACKӦ��   
void IIC_NAck(void)
{
 
IIC_SCL=0;
SDA_OUT();
IIC_SDA=1;
delay_us(2);
IIC_SCL=1;
delay_us(2);
IIC_SCL=0;
 
}     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ�� 
void IIC_Send_Byte(u8 txd)
{
                         
    u8 t;  
SDA_OUT();    
    IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {
               
        //IIC_SDA=(txd&0x80)>>7;
if((txd&0x80)>>7)
IIC_SDA=1;
else
IIC_SDA=0;
txd<<=1;  
delay_us(2);   //��TEA5767��������ʱ���Ǳ����
IIC_SCL=1;
delay_us(2);
IIC_SCL=0;
delay_us(2);
     
}
 
}    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK  
u8 IIC_Read_Byte(unsigned char ack)
{
 
unsigned char i,receive=0;
SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
{
 
        IIC_SCL=0;
        delay_us(2);
IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;  
delay_us(1);
     
}
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK  
    return receive;
 
}
 
u8 BMP180_ReadOneByte(u8 ReadAddr)
{
   
u8 temp=0;      
    IIC_Start(); 
IIC_Send_Byte(BMP180_WR_ADDR);   //����д����
IIC_Wait_Ack();  
    IIC_Send_Byte(ReadAddr);   //����Ҫ���ļĴ�����ַ
IIC_Wait_Ack();   
IIC_Start();     
IIC_Send_Byte(BMP180_RD_ADDR);           //�������ģʽ  
IIC_Wait_Ack();
    temp=IIC_Read_Byte(0);  
    IIC_Stop();//����һ��ֹͣ����   
return temp;
 
}
 
 
 
// u16 BMP180_ReadTowByte(u8 ReadAddr)
// {
   
// u16 temp=0;      
//     IIC_Start(); 
// IIC_Send_Byte(BMP180_WR_ADDR);   //����д����
// IIC_Wait_Ack();  
//     IIC_Send_Byte(ReadAddr);   //����Ҫ���ļĴ�����ַ
// IIC_Wait_Ack();   
// IIC_Start();     
// IIC_Send_Byte(BMP180_RD_ADDR);           //�������ģʽ  
// IIC_Wait_Ack();
// IIC_Send_Byte(BMP180_RD_ADDR);           //�������ģʽ  
// IIC_Wait_Ack();
//     temp=IIC_Read_Byte(0);  
//     IIC_Stop();//����һ��ֹͣ����   
// return temp;
//
}
//��AT24CXXָ����ַд��һ������
//WriteAddr  :д�����ݵ�Ŀ�ĵ�ַ   
//DataToWrite:Ҫд�������
void BMP180_WriteOneByte(u16 WriteAddr,u8 DataToWrite)
{
           
    IIC_Start(); 
 
IIC_Send_Byte(BMP180_WR_ADDR);    //����д����
  
IIC_Wait_Ack();  
    IIC_Send_Byte(WriteAddr);   //���͵�ַ
IIC_Wait_Ack();      
IIC_Send_Byte(DataToWrite);     //�����ֽ�  
IIC_Wait_Ack();        
    IIC_Stop();//����һ��ֹͣ����
delay_us(1);
 
}
 
// int BMP180_CRC_Read(u8 addr)
// {
 
// u16 temp;
// temp = (BMP180_ReadOneByte(addr)<<8)|BMP180_ReadOneByte(addr+1);   
// return temp;
//
}
short BMP180_CRC_Read(u8 addr)
{
 
	u8 msb,lsb;
	short data;
	IIC_Start();
	IIC_Send_Byte(BMP180_WR_ADDR);   //����д����
	IIC_Wait_Ack();  
	IIC_Send_Byte(addr);   //����Ҫ���ļĴ�����ַ
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(BMP180_RD_ADDR);
	 
	IIC_Wait_Ack();
	msb=IIC_Read_Byte(1);
	//IIC_Wait_Ack();
	lsb=IIC_Read_Byte(0);
	//IIC_Wait_Ack();
	IIC_Stop();
	delay_us(5);
	data= msb << 8;
	data|= lsb;
	return data;
 
}
u16 BMP085_Get_UT(void)
{
 
	IIC_Start();
	IIC_Send_Byte(BMP180_WR_ADDR);//����д����
	//IIC_Wait_Ack();
	IIC_Send_Byte(0xf4);        //������
	//IIC_Wait_Ack();
	IIC_Send_Byte(0x2E);
	IIC_Stop();
	delay_ms(5);
	return BMP180_CRC_Read(0xf6);
 
}
long  BMP_UP_Read(void)
{
 
	long temp;
	BMP180_WriteOneByte(0xF4,0x34+(oss<<6));
	delay_ms(27);
	temp=(BMP180_ReadOneByte(0xf6)<<16)|(BMP180_ReadOneByte(0xf7)<<8)|BMP180_ReadOneByte(0xf8);
	return temp>>(8-oss);
 
}
 
long BMP085_Get_UP(void)
{
 
	long pressure=0;
	u8 msb,lsb,xlsb;
	BMP180_WriteOneByte(0xF4,0x34+(oss<<6));
	delay_ms(2+(3<<oss));
	IIC_Start();
	IIC_Send_Byte(BMP180_RD_ADDR);
	IIC_Wait_Ack();
	msb=IIC_Read_Byte(1);
	lsb=IIC_Read_Byte(1);
	xlsb=IIC_Read_Byte(0);
	IIC_Stop();
	pressure=(msb<<16)|(lsb<<8)|xlsb;
	return pressure>>(8-oss);
 
}