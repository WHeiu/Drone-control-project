#include "ms5611.h"

//u8 exchange_Pres_num[8];//ѹ������
//u8 exchange_Temp_num[8];//�¶�����

u16 Cal_C[7];  //���ڴ��PROM�е�8������
//u32 D1_Pres,D2_Temp; // �������ѹ�����¶�
//float Pressure;				//�¶Ȳ�������ѹ
//float dT,Temperature,Temperature2;//ʵ�ʺͲο��¶�֮��Ĳ���,ʵ���¶�,�м�ֵ
//double OFF,SENS;  //ʵ���¶ȵ���,ʵ���¶�������
defMS5611data ms5611data;

//=========================================================
//******MS561101BA����********
//=========================================================
void MS561101BA_Init(void)
{
	IIC_Init();//��ʼ���ӿ�
	
	MS561101BA_RESET();
	delay_ms(100);
	MS561101BA_PROM_READ();
	delay_ms(100);
} 

void MS561101BA_RESET(void)//�����ݶ���������������ܲ���ȥ����ע�͵��˴�while(IIC_Wait_Ack());
{	
	IIC_Start();
	IIC_Send_Byte(MS561101BA_SlaveAddress);
	while(IIC_Wait_Ack());
	delay_us(100);
	IIC_Send_Byte(MS561101BA_RST);
	while(IIC_Wait_Ack());
	delay_us(100);
	IIC_Stop();
}

//��PROM��ȡ����У׼����
void MS561101BA_PROM_READ(void)
{
	u16 d1,d2;
	u8 i;
	for(i=0;i<=6;i++)
	{
		IIC_Start();
		IIC_Send_Byte(MS561101BA_SlaveAddress);
		while(IIC_Wait_Ack());
		IIC_Send_Byte((MS561101BA_PROM_RD+i*2));
		while(IIC_Wait_Ack());

		IIC_Start();
		IIC_Send_Byte(MS561101BA_SlaveAddress+0x01);
		while(IIC_Wait_Ack());
		d1=IIC_Read_Byte(1);
		d2=IIC_Read_Byte(0);
		IIC_Stop();

		delay_ms(10);

		Cal_C[i]=(d1<<8)|d2;
	}//��ӡPROM��ȡ����У׼���ݣ�������ݴ����Ƿ�����
//	printf("C1 =%d\n",Cal_C[1]);
//	printf("C2 =%d\n",Cal_C[2]);
//	printf("C3 =%d\n",Cal_C[3]);
//	printf("C4 =%d\n",Cal_C[4]);
//	printf("C5 =%d\n",Cal_C[5]);
//	printf("C6 =%d\n",Cal_C[6]);
}

u32 MS561101BA_DO_CONVERSION(u8 command)
{
	u32 conversion=0;
	u32 conv1,conv2,conv3; 
	IIC_Start();
	IIC_Send_Byte(MS561101BA_SlaveAddress);
	while(IIC_Wait_Ack());
	IIC_Send_Byte(command);	
	while(IIC_Wait_Ack());
	IIC_Stop();

	delay_ms(20);	

	IIC_Start();
	IIC_Send_Byte(MS561101BA_SlaveAddress);
	while(IIC_Wait_Ack());
	IIC_Send_Byte(0x00);
	while(IIC_Wait_Ack());
	IIC_Stop();

	IIC_Start();
	IIC_Send_Byte(MS561101BA_SlaveAddress+1);
	while(IIC_Wait_Ack());
	conv1=IIC_Read_Byte(1);
	conv2=IIC_Read_Byte(1);
	conv3=IIC_Read_Byte(0);
	IIC_Stop();

	conversion= (conv1<<16) + (conv2<<8) + conv3;

	return conversion;
}

/*
//��ȡ�����¶�
void MS561101BA_GetTemperature(u8 OSR_Temp)
{
	float readdata;
    
	readdata= MS561101BA_DO_CONVERSION(OSR_Temp);	
	delay_ms(10);
//	printf("%u /n",D2_Temp);
	dT=readdata - (((u32)Cal_C[5])<<8);
	Temperature=2000+dT*((u32)Cal_C[6])/8388608;
//	printf("Temperature:%f\n",Temperature);
}

//��ȡ������ѹ
void MS561101BA_GetPressure(u8 OSR_Pres)
{
	float Aux,OFF2,SENS2;  //�¶�У��ֵ	
	float readdata;
	
	readdata= MS561101BA_DO_CONVERSION(OSR_Pres);
	delay_ms(10); 
	
	OFF=(u32)(Cal_C[2]<<16)+((u32)Cal_C[4]*dT)/128.0;
	SENS=(u32)(Cal_C[1]<<15)+((u32)Cal_C[3]*dT)/256.0;
	//�¶Ȳ���
	if(Temperature < 2000)// second order temperature compensation when under 20 degrees C
	{
		Temperature2 = (dT*dT) / 0x80000000;
		Aux = (Temperature-2000)*(Temperature-2000);
		OFF2 = 2.5*Aux;
		SENS2 = 1.25*Aux;
		if(Temperature < -1500)
		{
			Aux = (Temperature+1500)*(Temperature+1500);
			OFF2 = OFF2 + 7*Aux;
			SENS2 = SENS + 5.5*Aux;
		}
	}else  //(Temperature > 2000)
	{
		Temperature2 = 0;
		OFF2 = 0;
		SENS2 = 0;
	}
	
	Temperature = Temperature - Temperature2;
	OFF = OFF - OFF2;
	SENS = SENS - SENS2;	

	Pressure=(readdata*SENS/2097152.0-OFF)/32768.0;
}
*/
void MS561101BA_readdata(defMS5611data *ms5611data)
{
	float dT,readdata,T1,T2,Aux,OFF2,SENS2;
	double OFF,SENS;  //ʵ���¶ȵ���,ʵ���¶�������
	readdata= MS561101BA_DO_CONVERSION(MS561101BA_D2_OSR_4096);	
//	delay_ms(10);
//	printf("%u /n",D2_Temp);
	dT=readdata - (((u32)Cal_C[5])<<8);
	T1=(s16)(2000+dT*((u32)Cal_C[6])/8388608);
//	ms5611data->temperature=T1;
	
	
	readdata= MS561101BA_DO_CONVERSION(MS561101BA_D1_OSR_4096);
	OFF=(u32)(Cal_C[2]<<16)+((u32)Cal_C[4]*dT)/128;
	SENS=(u32)(Cal_C[1]<<15)+((u32)Cal_C[3]*dT)/256;
	//�¶Ȳ���
	if(T1 < 2000)// second order temperature compensation when under 20 degrees C
	{
		T2 = (dT*dT) / 0x80000000;
		Aux = (T1-2000)*(T1-2000);
		OFF2 = 2.5*Aux;
		SENS2 = 1.25*Aux;
		if(T1 < -1500)
		{
			Aux = (T1+1500)*(T1+1500);
			OFF2 = OFF2 + 7*Aux;
			SENS2 = SENS + 5.5*Aux;
		}
	}else  //(Temperature > 2000)
	{
		T2 = 0;
		OFF2 = 0;
		SENS2 = 0;
	}	
	T1 -= T2;
	OFF = OFF - OFF2;
	SENS = SENS - SENS2;	
	ms5611data->pressure=(u32)((readdata*SENS/2097152-OFF)/32768);
	ms5611data->temperature=T1;
}

/***************����ת��******************* 
void Exchange_Number(void) 
{ 
	u32 ex_Pressure;	//���ڶ���ת��ֵ
	u32 ex_Temperature; //���ڶ���ת��ֵ

	MS561101BA_GetTemperature(MS561101BA_D2_OSR_4096);
	MS561101BA_GetPressure(MS561101BA_D1_OSR_4096);
	
	ex_Pressure=(long)(Pressure); 
	
	exchange_Pres_num[0]=ex_Pressure/1000000+'0';
	exchange_Pres_num[1]=ex_Pressure/100000%10+'0';
	exchange_Pres_num[2]=ex_Pressure/10000%10+'0';
	exchange_Pres_num[3]=ex_Pressure/1000%10+'0';
	exchange_Pres_num[4]=ex_Pressure/100%10+'0';
	exchange_Pres_num[5]='.';
	exchange_Pres_num[6]=ex_Pressure/10%10+'0'; 
	exchange_Pres_num[7]=ex_Pressure%10+'0'; 
	printf("Atm: ");
	Usart_Send(exchange_Pres_num,8);
	printf("mbar\n");
	
	ex_Temperature=(long)(Temperature); 
	
	exchange_Temp_num[0]=ex_Temperature/1000000+'0';
	exchange_Temp_num[1]=ex_Temperature/100000%10+'0';
	exchange_Temp_num[2]=ex_Temperature/10000%10+'0';
	exchange_Temp_num[3]=ex_Temperature/1000%10+'0';
	exchange_Temp_num[4]=ex_Temperature/100%10+'0';
	exchange_Temp_num[5]='.';
	exchange_Temp_num[6]=ex_Temperature/10%10+'0'; 
	exchange_Temp_num[7]=ex_Temperature%10+'0'; 
	printf("Tem: ");
	Usart_Send(exchange_Temp_num,8);
	printf("��\n");
} 
*/
		 


