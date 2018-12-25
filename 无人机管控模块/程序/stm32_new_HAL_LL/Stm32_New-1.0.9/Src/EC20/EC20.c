#include "EC20.h"
#include "main.h"
#include "usart.h"
#include "user.h"
GPS_DATA gspdata;
GSM_init GSMinit;
GSM_Status GSMdata;

extern char ReceiveBuff[];
extern char UART4_ReceiveBuff[];
extern char AT_QIOPEN[];

char EC20_rx_buff[512];
uint8_t EC20_rx_flag=0;
uint8_t EC20_rx_num=0;
uint8_t EC20_rx_tempcount=0;

char *gps_strx,*gps_extstrx,*gps_Readystrx;
char *strx0;
char *Re_strx;

//��ȡIMEI
void Get_IMEI(void)
{
    char i=0,*strx;
    //   printf("AT\r\n"); //ͬ��EC20����
	  //EC20_Transmit((uint8_t*)UART4_Tx_GPS_TEST0);
	  printff(USART3, "AT\r\n",strlen("AT\r\n"));
	  printff(UART4, "AT \r\n",strlen("AT\r\n"));
    LL_mDelay(300);
    strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//����OK
    while(strx==NULL)
    {
  	  printff(UART4, "AT\r\n",strlen("AT\r\n"));
       LL_mDelay(300);
        strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//����OK
    }	
 		 printff(USART3, "Get_IMEI_START_1\r\n",strlen("Get_IMEI_START_1\r\n"));
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();
	  printff(UART4, "AT+GSN\r\n",strlen("AT+GSN\r\n"));
	  printff(USART3, "AT+GSN\r\n",strlen("AT+GSN\r\n"));
	   LL_mDelay(300);
  while(1)
    {
		      if(UART4_ReceiveBuff[2]>='0'&&UART4_ReceiveBuff[2]<='9')//��ȡģ��IMEI��ֵ
							 	         break;
			      else
        {
         		UART4_CLR_RecvBuf();
            printff(UART4, "AT+GSN\r\n",strlen("AT+GSN\r\n"));
					  LL_mDelay(300);		 
        }
    }
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		  for(i=0;i<20;i++)
    {
       GSMdata.IMEI[i]=UART4_ReceiveBuff[i];
    }	
		//printff(USART3, "Get_IMEI_START_2\r\n",strlen("Get_IMEI_START_2\r\n"));
    for(i=0;i<30;i++)
    {
        GSMinit. IMEI[i]=UART4_ReceiveBuff[i+2];
        GSMinit.IMEI[15]=',';
    }	
   for(i=0;i<16;i++)
	 gspdata.senddata[i]=GSMinit.IMEI[i];//��ȡ��IMEIֵ
   printff(UART4, "AT+QGPS=1\r\n",strlen("AT+QGPS=1\r\n")); //��GPS��Դ
   LL_mDelay(300);	
	 printff(UART4, "AT+QICLOSE=0\r\n",strlen("AT+QICLOSE=0\r\n")); //�ر���һ������
   LL_mDelay(300);	
}

/*  ��ȡGPS�ϱ������� EC20��ʼ��*/
void  EC20_GPS_Init(void)
{
	  int i;
	  printff(USART3, "AT\r\n",strlen("AT\r\n"));
	  printff(UART4, "AT\r\n",strlen("AT\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//����OK
    while(gps_strx==NULL)
    {
        UART4_CLR_RecvBuf();	
			  printff(USART3, "AT\r\n",strlen("AT\r\n"));
			  printff(UART4, "AT\r\n",strlen("AT\r\n"));
        LL_mDelay(500);
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//����OK
    }
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();	
		printff(USART3, "AT+QGPS?\r\n",strlen("AT+QGPS?\r\n"));    //��ѯ��ǰ״̬
		printff(UART4, "AT+QGPS?\r\n",strlen("AT+QGPS?\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+QGPS: 1");//�����Ѿ��ϵ�
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		if(gps_strx==NULL)           //���û�ϵ���ϵ磬�ϵ�Ͳ�Ҫ�ظ��ϵ�
		{
			printff(USART3, "AT+QGPS=1\r\n",strlen("AT+QGPS=1\r\n"));    //��GNSS�ϵ�
			printff(UART4, "AT+QGPS=1\r\n",strlen("AT+QGPS=1\r\n"));
		}
    LL_mDelay(500);
 	  printff(USART3, "ATE0\r\n",strlen("ATE0\r\n"));   //�رջ���
	  printff(UART4, "ATE0\r\n",strlen("ATE0\r\n"));
    LL_mDelay(500);
	  printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();	
    printff(USART3, "AT+CSQ\r\n",strlen("AT+CSQ\r\n"));  //���CSQ
   	printff(UART4, "AT+CSQ\r\n",strlen("AT+CSQ\r\n"));
    LL_mDelay(500);
 	  printff(USART3, "ATI\r\n",strlen("ATI\r\n"));  //���ģ��İ汾��
	  printff(UART4, "ATI\r\n",strlen("ATI\r\n"));
		LL_mDelay(500);
		for(i=0;i<100;i++)
    {
       GSMdata.Module_version_number[i]=UART4_ReceiveBuff[i];
    }	
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();	
    /////////////////////////////////
	  printff(USART3, "AT+CPIN?\r\n",strlen("AT+CPIN?\r\n"));  //���SIM���Ƿ���λ
	  printff(UART4, "AT+CPIN?\r\n",strlen("AT+CPIN?\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CPIN: READY");//�鿴�Ƿ񷵻�ready
    while(gps_strx==NULL)
    {
        UART4_CLR_RecvBuf();
		  	printff(USART3, "AT+CPIN?\r\n",strlen("AT+CPIN?\r\n"));
		  	printff(UART4, "AT+CPIN?\r\n",strlen("AT+CPIN?\r\n"));
        LL_mDelay(500);
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CPIN: READY");//���SIM���Ƿ���λ���ȴ�����λ�������ʶ�𲻵���ʣ��Ĺ�����û������
    }
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();	
    ///////////////////////////////////
		printff(USART3, "AT+CREG?\r\n",strlen("AT+CREG?\r\n")); //�鿴�Ƿ�ע��GSM����
		printff(UART4, "AT+CREG?\r\n",strlen("AT+CREG?\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CREG: 0,1");//��������
    gps_extstrx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CREG: 0,5");//��������������
    while(gps_strx==NULL&&gps_extstrx==NULL)
    {
        UART4_CLR_RecvBuf();
			  printff(USART3, "AT+CREG?\r\n",strlen("AT+CREG?\r\n")); //�鿴�Ƿ�ע��GSM����
			  printff(UART4, "AT+CREG?\r\n",strlen("AT+CREG?\r\n"));
        LL_mDelay(500);
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CREG: 0,1");//��������
        gps_extstrx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CREG: 0,5");//��������������
    }
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();
		printff(USART3, "AT+CGREG?\r\n",strlen("AT+CGREG?\r\n")); //�鿴�Ƿ�ע��GPRS����
		printff(UART4, "AT+CGREG?\r\n",strlen("AT+CGREG?\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CGREG: 0,1");//��������Ҫ��ֻ��ע��ɹ����ſ��Խ���GPRS���ݴ��䡣
    gps_extstrx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CGREG: 0,5");//��������������
    while(gps_strx==NULL&&gps_extstrx==NULL)
    {
        UART4_CLR_RecvBuf();
			  printff(USART3, "AT+CGREG?\r\n",strlen("AT+CGREG?\r\n"));  //�鿴�Ƿ�ע��GPRS����
		   	printff(UART4, "AT+CGREG?\r\n",strlen("AT+CGREG?\r\n"));
        LL_mDelay(500);
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CGREG: 0,1");//��������Ҫ��ֻ��ע��ɹ����ſ��Խ���GPRS���ݴ��䡣
        gps_extstrx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CGREG: 0,5");//��������������
    }
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();
 		printff(UART4, "AT+COPS?\r\n",strlen("AT+COPS?\r\n")); //�鿴ע�ᵽ�ĸ���Ӫ�̣�֧���ƶ� ��ͨ ���� 
		printff(USART3, "AT+COPS?\r\n",strlen("AT+COPS?\r\n"));
    LL_mDelay(500);
				for(i=0;i<30;i++)
    {
         GSMdata.Operators[i]=UART4_ReceiveBuff[i];
    }	
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();
 		printff(USART3, "AT+QICLOSE=0\r\n",strlen("AT+QICLOSE=0\r\n")); //�ر�socket����
		printff(UART4, "AT+QICLOSE=0\r\n",strlen("AT+QICLOSE=0\r\n"));
    LL_mDelay(500);
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();
    //����APN�����û���������
		printff(USART3, "AT+QICSGP=1,1,\042CMNET\042,\042\042,\042\042,0\r\n",strlen("AT+QICSGP=1,1,\042CMNET\042,\042\042,\042\042,0\r\n"));
		printff(UART4, "AT+QICSGP=1,1,\042CMNET\042,\042\042,\042\042,0\r\n",strlen("AT+QICSGP=1,1,\042CMNET\042,\042\042,\042\042,0\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//�����ɹ�
  while(gps_strx==NULL)
    {
        LL_mDelay(500);
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");////�����ɹ�
    }
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();
		printff(USART3, "AT+QIDEACT=1\r\n",strlen("AT+QIDEACT=1\r\n")); //ȥ����
		printff(UART4, "AT+QIDEACT=1\r\n",strlen("AT+QIDEACT=1\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//�����ɹ�
    while(gps_strx==NULL)
    {
        LL_mDelay(500);
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//�����ɹ�
    }
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();
		printff(USART3, "AT+QIACT=1\r\n",strlen("AT+QIACT=1\r\n"));  //����
		printff(UART4, "AT+QIACT=1\r\n",strlen("AT+QIACT=1\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//�����ɹ�
    while(gps_strx==NULL)
    {
        LL_mDelay(500);
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//�����ɹ�
    }
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();
		printff(USART3, "AT+QIACT?\r\n",strlen("AT+QIACT?\r\n")); //��ȡ��ǰ����IP��ַ
		printff(UART4, "AT+QIACT?\r\n",strlen("AT+QIACT?\r\n"));
    LL_mDelay(500);
			for(i=0;i<15;i++)
    {
			if(UART4_ReceiveBuff[i+17]== '"')
				break;
       GSMdata.Current_Card_IP[i]=UART4_ReceiveBuff[i+17];
    }	
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();
    //��������Ҫ��½��IP���룬����ֱ���³�ģʽ
//		printff(USART3, "AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,1\r\n",strlen("AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,1\r\n"));
//		printff(UART4, "AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,1\r\n",strlen("AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,1\r\n"));
    printff(USART3, AT_QIOPEN,strlen(AT_QIOPEN)); //��������Ҫ��½��IP���룬����ֱ���³�ģʽ
		printff(UART4, AT_QIOPEN,strlen(AT_QIOPEN));
		LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+QIOPEN: 0,0");//����Ƿ��½�ɹ�
    while(gps_strx==NULL)
    {
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+QIOPEN: 0,0");//����Ƿ��½�ɹ�
        LL_mDelay(100);
       
    }
    LL_mDelay(500);
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();
}		

///�����ַ������ݸ�������
void EC20Send_StrData(char *bufferdata)
{
    uint8_t untildata=0xff;
	  printff(USART3, "AT+QISEND=0\r\n",strlen("AT+QISEND=0\r\n"));
		printff(UART4, "AT+QISEND=0\r\n",strlen("AT+QISEND=0\r\n"));
    LL_mDelay(100);
		printff(UART4, bufferdata,strlen(bufferdata));
    LL_mDelay(100);	
	  LL_USART_TransmitData8(UART4, (uint8_t) 0x1a); //������ɺ���
		while(LL_USART_IsActiveFlag_TC(UART4) == RESET)
		{
		}
		LL_mDelay(100);
		gps_strx=strstr((char*)UART4_ReceiveBuff,(char*)"SEND OK");//�Ƿ���ȷ����
		while(gps_strx==NULL)
		{
				gps_strx=strstr((char*)UART4_ReceiveBuff,(char*)"SEND OK");//�Ƿ���ȷ����
				LL_mDelay(10);
		}
    LL_mDelay(100);
	  printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();
 	  printff(USART3, "AT+QISEND=0,0\r\n",strlen("AT+QISEND=0,0\r\n"));
		printff(UART4, "AT+QISEND=0,0\r\n",strlen("AT+QISEND=0,0\r\n"));
    LL_mDelay(200);
    gps_strx=strstr((char*)UART4_ReceiveBuff,(char*)"+QISEND:");//����ʣ���ֽ�����
		while(untildata)
    {
 				printff(USART3, "AT+QISEND=0,0\r\n",strlen("AT+QISEND=0,0\r\n"));
		    printff(UART4, "AT+QISEND=0,0\r\n",strlen("AT+QISEND=0,0\r\n"));
        LL_mDelay(200);
        gps_strx=strstr((char*)UART4_ReceiveBuff,(char*)"+QISEND:");//����ʣ���ֽ�����
        gps_strx=strstr((char*)gps_strx,(char*)",");//��ȡ��һ��,
        gps_strx=strstr((char*)(gps_strx+1),(char*)",");//��ȡ�ڶ���,
        untildata=*(gps_strx+1)-0x30;
        UART4_CLR_RecvBuf();
    }
    printff(USART3, "Send Flish!!!!!\r\n",strlen("Send Flish!!!!!\r\n"));
}

///͸��ģʽ�½�������
void EC20Send_RecAccessMode(void)
{   
	    char Data_SendIMEI_Buffer[15]={0}; 
			int i;
		  Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x01");  //��¼
			if(Re_strx)
			{		
				unsigned short Str_Len = 0;
				printff(UART4, "78780A01",strlen("78780A01"));              //��������͸����������  
				for(i = 0; i < 15; i++)
				   Data_SendIMEI_Buffer[i] = GSMdata.IMEI[i+1];    //�޳���һ�����з�
				printff(UART4, Data_SendIMEI_Buffer,strlen(Data_SendIMEI_Buffer));
				printff(UART4, "010D0A",strlen("010D0A"));
				printff(UART4, "\r\n",strlen("\r\n"));
		  	UART4_CLR_RecvBuf();
      }
			Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x08");  //����������
			if(Re_strx)
			{		
				printff(UART4, "787801080D0A",strlen("787801080D0A"));     //�豸��������������������������ڴ���
				printff(UART4, "\r\n",strlen("\r\n"));
				UART4_CLR_RecvBuf();
      }
			Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x10");  //����GPS��λ���ݰ�
			if(Re_strx)
			{		
				while(1)
				{
				EC20Send_ChangeMode(1);    //�л�ģʽΪ����ģʽ����Ϊ͸��ģʽ��ͻ������������δ֪
			 	EC20_GetGps_Send();  //��ȡGPS���ݰ�������
				EC20Send_ChangeMode(0);
				UART4_CLR_RecvBuf();
				}
      }
		Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x13");  //״̬��
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
	
					Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x14");  //�豸����ָ��
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"NO CARRIER");
			 if(Re_strx)
			 {
				 while(1)
				 {
							 printff(USART3, "Server Is Closed!\r\n",strlen("Server Is Closed!\r\n"));
         }
       }


}

//�������·�EC20��ʼ��  
	void  EC20_TR_Init(void)
{
	  int i = 0;
	  printff(USART3, "AT\r\n",strlen("AT\r\n"));
	  printff(UART4, "AT\r\n",strlen("AT\r\n"));
		LL_mDelay(500);
		gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//����OK
		while(gps_strx==NULL)
		{
				UART4_CLR_RecvBuf();	
			  printff(USART3, "AT\r\n",strlen("AT\r\n"));
	      printff(UART4, "AT\r\n",strlen("AT\r\n"));
				LL_mDelay(500);
				gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//����OK
		}
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();	
		
		printff(USART3, "AT+QGPS?\r\n",strlen("AT+QGPS?\r\n"));    //��ѯ��ǰ״̬
		printff(UART4, "AT+QGPS?\r\n",strlen("AT+QGPS?\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+QGPS: 1");//�����Ѿ��ϵ�
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		if(gps_strx==NULL)           //���û�ϵ���ϵ磬�ϵ�Ͳ�Ҫ�ظ��ϵ�
		{
			printff(USART3, "AT+QGPS=1\r\n",strlen("AT+QGPS=1\r\n"));    //��GNSS�ϵ�
			printff(UART4, "AT+QGPS=1\r\n",strlen("AT+QGPS=1\r\n"));
		}
    LL_mDelay(500);
		
		printff(USART3, "ATE0\r\n",strlen("ATE0\r\n"));  //�رջ���
	  printff(UART4, "ATE0\r\n",strlen("ATE0\r\n"));
		LL_mDelay(500);
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();	
		
		printff(UART4, "AT+GSN\r\n",strlen("AT+GSN\r\n"));  //��ȡģ��IMEI��ֵ
	  printff(USART3, "AT+GSN\r\n",strlen("AT+GSN\r\n"));  
	  LL_mDelay(500);
		while(1)
			{
						if(UART4_ReceiveBuff[2]>='0'&&UART4_ReceiveBuff[2]<='9')//��ȡģ��IMEI��ֵ
													 break;
							else
					{
							UART4_CLR_RecvBuf();
							printff(UART4, "AT+GSN\r\n",strlen("AT+GSN\r\n"));
							LL_mDelay(500);		 
					}
			}
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		  for(i=0;i<16;i++)
    {
       GSMdata.IMEI[i]=UART4_ReceiveBuff[i+1];   //i+1��Ϊ���޳���һ�����з�
    }	
		printff(USART3,GSMdata.IMEI,strlen(GSMdata.IMEI));   //��ӡ�洢��IMEI
		UART4_CLR_RecvBuf();
		
		printff(USART3, "AT+CSQ\r\n",strlen("AT+CSQ\r\n")); //���CSQ
	  printff(UART4, "AT+CSQ\r\n",strlen("AT+CSQ\r\n"));
		LL_mDelay(500);
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		printff(USART3, "ATI\r\n",strlen("ATI\r\n")); //���ģ��İ汾��
	  printff(UART4, "ATI\r\n",strlen("ATI\r\n"));
		LL_mDelay(500);
			for(i=0;i<100;i++)
    {
       GSMdata.Module_version_number[i]=UART4_ReceiveBuff[i];
    }	
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();
		printff(USART3, "AT+CPIN?\r\n",strlen("AT+CPIN?\r\n"));  //���SIM���Ƿ���λ
	  printff(UART4, "AT+CPIN?\r\n",strlen("AT+CPIN?\r\n"));
		LL_mDelay(500);
		gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CPIN: READY"); //�鿴�Ƿ񷵻�ready
		while(gps_strx==NULL)
		{
				UART4_CLR_RecvBuf();
			  printff(USART3, "AT+CPIN?\r\n",strlen("AT+CPIN?\r\n"));
	      printff(UART4, "AT+CPIN?\r\n",strlen("AT+CPIN?\r\n"));
				LL_mDelay(500);
				gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CPIN: READY");//���SIM���Ƿ���λ���ȴ�����λ�������ʶ�𲻵���ʣ��Ĺ�����û������
		}
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();	
	
		printff(USART3, "AT+CGREG?\r\n",strlen("AT+CGREG?\r\n")); //�鿴�Ƿ�ע��GPRS����
	  printff(UART4, "AT+CGREG?\r\n",strlen("AT+CGREG?\r\n"));
		LL_mDelay(500);
		Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CGREG: 0,1");//��������Ҫ��ֻ��ע��ɹ����ſ��Խ���GPRS���ݴ��䡣
		gps_extstrx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CGREG: 0,5");//��������������
		while(gps_strx==NULL&&gps_extstrx==NULL)
		{
				UART4_CLR_RecvBuf();
			  printff(USART3, "AT+CGREG?\r\n",strlen("AT+CGREG?\r\n"));  //�鿴�Ƿ�ע��GPRS����
	      printff(UART4, "AT+CGREG?\r\n",strlen("AT+CGREG?\r\n"));
				LL_mDelay(500);
				Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CGREG: 0,1");//��������Ҫ��ֻ��ע��ɹ����ſ��Խ���GPRS���ݴ��䡣
				gps_extstrx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CGREG: 0,5");//��������������
		}
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();
		printff(USART3, "AT+COPS?\r\n",strlen("AT+COPS?\r\n")); //�鿴ע�ᵽ�ĸ���Ӫ�̣�֧���ƶ� ��ͨ ���� 
	  printff(UART4, "AT+COPS?\r\n",strlen("AT+COPS?\r\n"));
		LL_mDelay(500);
				for(i=0;i<30;i++)
    {
         GSMdata.Operators[i]=UART4_ReceiveBuff[i];
    }	
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();
		printff(USART3, "AT+QICLOSE=0\r\n",strlen("AT+QICLOSE=0\r\n")); //�ر�socket����
	  printff(UART4, "AT+QICLOSE=0\r\n",strlen("AT+QICLOSE=0\r\n"));
	  LL_mDelay(500);
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();
	  //����APN�����û���������
		printff(USART3, "AT+QICSGP=1,1,\042CMNET\042,\042\042,\042\042,0\r\n",strlen("AT+QICSGP=1,1,\042CMNET\042,\042\042,\042\042,0\r\n"));
	  printff(UART4, "AT+QICSGP=1,1,\042CMNET\042,\042\042,\042\042,0\r\n",strlen("AT+QICSGP=1,1,\042CMNET\042,\042\042,\042\042,0\r\n"));
		LL_mDelay(500);
		gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//�����ɹ�
		while(gps_strx==NULL)
		{
				LL_mDelay(500);
				gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");////�����ɹ�
		}
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();
		printff(USART3, "AT+QIDEACT=1\r\n",strlen("AT+QIDEACT=1\r\n")); //ȥ����
	  printff(UART4, "AT+QIDEACT=1\r\n",strlen("AT+QIDEACT=1\r\n"));
		LL_mDelay(500);
		gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//�����ɹ�
		while(gps_strx==NULL)
		{
				LL_mDelay(500);
				gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//�����ɹ�
		}
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();
		printff(USART3, "AT+QIACT=1\r\n",strlen("AT+QIACT=1\r\n")); //����
	  printff(UART4, "AT+QIACT=1\r\n",strlen("AT+QIACT=1\r\n"));		
		LL_mDelay(500);
		gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//�����ɹ�
		while(gps_strx==NULL)
		{
				LL_mDelay(500);
				gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//�����ɹ�
		}
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();
		printff(USART3, "AT+QIACT?\r\n",strlen("AT+QIACT?\r\n")); //��ȡ��ǰ����IP��ַ
	  printff(UART4, "AT+QIACT?\r\n",strlen("AT+QIACT?\r\n"));
		LL_mDelay(500);
		for(i=0;i<15;i++)
    {
			if(UART4_ReceiveBuff[i+17]== '"')
				break;
       GSMdata.Current_Card_IP[i]=UART4_ReceiveBuff[i+17];
    }	
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();
    //����Ϊ͸��ģʽ
		printff(USART3, "AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,2\r\n",strlen("AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,2\r\n"));
	  printff(UART4, "AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,2\r\n",strlen("AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,2\r\n"));
		LL_mDelay(500);
		gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"CONNECT");//����Ƿ��½�ɹ�
		while(gps_strx==NULL)
		{
				gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"CONNECT");//����Ƿ��½�ɹ�
				LL_mDelay(100);
		}
    printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		LL_mDelay(500);
		UART4_CLR_RecvBuf();
}		

//��ȡGPS������Ϣ�����͸�������
void EC20_GetGps_Send(void)
{
		UART4_CLR_RecvBuf();  
		printff(USART3, "AT+QGPSGNMEA=\042GGA\042\r\n",strlen("AT+QGPSGNMEA=\042GGA\042\r\n"));  //��ȡGPS������λ����
		printff(UART4, "AT+QGPSGNMEA=\042GGA\042\r\n",strlen("AT+QGPSGNMEA=\042GGA\042\r\n"));			
		LL_mDelay(500);
		strx0=strstr((const char*)UART4_ReceiveBuff,(const char*)"+QGPSGNMEA:");//����OK
		while(strx0 == NULL)
		{
				LL_mDelay(50);
				strx0=strstr((const char*)UART4_ReceiveBuff,(const char*)"+QGPSGNMEA:");//����OK
				printff(USART3, "AT+QGPSGNMEA=\042GGA\042\r\n",strlen("AT+QGPSGNMEA=\042GGA\042\r\n"));
				printff(UART4, "AT+QGPSGNMEA=\042GGA\042\r\n",strlen("AT+QGPSGNMEA=\042GGA\042\r\n"));
		}			  	
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		EC20Send_StrData(UART4_ReceiveBuff);//ͨ��EC20�����ݷ��ͳ�ȥ
};

void EC20_Init(void)
{	 
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
		LL_mDelay(200);	
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);//���߿���
		LL_mDelay(300);	
	 //	EC20_UART_PROT_INIT();//�ı�EC20 �Ĵ���ģʽ
		LL_mDelay(200);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);//��ʼ�����
}

void EC20_Get_Data(void)
{
	  printff(USART3, "--------------------------------\r\n", strlen("--------------------------------\r\n"));
	  printff(USART3,"**************EC20**************\r\n",strlen("**************EC20**************\r\n"));
  	printff(USART3,"IMEI��",strlen("IMEI��"));        //��ӡIMEI��
	  printff(USART3,GSMdata.IMEI,strlen(GSMdata.IMEI));
	  printff(USART3,"\r\n\r\nCurrent_Card_IP��\r\n",strlen("\r\n\r\nCurrent_Card_IP��\r\n"));   // ��ӡ��ǰ��IP
	  printff(USART3,GSMdata.Current_Card_IP,strlen(GSMdata.Current_Card_IP));
		printff(USART3,"\r\n\r\nModule_version_number��",strlen("\r\n\r\nModule_version_number��"));   // ��ӡģ��汾��
	  printff(USART3,GSMdata.Module_version_number,strlen(GSMdata.Module_version_number));
	  printff(USART3,"\r\nOperators��",strlen("\r\nOperators��"));     //��ӡӪ����
	  printff(USART3,GSMdata.Operators,strlen(GSMdata.Operators));
	
  	/* ��ѯSIM���Ƿ���λ*/
	  printff(USART3, "\r\n\r\nWhether SIM card is in place��\r\n",strlen("\r\n\r\nWhether SIM card is in place��\r\n"));  //���SIM���Ƿ���λ
	  printff(UART4, "AT+CPIN?\r\n",strlen("AT+CPIN?\r\n"));
    LL_mDelay(500);
	  if(strstr((const char*)UART4_ReceiveBuff,(const char*)"+CPIN: READY") != NULL)
		  printff(USART3, "SIM READY!\r\n",strlen("SIM READY!\r\n"));
		else 
			printff(USART3, "SIM Error!",strlen("SIM Error!"));  //���SIM���Ƿ���λ;
	//  printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
	
		
    printff(USART3,"\r\n",strlen("\r\n"));
		printff(USART3, "-------------------------------\r\n\r\n", strlen("-------------------------------\r\n\r\n"));
};	

void EC20_Change_Server_IP(void) //�����ƣ�����洢�����⣬��������
{
	  int i = 0;
	  printff(USART3,"The original server IP configuration��\r\n",strlen("The original server IP configuration��\r\n"));
	  printff(USART3, AT_QIOPEN,strlen(AT_QIOPEN));       //ԭʼ������IP���˿�����
	  printff(USART3,"Please enter the modified IP��\r\n",strlen("Please enter the modified IP��\r\n"));
	  while(1)
		{
			for(i = 0; AT_QIOPEN[i] != '\0' ; i++)

					printff(UART4, AT_QIOPEN,strlen(AT_QIOPEN));
		}
};

///�л�����ģʽ͸����ָ��ģʽ
void EC20Send_ChangeMode(int Mode)   //Mode����ָ��ģʽ��1 Ϊ����ģʽ�� 0 Ϊ͸��ģʽ
{
  if(Mode)//�л�Ϊ����ģʽ
	{
		LL_mDelay(500);
	//	printf("+++");//�л�͸��
		printff(UART4, "+++",strlen("+++"));  //�л�͸��
		LL_mDelay(800);
		LL_mDelay(800);
    }
  else//�л�Ϊ����ģʽ
    {
        UART4_CLR_RecvBuf();
   //     printf("AT+QISWTMD=0,2\r\n");//�л�Ϊ͸��ģʽ
			  printff(UART4, "AT+QISWTMD=0,2\r\n",strlen("AT+QISWTMD=0,2\r\n"));
        LL_mDelay(300);
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"CONNECT");//�����л����
      while(gps_strx==NULL)
        {
            gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"CONNECT");//�����л����
        }
        UART4_CLR_RecvBuf();
        LL_mDelay(300);
    }
}
