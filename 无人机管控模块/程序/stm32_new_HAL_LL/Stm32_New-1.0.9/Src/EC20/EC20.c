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

//获取IMEI
void Get_IMEI(void)
{
    char i=0,*strx;
    //   printf("AT\r\n"); //同步EC20开机
	  //EC20_Transmit((uint8_t*)UART4_Tx_GPS_TEST0);
	  printff(USART3, "AT\r\n",strlen("AT\r\n"));
	  printff(UART4, "AT \r\n",strlen("AT\r\n"));
    LL_mDelay(300);
    strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//返回OK
    while(strx==NULL)
    {
  	  printff(UART4, "AT\r\n",strlen("AT\r\n"));
       LL_mDelay(300);
        strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//返回OK
    }	
 		 printff(USART3, "Get_IMEI_START_1\r\n",strlen("Get_IMEI_START_1\r\n"));
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();
	  printff(UART4, "AT+GSN\r\n",strlen("AT+GSN\r\n"));
	  printff(USART3, "AT+GSN\r\n",strlen("AT+GSN\r\n"));
	   LL_mDelay(300);
  while(1)
    {
		      if(UART4_ReceiveBuff[2]>='0'&&UART4_ReceiveBuff[2]<='9')//获取模块IMEI的值
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
	 gspdata.senddata[i]=GSMinit.IMEI[i];//获取到IMEI值
   printff(UART4, "AT+QGPS=1\r\n",strlen("AT+QGPS=1\r\n")); //打开GPS电源
   LL_mDelay(300);	
	 printff(UART4, "AT+QICLOSE=0\r\n",strlen("AT+QICLOSE=0\r\n")); //关闭上一次连接
   LL_mDelay(300);	
}

/*  获取GPS上报服务器 EC20初始化*/
void  EC20_GPS_Init(void)
{
	  int i;
	  printff(USART3, "AT\r\n",strlen("AT\r\n"));
	  printff(UART4, "AT\r\n",strlen("AT\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//返回OK
    while(gps_strx==NULL)
    {
        UART4_CLR_RecvBuf();	
			  printff(USART3, "AT\r\n",strlen("AT\r\n"));
			  printff(UART4, "AT\r\n",strlen("AT\r\n"));
        LL_mDelay(500);
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//返回OK
    }
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();	
		printff(USART3, "AT+QGPS?\r\n",strlen("AT+QGPS?\r\n"));    //查询当前状态
		printff(UART4, "AT+QGPS?\r\n",strlen("AT+QGPS?\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+QGPS: 1");//返回已经上电
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		if(gps_strx==NULL)           //如果没上电就上电，上电就不要重复上电
		{
			printff(USART3, "AT+QGPS=1\r\n",strlen("AT+QGPS=1\r\n"));    //对GNSS上电
			printff(UART4, "AT+QGPS=1\r\n",strlen("AT+QGPS=1\r\n"));
		}
    LL_mDelay(500);
 	  printff(USART3, "ATE0\r\n",strlen("ATE0\r\n"));   //关闭回显
	  printff(UART4, "ATE0\r\n",strlen("ATE0\r\n"));
    LL_mDelay(500);
	  printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();	
    printff(USART3, "AT+CSQ\r\n",strlen("AT+CSQ\r\n"));  //检查CSQ
   	printff(UART4, "AT+CSQ\r\n",strlen("AT+CSQ\r\n"));
    LL_mDelay(500);
 	  printff(USART3, "ATI\r\n",strlen("ATI\r\n"));  //检查模块的版本号
	  printff(UART4, "ATI\r\n",strlen("ATI\r\n"));
		LL_mDelay(500);
		for(i=0;i<100;i++)
    {
       GSMdata.Module_version_number[i]=UART4_ReceiveBuff[i];
    }	
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();	
    /////////////////////////////////
	  printff(USART3, "AT+CPIN?\r\n",strlen("AT+CPIN?\r\n"));  //检查SIM卡是否在位
	  printff(UART4, "AT+CPIN?\r\n",strlen("AT+CPIN?\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CPIN: READY");//查看是否返回ready
    while(gps_strx==NULL)
    {
        UART4_CLR_RecvBuf();
		  	printff(USART3, "AT+CPIN?\r\n",strlen("AT+CPIN?\r\n"));
		  	printff(UART4, "AT+CPIN?\r\n",strlen("AT+CPIN?\r\n"));
        LL_mDelay(500);
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CPIN: READY");//检查SIM卡是否在位，等待卡在位，如果卡识别不到，剩余的工作就没法做了
    }
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();	
    ///////////////////////////////////
		printff(USART3, "AT+CREG?\r\n",strlen("AT+CREG?\r\n")); //查看是否注册GSM网络
		printff(UART4, "AT+CREG?\r\n",strlen("AT+CREG?\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CREG: 0,1");//返回正常
    gps_extstrx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CREG: 0,5");//返回正常，漫游
    while(gps_strx==NULL&&gps_extstrx==NULL)
    {
        UART4_CLR_RecvBuf();
			  printff(USART3, "AT+CREG?\r\n",strlen("AT+CREG?\r\n")); //查看是否注册GSM网络
			  printff(UART4, "AT+CREG?\r\n",strlen("AT+CREG?\r\n"));
        LL_mDelay(500);
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CREG: 0,1");//返回正常
        gps_extstrx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CREG: 0,5");//返回正常，漫游
    }
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();
		printff(USART3, "AT+CGREG?\r\n",strlen("AT+CGREG?\r\n")); //查看是否注册GPRS网络
		printff(UART4, "AT+CGREG?\r\n",strlen("AT+CGREG?\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CGREG: 0,1");//，这里重要，只有注册成功，才可以进行GPRS数据传输。
    gps_extstrx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CGREG: 0,5");//返回正常，漫游
    while(gps_strx==NULL&&gps_extstrx==NULL)
    {
        UART4_CLR_RecvBuf();
			  printff(USART3, "AT+CGREG?\r\n",strlen("AT+CGREG?\r\n"));  //查看是否注册GPRS网络
		   	printff(UART4, "AT+CGREG?\r\n",strlen("AT+CGREG?\r\n"));
        LL_mDelay(500);
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CGREG: 0,1");//，这里重要，只有注册成功，才可以进行GPRS数据传输。
        gps_extstrx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CGREG: 0,5");//返回正常，漫游
    }
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();
 		printff(UART4, "AT+COPS?\r\n",strlen("AT+COPS?\r\n")); //查看注册到哪个运营商，支持移动 联通 电信 
		printff(USART3, "AT+COPS?\r\n",strlen("AT+COPS?\r\n"));
    LL_mDelay(500);
				for(i=0;i<30;i++)
    {
         GSMdata.Operators[i]=UART4_ReceiveBuff[i];
    }	
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();
 		printff(USART3, "AT+QICLOSE=0\r\n",strlen("AT+QICLOSE=0\r\n")); //关闭socket连接
		printff(UART4, "AT+QICLOSE=0\r\n",strlen("AT+QICLOSE=0\r\n"));
    LL_mDelay(500);
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();
    //接入APN，无用户名和密码
		printff(USART3, "AT+QICSGP=1,1,\042CMNET\042,\042\042,\042\042,0\r\n",strlen("AT+QICSGP=1,1,\042CMNET\042,\042\042,\042\042,0\r\n"));
		printff(UART4, "AT+QICSGP=1,1,\042CMNET\042,\042\042,\042\042,0\r\n",strlen("AT+QICSGP=1,1,\042CMNET\042,\042\042,\042\042,0\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//开启成功
  while(gps_strx==NULL)
    {
        LL_mDelay(500);
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");////开启成功
    }
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();
		printff(USART3, "AT+QIDEACT=1\r\n",strlen("AT+QIDEACT=1\r\n")); //去激活
		printff(UART4, "AT+QIDEACT=1\r\n",strlen("AT+QIDEACT=1\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//开启成功
    while(gps_strx==NULL)
    {
        LL_mDelay(500);
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//开启成功
    }
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();
		printff(USART3, "AT+QIACT=1\r\n",strlen("AT+QIACT=1\r\n"));  //激活
		printff(UART4, "AT+QIACT=1\r\n",strlen("AT+QIACT=1\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//开启成功
    while(gps_strx==NULL)
    {
        LL_mDelay(500);
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//开启成功
    }
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();
		printff(USART3, "AT+QIACT?\r\n",strlen("AT+QIACT?\r\n")); //获取当前卡的IP地址
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
    //这里是需要登陆的IP号码，采用直接吐出模式
//		printff(USART3, "AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,1\r\n",strlen("AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,1\r\n"));
//		printff(UART4, "AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,1\r\n",strlen("AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,1\r\n"));
    printff(USART3, AT_QIOPEN,strlen(AT_QIOPEN)); //这里是需要登陆的IP号码，采用直接吐出模式
		printff(UART4, AT_QIOPEN,strlen(AT_QIOPEN));
		LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+QIOPEN: 0,0");//检查是否登陆成功
    while(gps_strx==NULL)
    {
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+QIOPEN: 0,0");//检查是否登陆成功
        LL_mDelay(100);
       
    }
    LL_mDelay(500);
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();
}		

///发送字符型数据给服务器
void EC20Send_StrData(char *bufferdata)
{
    uint8_t untildata=0xff;
	  printff(USART3, "AT+QISEND=0\r\n",strlen("AT+QISEND=0\r\n"));
		printff(UART4, "AT+QISEND=0\r\n",strlen("AT+QISEND=0\r\n"));
    LL_mDelay(100);
		printff(UART4, bufferdata,strlen(bufferdata));
    LL_mDelay(100);	
	  LL_USART_TransmitData8(UART4, (uint8_t) 0x1a); //发送完成函数
		while(LL_USART_IsActiveFlag_TC(UART4) == RESET)
		{
		}
		LL_mDelay(100);
		gps_strx=strstr((char*)UART4_ReceiveBuff,(char*)"SEND OK");//是否正确发送
		while(gps_strx==NULL)
		{
				gps_strx=strstr((char*)UART4_ReceiveBuff,(char*)"SEND OK");//是否正确发送
				LL_mDelay(10);
		}
    LL_mDelay(100);
	  printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();
 	  printff(USART3, "AT+QISEND=0,0\r\n",strlen("AT+QISEND=0,0\r\n"));
		printff(UART4, "AT+QISEND=0,0\r\n",strlen("AT+QISEND=0,0\r\n"));
    LL_mDelay(200);
    gps_strx=strstr((char*)UART4_ReceiveBuff,(char*)"+QISEND:");//发送剩余字节数据
		while(untildata)
    {
 				printff(USART3, "AT+QISEND=0,0\r\n",strlen("AT+QISEND=0,0\r\n"));
		    printff(UART4, "AT+QISEND=0,0\r\n",strlen("AT+QISEND=0,0\r\n"));
        LL_mDelay(200);
        gps_strx=strstr((char*)UART4_ReceiveBuff,(char*)"+QISEND:");//发送剩余字节数据
        gps_strx=strstr((char*)gps_strx,(char*)",");//获取第一个,
        gps_strx=strstr((char*)(gps_strx+1),(char*)",");//获取第二个,
        untildata=*(gps_strx+1)-0x30;
        UART4_CLR_RecvBuf();
    }
    printff(USART3, "Send Flish!!!!!\r\n",strlen("Send Flish!!!!!\r\n"));
}

///透传模式下接受数据
void EC20Send_RecAccessMode(void)
{   
	    char Data_SendIMEI_Buffer[15]={0}; 
			int i;
		  Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x01");  //登录
			if(Re_strx)
			{		
				unsigned short Str_Len = 0;
				printff(UART4, "78780A01",strlen("78780A01"));              //串口数据透传至服务器  
				for(i = 0; i < 15; i++)
				   Data_SendIMEI_Buffer[i] = GSMdata.IMEI[i+1];    //剔除第一个换行符
				printff(UART4, Data_SendIMEI_Buffer,strlen(Data_SendIMEI_Buffer));
				printff(UART4, "010D0A",strlen("010D0A"));
				printff(UART4, "\r\n",strlen("\r\n"));
		  	UART4_CLR_RecvBuf();
      }
			Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x08");  //发送心跳包
			if(Re_strx)
			{		
				printff(UART4, "787801080D0A",strlen("787801080D0A"));     //设备向服务器发送心跳包，发送周期待定
				printff(UART4, "\r\n",strlen("\r\n"));
				UART4_CLR_RecvBuf();
      }
			Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x10");  //发送GPS定位数据包
			if(Re_strx)
			{		
				while(1)
				{
				EC20Send_ChangeMode(1);    //切换模式为命令模式，因为透传模式冲突，产生的问题未知
			 	EC20_GetGps_Send();  //获取GPS数据包并发送
				EC20Send_ChangeMode(0);
				UART4_CLR_RecvBuf();
				}
      }
		Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x13");  //状态包
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
	
					Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x14");  //设备休眠指令
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

//服务器下发EC20初始化  
	void  EC20_TR_Init(void)
{
	  int i = 0;
	  printff(USART3, "AT\r\n",strlen("AT\r\n"));
	  printff(UART4, "AT\r\n",strlen("AT\r\n"));
		LL_mDelay(500);
		gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//返回OK
		while(gps_strx==NULL)
		{
				UART4_CLR_RecvBuf();	
			  printff(USART3, "AT\r\n",strlen("AT\r\n"));
	      printff(UART4, "AT\r\n",strlen("AT\r\n"));
				LL_mDelay(500);
				gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//返回OK
		}
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();	
		
		printff(USART3, "AT+QGPS?\r\n",strlen("AT+QGPS?\r\n"));    //查询当前状态
		printff(UART4, "AT+QGPS?\r\n",strlen("AT+QGPS?\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+QGPS: 1");//返回已经上电
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		if(gps_strx==NULL)           //如果没上电就上电，上电就不要重复上电
		{
			printff(USART3, "AT+QGPS=1\r\n",strlen("AT+QGPS=1\r\n"));    //对GNSS上电
			printff(UART4, "AT+QGPS=1\r\n",strlen("AT+QGPS=1\r\n"));
		}
    LL_mDelay(500);
		
		printff(USART3, "ATE0\r\n",strlen("ATE0\r\n"));  //关闭回显
	  printff(UART4, "ATE0\r\n",strlen("ATE0\r\n"));
		LL_mDelay(500);
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();	
		
		printff(UART4, "AT+GSN\r\n",strlen("AT+GSN\r\n"));  //获取模块IMEI的值
	  printff(USART3, "AT+GSN\r\n",strlen("AT+GSN\r\n"));  
	  LL_mDelay(500);
		while(1)
			{
						if(UART4_ReceiveBuff[2]>='0'&&UART4_ReceiveBuff[2]<='9')//获取模块IMEI的值
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
       GSMdata.IMEI[i]=UART4_ReceiveBuff[i+1];   //i+1是为了剔除第一个换行符
    }	
		printff(USART3,GSMdata.IMEI,strlen(GSMdata.IMEI));   //打印存储的IMEI
		UART4_CLR_RecvBuf();
		
		printff(USART3, "AT+CSQ\r\n",strlen("AT+CSQ\r\n")); //检查CSQ
	  printff(UART4, "AT+CSQ\r\n",strlen("AT+CSQ\r\n"));
		LL_mDelay(500);
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		printff(USART3, "ATI\r\n",strlen("ATI\r\n")); //检查模块的版本号
	  printff(UART4, "ATI\r\n",strlen("ATI\r\n"));
		LL_mDelay(500);
			for(i=0;i<100;i++)
    {
       GSMdata.Module_version_number[i]=UART4_ReceiveBuff[i];
    }	
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();
		printff(USART3, "AT+CPIN?\r\n",strlen("AT+CPIN?\r\n"));  //检查SIM卡是否在位
	  printff(UART4, "AT+CPIN?\r\n",strlen("AT+CPIN?\r\n"));
		LL_mDelay(500);
		gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CPIN: READY"); //查看是否返回ready
		while(gps_strx==NULL)
		{
				UART4_CLR_RecvBuf();
			  printff(USART3, "AT+CPIN?\r\n",strlen("AT+CPIN?\r\n"));
	      printff(UART4, "AT+CPIN?\r\n",strlen("AT+CPIN?\r\n"));
				LL_mDelay(500);
				gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CPIN: READY");//检查SIM卡是否在位，等待卡在位，如果卡识别不到，剩余的工作就没法做了
		}
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();	
	
		printff(USART3, "AT+CGREG?\r\n",strlen("AT+CGREG?\r\n")); //查看是否注册GPRS网络
	  printff(UART4, "AT+CGREG?\r\n",strlen("AT+CGREG?\r\n"));
		LL_mDelay(500);
		Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CGREG: 0,1");//，这里重要，只有注册成功，才可以进行GPRS数据传输。
		gps_extstrx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CGREG: 0,5");//返回正常，漫游
		while(gps_strx==NULL&&gps_extstrx==NULL)
		{
				UART4_CLR_RecvBuf();
			  printff(USART3, "AT+CGREG?\r\n",strlen("AT+CGREG?\r\n"));  //查看是否注册GPRS网络
	      printff(UART4, "AT+CGREG?\r\n",strlen("AT+CGREG?\r\n"));
				LL_mDelay(500);
				Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CGREG: 0,1");//，这里重要，只有注册成功，才可以进行GPRS数据传输。
				gps_extstrx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CGREG: 0,5");//返回正常，漫游
		}
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();
		printff(USART3, "AT+COPS?\r\n",strlen("AT+COPS?\r\n")); //查看注册到哪个运营商，支持移动 联通 电信 
	  printff(UART4, "AT+COPS?\r\n",strlen("AT+COPS?\r\n"));
		LL_mDelay(500);
				for(i=0;i<30;i++)
    {
         GSMdata.Operators[i]=UART4_ReceiveBuff[i];
    }	
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();
		printff(USART3, "AT+QICLOSE=0\r\n",strlen("AT+QICLOSE=0\r\n")); //关闭socket连接
	  printff(UART4, "AT+QICLOSE=0\r\n",strlen("AT+QICLOSE=0\r\n"));
	  LL_mDelay(500);
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();
	  //接入APN，无用户名和密码
		printff(USART3, "AT+QICSGP=1,1,\042CMNET\042,\042\042,\042\042,0\r\n",strlen("AT+QICSGP=1,1,\042CMNET\042,\042\042,\042\042,0\r\n"));
	  printff(UART4, "AT+QICSGP=1,1,\042CMNET\042,\042\042,\042\042,0\r\n",strlen("AT+QICSGP=1,1,\042CMNET\042,\042\042,\042\042,0\r\n"));
		LL_mDelay(500);
		gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//开启成功
		while(gps_strx==NULL)
		{
				LL_mDelay(500);
				gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");////开启成功
		}
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();
		printff(USART3, "AT+QIDEACT=1\r\n",strlen("AT+QIDEACT=1\r\n")); //去激活
	  printff(UART4, "AT+QIDEACT=1\r\n",strlen("AT+QIDEACT=1\r\n"));
		LL_mDelay(500);
		gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//开启成功
		while(gps_strx==NULL)
		{
				LL_mDelay(500);
				gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//开启成功
		}
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();
		printff(USART3, "AT+QIACT=1\r\n",strlen("AT+QIACT=1\r\n")); //激活
	  printff(UART4, "AT+QIACT=1\r\n",strlen("AT+QIACT=1\r\n"));		
		LL_mDelay(500);
		gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//开启成功
		while(gps_strx==NULL)
		{
				LL_mDelay(500);
				gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//开启成功
		}
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();
		printff(USART3, "AT+QIACT?\r\n",strlen("AT+QIACT?\r\n")); //获取当前卡的IP地址
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
    //设置为透传模式
		printff(USART3, "AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,2\r\n",strlen("AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,2\r\n"));
	  printff(UART4, "AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,2\r\n",strlen("AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,2\r\n"));
		LL_mDelay(500);
		gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"CONNECT");//检查是否登陆成功
		while(gps_strx==NULL)
		{
				gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"CONNECT");//检查是否登陆成功
				LL_mDelay(100);
		}
    printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		LL_mDelay(500);
		UART4_CLR_RecvBuf();
}		

//获取GPS数据信息并发送给服务器
void EC20_GetGps_Send(void)
{
		UART4_CLR_RecvBuf();  
		printff(USART3, "AT+QGPSGNMEA=\042GGA\042\r\n",strlen("AT+QGPSGNMEA=\042GGA\042\r\n"));  //读取GPS北斗定位数据
		printff(UART4, "AT+QGPSGNMEA=\042GGA\042\r\n",strlen("AT+QGPSGNMEA=\042GGA\042\r\n"));			
		LL_mDelay(500);
		strx0=strstr((const char*)UART4_ReceiveBuff,(const char*)"+QGPSGNMEA:");//返回OK
		while(strx0 == NULL)
		{
				LL_mDelay(50);
				strx0=strstr((const char*)UART4_ReceiveBuff,(const char*)"+QGPSGNMEA:");//返回OK
				printff(USART3, "AT+QGPSGNMEA=\042GGA\042\r\n",strlen("AT+QGPSGNMEA=\042GGA\042\r\n"));
				printff(UART4, "AT+QGPSGNMEA=\042GGA\042\r\n",strlen("AT+QGPSGNMEA=\042GGA\042\r\n"));
		}			  	
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		EC20Send_StrData(UART4_ReceiveBuff);//通过EC20将数据发送出去
};

void EC20_Init(void)
{	 
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
		LL_mDelay(200);	
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);//拉高控制
		LL_mDelay(300);	
	 //	EC20_UART_PROT_INIT();//改变EC20 的串口模式
		LL_mDelay(200);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);//初始化完成
}

void EC20_Get_Data(void)
{
	  printff(USART3, "--------------------------------\r\n", strlen("--------------------------------\r\n"));
	  printff(USART3,"**************EC20**************\r\n",strlen("**************EC20**************\r\n"));
  	printff(USART3,"IMEI：",strlen("IMEI："));        //打印IMEI码
	  printff(USART3,GSMdata.IMEI,strlen(GSMdata.IMEI));
	  printff(USART3,"\r\n\r\nCurrent_Card_IP：\r\n",strlen("\r\n\r\nCurrent_Card_IP：\r\n"));   // 打印当前卡IP
	  printff(USART3,GSMdata.Current_Card_IP,strlen(GSMdata.Current_Card_IP));
		printff(USART3,"\r\n\r\nModule_version_number：",strlen("\r\n\r\nModule_version_number："));   // 打印模块版本号
	  printff(USART3,GSMdata.Module_version_number,strlen(GSMdata.Module_version_number));
	  printff(USART3,"\r\nOperators：",strlen("\r\nOperators："));     //打印营运商
	  printff(USART3,GSMdata.Operators,strlen(GSMdata.Operators));
	
  	/* 查询SIM卡是否在位*/
	  printff(USART3, "\r\n\r\nWhether SIM card is in place：\r\n",strlen("\r\n\r\nWhether SIM card is in place：\r\n"));  //检查SIM卡是否在位
	  printff(UART4, "AT+CPIN?\r\n",strlen("AT+CPIN?\r\n"));
    LL_mDelay(500);
	  if(strstr((const char*)UART4_ReceiveBuff,(const char*)"+CPIN: READY") != NULL)
		  printff(USART3, "SIM READY!\r\n",strlen("SIM READY!\r\n"));
		else 
			printff(USART3, "SIM Error!",strlen("SIM Error!"));  //检查SIM卡是否在位;
	//  printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
	
		
    printff(USART3,"\r\n",strlen("\r\n"));
		printff(USART3, "-------------------------------\r\n\r\n", strlen("-------------------------------\r\n\r\n"));
};	

void EC20_Change_Server_IP(void) //待完善，数组存储有问题，改用链表
{
	  int i = 0;
	  printff(USART3,"The original server IP configuration：\r\n",strlen("The original server IP configuration：\r\n"));
	  printff(USART3, AT_QIOPEN,strlen(AT_QIOPEN));       //原始服务器IP、端口配置
	  printff(USART3,"Please enter the modified IP：\r\n",strlen("Please enter the modified IP：\r\n"));
	  while(1)
		{
			for(i = 0; AT_QIOPEN[i] != '\0' ; i++)

					printff(UART4, AT_QIOPEN,strlen(AT_QIOPEN));
		}
};

///切换工作模式透传和指令模式
void EC20Send_ChangeMode(int Mode)   //Mode参数指定模式，1 为命令模式， 0 为透传模式
{
  if(Mode)//切换为命令模式
	{
		LL_mDelay(500);
	//	printf("+++");//切换透传
		printff(UART4, "+++",strlen("+++"));  //切换透传
		LL_mDelay(800);
		LL_mDelay(800);
    }
  else//切换为数据模式
    {
        UART4_CLR_RecvBuf();
   //     printf("AT+QISWTMD=0,2\r\n");//切换为透传模式
			  printff(UART4, "AT+QISWTMD=0,2\r\n",strlen("AT+QISWTMD=0,2\r\n"));
        LL_mDelay(300);
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"CONNECT");//命令切换完成
      while(gps_strx==NULL)
        {
            gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"CONNECT");//命令切换完成
        }
        UART4_CLR_RecvBuf();
        LL_mDelay(300);
    }
}
