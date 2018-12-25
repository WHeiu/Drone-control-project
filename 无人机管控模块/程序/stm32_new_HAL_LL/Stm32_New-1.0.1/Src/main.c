
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"

/* USER CODE BEGIN Includes */
#include<math.h>
#include <stdio.h>
#include <string.h>
#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"
#include "bmp280.h"
#include "mp9250.h"
#include "EC20.h"
#include <math.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CRC_HandleTypeDef hcrc;

QSPI_HandleTypeDef hqspi;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_rx;

HCD_HandleTypeDef hhcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

void uartdamget(void);

#define BUFFERSIZE 300                              
uint8_t ReceiveBuff[BUFFERSIZE];
uint8_t GPS_DATA_RECEIVE[BUFFERSIZE]="Connect the device\r\n";; 
uint8_t recv_end_flag = 0,Rx_len;       

__IO ITStatus UartReady = RESET;

#define RTC_ASYNCH_PREDIV    0x7F
#define RTC_SYNCH_PREDIV     0xF9
#define SENSORS_STATE_OK     0x01
#define CONNECT_STATE_OK     0x02

#define EC20_engine          0x00 //EC20 engine
#define ADC_engine           0x00	//ADC  engine

#define TEST_SEND            0x00
#define ERROR_STA            0X01
#define SUCCFULL_STA         0X00

int Uart_count=0;
struct {
  int32_t v_actual_temp_s32;
  int32_t v_actual_press_u32;
  int32_t v_actual_temp_combined_s32;
  int32_t v_actual_press_combined_u32;
  uint32_t actual_press_u64;
  float v_actual_temp_float;
  float v_actual_press_float;
  double v_actual_temp_double;
  double v_actual_press_double;
  int32_t err_cnt;
} bmp_data;

char str[100];

/**
  * @brief  Set the communication protocol on GPRS.
  *
  * @note   Tracker report:AAA, 140,latitude, longitude, altitude, Speed ,roll, pitch, yaw, 
						time_boot_ms, voltage_battery, battery_remaining, type, system_status
  *
  * @param  Longitude/Lantitude: Unit: degree ,Decimal. For example: -23.256438 .
																 When a minus (-) exists, the tracker is in the southern hemisphere.
																 When no minus (-) exists, the tracker is in the northern hemisphere.
	* @param	Altitude: Unit: meter.
  * @param 	Speed: 		External Gps provides speed data.
  * @param  Battery_status:   Battery_status <0, low battery; 
															Battery_status >0, normal status.
  * @param 	Uav_type: save the UAV of information.	
	*	@param  Vendor:	Manufacture name: HWA type defined "HW"
	* @param  System_status:Device status :    bit 0 = 1:sensors ok!;			bit 1 = 1:comunication to flying control system ok!
																						 bit 2 = 1:???        ;			bit 3 = 1:???
																						 bit 4 = 1:???        ;		  bit 5 = 1:???
																						 bit 6 = 1:???        ;			bit 7 = 1:???
  * @retval None
  */
	
typedef struct  
{  
    float 		Longitude;  
    float 		Lantitude; 
		int 			Altitude;
	  float		  Speed_x;
		float		  Speed_y;
		float		  Speed_z;
		float			Roll_angle;
		float 		Pitch_angle;
		float 		Yaw_angle;
		uint32_t  Timestamp;
		int8_t 		Battery_status;
		int8_t 		*Uav_type;
		char 			Vendor[3];
		uint8_t 	System_status; 
} Uavcomunication; 

/***********extern***************/
extern MPU_value mpu_value; 
extern char EC20_rx_buff[512];

extern char *EC20_sendmsg_cmd;
extern char *EC20_init_cmd;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CRC_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_RTC_Init(void);
static void MX_USB_OTG_FS_HCD_Init(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

extern int8_t BMP280_Begin(void);
extern float BMP280_ReadTemperature(void);
extern float BMP280_ReadPressure(void);
extern uint32_t BMP280_ReadPressure_int64(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
   //printf重定向串口3
int fputc(int ch, FILE *f)
{
	uint8_t c=(uint8_t) (ch & 0x00FF);
  HAL_UART_Transmit(&huart3,&c,1,10);
  
  return ch;
}

void Delay(unsigned int xms)  // xms
{
    unsigned int x,y;
    for(x=xms;x>0;x--)
        for(y=2000;y>0;y--);
}
char * EC20_sendmsg_cmd1[]={
	"AT+CPMS?\r\n",
	"AT+CNMI=1\r\n",
	"AT+CSCA?\r\n",
	"AT+CMGF=1\r\n",
};
void EC20_Transmit(uint8_t* UART4_Tx)
{ 
	 int size=strlen((const char*)UART4_Tx);
   HAL_UART_Transmit(&huart4, (uint8_t*)UART4_Tx,size,50);
}

void EC20_Transmit_Printf(uint8_t* UART4_Tx)
{ 
	 int size=strlen((const char*)UART4_Tx);
	 HAL_UART_Transmit(&huart3, (uint8_t*)UART4_Tx,size,50);
}
void EC20_Receive()
{
	   HAL_UART_Receive(&huart4, (uint8_t*)ReceiveBuff, COUNTOF(ReceiveBuff),50);
		/*打印收到的数据*/
    for(int i=0;i<COUNTOF(ReceiveBuff);i++)
    {
			  if(ReceiveBuff[i]=='O'&&ReceiveBuff[i+1]=='K')
				{  
					printf("OK\r\n");
					break;
				}
				if(ReceiveBuff[i]=='\0'&&ReceiveBuff[i+1]=='\0'&&ReceiveBuff[i+2]=='\0'&&ReceiveBuff[i+3]=='\0')
				{  
					printf("\n****** 	\r\n");
					break;
				}
				if(ReceiveBuff[i]=='0'&&ReceiveBuff[i+1]=='0'&&ReceiveBuff[i+2]=='0'&&ReceiveBuff[i+3]=='0')
				{  
					break;
				}
        printf("%c",ReceiveBuff[i]);
    }
    printf("\r\n"); 
    for(int i=0;i<=(COUNTOF(ReceiveBuff) - 1);i++)
		{
			if(ReceiveBuff[i]==':'&&ReceiveBuff[i-1]=='C'&&ReceiveBuff[i-2]=='O')
			{
				 i++;
				 for(int j=0;i<=(COUNTOF(ReceiveBuff) - 1);i++,j++)
					{
							/* 转换：之后的GPS数据 给GPS_DATA*/
						   if(ReceiveBuff[i]=='\0'&&ReceiveBuff[i+1]=='\0'&&ReceiveBuff[i+2]=='\0'&&ReceiveBuff[i+3]=='\0')
								{  
									printf("\n****** 	\r\n");
									break;
								}
								
						   GPS_DATA_RECEIVE[j]=ReceiveBuff[i];
							 if(ReceiveBuff[i]=='O'&&ReceiveBuff[i+1]=='K')
								{
									 printf("GPS数据\n");   
									break;
								}
								
							 printf("%c",ReceiveBuff[i]);
							 
					}
				}
		}			
		/*清空数据缓存*/
    for(int i = 0; i < COUNTOF(ReceiveBuff); i++) 
    ReceiveBuff[i]=0;
			
}
void EC20_Init()
{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
			HAL_Delay(200);	
			
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);//拉高控制
			HAL_Delay(300);	

			EC20_UART_PROT_INIT();//改变EC20 的串口模式
			HAL_Delay(200);
			
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);//初始化完成
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  //int i=0;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
 
	uint32_t Uart_count=0;  //Uart_count is used to test uart 
	
	uint8_t Flag_test=SUCCFULL_STA;    //set one
	uint8_t Flag_run =SUCCFULL_STA-1;  //set zero
	
	uint32_t ADC_Value[50];
	uint32_t ad1;
	uint8_t  ADC_i;

	uint8_t UART4_Tx_TEST1[]="AT+CSQ\r\n";
	uint8_t UART4_Tx_TEST2[]="ATD13036699696\r\n";
	uint8_t UART4_Tx_TEST3[]="AT+CPIN?\r\n";
	uint8_t UART4_Tx_TEST4[]="AT+CMGF=1\r\n";
	uint8_t UART4_Tx_TEST5[]="ATI\r\n";
	
	/*电话（可变）*/
	uint8_t UART4_Tx_TEST6[]="AT+CMGS=\"13036699696\"\r\n";
	uint8_t UART4_Tx_TEST7[]="AT+CMEE=2\r\n";
	
	uint8_t UART4_Tx_GPS_TEST0[]="ATI\r\n";
	uint8_t UART4_Tx_GPS_TEST1[]="AT+QGPS=1\r\n";
	uint8_t UART4_Tx_GPS_TEST2[]="AT+QGPSLOC?\r\n";
	uint8_t UART4_Tx_GPS_TEST3[]="AT+QGPSCFG=\"gpsnmeatype\",1\r\n";

	/*IP 地址口（可变）*/
  uint8_t UART4_Tx_TCP_TEST1[]="AT+QIOPEN=1,0,\"TCP\",\"119.27.184.248\",1234,0,1\r\n";
	uint8_t UART4_Tx_TCP_TEST2[]="AT+QISEND=0\r\n";
	uint8_t UART4_Tx_TCP_TEST3[]="AT+QICLOSE=0\r\n";
	
  uint8_t UART4_Tx_Flag_end[]={0x1A};
	uint8_t UART4_Tx_CMGS_ER[]="ERROR!!\r\n";
	uint8_t UART4_Tx_CMGS_end[]={0x1A};
	
	uint8_t UART4_Tx_MESSAGE1[]="AT+CPMS?\r\n";
	uint8_t UART4_Tx_MESSAGE2[]="AT+CNMI=1,2,0,1,0\r\n";
	uint8_t UART4_Tx_MESSAGE3[]="AT+CSCA?\r\n";
	uint8_t UART4_Tx_MESSAGE4[]="AT+CMGF=1\r\n";
	
	
	uint8_t UART4_Tx_CMGS[]="HELLO,WORLD!!!CONNECTED SUCUSSFUL!!!\r\n";

	
	uint8_t tempdata[50]={0};
	
	uint8_t Atempdata1[50]={0};
	uint8_t Atempdata2[50]={0};
	uint8_t Atempdata3[50]={0};
	
	uint8_t tempdata1[20]={0};
	uint8_t tempdata2[20]={0};
	uint8_t tempdata3[20]={0};
	uint8_t tempdata4[20]={0};
	uint8_t tempdata5[20]={0};
	uint8_t tempdata6[20]={0};
	uint8_t tempdata7[20]={0};
	
	uint8_t test_count=0;
	uint8_t test_flag=1;
	uint8_t count_ex=100;
	uint8_t count_OK=0;
	uint8_t *pTempdate;
	uint8_t m=0;
	uint8_t test_counter_polling=0;
	float altitude1; //存放初始高度
	pTempdate=tempdata;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
 	MX_GPIO_Init();
  MX_CRC_Init();
	MX_RTC_Init();
//   MX_DMA_Init();
	MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_QUADSPI_Init();

  MX_SPI1_Init();
  MX_SPI2_Init();
 // MX_ADC1_Init();

 // MX_USB_OTG_FS_HCD_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	

	/*************init the BMP280****************/
//  if (BMP280_Begin() == -1)
//  bmp_data.err_cnt++;
//	printf("Finshed the inition of BMP280.  \r\n");
//	
//	/*************init the MPU9250****************/
//	Init_MPU9250();
//	printf("Finshed the inition of MPU9250.  \r\n");
//	
//	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET);//TURN ON LED_RED 
//	
//	/*************init the EC20****************/	
  //EC20初始化 ,先拉高在拉低
  EC20_Init();
	printf("Finished the inition of EC20\r\n");
	
	printf("Inition Finished\r\n");

	/********init the RTC*******************/
	#if 0
	RTC_HandleTypeDef RtcHandle;
	RTC_TimeTypeDef stimestructureget;
	__HAL_RTC_RESET_HANDLE_STATE(&RtcHandle);
  RtcHandle.Instance = RTC;
  RtcHandle.Init.HourFormat     = RTC_HOURFORMAT_12;
  RtcHandle.Init.AsynchPrediv   = RTC_ASYNCH_PREDIV;
  RtcHandle.Init.SynchPrediv    = RTC_SYNCH_PREDIV;
  RtcHandle.Init.OutPut         = RTC_OUTPUT_DISABLE;
  RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  RtcHandle.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;
	#endif
	
	/*********SEND THE DATA TO GROUND AND SET THE FLAG********/
	#if 0
	
	/*要求所需的矫正*/
	tcp_connect_sever("101.207.17.199","1113");
	uint8_t Test_senddata[11]={'H' ,'E' , 'L' , 'L' , 'O',  'I',  'T',  'I',  'S' , 'M' , 'E' , '!'};
	uint8_t Test_receivedata_example[11]={'!' ,'E'  , 'M'  ,'S' , 'I' , 'T' , 'I' , 'O'  ,'L'  ,'L' , 'E' , 'H' };
	uint8_t Test_receivedata[11]=(0);
	#endif
	
	//HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_Value, 100); //send ADC_data to ADC_value
	     
	printf("\r\n\r\n\r\n");
     
//  /*启动GPS接收*/
//	EC20_uart4_send((uint8_t*)UART4_Tx_GPS_TEST1);    // 发送AT+QGPS=1
//	HAL_Delay(500);
//	/*打印AT错误信息*/
//	EC20_uart4_send((uint8_t*)UART4_Tx_TEST7);

//	/*开启IDLE中断*/
//	__HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
//	/*启动DMA接收*/
//  HAL_UART_Receive_DMA(&huart4,ReceiveBuff,BUFFERSIZE); 

	
//	bmp_data.v_actual_press_float = BMP280_ReadPressure();    //读取初始气压高度
//		altitude1 = (1013.25-(bmp_data.v_actual_press_float/100))*9;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_2);

#if 1
#if 1
		//MP9250
		printf("-------------------------------\r\n");
		printf("MPU-9520 \r\n");
		READ_MPU9250_MAG();    //读取磁力计
		READ_MPU9250_ACCEL();  //读取加速度
		READ_MPU9250_GYRO();   //读取陀螺仪

		printf("Mag: x:%d \ty:%d\tz:%d    \r\n", mpu_value.Mag[0],mpu_value.Mag[1],mpu_value.Mag[2]);        //磁力计
		printf("Gyro: x:%d \ty:%d\tz:%d   \r\n", mpu_value.Gyro[0],mpu_value.Gyro[1],mpu_value.Gyro[2]);     // 陀螺仪
		printf("Accel: x:%d \ty:%d\tz:%d  \r\n", mpu_value.Accel[0],mpu_value.Accel[1],mpu_value.Accel[2]);   //加速度
		printf("-------------------------------\r\n\r\n\r\n");
#endif 
//		//Bmp280
//		bmp_data.v_actual_temp_float = BMP280_ReadTemperature();
//		bmp_data.actual_press_u64 = BMP280_ReadPressure_int64();
//		bmp_data.v_actual_press_float = BMP280_ReadPressure();

//		printf("-------------------------------\r\n");
//	
//		printf("BMP280 \r\n");
//		printf("  T32: %u ℃  \r\n", bmp_data.v_actual_temp_s32);   //直接读的温度值32位
//		printf("  P32: %u Pa  \r\n", bmp_data.v_actual_press_u32);   //直接读的气压值32位
//		printf("  P64: %.3f KPa \r\n", ((float)bmp_data.actual_press_u64 / 256.0f)/1000);   //将气压转换为64位
//   	printf("  Tf: %.3f ℃\r\n", bmp_data.v_actual_temp_float);    //将温度转换为浮点型
//		printf("  Pf: %.3f KPa  \r\n", bmp_data.v_actual_press_float/1000);   //将气压转换为浮点型

//		float altitude2,altitude ,seaLevelhPa, mmHg;
//		mmHg = bmp_data.v_actual_press_float / 133.321995f;   // 同为压强
//		printf("  %.2f mmHg \r\n", mmHg);         

//		seaLevelhPa = 101325.0f;     //标准大气压
//	  altitude2 = (1013.25-((bmp_data.v_actual_press_float)/100))*9;   //测试点海拔高度
//	  altitude = altitude2-altitude1;     //相对高度
//	  //altitude = 44330 * (1.0 - pow((double)(bmp_data.v_actual_press_float / seaLevelhPa), 0.19029)); //pow函数？
//		printf( "  %.2f m  \r\n", altitude);   //高度
//   

//		printf("  nerror: %u \r\n", bmp_data.err_cnt);
//  	printf("-------------------------------\r\n\r\n");  
		
#endif
#if 0

	  printf("---------------------\r\n");
		printf("数据转换为字符串并发送\r\n");
		
		sprintf((char*)Atempdata1,"Mag: x:%d \ty:%d\tz:%d   \r\n", mpu_value.Mag[0],mpu_value.Mag[1],mpu_value.Mag[2]);
		sprintf((char*)Atempdata2,"Gyro: x:%d \ty:%d\tz:%d  \r\n", mpu_value.Gyro[0],mpu_value.Gyro[1],mpu_value.Gyro[2]);
		sprintf((char*)Atempdata3,"Accel: x:%d \ty:%d\tz:%d  \r\n", mpu_value.Accel[0],mpu_value.Accel[1],mpu_value.Accel[2]);
		
		
		sprintf((char*)tempdata1,"T32: %u 'C  \r\n", bmp_data.v_actual_temp_s32);
		sprintf((char*)tempdata2,"P32: %u Pa  \r\n", bmp_data.v_actual_press_u32);
		sprintf((char*)tempdata3,"P64: %.3f Pa \r\n", (float)bmp_data.actual_press_u64 / 256.0f);
		sprintf((char*)tempdata4,"Tf: %.3f  'C \r\n", bmp_data.v_actual_temp_float);
		sprintf((char*)tempdata5,"Pf: %.3f Pa  \r\n", bmp_data.v_actual_press_float);
		
		sprintf((char*)tempdata6,"%.2f mmHg \r\n", mmHg);
		sprintf((char*)tempdata7,"%.2f m  \r\n", altitude);

		//数据转换为字符串并发送
		EC20_Transmit_Printf((uint8_t*)Atempdata1);
		EC20_Transmit_Printf((uint8_t*)Atempdata2);
		EC20_Transmit_Printf((uint8_t*)Atempdata3);
		
		EC20_Transmit_Printf((uint8_t*)tempdata1);
		EC20_Transmit_Printf((uint8_t*)tempdata2);
		EC20_Transmit_Printf((uint8_t*)tempdata3);
		EC20_Transmit_Printf((uint8_t*)tempdata4);
		EC20_Transmit_Printf((uint8_t*)tempdata5);
		EC20_Transmit_Printf((uint8_t*)tempdata6);
		EC20_Transmit_Printf((uint8_t*)tempdata7);
		printf("END\r\n");
		printf("---------------------\r\n\r\n");
	#endif	

#if 0//GPRS test
	
		/*数据连接中断一次不可再连接，需重新启动或者关闭网络20秒左右后方能连接*/
		if(test_counter_polling==20)
		{
		  EC20_Transmit((uint8_t*)UART4_Tx_GPS_TEST1);
			test_counter_polling=0;
		}
		test_counter_polling++;
		


			
		#if 0
		/*询问GPS定位信息*/
		
		printf("444444\r\n");
		EC20_Transmit((uint8_t*)UART4_Tx_GPS_TEST2);
		
		/*接受串口数据*/		
	  EC20_Receive();
	 
		
		
		/*开启直传模式*/
		EC20_Transmit((uint8_t*)UART4_Tx_TCP_TEST1);
	  EC20_Receive();
		
    /*开启传送开关*/
		EC20_Transmit((uint8_t*)UART4_Tx_TCP_TEST2);
	  EC20_Receive();
		
		
		/*传送TCP数据*/
			printf("5555555\n");
		/*（1）传送GPS数据*/
	  EC20_Transmit((uint8_t*)GPS_DATA_RECEIVE);
		EC20_Transmit((uint8_t*)UART4_Tx_Flag_end);//结束标志0x1A
		EC20_Receive();
		
		/*（2）传送传感器数据*/
		
		/*传送BMP9250数据*/
		EC20_Transmit((uint8_t*)Atempdata1);
		EC20_Transmit((uint8_t*)Atempdata2);
		EC20_Transmit((uint8_t*)Atempdata3);
		
		/*传送BMP280数据*/
		EC20_Transmit((uint8_t*)tempdata1);
		EC20_Transmit((uint8_t*)tempdata2);
		EC20_Transmit((uint8_t*)tempdata3);
		EC20_Transmit((uint8_t*)tempdata4);
		EC20_Transmit((uint8_t*)tempdata5);
		EC20_Transmit((uint8_t*)tempdata6);
		EC20_Transmit((uint8_t*)tempdata7);
		
		printf("666666\n");
		EC20_Transmit((uint8_t*)UART4_Tx_Flag_end);//结束标志0x1A
		EC20_Receive();
		
		
		/*打印并清除数据*/
		printf("\r\n数据如下：\r\n");
		 for(int i = 0; i < COUNTOF(GPS_DATA_RECEIVE); i++) 
    {
			printf("%c",ReceiveBuff[i]);
			GPS_DATA_RECEIVE[i]=0;
		}
		
		/*初始化检测*/
    test_flag=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);
		
    /*如果检测STATE口为低，重新检测并初始化*/
		if(test_flag==1)
		{
			
      EC20_Init();
			test_count++;
			if(test_count==10)
			{
				printf("EC20断电失败\n");
				test_count=0;
				break;
			}
	  }
		#endif
#endif
				

#if 0 //adc采样，采样电池
				HAL_Delay(500);
        for(ADC_i = 0,ad1 =0; ADC_i< 100;)
        {
            ad1 += ADC_Value[ADC_i++];

        }
        ad1 /= 50;
        printf("\r\n******** ADC DMA Example ********\r\n\r\n");
        printf(" AD1 value = %1.3fV \r\n", ad1*3.3f/4096);


#endif


#if 0 //ec20数据整合
		
			/**********Send the data to the Ground and check it****************/
//			if(HAL_UART_Transmit_IT(&huart4, (uint8_t *)Test_senddata, sizeof(Test_senddata)) != HAL_OK)
//			{
//					Error_Handler();
//			}
//			Delay(100);
//		
//			if(HAL_UART_Receive_IT(&huart4, (uint8_t *)Test_receivedata, sizeof(Test_receivedata)) != HAL_OK)
//			{
//				Error_Handler();
//			}
//			
//		for(uint8_t comparecount=0,Different_flag=0;comparecount<=sizeof(Test_receivedata);comparecount++)//??辫?箨?????陏?Y
//		{	
//			/*****************COMPARE the flag of the data which form Ground*********************/
//			if(Test_receivedata[comparecount]==Test_receivedata_example[comparecount])
//			{
//				Different_flag++;
//				printf(" Connect %u  \n", Different_flag);
//			}
//			/***************If receive the data from Ground,begain send the data*************/
//			if(Different_flag==sizeof(Test_senddata))
//			{
//				Flag_run=SUCCFULL_STA;//enable the data of GPRS
//			}
//		}
	
		if(1)
		{
		tcp_connect_sever("101.207.17.199","1113");
		HAL_RTC_GetTime(&RtcHandle, &stimestructureget, RTC_FORMAT_BIN);//Get the time of RTC
		
		/***********Set the communication protocol on GPRS.***************/
		/*结构体赋值*/
		Uavcomunication.Longitude=mpu_value.Mag[0];
		Uavcomunication.Lantitude=mpu_value.Mag[1];
		Uavcomunication.Altitude=altitude;
		Uavcomunication.Speed_x=mpu_value.Accel[0];
		Uavcomunication.Speed_y=mpu_value.Accel[1];
		Uavcomunication.Speed_z=mpu_value.Accel[2];
		Uavcomunication.Roll_angle=mpu_value.Gyro[0];
		Uavcomunication.Pitch_angle=mpu_value.Gyro[1];
		Uavcomunication.Yaw_angle=mpu_value.Gyro[2];
		Uavcomunication.Timestamp=stimestructureget.Seconds;
		Uavcomunication.Battery_status=0;
		Uavcomunication.Uav_type=0;
		Uavcomunication.Vendor="HW";
		Uavcomunication.System_status=0;
		
		/*********ACCORDING TO STATE INIT THE Uavcomunication.System_status*************/
		if(bmp_data.err_cnt==0)
		{Uavcomunication.System_status=Uavcomunication.System_status&SENSORS_STATE_OK;}
		
		if(Flag_run==1)
		{Uavcomunication.System_status=Uavcomunication.System_status&CONNECT_STATE_OK;}
		
		/* To transmit the Uav of date */
		/*传送数据*/
		EC20_Transmit((uint8_t*)Uavcomunication);
		/*开启直传模式*/
		EC20_Transmit((uint8_t*)UART4_Tx_TCP_TEST1);
	  EC20_Receive();
		
    /*开启传送开关*/
		EC20_Transmit((uint8_t*)UART4_Tx_TCP_TEST2);
	  EC20_Receive();
		
		
		/*传送TCP数据*/
		
		/*传送UAV数据*/
	  EC20_Transmit((uint8_t*)Uavcomunication);
		EC20_Transmit((uint8_t*)UART4_Tx_Flag_end);//结束标志0x1A
		EC20_Receive();
		}
		
#if 0//message test
		
		/*设置短信存储区*/
		EC20_Transmit((uint8_t*)UART4_Tx_MESSAGE1);
	  EC20_Receive();

		/*设置短信存储区格式*/
		EC20_Transmit((uint8_t*)UART4_Tx_MESSAGE2);
		EC20_Receive();
			
		/*查询短信中心*/
		EC20_Transmit((uint8_t*)UART4_Tx_MESSAGE3);
	  EC20_Receive();

		/*设置短信文本格式*/
		EC20_Transmit((uint8_t*)UART4_Tx_MESSAGE4);
		EC20_Receive();
		
		/*发送短信，手机号码确定，结束标志位0X1A*/
		EC20_Transmit((uint8_t*)UART4_Tx_TEST6);
		EC20_Receive();
		
		/*发送字符串内容，字符串内容自定义*/
		EC20_Transmit((uint8_t*)GPS_DATA_RECEIVE);
	
    /*发送结束符，结束后需等待5秒收到短信*/
		EC20_Transmit((uint8_t*)UART4_Tx_Flag_end);
	
#endif
#endif
//		EC20_Transmit((uint8_t*)UART4_Tx_GPS_TEST0); 
//		EC20_Receive();
// HAL_Delay(3000);
  }
	
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_UART4|RCC_PERIPHCLK_USB
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* UART4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UART4_IRQn, 8, 0);
  HAL_NVIC_EnableIRQ(UART4_IRQn);
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 11, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* DMA2_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the ADC multi-mode 
    */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_VBAT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* CRC init function */
static void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* QUADSPI init function */
static void MX_QUADSPI_Init(void)
{

  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 255;
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 1;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* UART4 init function */
static void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart4.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  huart4.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USB_OTG_FS init function */
static void MX_USB_OTG_FS_HCD_Init(void)
{

  hhcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hhcd_USB_OTG_FS.Init.Host_channels = 8;
  hhcd_USB_OTG_FS.Init.speed = HCD_SPEED_FULL;
  hhcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hhcd_USB_OTG_FS.Init.phy_itface = HCD_PHY_EMBEDDED;
  hhcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  if (HAL_HCD_Init(&hhcd_USB_OTG_FS) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MP_CS_GPIO_Port, MP_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, EC20_RESET_Pin|EC20_CTL_Pin|EC20_DISABLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EC20_NETMODE_Pin|LE_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : EC20_STA_Pin */
  GPIO_InitStruct.Pin = EC20_STA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(EC20_STA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MP_CS_Pin */
  GPIO_InitStruct.Pin = MP_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MP_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EC20_RESET_Pin EC20_CTL_Pin BMP_CS_Pin */
  GPIO_InitStruct.Pin = EC20_RESET_Pin|EC20_CTL_Pin|BMP_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : EC20_DISABLE_Pin */
  GPIO_InitStruct.Pin = EC20_DISABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EC20_DISABLE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EC20_RI_Pin */
  GPIO_InitStruct.Pin = EC20_RI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EC20_RI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EC20_NETMODE_Pin LE_RED_Pin */
  GPIO_InitStruct.Pin = EC20_NETMODE_Pin|LE_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : UAV_232_RDY_Pin */
  GPIO_InitStruct.Pin = UAV_232_RDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UAV_232_RDY_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void uartdamget(void)
{
			if(recv_end_flag ==1)
		{		 
			
				/*打印收到的数据长度*/
				printf("\nrx_len=%d\r\n",Rx_len);
			
				/*打印收到的数据*/
				for(int i=0;i<Rx_len;i++)
				{
						 printf("%c",ReceiveBuff[i]);
				}
				printf("\r\n"); 
				
				/*清空数据缓存*/
				for(int i = 0; i < Rx_len ; i++) 
				ReceiveBuff[i]=0;
	
				Rx_len=0;
				recv_end_flag=0;
		}
		HAL_UART_Receive_DMA(&huart4,(uint8_t*)ReceiveBuff,BUFFERSIZE);
}
/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle. 
  * @note   This example shows a simple way to report end of IT Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */
  UartReady = SET;


}
/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */

  UartReady = SET;

}

/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
    Error_Handler();
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
