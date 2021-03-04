/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h> //strlen
#include <stdio.h> //printf
#include <stdlib.h> //printf
#include "fatfs_sd.h" //sd
#include "lsm6dsl_reg.h"
#include "edmi.h"
#include "mdm_sim7070g.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef union{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

typedef union{
  int16_t i16bit;
  uint8_t u8bit[2];
} axis1bit16_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TX_BUF_DIM          1000
#define BIT_0	( 1 << 0 )
#define BIT_1	( 1 << 1 )
#define BIT_2	( 1 << 2 )
#define SECOND	1000
#define MINUTE	60*SECOND
#define HOUR	60*MINUTE
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

osThreadId mainProgramHandle;
uint32_t mainProgramBuffer[ 512 ];
osStaticThreadDef_t mainProgramControlBlock;
osThreadId readGyroHandle;
uint32_t readGyroBuffer[ 128 ];
osStaticThreadDef_t readGyroControlBlock;
osThreadId sendDataToServeHandle;
uint32_t sendDataToServeBuffer[ 512 ];
osStaticThreadDef_t sendDataToServeControlBlock;
osThreadId readEoBHandle;
uint32_t readEoBBuffer[ 700 ];
osStaticThreadDef_t readEoBControlBlock;
osThreadId readLPHandle;
uint32_t readLPBuffer[ 700 ];
osStaticThreadDef_t readLPControlBlock;
osThreadId readInstantHandle;
uint32_t readInstantBuffer[ 700 ];
osStaticThreadDef_t readInstantControlBlock;
osThreadId sendHardBitHandle;
uint32_t sendHardBitBuffer[ 512 ];
osStaticThreadDef_t sendHardBitControlBlock;
osThreadId checkVbatHandle;
uint32_t checkVbatBuffer[ 128 ];
osStaticThreadDef_t checkVbatControlBlock;
osThreadId checkVMonHandle;
uint32_t checkVMonBuffer[ 128 ];
osStaticThreadDef_t checkVMonControlBlock;
osMutexId myMutex01Handle;
osStaticMutexDef_t myMutex01ControlBlock;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
void main_program(void const * argument);
void read_gyro(void const * argument);
void send_data_to_server(void const * argument);
void read_EoB(void const * argument);
void read_LP(void const * argument);
void read_instant(void const * argument);
void send_hard_bit(void const * argument);
void check_vbat(void const * argument);
void check_vmon(void const * argument);

/* USER CODE BEGIN PFP */
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static int32_t platform_write(void *handle, uint8_t Reg, uint8_t *Bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t Reg, uint8_t *Bufp,
                             uint16_t len);
int fputc(int ch, FILE *f)
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
//  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}
int _write(int file, char *ptr, int len){
  HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 0xFFFF);
  return len;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//uint8_t msg[] = "\nhello world\n";

float vbat_percentage;
uint8_t pwr_vmon, pwr_state, vbat_state;
uint8_t jmlChMK6N;
bool uart3done, uart2busy, uart3disc;
bool usemqtt,uart2done;
int cntpok=0;

char uart1_temp[1100],uart1_buf[1];
char uart2_temp[1100],uart2_buf[1];
char uart3_temp[1100],uart3_buf[1];
char uart2_at[20];
int i,j,k;
int len_mydata;
char bytecat[120];
char time_token[120];

char csq[20],cgatt[27];
uint8_t csqint,cgattint,smstateint,secmin1;
int signal;
char *networkstr;
char *smstatestr;
char *ATSMSTATE="AT+SMSTATE?\r\n";
char *ATSMPUB;
char * temp_data;
char * temp_data2;

FATFS fs;
FIL fil;
FRESULT fresult;
char buffer[1100], mydata[1100];
//char buffer1[1024];
char buffer2[1100];
char buffer3[1100];
UINT br,bw;
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
static axis1bit16_t data_raw_temperature;
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float temperature_degC;
static uint8_t whoamI, rst;
uint8_t tx_buffer[TX_BUF_DIM];

stmdev_ctx_t dev_ctx;

bool onlineMode;
int typemeter;
int net_err_cnt, send_err_cnt, res, error_status, state;
char * snMeter="000000000", buffer1;
long timestamp_now;
int cnt_instant=0, cnt_eob=0, cnt_lp=0;

char eob_date,eob_hour, eob_minute, lp_hour, lp_minute, instant_hour, instant_minute;
long lp_rec=0;
int lpregm[14];
char st_hour, st_min, st_sec;
int cekNTP = 0;
void hard_reset_rtos(){
	  HAL_GPIO_WritePin(HARD_RST_GPIO_Port, HARD_RST_Pin, GPIO_PIN_RESET);
	  osDelay(1000);
	  HAL_GPIO_WritePin(HARD_RST_GPIO_Port, HARD_RST_Pin, GPIO_PIN_SET);
}

void restartModem(){
	enable_modem();
	if(onlineMode)  setup_modem();
}

void runGyro()
{
  osSignalSet( readGyroHandle, BIT_1 | BIT_2);
}

int datauart3size(char buf[])
{
	int i = 0;
	while(buf[i] != 0x03) i++;
	return i+1;
}

int bufsize(char *buf)
{
	int i = 0;
	while(*buf++ != '\0') i++;
	return i;
}

int bufsizechararray(char buf[])
{
	int i = 0;
	while(buf[i] != '\0') i++;
	return i+1;
}

void bufclear(void)
{
	for (int i=0; i < 1024; i++)
	{
		buffer[i] = '\0';
		mydata[i] = '\0';
	}
}

int charToInt(char c){

return c - '0';
}

static void tx_com( uint8_t *tx_buffer, uint16_t len )
{
  HAL_UART_Transmit( &huart1, tx_buffer, len, 1000 );
}

static int32_t platform_write(void *handle, uint8_t Reg, uint8_t *Bufp,
                              uint16_t len)
{
  HAL_I2C_Mem_Write(handle, LSM6DSL_I2C_ADD_H, Reg,
					  I2C_MEMADD_SIZE_8BIT, Bufp, len, 1000);
  return 0;
}

static int32_t platform_read(void *handle, uint8_t Reg, uint8_t *Bufp,
                             uint16_t len)
{
  HAL_I2C_Mem_Read(handle, LSM6DSL_I2C_ADD_H, Reg,
				   I2C_MEMADD_SIZE_8BIT, Bufp, len, 1000);
  return 0;
}

void sendMQTTMAIN(){
	uart2done = false;
	sprintf(uart2_at,"AT+CPSI?");
	printf(uart2_at);
	SerialATprintln("AT+CPSI?", 8, 0xff);
	osDelay(2000);
	for(int y=0;y<30;y++){
				osDelay(200);
				if(uart2done)break;
			}
	int cmpint=strcmp(networkstr," GSM");
	if(cmpint != 0){
		if(signal<-115){
				SerialATprintln("AT+CNMP=13", 10, 0xff);
				osDelay(2000);
				reset_mqtt();
				SerialATprintln("AT+SMCONN",9,0xff);
				uart2done = false;
				for(int y=0;y<30;y++){
					osDelay(200);
					if(uart2done)break;
				}
			}
	}
	else {
		if(signal<=-100){
			for(int z=0;z<3;z++){
				printf("modem reboot\n");
				modem_reset_rtos();
				reset_mqtt();
				SerialATprintln("AT+CPSI?", 8, 0xff);
				osDelay(2000);
				for(int y=0;y<30;y++){
					osDelay(200);
					if(uart2done)break;
				}
				if(signal > -100) break;
			}

		}
	}
	SerialATprintln("AT+CNMP?", 8, 0xff);
	osDelay(2000);
	memset(ATSMPUB,0,48);
	memset(smstatestr,0,11);
	len_mydata = bufsizechararray(mydata);
//	HAL_UART_Transmit(&huart2,"AT+SMSTATE?\r\n",13,0xff);
	for(int z=0;z<3;z++){
		uart2done = false;
		HAL_UART_Transmit(&huart2,(uint8_t *) ATSMSTATE,strlen(ATSMSTATE),0xff);
		for(int y=0;y<30;y++){
				osDelay(200);
				if(uart2done)break;
			}
		temp_data =strstr((char *)buffer2,"+SMSTATE:");
		memcpy(smstatestr,temp_data,11);
		smstateint = strcmp(smstatestr,"+SMSTATE: 0");
		uart2done = false;
		if(smstateint!=0) break;
		reset_mqtt();
		SerialATprintln("AT+SMCONN",9,0xff);
		uart2done = false;
		for(int y=0;y<30;y++){
			osDelay(200);
			if(uart2done)break;
		}
	}
	uart2done = false;
	SerialATprintln("AT+SMCONN",9,0xff);
	for(int i=0;i<20;i++){
		osDelay(200);
		if(uart2done)break;
	}
	uart2done = false;
//		SerialATprintln("AT+SMUNSUB=\"tayra/incoming\"", 27, 0xff); //tayra/message
	SerialATprintln("AT+SMUNSUB=\"tayra/config\"", 25, 0xff); //tayra/message
		for(int i=0;i<25;i++){
			osDelay(200);
			if(uart2done)break;
		}
	uart2done = false;
	sprintf(ATSMPUB,"AT+SMPUB=\"tayra/incoming\",%d,1,1\r\n",len_mydata);
	HAL_UART_Transmit(&huart2,(uint8_t *) ATSMPUB,strlen(ATSMPUB),0xff);
	for(int y=0;y<20;y++){
		osDelay(200);
		if(uart2done)break;
	}
	uart2done = false;
		printf("mydata: %s\r\n",mydata);
		HAL_UART_Transmit(&huart2, (uint8_t *)mydata, len_mydata,0x3ff);
		for(int y=0;y<20;y++){
			osDelay(200);
			if(uart2done)break;
		}
	uart2done = false;
//		SerialATprintln("AT+SMSUB=\"tayra/incoming\",1", 27, 0xff); //tayra/message
		SerialATprintln("AT+SMSUB=\"tayra/config\",1", 25, 0xff); //tayra/message
		for(int i=0;i<20;i++){
			osDelay(200);
			if(uart2done)break;
		}
	// change cnmp to 2 (nbiot)
		uart2done = false;
		SerialATprintln("AT+CNMP=2", 9, 0xff);
		osDelay(2000);
		SerialATprintln("AT+CNMP?", 8, 0xff);
		osDelay(2000);
}

void sendMQTTpwr(){
	bufclear();
//	temp_data=malloc(200);
//	temp_data2=malloc(50);
	memset((char *)mydata,0,1100);
	uart2done = false;
		sprintf(uart2_at,"AT+CPSI?");
		printf(uart2_at);
		SerialATprintln("AT+CPSI?", 8, 0xff);
		osDelay(4000);
		for(int y=0;y<40;y++){
					osDelay(200);
					if(uart2done)break;
				}
		int cmpint=strcmp(networkstr," GSM");
			if(cmpint != 0){
				if(signal<-115){
						SerialATprintln("AT+CNMP=13", 10, 0xff);
						osDelay(2000);
						reset_mqtt();
						SerialATprintln("AT+SMCONN",9,0xff);
						uart2done = false;
						for(int y=0;y<30;y++){
							osDelay(200);
							if(uart2done)break;
						}
					}
			}
			else {
				if(signal<=-100){
					for(int z=0;z<3;z++){
						printf("modem reboot\n");
						modem_reset_rtos();
						reset_mqtt();
						SerialATprintln("AT+CPSI?", 8, 0xff);
						osDelay(2000);
						for(int y=0;y<30;y++){
							osDelay(200);
							if(uart2done)break;
						}
						if(signal > -100) break;
					}

				}
			}
		SerialATprintln("AT+CNMP?", 8, 0xff);
		osDelay(2000);
	memset((char *)mydata,0,1100);
	cmpint = strcmp(snMeter, "00000000");
	if(cmpint == 0) read_sn_edmi();
	if(typemeter==6) sprintf(temp_data,"{\"mid\":%s,\"pid\":\"Event\",\"mtr\":\"MK6N\"",snMeter);
	else sprintf(temp_data,"{\"mid\":%s,\"pid\":\"Event\",\"mtr\":\"MK10E\"",snMeter);
	sprintf(temp_data2,",\"net\":\"");
	strcat(temp_data, temp_data2);
	strcat(temp_data, networkstr);
	strcat(temp_data, "\"");
	sprintf(temp_data2,",\"sig\":\"%d\"",signal);
	strcat(temp_data, temp_data2);
	sprintf(temp_data2,",\"bat\":\"%.2f\"",vbat_percentage);
	strcat(temp_data, temp_data2);
	sprintf(temp_data2,",\"pwr\":\"%d\"}",pwr_vmon);
	strcat(temp_data, temp_data2);
	strcpy(mydata,temp_data);
	uart2done = false;
	memset(ATSMPUB,0,48);
	memset(smstatestr,0,11);
	len_mydata = bufsizechararray(mydata);
//	HAL_UART_Transmit(&huart2,"AT+SMSTATE?\r\n",13,0xff);
	for(int i=0;i<10;i++){
			osDelay(200);
			if(uart2done)break;
		}
	for(int z=0;z<3;z++){
			uart2done = false;
			HAL_UART_Transmit(&huart2,(uint8_t *) ATSMSTATE,strlen(ATSMSTATE),0xff);
			for(int y=0;y<30;y++){
					osDelay(200);
					if(uart2done)break;
				}
			temp_data =strstr((char *)buffer2,"+SMSTATE:");
			memcpy(smstatestr,temp_data,11);
			smstateint = strcmp(smstatestr,"+SMSTATE: 0");
			uart2done = false;
			if(smstateint!=0) break;
			reset_mqtt();
			SerialATprintln("AT+SMCONN",9,0xff);
			uart2done = false;
			for(int y=0;y<30;y++){
				osDelay(200);
				if(uart2done)break;
			}
		}
		uart2done = false;
		SerialATprintln("AT+SMCONN",9,0xff);
		for(int i=0;i<20;i++){
			osDelay(200);
			if(uart2done)break;
		}
//	uint8_t test = strcmp(smstatestr,"+SMSTATE: 1");
	uart2done = false;
//	SerialATprintln("AT+SMUNSUB=\"tayra/incoming\"", 27, 0xff); //tayra/message
	SerialATprintln("AT+SMUNSUB=\"tayra/config\"", 25, 0xff); //tayra/message
	for(int i=0;i<25;i++){
		osDelay(200);
		if(uart2done)break;
	}
	uart2done = false;
	sprintf(ATSMPUB,"AT+SMPUB=\"tayra/incoming\",%d,1,1\r\n",len_mydata);
	HAL_UART_Transmit(&huart2,(uint8_t *) ATSMPUB,strlen(ATSMPUB),0xff);
	for(int y=0;y<20;y++){
		osDelay(200);
		if(uart2done)break;
	}
	uart2done = false;
		printf("mydata: %s\r\n",mydata);
		HAL_UART_Transmit(&huart2, (uint8_t *)mydata, len_mydata,0x3ff);
		for(int y=0;y<20;y++){
			osDelay(200);
			if(uart2done)break;
		}
	uart2done = false;
//	SerialATprintln("AT+SMSUB=\"tayra/incoming\",1", 27, 0xff); //tayra/message
	SerialATprintln("AT+SMSUB=\"tayra/config\",1", 25, 0xff); //tayra/message
	for(int i=0;i<20;i++){
		osDelay(200);
		if(uart2done)break;
	}
	// change cnmp to 2 (nbiot)
	uart2done = false;
	SerialATprintln("AT+CNMP=2", 9, 0xff);
	osDelay(2000);
	SerialATprintln("AT+CNMP?", 8, 0xff);
	osDelay(2000);
//	osDelay(2000);
}

void sendSimple(){
	uart2done = false;
		sprintf(uart2_at,"AT+CPSI?");
		printf(uart2_at);
		SerialATprintln("AT+CPSI?", 8, 0xff);
		osDelay(4000);
		for(int y=0;y<40;y++){
					osDelay(200);
					if(uart2done)break;
				}
		int cmpint=strcmp(networkstr," GSM");
					if(cmpint != 0){
						if(signal<-115){
								SerialATprintln("AT+CNMP=13", 10, 0xff);
								osDelay(2000);
								reset_mqtt();
								SerialATprintln("AT+SMCONN",9,0xff);
								uart2done = false;
								for(int y=0;y<30;y++){
									osDelay(200);
									if(uart2done)break;
								}
							}
					}
					else {
						if(signal<=-100){
							for(int z=0;z<3;z++){
								printf("modem reboot\n");
								modem_reset_rtos();
								reset_mqtt();
								SerialATprintln("AT+CPSI?", 8, 0xff);
								osDelay(2000);
								for(int y=0;y<30;y++){
									osDelay(200);
									if(uart2done)break;
								}
								if(signal > -100) break;
							}

						}
					}
	SerialATprintln("AT+CNMP?", 8, 0xff);
	osDelay(2000);
	uart2done = false;
	bufclear();
//	temp_data=malloc(200);
//	temp_data2=malloc(50);
	memset((char *)mydata,0,1100);
	cmpint = strcmp(snMeter, "00000000");
	if(cmpint == 0) read_sn_edmi();
	if(typemeter==6) {
		if(uart3disc) sprintf(temp_data,"{\"mid\":%s,\"pid\":\"Heartbeat\",\"mtr\":\"MK6N\",\"mtrcon\":\"disconnect\"",snMeter);
		else sprintf(temp_data,"{\"mid\":%s,\"pid\":\"Heartbeat\",\"mtr\":\"MK6N\",\"mtrcon\":\"connect\"",snMeter);
	}
	else {
		if(uart3disc) sprintf(temp_data,"{\"mid\":%s,\"pid\":\"Heartbeat\",\"mtr\":\"MK10E\",\"mtrcon\":\"disconnect\"",snMeter);
		else sprintf(temp_data,"{\"mid\":%s,\"pid\":\"Heartbeat\",\"mtr\":\"MK10E\",\"mtrcon\":\"connect\"",snMeter);
	}
	sprintf(temp_data2,",\"net\":\"");
	strcat(temp_data, temp_data2);
	strcat(temp_data, networkstr);
	strcat(temp_data, "\"");
	sprintf(temp_data2,",\"sig\":\"%d\"",signal);
	strcat(temp_data, temp_data2);
  	sprintf(temp_data2,",\"bat\":\"%.2f\"",vbat_percentage);
	strcat(temp_data, temp_data2);
	sprintf(temp_data2,",\"pwr\":\"%d\"}",pwr_vmon);
	strcat(temp_data, temp_data2);
	strcpy(mydata,temp_data);
	printf("mydata: %s\r\n",mydata);
//	  strcat(buffer, snMeter);
//	  strcat(buffer, ",\"pid\":\"Heartbeat\"}");
//	  printf(buffer);
//  	if(onlineMode) sendToServer((char*)mydata,bufsizechararray(mydata));
	csqint = 0;
			cgattint = 1;
//			cgattint = check_cgatt_rtos();
			SerialATprintln("AT+CPSI?", 8, 0xff);
			osDelay(2000);

			uart2done=false;
			memset((char *)buffer2,0,sizeof(buffer2));
			memset((char *)csq,0,sizeof(csq));
			char *strcsq = malloc(sizeof(csq));


			SerialATprintln("AT+CSQ",6,0xff);
			osDelay(100);
			for(int y=0;y<20;y++){
				osDelay(200);
				if(uart2done)break;
			}
			strcsq = strstr((char *)buffer2,"+CSQ:");
			strcpy((char *)csq,strcsq);
			for(int x=0;x<20;x++){
				if(csq[x]=='\r' || csq[x]=='\n') csq[x]='\0';
				if(x>10) csq[x]='\0';
			}
			memset((char *)strcsq,0,sizeof(csq));
			strcsq = strstr((char *)csq,"+CSQ: 99,99");
			csqint = strcmp(strcsq,"+CSQ: 99,99");
			if(csqint!=0) csqint = 1;
			if(csqint || cgattint) {
	//			printf("online True\r\n");
				onlineMode = true;
			}
			else {
				onlineMode = false;
				net_err_cnt++;
			}
	if(usemqtt){
		len_mydata=bufsizechararray(mydata);
		memset(ATSMPUB,0,48);
			memset(smstatestr,0,11);
			len_mydata = bufsizechararray(mydata);
		//	HAL_UART_Transmit(&huart2,"AT+SMSTATE?\r\n",13,0xff);
			for(int z=0;z<3;z++){
				uart2done = false;
				HAL_UART_Transmit(&huart2,(uint8_t *) ATSMSTATE,strlen(ATSMSTATE),0xff);
				for(int y=0;y<30;y++){
						osDelay(200);
						if(uart2done)break;
					}
				temp_data =strstr((char *)buffer2,"+SMSTATE:");
				memcpy(smstatestr,temp_data,11);
				smstateint = strcmp(smstatestr,"+SMSTATE: 0");
				uart2done = false;
				if(smstateint!=0) break;
				reset_mqtt();
				SerialATprintln("AT+SMCONN",9,0xff);
				uart2done = false;
				for(int y=0;y<30;y++){
					osDelay(200);
					if(uart2done)break;
				}
			}
			uart2done = false;
			SerialATprintln("AT+SMCONN",9,0xff);
			for(int i=0;i<20;i++){
				osDelay(200);
				if(uart2done)break;
			}
			uart2done = false;
			sprintf(ATSMPUB,"AT+SMPUB=\"tayra/incoming\",%d,1,1\r\n",len_mydata);
			HAL_UART_Transmit(&huart2,(uint8_t *) ATSMPUB,strlen(ATSMPUB),0xff);
			for(int y=0;y<20;y++){
				osDelay(200);
				if(uart2done)break;
			}
			uart2done = false;
			printf("mydata: %s\r\n",mydata);
			HAL_UART_Transmit(&huart2, (uint8_t *)mydata, len_mydata,0x3ff);
			for(int y=0;y<20;y++){
				osDelay(200);
				if(uart2done)break;
			}
			// change cnmp to 2 (nbiot)
				uart2done = false;
				SerialATprintln("AT+CNMP=2", 9, 0xff);
				osDelay(2000);
				SerialATprintln("AT+CNMP?", 8, 0xff);
				osDelay(2000);

//		sendMQTTMAIN();
//				osDelay(2000);
	}

}

void reset_mqtt(){
	SerialATprintln("AT+CPSI?", 8, 0xff);
	osDelay(2000);

	SerialATprintln("AT+CGDCONT=1,\"IP\",\"nb1internet\",\"0.0.0.0\",0,0,0", 44, 0xff);
	osDelay(2000);

	SerialATprintln("AT+CGDCONT?", 11, 0xff);
	osDelay(2000);

	SerialATprintln("AT+CNACT=0,1", 12, 0xff);
	osDelay(2000);

	SerialATprintln("AT+CNACT?", 9, 0xff);
	osDelay(2000);

	SerialATprintln("AT+SMDISC",9,700);
	  osDelay(1000);

//	  SerialATprintln("AT+SMCONF=\"URL\",broker.mqttdashboard.com,1883",45,0xff);
	  SerialATprintln("AT+SMCONF=\"URL\",broker.hivemq.com,1883",38,0xff);
	  osDelay(1000);

	  SerialATprintln("AT+SMCONF=\"KEEPTIME\",60",23,0xff);
	  osDelay(1000);

	  SerialATprintln("AT+SMCONF=\"CLEANSS\",1",21,0xff);
	  osDelay(1000);

	  uart2done=false;
	  SerialATprintln("AT+SMCONN",9,0xff);
	  		for(int y=0;y<20;y++){
	  				osDelay(200);
	  				if(uart2done)break;
	  			}
}

void NB_rst_rtos(void)
{
  // pull pwrkey low around 1 second then pull high (prevent automatically on / off)
  HAL_GPIO_WritePin(PWR_EN_GPIO_Port, PWR_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PWR_EN_GPIO_Port, PWR_EN_Pin, GPIO_PIN_SET);
  osDelay(1000);
  HAL_GPIO_WritePin(PWR_EN_GPIO_Port, PWR_EN_Pin, GPIO_PIN_RESET);
  osDelay(2*1000);
}

void NB_rst1(void)
{
  // pull pwrkey low around 1 second then pull high (prevent automatically on / off)
  HAL_GPIO_WritePin(RSTNB_GPIO_Port, RSTNB_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RSTNB_GPIO_Port, RSTNB_Pin, GPIO_PIN_SET);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(RSTNB_GPIO_Port, RSTNB_Pin, GPIO_PIN_RESET);
}

void NB_rst1_rtos(void)
{
  // pull pwrkey low around 1 second then pull high (prevent automatically on / off)
  HAL_GPIO_WritePin(RSTNB_GPIO_Port, RSTNB_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RSTNB_GPIO_Port, RSTNB_Pin, GPIO_PIN_SET);
  osDelay(1000);
  HAL_GPIO_WritePin(RSTNB_GPIO_Port, RSTNB_Pin, GPIO_PIN_RESET);
}

void NB_rst(void)
{
  // pull pwrkey low around 1 second then pull high (prevent automatically on / off)
  HAL_GPIO_WritePin(PWR_EN_GPIO_Port, PWR_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PWR_EN_GPIO_Port, PWR_EN_Pin, GPIO_PIN_SET);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(PWR_EN_GPIO_Port, PWR_EN_Pin, GPIO_PIN_RESET);
  HAL_Delay(2*1000);
}

void ATCOMMAND(char *command, char* res)
{
	char respond[120] = {0};
//	char *ptr= strstr(respond,res);
//	while(ptr == NULL){
	memset(respond,0,strlen(respond));
	HAL_UART_Transmit(&huart2,(uint8_t *)command,strlen(command),0xff);
	HAL_UART_Receive(&huart2,(uint8_t *)respond,120,0xff);
//	ptr = strstr(respond,res);
//	}
	printf("%s\r\n",respond);
}

void modem_reset_rtos()
{
	printf("\r\nModem Reboot\r\n");

	SerialATprintln("AT+CREBOOT", 10, 0xff);
	osDelay(4000);
	for(int y=0;y<40;y++){
		osDelay(200);
		if(uart2done)break;
	}

	printf("\r\nNBIOT Setup\r\n");

	char at[]="AT\r\n";
	SerialATprintln(at,2,0xff);
	osDelay(1000);

	char cpin[]="AT+CPIN?";
	SerialATprintln(cpin,8,0xff);
	osDelay(1000);

	char csq[]="AT+CSQ";
	SerialATprintln(csq,6,0xff);
	osDelay(1000);

	char creg[]="AT+CREG=1";
	SerialATprintln(creg,9,0xff);
	osDelay(1000);

	char cgreg[]="AT+CGREG=1";
	SerialATprintln(cgreg,10,0xff);
	osDelay(1000);

	char cgatt1[]="AT+CGATT=1";
	SerialATprintln(cgatt1,9,0xff);
	osDelay(1000);

	char cgatt[]="AT+CGATT?";
	SerialATprintln(cgatt,9,0xff);
	osDelay(1000);

	char cops[]="AT+COPS?";
	SerialATprintln(cops,8,0xff);
	osDelay(1000);

	char cgnapn[]="AT+CGNAPN";
	SerialATprintln(cgnapn,9,0xff);
	osDelay(1000);

//	char cgdcont[]="AT+CGDCONT=1,\"IP\",\"internet\"\r\n";
	char cgdcont[]="AT+CGDCONT=1,\"IP\",\"nb1internet\"";
	SerialATprintln(cgdcont,31,0xff);
	osDelay(1000);

	char cgdcont1[]="AT+CGDCONT?";
	SerialATprintln(cgdcont1,11,0xff);
	osDelay(1000);

	char cnact1[]="AT+CNACT=0,1";
	SerialATprintln(cnact1,12,0xff);
	osDelay(1000);

	printf("NBIOT done\r\n");
}
void NBIOT_setup(void)
{
	printf("\r\nNBIOT setup\r\n");

	char at[]="AT\r\n";
	char at_cmp[] = "OK";
	ATCOMMAND(at,at_cmp);

	char cpin[]="AT+CPIN?\r\n";
	char cpin_cmp[] = "+CPIN:";
	ATCOMMAND(cpin,cpin_cmp);

	char csq[]="AT+CSQ\r\n";
	char csq_cmp[] = "+CSQ:";
	ATCOMMAND(csq,csq_cmp);

	char creg[]="AT+CREG=1\r\n";
	char creg_cmp[] = "+CREG";
	ATCOMMAND(creg,creg_cmp);

	char cgreg[]="AT+CGREG=1\r\n";
	char cgreg_cmp[] = "+CGREG";
	ATCOMMAND(cgreg,cgreg_cmp);

	char cgatt1[]="AT+CGATT=1\r\n";
	char cgatt1_cmp[] = "+CGATT";
	ATCOMMAND(cgatt1,cgatt1_cmp);

	char cgatt[]="AT+CGATT?\r\n";
	char cgatt_cmp[] = "+CGATT:";
	ATCOMMAND(cgatt,cgatt_cmp);

	char cops[]="AT+COPS?\r\n";
	char cops_cmp[] = "+COPS:";
	ATCOMMAND(cops,cops_cmp);

	char cgnapn[]="AT+CGNAPN\r\n";
	char cgnapn_cmp[] = "+CGNAPN:";
	ATCOMMAND(cgnapn,cgnapn_cmp);

//	char cgdcont[]="AT+CGDCONT=1,\"IP\",\"internet\"\r\n";
	char cgdcont[]="AT+CGDCONT=1,\"IP\",\"nb1internet\"\r\n";
	char cgdcont_cmp[] = "AT+CGDCONT";
	ATCOMMAND(cgdcont,cgdcont_cmp);

	char cgdcont1[]="AT+CGDCONT?\r\n";
	char cgdcont1_cmp[] = "+CGDCONT:";
	ATCOMMAND(cgdcont1,cgdcont1_cmp);
	
//	char cnact[]="AT+CNACT=0,0\r\n";
//	char cnact_cmp[] = "+CNACT";
//	ATCOMMAND(cnact,cnact_cmp);
	
	char cnact1[]="AT+CNACT=0,1\r\n";
	char cnact1_cmp[] = "OK";
	ATCOMMAND(cnact1,cnact1_cmp);
	
	printf("NBIOT done\r\n");
}

void NBIOT_setup_IT(void)
{
	printf("\r\nNBIOT setup\r\n");

	SerialATprintln("AT",2,0xff);
	HAL_Delay(500);

	SerialATprintln("AT+CPIN",7,0xff);
	HAL_Delay(500);

	SerialATprintln("AT+CSQ",6,0xff);
	HAL_Delay(500);

	SerialATprintln("AT+CREG=1",9,0xff);
	HAL_Delay(500);

	SerialATprintln("AT+CGREG=1",10,0xff);
	HAL_Delay(500);

	SerialATprintln("AT+CGATT=1",10,0xff);
	HAL_Delay(500);

	SerialATprintln("AT+CGATT?",9,0xff);
	HAL_Delay(500);

	SerialATprintln("AT+COPS?",8,0xff);
	HAL_Delay(500);

	char cgnapn[]="AT+CGNAPN\r\n";
	char cgnapn_cmp[] = "+CGNAPN:";
	SerialATprintln("AT+CGNAPN",9,0xff);
	HAL_Delay(500);

//	char cgdcont[]="AT+CGDCONT=1,\"IP\",\"internet\"\r\n";
	SerialATprintln("AT+CGDCONT=1,\"IP\",\"nb1internet\"",31,0xff);
	HAL_Delay(500);

	SerialATprintln("AT+CGDCONT?",11,0xff);
	HAL_Delay(500);

	SerialATprintln("AT+CGNACT=0,1",13,0xff);
	HAL_Delay(500);

	printf("NBIOT done\r\n");
}

void MQTT_setup(void)
{
	usemqtt=true;
	printf("\r\nMQTT setup\r\n");
	HAL_Delay(1000);
	
	SerialATprintln("AT+SMDISC",9,0xff);
	HAL_Delay(1000);
	printf("%s\r\n",uart2_temp);
	
//	char smconf_url[]="AT+SMCONF=\"URL\",broker.mqttdashboard.com,1883\r\n";
//	char smconf_url[]="AT+SMCONF=\"URL\",broker.hivemq.com,1883\r\n";
//	char smconf_url_cmp[] = "OK";
//	ATCOMMAND(smconf_url,smconf_url_cmp);
	SerialATprintln("AT+SMCONF=\"URL\",broker.hivemq.com,1883",38,0xff);
	HAL_Delay(1000);
	printf("%s\r\n",uart2_temp);

	SerialATprintln("AT+SMCONF=\"KEEPTIME\",60",23,0xff);
	HAL_Delay(1000);
	printf("%s\r\n",uart2_temp);
	
	SerialATprintln("AT+SMCONF=\"CLEANSS\",1",21,0xff);
	HAL_Delay(1000);
	printf("%s\r\n",uart2_temp);

	SerialATprintln("AT+SMCONN",9,0xff);
	HAL_Delay(1000);
	printf("%s\r\n",uart2_temp);

	printf("MQTT done\r\n");
}

void LSM6DSL_readRegister(uint16_t* output, uint8_t i2cReg)
{
	uint8_t tbuf[1];
	uint8_t rbuf[2];
	uint16_t out;
	
	tbuf[0] = i2cReg;
	HAL_I2C_Master_Transmit(&hi2c1,0X6B<<1,tbuf,1,10);
	HAL_I2C_Master_Receive(&hi2c1,0X6B<<1,rbuf,2,10);
	out = ((uint16_t)(rbuf[1] << 8)|(uint16_t)rbuf[0]);

	*output = out;
}

void LSM6DSL_setup(void)
{
	dev_ctx.write_reg = platform_write;
	dev_ctx.read_reg = platform_read;
	dev_ctx.handle = &hi2c1;

	whoamI = 0;
	lsm6dsl_device_id_get(&dev_ctx, &whoamI);\
	if ( whoamI != LSM6DSL_ID )
	{
		while(whoamI != LSM6DSL_ID)
		{
			sprintf((char*)tx_buffer, "Sensor not found\n");
			tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
			lsm6dsl_device_id_get(&dev_ctx, &whoamI);
			osDelay(20);
		}
	}
	sprintf((char*)tx_buffer, "Sensor found\n");
	tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );

	lsm6dsl_reset_set(&dev_ctx, PROPERTY_ENABLE);
	do {
		lsm6dsl_reset_get(&dev_ctx, &rst);
	} while (rst);

	lsm6dsl_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

	lsm6dsl_xl_data_rate_set(&dev_ctx, LSM6DSL_XL_ODR_416Hz);
	lsm6dsl_gy_data_rate_set(&dev_ctx, LSM6DSL_GY_ODR_416Hz);

	lsm6dsl_xl_full_scale_set(&dev_ctx, LSM6DSL_2g);
	lsm6dsl_gy_full_scale_set(&dev_ctx, LSM6DSL_2000dps);

	lsm6dsl_xl_filter_analog_set(&dev_ctx, LSM6DSL_XL_ANA_BW_400Hz);

	lsm6dsl_xl_lp2_bandwidth_set(&dev_ctx, LSM6DSL_XL_LOW_NOISE_LP_ODR_DIV_100);

	lsm6dsl_gy_band_pass_set(&dev_ctx, LSM6DSL_HP_260mHz_LP1_STRONG);

	lsm6dsl_wrist_tilt_sens_set(&dev_ctx, PROPERTY_ENABLE);
	lsm6dsl_tilt_sens_set(&dev_ctx, PROPERTY_ENABLE);
	lsm6dsl_motion_sens_set(&dev_ctx, PROPERTY_ENABLE);
	lsm6dsl_all_on_int1_set(&dev_ctx, PROPERTY_ENABLE);
	lsm6dsl_tap_detection_on_x_set(&dev_ctx, PROPERTY_ENABLE);
	lsm6dsl_tap_detection_on_y_set(&dev_ctx, PROPERTY_ENABLE);
	lsm6dsl_tap_detection_on_z_set(&dev_ctx, PROPERTY_ENABLE);
	lsm6dsl_tap_threshold_x_set(&dev_ctx, 0x08);
	lsm6dsl_tap_shock_set(&dev_ctx, 0x03);
	lsm6dsl_tap_quiet_set(&dev_ctx, 0x03);
	lsm6dsl_tap_dur_set(&dev_ctx, 0x08);
	lsm6dsl_tap_mode_set(&dev_ctx, LSM6DSL_BOTH_SINGLE_DOUBLE);
	lsm6dsl_ff_dur_set(&dev_ctx, 0x06);
	lsm6dsl_wkup_dur_set(&dev_ctx, 0x00);
	lsm6dsl_timestamp_res_set(&dev_ctx, LSM6DSL_LSB_6ms4);
	lsm6dsl_act_sleep_dur_set(&dev_ctx, 0x00);
	lsm6dsl_ff_threshold_set(&dev_ctx, LSM6DSL_FF_TSH_312mg);

	lsm6dsl_int1_route_t prop;
	prop.int1_boot = PROPERTY_DISABLE;
	prop.int1_fth = PROPERTY_DISABLE;
	prop.int1_full_flag = PROPERTY_DISABLE;
	prop.int1_timer = PROPERTY_DISABLE;
	prop.int1_ff = PROPERTY_ENABLE;
	prop.int1_double_tap = PROPERTY_ENABLE;
	prop.int1_sign_mot = PROPERTY_ENABLE;
	prop.int1_step_detector = PROPERTY_ENABLE;
	prop.int1_tilt = PROPERTY_ENABLE;
	prop.int1_wu = PROPERTY_DISABLE;
	prop.int1_6d = PROPERTY_DISABLE;

	lsm6dsl_pin_int1_route_set(&dev_ctx, prop);

	sprintf((char*)tx_buffer, "Sensor configured\n");
	tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
}

// Meter Function
void writeToSD1(char * data, char * fn){
	fresult = f_mount(&fs, "/", 1);
	  if(fresult != FR_OK)
	  {
		  sprintf((char*)tx_buffer, "Failed to mount SD Card\n");
		  tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
	  }
	  else
	  {
		  sprintf((char*)tx_buffer, "SD Card Successfully Mounted\n");
		  tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
		  fresult = f_open(&fil, fn, FA_OPEN_APPEND | FA_READ | FA_WRITE);
		  if(fresult != FR_OK)
		  {
			  sprintf((char*)tx_buffer, "Failed to Open File\n");
			  tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
		  }
		  else
		  {
			  sprintf((char*)tx_buffer, "File Successfully Opened\n");
			  tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
			   f_close(&fil);
		  }
		  fresult = f_open(&fil, fn, FA_OPEN_APPEND | FA_READ | FA_WRITE);
		  if(fresult != FR_OK)
		  	  {
		  		  sprintf((char*)tx_buffer, "Failed to Open File\n");
		  		  tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
		  	  }
		  	  else
		  	  {
		  		  sprintf((char*)tx_buffer, "File Successfully Opened\n");
		  		  tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
		  		  f_puts(data, &fil);
				  printf("\nFile %s is created and the data is written\r\n",fn);
		  		  f_close(&fil);
		  	  }
	  }
}

void read_instant_mk6n(){
	dataallclearMK6N();
	loginToMeterRTOS();
	int cmpint = strcmp(snMeter, "00000000");
	if(cmpint == 0) read_sn_edmi();
	SerialMonprintln("Read Instant MK6N");
	dataallclearMK6N();
	uart2done=false;
	uart3disc = true;
	uart3done=false;
	memset(mydata,0,1100);
	memset(uart3_temp,0,1100);
	memset(buffer3,0,1100);
	for(int x=0;x<3;x++){
		txInstantMK6N(x);
		for(int y=0;y<20;y++){
			osDelay(200);
			if(uart3done){
				uart3disc = false;
				break;
			}
		}
		memset(mydata,0,1100);
		if (!uart3disc) parsingDataRawInstantMK6N(buffer3,datauart3size(buffer3),x);
		  else {
				memset((char *)mydata,0,1100);
				sprintf(temp_data,"{\"mid\":%s,\"pid\":\"Event\",\"mtr\":\"MK6N\",\"csq\":\"",snMeter);
				strcat(temp_data, (char*)csq);
				strcat(temp_data,"\"");
			  	sprintf(temp_data2,",\"mtrcon\":\"disconnect\"");
				strcat(temp_data, temp_data2);
			  	sprintf(temp_data2,",\"bat\":\"%.2f\"",vbat_percentage);
				strcat(temp_data, temp_data2);
				sprintf(temp_data2,",\"pwr\":\"%d\"}",pwr_vmon);
				strcat(temp_data, temp_data2);
				strcpy(mydata,temp_data);
				cnt_instant++;
			}
		memset(buffer3,0,1000);
	}
	decodeJsonInstantMK6N(mydata);
	printf(mydata);
	csqint = 1;
	cgattint = 1;
	if(csqint || cgattint) {
//		printf("online True\r\n");
		onlineMode = true;
	}
	else {
		onlineMode = false;
		net_err_cnt++;
	}
	if(onlineMode) {
		if(usemqtt){
			len_mydata=bufsizechararray(mydata);
			sendMQTTMAIN();
			osDelay(2000);
		}
		else sendToServer(mydata,bufsizechararray(mydata));
	}
	else {
		if(net_err_cnt >= 3) {
			printf("Restart Modem \r\n");
			NB_rst1_rtos();
			net_err_cnt=0;
			osDelay(5000);
			setup_modem();
		}
	}
}

void read_eob_mk6n(){
	loginToMeterRTOS();
	dataallclearMK6N();
	memset(mydata,0,1100);
	osDelay(1000);
	int cmpint = strcmp(snMeter, "00000000");
	if(cmpint == 0) read_sn_edmi();
	SerialMonprintln("Read EoB init MK6N");
	uart3disc = true;
	uart3done=false;
	txEoBInit1();
	for(int y=0;y<20;y++){
		osDelay(200);
		if(uart3done){
			uart3disc = false;
			break;
		}
	}
	if (!uart3disc) rxEoBInit1(buffer3,datauart3size(buffer3));
	  else {
			memset((char *)mydata,0,1100);
			sprintf(temp_data,"{\"mid\":%s,\"pid\":\"Event\",\"mtr\":\"MK6N\",\"csq\":\"",snMeter);
			strcat(temp_data, (char*)csq);
			strcat(temp_data,"\"");
		  	sprintf(temp_data2,",\"mtrcon\":\"disconnect\"");
			strcat(temp_data, temp_data2);
		  	sprintf(temp_data2,",\"bat\":\"%.2f\"",vbat_percentage);
			strcat(temp_data, temp_data2);
			sprintf(temp_data2,",\"pwr\":\"%d\"}",pwr_vmon);
			strcat(temp_data, temp_data2);
			strcpy(mydata,temp_data);
			cnt_eob++;
		}
	memset(buffer3,0,1000);
//	HAL_Delay(1000);
	SerialMonprintln("Read EoB MK6N");
	memset(mydata,0,1100);
	for(int i=0;i<9;i++){
		uart3disc = true;
		uart3done=false;
		readEoB2(i);
		for(int y=0;y<20;y++){
			osDelay(200);
			if(uart3done){
				uart3disc = false;
				break;
			}
		}
		if (!uart3disc) readEoB3(buffer3,datauart3size(buffer3),i);
		  else {
				memset((char *)mydata,0,1100);
				sprintf(temp_data,"{\"mid\":%s,\"pid\":\"Event\",\"mtr\":\"MK6N\",\"csq\":\"",snMeter);
				strcat(temp_data, (char*)csq);
				strcat(temp_data,"\"");
			  	sprintf(temp_data2,",\"mtrcon\":\"disconnect\"");
				strcat(temp_data, temp_data2);
			  	sprintf(temp_data2,",\"bat\":\"%.2f\"",vbat_percentage);
				strcat(temp_data, temp_data2);
				sprintf(temp_data2,",\"pwr\":\"%d\"}",pwr_vmon);
				strcat(temp_data, temp_data2);
				strcpy(mydata,temp_data);
				cnt_eob++;
			}
		memset(buffer3,0,1000);
	}
	if(!uart3disc) decodeEoB3(mydata);

	printf(mydata);
	csqint = 1;
	cgattint = 1;
	if(csqint || cgattint) {
//		printf("online True\r\n");
		onlineMode = true;
	}
	else {
		onlineMode = false;
		net_err_cnt++;
	}
	if(onlineMode) {
		if(usemqtt){
			len_mydata=bufsizechararray(mydata);
			sendMQTTMAIN();
			osDelay(2000);
		}
		else sendToServer(mydata,bufsizechararray(mydata));
	}
	else {
		if(net_err_cnt >= 3) {
			printf("Restart Modem \r\n");
			NB_rst1_rtos();
			net_err_cnt=0;
			osDelay(5000);
			setup_modem();
		}
	}
}

void read_lp_mk6n(){
	  dataallclearMK6N();
	  loginToMeterRTOS();
	  dataallclearMK6N();
	  osDelay(1000);
	  int cmpint = strcmp(snMeter, "00000000");
	  if(cmpint == 0) read_sn_edmi();
	  SerialMonprintln("Read LP MK6N");
	  memset(mydata,0,1100);
	  uart3disc = true;
	  uart3done=false;
	  txlpInit1MK6N();
	  for(int y=0;y<20;y++){
			osDelay(200);
			if(uart3done){
				uart3disc = false;
				break;
			}
	  }
	  if(!uart3disc){
		  rxlpInit1MK6N(buffer3,datauart3size(buffer3));
	  }
	  else {
			memset((char *)mydata,0,1100);
			sprintf(temp_data,"{\"mid\":%s,\"pid\":\"Event\",\"mtr\":\"MK6N\",\"csq\":\"",snMeter);
			strcat(temp_data, (char*)csq);
			strcat(temp_data,"\"");
		  	sprintf(temp_data2,",\"mtrcon\":\"disconnect\"");
			strcat(temp_data, temp_data2);
		  	sprintf(temp_data2,",\"bat\":\"%.2f\"",vbat_percentage);
			strcat(temp_data, temp_data2);
			sprintf(temp_data2,",\"pwr\":\"%d\"}",pwr_vmon);
			strcat(temp_data, temp_data2);
			strcpy(mydata,temp_data);
			cnt_lp++;
		}

		memset(buffer3,0,1100);
		uart3disc = true;
		uart3done=false;
	  txlpMK6NInit2();
	  for(int y=0;y<20;y++){
		osDelay(200);
		if(uart3done){
			uart3disc = false;
			break;
		}
	  }
	  if(!uart3disc){
		  rxlpMK6NInit2(buffer3,datauart3size(buffer3));
		  memset(buffer3,0,1100);
		  jmlChMK6N = getJumlahChannelMK6N();
	  }
	  else {
			memset((char *)mydata,0,1100);
			sprintf(temp_data,"{\"mid\":%s,\"pid\":\"Event\",\"mtr\":\"MK6N\",\"csq\":\"",snMeter);
			strcat(temp_data, (char*)csq);
			strcat(temp_data,"\"");
		  	sprintf(temp_data2,",\"mtrcon\":\"disconnect\"");
			strcat(temp_data, temp_data2);
		  	sprintf(temp_data2,",\"bat\":\"%.2f\"",vbat_percentage);
			strcat(temp_data, temp_data2);
			sprintf(temp_data2,",\"pwr\":\"%d\"}",pwr_vmon);
			strcat(temp_data, temp_data2);
			strcpy(mydata,temp_data);
			cnt_lp++;
		}
	  memset(buffer3,0,1100);
	  jmlChMK6N = getJumlahChannelMK6N();
//	  HAL_Delay(500);
//	  readlpMK6NInit2a();
	  for(int i=0;i<jmlChMK6N;i++){
			uart3disc = true;
			uart3done=false;
		txlpMK6NInit2a(i);
		for(int y=0;y<20;y++){
			osDelay(200);
			if(uart3done){
				uart3disc = false;
				break;
			}
		}
		if(!uart3disc){
		rxlpMK6NInit2a(buffer3,datauart3size(buffer3),i);
		}
		else {
			memset((char *)mydata,0,1100);
			sprintf(temp_data,"{\"mid\":%s,\"pid\":\"Event\",\"mtr\":\"MK6N\",\"csq\":\"",snMeter);
			strcat(temp_data, (char*)csq);
			strcat(temp_data,"\"");
		  	sprintf(temp_data2,",\"mtrcon\":\"disconnect\"");
			strcat(temp_data, temp_data2);
		  	sprintf(temp_data2,",\"bat\":\"%.2f\"",vbat_percentage);
			strcat(temp_data, temp_data2);
			sprintf(temp_data2,",\"pwr\":\"%d\"}",pwr_vmon);
			strcat(temp_data, temp_data2);
			strcpy(mydata,temp_data);
			cnt_lp++;
		}
		memset(buffer3,0,1100);
	  }
	  //	  HAL_Delay(500);
		memset(buffer3,0,1100);
		uart3disc = true;
		uart3done=false;
		if(lp_rec !=0){
			txlpMK6NRec(lp_rec);
			lp_rec = 0;
		}
		else 	txlpMK6N(1+cnt_lp);
//	  txlpMK6N(1);
		for(int y=0;y<20;y++){
			osDelay(200);
			if(uart3done){
				uart3disc = false;
				break;
			}
		}
		memset(mydata,0,1100);
		if(!uart3disc){
	  rxlpMK6N(mydata,buffer3,datauart3size(buffer3));
	  if(cnt_lp>0)cnt_lp--;
		}
		else {
			memset((char *)mydata,0,1100);
			sprintf(temp_data,"{\"mid\":%s,\"pid\":\"Event\",\"mtr\":\"MK6N\",\"csq\":\"",snMeter);
			strcat(temp_data, (char*)csq);
			strcat(temp_data,"\"");
			sprintf(temp_data2,",\"mtrcon\":\"disconnect\"");
			strcat(temp_data, temp_data2);
			sprintf(temp_data2,",\"bat\":\"%.2f\"",vbat_percentage);
			strcat(temp_data, temp_data2);
			sprintf(temp_data2,",\"pwr\":\"%d\"}",pwr_vmon);
			strcat(temp_data, temp_data2);
			strcpy(mydata,temp_data);
			cnt_lp++;
		}

	  printf(mydata);
	  csqint = 1;
	  cgattint = 1;
	  if(csqint || cgattint) {
//			printf("online True\r\n");
		onlineMode = true;
	  }
	  else {
		onlineMode = false;
		net_err_cnt++;
	  }
	  if(onlineMode) {
		if(usemqtt){
			len_mydata=bufsizechararray(mydata);
			sendMQTTMAIN();
			osDelay(2000);
		}
			else sendToServer(mydata,bufsizechararray(mydata));
		}
		else {
			if(net_err_cnt >= 3) {
				printf("Restart Modem \r\n");
				NB_rst1_rtos();
				net_err_cnt=0;
				osDelay(5000);
				setup_modem();
			}
		}
}

void read_instant_mk10e(){
	memset((char *)mydata,0,1100);
	loginToMeterRTOS();
	osDelay(1000);
	int cmpint = strcmp(snMeter, "00000000");
	if(cmpint == 0) read_sn_edmi();
	uart3done = false;
	SerialMonprintln("Read Instant MK10E");
	uart2done=false;
	memset((char *)uart3_temp,0,1100);
	memset((char *)buffer3,0,1100);
	uart3disc = true;
	txInstantMK10E();
	for(int y=0;y<20;y++){
		osDelay(200);
		if(uart3done){
			uart3disc = false;
			break;
		}
	}
	memset(mydata,0,1100);
	if(!uart3disc){
		rxInstantMK10E((char *)mydata,buffer3,datauart3size(buffer3));
		memset((char *)buffer3,0,1100);
	//	printf(mydata);
	}
	else {
		memset((char *)mydata,0,1100);
		sprintf(temp_data,"{\"mid\":%s,\"pid\":\"Event\",\"mtr\":\"MK10E\",\"csq\":\"",snMeter);
		strcat(temp_data, (char*)csq);
		strcat(temp_data,"\"");
	  	sprintf(temp_data2,",\"mtrcon\":\"disconnect\"");
		strcat(temp_data, temp_data2);
	  	sprintf(temp_data2,",\"bat\":\"%.2f\"",vbat_percentage);
		strcat(temp_data, temp_data2);
		sprintf(temp_data2,",\"pwr\":\"%d\"}",pwr_vmon);
		strcat(temp_data, temp_data2);
		strcpy(mydata,temp_data);
	}
	csqint = 1;
	cgattint = 1;
	if(csqint || cgattint) {
//		printf("online True\r\n");
		onlineMode = true;
	}
	else {
		onlineMode = false;
		net_err_cnt++;
	}
	if(onlineMode) {
//		usemqtt = true;
		if(usemqtt){
			len_mydata=bufsizechararray(mydata);
			sendMQTTMAIN();
			osDelay(2000);
		}
		else sendToServer(mydata,bufsizechararray(mydata));
//		sendMQTT(mydata,bufsizechararray(mydata));
	}
	else{
		if(net_err_cnt >= 3) {
			printf("Restart Modem \r\n");
			NB_rst1_rtos();
			net_err_cnt=0;
			osDelay(5000);
			setup_modem();
		}
	}

}

void read_eob_mk10e(){
	loginToMeterRTOS();
	osDelay(1000);
	int cmpint = strcmp(snMeter, "00000000");
	if(cmpint == 0) read_sn_edmi();
	SerialMonprintln("Read EoB Init LP MK10E");
	uart2done=false;
	memset((char *)mydata,0,1100);
	memset((char *)uart3_temp,0,1100);
	memset((char *)buffer3,0,1100);
	uart3done = false;
	uart3disc = true;
	txEobLpInitMK10E();
	for(int y=0;y<20;y++){
		osDelay(200);
		if(uart3done){
			uart3disc = false;
			break;
		}
	}
	if(!uart3disc){
	rxEobLpInitMK10E((char *)buffer3,datauart3size(buffer3));
	memset((char *)buffer3,0,1100);
	}
	else {
		memset((char *)mydata,0,1100);
		sprintf(temp_data,"{\"mid\":%s,\"pid\":\"Event\",\"mtr\":\"MK10E\",\"csq\":\"",snMeter);
		strcat(temp_data, (char*)csq);
		strcat(temp_data,"\"");
	  	sprintf(temp_data2,",\"mtrcon\":\"disconnect\"");
		strcat(temp_data, temp_data2);
	  	sprintf(temp_data2,",\"bat\":\"%.2f\"",vbat_percentage);
		strcat(temp_data, temp_data2);
		sprintf(temp_data2,",\"pwr\":\"%d\"}",pwr_vmon);
		strcat(temp_data, temp_data2);
		strcpy(mydata,temp_data);
	}

	loginToMeterRTOS();
	SerialMonprintln("Read EoB MK10E");
	memset((char *)uart3_temp,0,1100);
	memset((char *)buffer3,0,1100);
	memset(mydata,0,1100);
	uart3done = false;
	uart3disc = true;
	txEoBCurrentMK10E();
	for(int y=0;y<20;y++){
		osDelay(200);
		if(uart3done){
			uart3disc = false;
			break;
		}
	}
	memset(mydata,0,1100);
	if(!uart3disc){
	rxEoBCurrentMK10E((char *)mydata,buffer3,datauart3size(buffer3));
	memset((char *)buffer3,0,1100);
	}
	else {
		memset((char *)mydata,0,1100);
		sprintf(temp_data,"{\"mid\":%s,\"pid\":\"Event\",\"mtr\":\"MK10E\",\"csq\":\"",snMeter);
		strcat(temp_data, (char*)csq);
		strcat(temp_data,"\"");
	  	sprintf(temp_data2,",\"mtrcon\":\"disconnect\"");
		strcat(temp_data, temp_data2);
	  	sprintf(temp_data2,",\"bat\":\"%.2f\"",vbat_percentage);
		strcat(temp_data, temp_data2);
		sprintf(temp_data2,",\"pwr\":\"%d\"}",pwr_vmon);
		strcat(temp_data, temp_data2);
		strcpy(mydata,temp_data);
	}

	csqint = 1;
	cgattint = 1;
//	cgattint = check_cgatt_rtos();
//	csqint = check_csq_rtos();
	if(csqint || cgattint) {
//		printf("online True\r\n");
		onlineMode = true;
	}
	else {
		onlineMode = false;
		net_err_cnt++;
	}
	if(onlineMode) {
//		sendToServer(mydata,bufsizechararray(mydata));
//		usemqtt = true;
		if(usemqtt){
			len_mydata=bufsizechararray(mydata);
			sendMQTTMAIN();
			osDelay(2000);
		}
		else sendToServer((char *)mydata,bufsizechararray(mydata));
	}
	else{
		if(net_err_cnt >= 3) {
			printf("Restart Modem \r\n");
			NB_rst1_rtos();
			net_err_cnt=0;
			osDelay(1000);
			setup_modem();
		}
	}
}

void read_lp_mk10e(){
	loginToMeterRTOS();
	osDelay(1000);
	int cmpint = strcmp(snMeter, "00000000");
	if(cmpint == 0) read_sn_edmi();
	SerialMonprintln("Read EoB Init LP MK10E");
	uart2done=false;
	memset((char *)mydata,0,1100);
	memset((char *)uart3_temp,0,1100);
	memset((char *)buffer3,0,1100);
	uart3done = false;
	uart3disc = true;
	txEobLpInitMK10E();
	for(int y=0;y<20;y++){
		osDelay(200);
		if(uart3done){
			uart3disc = false;
			break;
		}
	}
	if(!uart3disc){
	rxEobLpInitMK10E((char *)buffer3,datauart3size(buffer3));
	memset((char *)buffer3,0,1100);
	}
	else {
		memset((char *)mydata,0,1100);
		sprintf(temp_data,"{\"mid\":%s,\"pid\":\"Event\",\"mtr\":\"MK10E\",\"csq\":\"",snMeter);
		strcat(temp_data, (char*)csq);
		strcat(temp_data,"\"");
	  	sprintf(temp_data2,",\"mtrcon\":\"disconnect\"");
		strcat(temp_data, temp_data2);
	  	sprintf(temp_data2,",\"bat\":\"%.2f\"",vbat_percentage);
		strcat(temp_data, temp_data2);
		sprintf(temp_data2,",\"pwr\":\"%d\"}",pwr_vmon);
		strcat(temp_data, temp_data2);
		strcpy(mydata,temp_data);
	}

	loginToMeterRTOS();
	SerialMonprintln("Read LP MK10E");
	uart2done=false;
	memset((char *)uart3_temp,0,1100);
	memset((char *)buffer3,0,1100);
	memset((char *)mydata,0,1100);
	uart3done = false;
	uart3disc = true;
	if(lp_rec !=0){
		txLPMK10ERec(lp_rec);
		lp_rec = 0;
	}
	else 	txLPMK10E(1+cnt_lp);
	for(int y=0;y<20;y++){
		osDelay(200);
		if(uart3done){
			uart3disc = false;
			break;
		}
	}
	memset(mydata,0,1100);
	if(!uart3disc){
	rxLPMK10E((char *)mydata,buffer3,datauart3size(buffer3));
	memset((char *)buffer3,0,1100);
	if(cnt_lp>0)cnt_lp--;
	}
	else {
		memset((char *)mydata,0,1100);
		sprintf(temp_data,"{\"mid\":%s,\"pid\":\"Event\",\"mtr\":\"MK10E\",\"csq\":\"",snMeter);
		strcat(temp_data, (char*)csq);
		strcat(temp_data,"\"");
	  	sprintf(temp_data2,",\"mtrcon\":\"disconnect\"");
		strcat(temp_data, temp_data2);
	  	sprintf(temp_data2,",\"bat\":\"%.2f\"",vbat_percentage);
		strcat(temp_data, temp_data2);
		sprintf(temp_data2,",\"pwr\":\"%d\"}",pwr_vmon);
		strcat(temp_data, temp_data2);
		strcpy(mydata,temp_data);
		cnt_lp++;
	}
	csqint = 1;
	cgattint = 1;
	if(csqint || cgattint) {
		onlineMode = true;
	}
	else {
		onlineMode = false;
		net_err_cnt++;
	}
	if(onlineMode) {
		if(usemqtt){
			len_mydata=bufsizechararray(mydata);
			sendMQTTMAIN();
			osDelay(2000);
		}
		else sendToServer(mydata,bufsizechararray(mydata));
	}
	else{
		if(net_err_cnt >= 3) {
			printf("Restart Modem \r\n");
			NB_rst1_rtos();
			net_err_cnt=0;
			osDelay(1000);
			setup_modem();
		}
	}
}

void read_sn_edmi(){
	memset((char *)mydata,0,1100);
	loginToMeterRTOS();
	uart3done = false;
	uart2done=false;
	memset((char *)uart3_temp,0,1100);
	memset((char *)buffer3,0,1100);
	uart3disc = true;
	txSNMK10E_rtos();
	for(int y=0;y<20;y++){
		osDelay(200);
		if(uart3done){
			uart3disc = false;
			break;
		}
	}
	if(!uart3disc){
	snMeter = rxSNMK10E_rtos(buffer3,datauart3size(buffer3));
	if(typemeter == 6) setSNMK6N(snMeter,strlen(snMeter));
	memset((char *)buffer3,0,1100);
	}
	else {
		snMeter = "00000000";
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	ATSMPUB=malloc(26);
	smstatestr=malloc(11);
	temp_data=malloc(200);
	temp_data2=malloc(50);
	networkstr=malloc(20);
	uart2busy = false;
	eob_date = 1;
	eob_hour = 10;
	eob_minute = 0;
	instant_hour = 19;
	instant_minute = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(HARD_RST_GPIO_Port, HARD_RST_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
  HAL_Delay(1000);
  sprintf((char*)tx_buffer, "\nV1.10 31-12-2020\n");
  tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );

  onlineMode = false;
  onlineMode = true;
  //NBIOT
//  NB_rst1();
  NB_rst();
  HAL_Delay(5000);
//  NBIOT_setup();
//  HAL_Delay(1000);
//  setup_modem_nortos();
  HAL_Delay(1000);
  usemqtt=false;

    SerialMonprintln("Login To Meter");
//    loginToMeter();
//    HAL_Delay(500);
    SerialMonprintln("Read SN Meter");
//    snMeter = readSNMK6N();
//    snMeter = readSNMK10E();
//    read_sn_edmi();
//    printf("%s\r\n",snMeter);

//  if(onlineMode) setup_modem();
	
  //Main power fail detection
  pwr_vmon = HAL_GPIO_ReadPin(PWR_VMON_GPIO_Port,PWR_VMON_Pin);
  sprintf((char*)tx_buffer, "pwr_vmon: %d\n", pwr_vmon);
  tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
  pwr_state=1;
//  if(pwr_vmon==0) pwr_state=0;
//  else pwr_state=1;

//  uint8_t pwr_vmon = HAL_GPIO_ReadPin(PWR_VMON_GPIO_Port,PWR_VMON_Pin);
//  sprintf((char*)tx_buffer, "pwr_vmon: %d\n", pwr_vmon);
//  tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );

  //Mount SD Card
  //SD CARD
  fresult = f_mount(&fs, "/", 1);
  if(fresult != FR_OK)
  {
	  sprintf((char*)tx_buffer, "Failed to mount SD Card\n");
	  tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
  }
  else
  {
	  sprintf((char*)tx_buffer, "SD Card Successfully Mounted\n");
	  tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
	  fresult = f_open(&fil, "sdfile1.txt", FA_OPEN_APPEND | FA_READ | FA_WRITE);
	  if(fresult != FR_OK)
	  {
		  sprintf((char*)tx_buffer, "Failed to Open File\n");
		  tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
	  }
	  else
	  {
		  sprintf((char*)tx_buffer, "File Successfully Opened\n");
		  tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
		   f_close(&fil);
	  }
	  fresult = f_open(&fil, "config.txt", FA_OPEN_APPEND | FA_READ | FA_WRITE);
	  if(fresult != FR_OK)
	  	  {
	  		  sprintf((char*)tx_buffer, "Failed to Open File\n");
	  		  tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
	  	  }
	  	  else
	  	  {
	  		  sprintf((char*)tx_buffer, "File Successfully Opened\n");
	  		  tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
	  		  f_puts("13\n", &fil);
			  f_puts("10\n", &fil);
			  f_puts("30\n", &fil);
			  printf("\nConfig is created and the data is written\r\n");
	  		  f_close(&fil);
	  	  }
	  // Open file to read
	  fresult = f_open(&fil, "configmtr.txt", FA_READ);
	  if(fresult != FR_OK)
	  	  {
	  		  sprintf((char*)tx_buffer, "Failed to Open File\n");
	  		  tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
	  	  }
	  	  else
	  	  {
	  		  bufclear();
	  		  // Read data from file
	  		  f_gets(buffer,sizeof(buffer),&fil);
	  		  typemeter = 6;
	  		  printf(buffer);
	  		  typemeter = charToInt(buffer[11]);
	  		  if(typemeter!=6){
	  			  if(typemeter==1){
	  				  typemeter=10;
	  			  }
	  		  }
//	  		  typemeter = 6;
	  		  printf("\r\nType Meter : %d\r\n",typemeter);
	  		  // Close file
	  		  f_close(&fil);
	  		  bufclear();
	  	  }
  }

  //i2cdevice , slave address:0x6B<<1
  if(HAL_I2C_IsDeviceReady(&hi2c1,0x6B<<1,2,10) ==HAL_OK){
	  sprintf((char*)tx_buffer, "LSM6DSL found\n");
	  tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
  }
  else
  {
	  sprintf((char*)tx_buffer, "LSM6DSL not found\n");
	  tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
  }

  bufclear();
  HAL_Delay(1000);
	
  //check i2c communication , return:0x6A
  uint16_t output;
  LSM6DSL_readRegister(&output,0x0F);
  LSM6DSL_setup();
  printf("read:%X\r\n",(uint8_t)output);
	
//
	
  // Check free space
  f_getfree("", &fre_clust, &pfs);
  total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
  bufclear();
  free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);

  // Open file to write/ create a file if it doesn't exist 
  fresult = f_open(&fil, "sdfile1.txt", FA_OPEN_APPEND | FA_READ | FA_WRITE);

  // Writing text 
  f_puts("Message: Hello World 123\n", &fil);
  printf("\nFile1 is created and the data is written\r\n");

  // Close file 
  fresult = f_close(&fil);

  // Open file to read
  fresult = f_open(&fil, "sdfile1.txt", FA_READ);

  //Read String from the file
  f_gets(buffer,sizeof(buffer),&fil);
  printf("\nSD Card read:%s\r\n",buffer);

  //Close file
  f_close(&fil);
  bufclear();
  f_mount(0, "", 0);

  //NBIOT
//  printf("\nOK\n");
//  HAL_Delay(1000);
//  NBIOT_setup();
//  HAL_Delay(1000);
//  setup_modem_nortos();
//  HAL_Delay(1000);
  usemqtt=true;
//  MQTT_setup();
  //enable uart interrupt
  HAL_UART_Receive_IT(&huart1,(uint8_t *)uart1_buf,1);
  HAL_UART_Receive_IT(&huart2,(uint8_t *)uart2_buf,1);
  HAL_UART_Receive_IT(&huart3,(uint8_t *)uart3_buf,1);
  NBIOT_setup_IT();
  MQTT_setup();

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of myMutex01 */
  osMutexStaticDef(myMutex01, &myMutex01ControlBlock);
  myMutex01Handle = osMutexCreate(osMutex(myMutex01));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of mainProgram */
  osThreadStaticDef(mainProgram, main_program, osPriorityNormal, 0, 512, mainProgramBuffer, &mainProgramControlBlock);
  mainProgramHandle = osThreadCreate(osThread(mainProgram), NULL);

  /* definition and creation of readGyro */
  osThreadStaticDef(readGyro, read_gyro, osPriorityLow, 0, 128, readGyroBuffer, &readGyroControlBlock);
  readGyroHandle = osThreadCreate(osThread(readGyro), NULL);

  /* definition and creation of sendDataToServe */
  osThreadStaticDef(sendDataToServe, send_data_to_server, osPriorityNormal, 0, 512, sendDataToServeBuffer, &sendDataToServeControlBlock);
  sendDataToServeHandle = osThreadCreate(osThread(sendDataToServe), NULL);

  /* definition and creation of readEoB */
  osThreadStaticDef(readEoB, read_EoB, osPriorityNormal, 0, 700, readEoBBuffer, &readEoBControlBlock);
  readEoBHandle = osThreadCreate(osThread(readEoB), NULL);

  /* definition and creation of readLP */
  osThreadStaticDef(readLP, read_LP, osPriorityNormal, 0, 700, readLPBuffer, &readLPControlBlock);
  readLPHandle = osThreadCreate(osThread(readLP), NULL);

  /* definition and creation of readInstant */
  osThreadStaticDef(readInstant, read_instant, osPriorityNormal, 0, 700, readInstantBuffer, &readInstantControlBlock);
  readInstantHandle = osThreadCreate(osThread(readInstant), NULL);

  /* definition and creation of sendHardBit */
  osThreadStaticDef(sendHardBit, send_hard_bit, osPriorityNormal, 0, 512, sendHardBitBuffer, &sendHardBitControlBlock);
  sendHardBitHandle = osThreadCreate(osThread(sendHardBit), NULL);

  /* definition and creation of checkVbat */
  osThreadStaticDef(checkVbat, check_vbat, osPriorityNormal, 0, 128, checkVbatBuffer, &checkVbatControlBlock);
  checkVbatHandle = osThreadCreate(osThread(checkVbat), NULL);

  /* definition and creation of checkVMon */
  osThreadStaticDef(checkVMon, check_vmon, osPriorityNormal, 0, 128, checkVMonBuffer, &checkVMonControlBlock);
  checkVMonHandle = osThreadCreate(osThread(checkVMon), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  read_eob_mk6n();
//	  read_lp_mk6n();
//	  HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
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
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
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
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x19;
  sTime.Minutes = 0x15;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_FRIDAY;
  sDate.Month = RTC_MONTH_OCTOBER;
  sDate.Date = 0x23;
  sDate.Year = 0x20;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2); //backup register
  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PWR_EN_Pin|RSTNB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HARD_RST_GPIO_Port, HARD_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PWR_VMON_Pin */
  GPIO_InitStruct.Pin = PWR_VMON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PWR_VMON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PWR_EN_Pin RSTNB_Pin SPI2_CS_Pin */
  GPIO_InitStruct.Pin = PWR_EN_Pin|RSTNB_Pin|SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : HARD_RST_Pin PA15 */
  GPIO_InitStruct.Pin = HARD_RST_Pin|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LSM6DSL_INT1_Pin */
  GPIO_InitStruct.Pin = LSM6DSL_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LSM6DSL_INT1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
	if(huart == &huart1)
		{
			uart1_temp[i]=uart1_buf[0]; //store byte in temp buffer
			if(uart1_temp[i] == 0x0A){
				HAL_UART_Transmit(&huart2, (uint8_t *)uart1_temp, strlen(uart1_temp),0xff);
				memset(uart1_temp,0,strlen(uart1_temp));
				i=0;
			}
			else if(i>1099){
				printf("%s\r\n",uart1_temp);
				memset(uart1_temp,0,1100);
				i=0;
			}
			else
				i++;
			HAL_UART_Receive_IT(&huart1,(uint8_t *)uart1_buf,1);
		}
	if(huart == &huart2)
		{
			uart2_temp[j]=uart2_buf[0]; //store byte in temp2 buffer
			if(j>6){
//				if(uart2_temp[j-1] == '\r'  && uart2_temp[j] == '\n')
				if(uart2_temp[j] == '\n' && (uart2_temp[j-3] == 'O' && uart2_temp[j-2] == 'K') || (uart2_temp[j-3] == 'O' && uart2_temp[j-2] == 'R'))
				{
						HAL_UART_Transmit(&huart1, (uint8_t *)uart2_temp, strlen(uart2_temp),0xff);
						printf("\r\n");
						uart2done = true;
						uart2busy = false;
						int cmpint=strcmp(uart2_at,"AT+CPSI?");
						if(cmpint==0){
							uart2_temp[j] = 0;
							uart2_temp[j-1] = 0;
							uart2_temp[j-2] = 0;
							uart2_temp[j-3] = 0;
							uart2_temp[j-4] = 0;
							uart2_temp[j-5] = 0;
							j=j-5;
							char *token = strtok(uart2_temp, "?");
							token = strtok(NULL, ":");
							token = strtok(NULL, ",");
							strcpy(networkstr,token);
							strcpy(temp_data2,token);
							printf("net: %s\n",networkstr);

							if(j<82 && j>60){
								for(int y=0;y<6;y++){
									token = strtok(NULL, ",");
	//								printf(" %s ",token);
								}
								signal = atoi(token);
								printf("sig: %d\n",signal);
							}
							else {
								for(int y=0;y<11;y++){
									token = strtok(NULL, ",");
	//								printf(" %s ",token);
								}
								signal = atoi(token);
								printf("sig: %d\n",signal);
							}
							memset(uart2_at,0,20);
						}
						memcpy(buffer2,uart2_temp,strlen(uart2_temp));
						memset(uart2_temp,0,1100);
						j=0;
				}
				else if(uart2_temp[j] == '>')
				{
					HAL_UART_Transmit(&huart1, (uint8_t *)uart2_temp, strlen(uart2_temp),0x3ff);
					printf("\r\n");
					uart2done = true;
					uart2busy = false;
					memset(uart2_temp,0,1100);
//					memset(mydata,0,1100);
					j=0;
				}
				else if(uart2_temp[j] == '}')
				{
					printf("%s\r\n",uart2_temp);
					if(j>59){
						if(uart2_temp[j-60] == 'C' && uart2_temp[j-59] == 'o' && uart2_temp[j-58] == 'n' && uart2_temp[j-57] == 'f' && uart2_temp[j-56] == 'i' && uart2_temp[j-55] == 'g')
						{
							printf("New Configuration Received\n");
							char *token = strtok(uart2_temp, ":");
							token = strtok(NULL, "=");
							token = strtok(NULL, ";");
							char *snMeterSub = token;
							int strint = strcmp(snMeter,snMeterSub);
							if(strint == 0){
								token = strtok(NULL, "=");
								token = strtok(NULL, ":");
								instant_hour = atoi(token);
								token = strtok(NULL, ";");
								instant_minute = atoi(token);
								token = strtok(NULL, "=");
	//							token = strtok(NULL, ":");
	//							lp_hour = atoi(token);
								token = strtok(NULL, ";");
								lp_rec = atol(token);
								token = strtok(NULL, "=");
							//					printf("%s\n",token);
								token = strtok(NULL, "-");
								eob_date = atoi(token);
								if(eob_date!=0){
									token = strtok(NULL, ":");
									eob_hour = atoi(token);
									token = strtok(NULL, "}");
									eob_minute = atoi(token);
								}
								else eob_date =0;
								printf("%d:%d %d %d-%d:%d\n",instant_hour, instant_minute, lp_rec, eob_date, eob_hour,eob_minute);
								uart2done = true;
								uart2busy = false;
								memset(uart2_temp,0,1100);
								j=0;
							}
						}
						else if(j>1099){
							printf("%s\r\n",uart2_temp);
							memset(uart2_temp,0,1100);
							j=0;
						}
//						else {
//							HAL_UART_Transmit(&huart1, (uint8_t *)uart2_temp, strlen(uart2_temp),0xff);
//						}
					}
					uart2done = true;
					uart2busy = false;
					memset(uart2_temp,0,1100);
					j=0;

				}
				/* Receive Network Time Protocol and Sync RTC Time with NTP */
				else if(j>29){
					if(uart2_temp[j-29] == 'C' && uart2_temp[j-28] == 'N' && uart2_temp[j-27] == 'T' && uart2_temp[j-26] == 'P' && uart2_temp[j-25] == ':'  && uart2_temp[j-23] == '1')
						{
						cntpok=2; // cntp berhasil
							printf("insert rtc date\n");
							printf("%s\n", uart2_temp);
							set_rtc_time(uart2_temp);
							uart2done = true;
							uart2busy = false;
							memset(uart2_temp,0,1100);
							j=0;
						}
					if(uart2_temp[j-28] == 'C' && uart2_temp[j-27] == 'C' && uart2_temp[j-26] == 'L' && uart2_temp[j-25] == 'K' && uart2_temp[j-24] == ':')
						{
							cntpok=2; // cntp berhasil
							printf("insert rtc date CCLK\r\n");
							printf("%s\n", uart2_temp);
							set_rtc_time_cclk(uart2_temp);
							uart2done = true;
							uart2busy = false;
							memset(uart2_temp,0,1100);
							j=0;
						}
					else if(j>1099){
						printf("%s\r\n",uart2_temp);
						memset(uart2_temp,0,1100);
						j=0;
					}
					else
						j++;
				}
				else if(j>1099){
					printf("%s\r\n",uart2_temp);
					memset(uart2_temp,0,1100);
					j=0;
				}
				else
					j++;
//				HAL_UART_Receive_IT(&huart2,(uint8_t *)uart2_buf,1);
			}
			else if(uart2_temp[j] == '>')
			{
				HAL_UART_Transmit(&huart1, (uint8_t *)uart2_temp, strlen(uart2_temp),0xff);
				printf("\r\n");
				uart2done = true;
				uart2busy = false;
				memset(uart2_temp,0,1100);
				j=0;
			}
			else if(j>1099){
				printf("%s\r\n",uart2_temp);
				memset(uart2_temp,0,1100);
				j=0;
			}
			else
				j++;
			HAL_UART_Receive_IT(&huart2,(uint8_t *)uart2_buf,1);
			osSignalSet( sendDataToServeHandle, BIT_1 | BIT_2);
		}
	if(huart == &huart3)
	{
			uart3_temp[k]=uart3_buf[0]; //store byte in temp buffer
			if(uart3_temp[k] == 0x03){
//				printf("%d\r\n",datauart3size(uart3_temp));
				memcpy(buffer3, uart3_temp,datauart3size(uart3_temp));
				HAL_UART_Transmit(&huart1,(uint8_t *)uart3_temp,datauart3size(uart3_temp),0xff);
				memset(uart3_temp,0,1100);
				uart3done = true;
				k=0;
			}
			else if(k>1099){
				printf("%s\r\n",uart3_temp);
				memset(uart3_temp,0,1100);
				memset(buffer3,0,1100);
				uart3done = true;
				k=0;
			}
			else
				k++;
			HAL_UART_Receive_IT(&huart3,(uint8_t *)uart3_buf,1);
			osSignalSet( readEoBHandle, BIT_1 | BIT_2);
	}
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_RxCpltCallback can be implemented in the user file.
   */
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_main_program */
/**
  * @brief  Function implementing the mainProgram thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_main_program */
void main_program(void const * argument)
{
  /* USER CODE BEGIN 5 */
  osStatus status, status_mutex;
  FATFS fs_t;
  FIL fil_t;
  FRESULT fresult_t;
  RTC_DateTypeDef gDate;
  RTC_TimeTypeDef gTime;
  /* Infinite loop */
  HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN); //current date
  HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN); //current time
  st_hour = gTime.Hours;
  st_min = gTime.Minutes;
  st_sec = gTime.Seconds;
  for(;;)
  {
	  status = osDelay(SECOND);
	  if(status == osOK)
	  {
		  GPIOA->BRR |= 1<<15;
//		  sprintf((char*)tx_buffer, "Main Program\n");
//		  tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
		  status_mutex = osMutexWait(myMutex01Handle, osWaitForever);
		  if(status_mutex == osOK)
		  {
					  HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN); //current date
					  HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN); //current time
//					  if(gTime.Minutes > st_min + 3) hard_reset_rtos();
					  if(pwr_vmon==1) {
						  if(pwr_state==0) {
						    	if(!uart2busy){
						    		uart2busy = true;
						    		sendMQTTpwr();
									pwr_state=1;
						    	}
						  }
					  }
					  else if(pwr_vmon==0) {
						  if(pwr_state==1) {
						    	if(!uart2busy){
						    		uart2busy = true;
						    		sendMQTTpwr();
									pwr_state=0;
						    	}
						  }
					  }
					  if (gDate.Date == 32) {
					  	printf("Setup NTP\r\n");
					  //						  set_cntp();
					  	setup_RTC_periode();
					  } else {
						cekNTP++;
					  	if (cekNTP == 20) {
					  		printf("Setup RTC Periode\r\n");
					  		setup_RTC_periode();
					  		cekNTP = 0;
					  	}
					  	printf("Tanggal: %d, %2d:%2d:%2d pwr_vmon: %d\r\n", gDate.Date, gTime.Hours, gTime.Minutes, gTime.Seconds, pwr_vmon);
					  }//						  sprintf((char*)tx_buffer, "Tanggal: %d, %2d:%2d:%2d pwr_vmon: %d\n", gDate.Date, gTime.Hours, gTime.Minutes, gTime.Seconds, pwr_vmon);
//						  tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
			  osMutexRelease(myMutex01Handle);
		  }
	  }
	  status = osDelay(SECOND);
	  if(status == osOK)
	  {
		  GPIOA->BSRR |= 0<<15;
	  }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_read_gyro */
/**
* @brief Function implementing the readGyro thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_read_gyro */
void read_gyro(void const * argument)
{
  /* USER CODE BEGIN read_gyro */
  osEvent event;
  /* Infinite loop */
  for(;;)
  {
	event = osSignalWait( BIT_1 | BIT_2, osWaitForever);
	if(event.value.signals == (BIT_1 | BIT_2))
	{
		lsm6dsl_reg_t reg;
		lsm6dsl_status_reg_get(&dev_ctx, &reg.status_reg);

		if (reg.status_reg.xlda)
		{
		  /* Read magnetic field data */
		  memset(data_raw_acceleration.u8bit, 0x00, 3*sizeof(int16_t));
		  lsm6dsl_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
		  acceleration_mg[0] = lsm6dsl_from_fs2g_to_mg( data_raw_acceleration.i16bit[0]);
		  acceleration_mg[1] = lsm6dsl_from_fs2g_to_mg( data_raw_acceleration.i16bit[1]);
		  acceleration_mg[2] = lsm6dsl_from_fs2g_to_mg( data_raw_acceleration.i16bit[2]);

		  sprintf((char*)tx_buffer, "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
				  acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
		  tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
		}
		if (reg.status_reg.gda)
		{
		  /* Read magnetic field data */
		  memset(data_raw_angular_rate.u8bit, 0x00, 3*sizeof(int16_t));
		  lsm6dsl_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate.u8bit);
		  angular_rate_mdps[0] = lsm6dsl_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[0]);
		  angular_rate_mdps[1] = lsm6dsl_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[1]);
		  angular_rate_mdps[2] = lsm6dsl_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[2]);

		  sprintf((char*)tx_buffer, "Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
				  angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
		  tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
		}
		if (reg.status_reg.tda)
		{
		  /* Read temperature data */
		  memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
		  lsm6dsl_temperature_raw_get(&dev_ctx, data_raw_temperature.u8bit);
		  temperature_degC = lsm6dsl_from_lsb_to_celsius( data_raw_temperature.i16bit );

		  sprintf((char*)tx_buffer, "Temperature [degC]:%6.2f\r\n", temperature_degC );
		  tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
		}
		osSignalSet( readGyroHandle, BIT_1);
	}
	osDelay(10);
  }
  /* USER CODE END read_gyro */
}

/* USER CODE BEGIN Header_send_data_to_server */
/**
* @brief Function implementing the sendDataToServe thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_send_data_to_server */
void send_data_to_server(void const * argument)
{
  /* USER CODE BEGIN send_data_to_server */
  osStatus status_delay, status_mutex;
  RTC_DateTypeDef gDate;
  RTC_TimeTypeDef gTime;
  uint8_t secnow,minnow;
  status_mutex = osMutexWait(myMutex01Handle, osWaitForever);
  if(status_mutex == osOK)
  {
	  read_sn_edmi();
	  printf("%s\r\n",snMeter);
	  setup_modem_1st();
	  cntpok =1;
	  sprintf((char*)tx_buffer, "MQTT Demo\n");
	  tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
	  mqtt_demo();
	  HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN); //current date
	  HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN); //current time
	  st_hour = gTime.Hours;
	  st_min = gTime.Minutes;
	  st_sec = gTime.Seconds;
	  sprintf((char*)tx_buffer, "MQTT Demo Finished\n");
	  tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
//	  get_config();
	  osMutexRelease(myMutex01Handle);
  }
  /* Infinite loop */
  for(;;)
  {
//    status_delay = osDelay(52*SECOND);
//    status_delay = osDelay(47*SECOND);
//    status_delay = osDelay(49*SECOND);
	  status_delay = osDelay(SECOND);
    if(status_delay == osOK)
    {
    	HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN); //current date
    	HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN); //current time
    	secnow = gTime.Seconds;
    	minnow = gTime.Minutes;
    	status_mutex = osMutexWait(myMutex01Handle, osWaitForever);
//    	status_mutex = osMutexWait(myMutex01Handle, 500);
    	if(status_mutex == osOK)
    	{
//    		if(!uart2busy){
    	    	if((minnow == 2 || minnow == 7 || minnow == 12 || minnow == 17 || minnow == 22 || minnow == 27 || minnow == 32 || minnow == 37 || minnow == 42 || minnow == 47 || minnow == 52 || minnow == 57 ) && secnow == 0){
        			uart2busy = true;
    				sendSimple();
    	    	}
    	    	else if(cntpok == 0 || cntpok == 1) {
        			uart2busy = true;
        			cntpok =1;
        			set_cntp();
        			osDelay(7000);
    	    	}
				else if(secnow == 0){
					if(minnow == 3 || minnow == 33)cntpok=0;
//					SerialATprintln("AT+CNMP=2", 10, 0xff);
//					osDelay(1000);
					uart2done = false;
						sprintf(uart2_at,"AT+CPSI?");
						printf(uart2_at);
						SerialATprintln("AT+CPSI?", 8, 0xff);
//						osDelay(1000);
						for(int y=0;y<30;y++){
									osDelay(200);
									if(uart2done)break;
								}
						int cmpint=strcmp(networkstr," GSM");
						if(cmpint != 0){ // cek network LTE NBIOT
							if(signal<-115){ // cek sinyal Minimum LTE
									SerialATprintln("AT+CNMP=13", 10, 0xff);
									osDelay(2000);
									reset_mqtt();
									SerialATprintln("AT+SMCONN",9,0xff);
									uart2done = false;
									for(int y=0;y<30;y++){
										osDelay(200);
										if(uart2done)break;
									}
								}
						}
						else {
							if(signal<=-100){ // cek sinyal minimum GSM
								for(int z=0;z<3;z++){
									printf("modem reboot\n");
									modem_reset_rtos();
									reset_mqtt();
									SerialATprintln("AT+CPSI?", 8, 0xff);
//									osDelay(2000);
									for(int y=0;y<30;y++){
										osDelay(200);
										if(uart2done)break;
									}
									if(signal > -100) break;
								}

							}
						}
					SerialATprintln("AT+CNMP?", 8, 0xff);
					osDelay(1000);
					for(int z=0;z<3;z++){
						uart2done = false;
						HAL_UART_Transmit(&huart2,(uint8_t *) ATSMSTATE,strlen(ATSMSTATE),0xff);
						for(int y=0;y<25;y++){
									osDelay(200);
									if(uart2done)break;
								}
							temp_data =strstr((char *)buffer2,"+SMSTATE:");
							memcpy(smstatestr,temp_data,11);
							smstateint = strcmp(smstatestr,"+SMSTATE: 0");
							uart2done = false;
							if(smstateint!=0) break;
							reset_mqtt();
							SerialATprintln("AT+SMCONN",9,0xff);
							uart2done = false;
							for(int y=0;y<25;y++){
								osDelay(200);
								if(uart2done)break;
							}
						}
					uart2done = false;
//							SerialATprintln("AT+SMUNSUB=\"tayra/incoming\"", 27, 0xff); //tayra/message
					SerialATprintln("AT+SMUNSUB=\"tayra/config\"", 25, 0xff); //tayra/message
							for(int i=0;i<25;i++){
								osDelay(200);
								if(uart2done)break;
							}
					osDelay(3000);
					uart2done = false;
//						SerialATprintln("AT+SMSUB=\"tayra/incoming\",1", 27, 0xff); //tayra/message
					SerialATprintln("AT+SMSUB=\"tayra/config\",1", 25, 0xff); //tayra/message
						for(int i=0;i<25;i++){
							osDelay(200);
							if(uart2done)break;
						}
					uart2done = false;
					SerialATprintln("AT+CNMP=2", 9, 0xff); // set CNMP automatic
					osDelay(1000);
					SerialATprintln("AT+CNMP?", 8, 0xff);
					osDelay(1000);
				}
//    		}
//    		uart2busy = false;
			osMutexRelease(myMutex01Handle);
    	}
    }
  }
  /* USER CODE END send_data_to_server */
}

/* USER CODE BEGIN Header_read_EoB */
/**
* @brief Function implementing the readEoB thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_read_EoB */
void read_EoB(void const * argument)
{
  /* USER CODE BEGIN read_EoB */
  osStatus status_mutex;
  RTC_DateTypeDef gDate;
  RTC_TimeTypeDef gTime;
  FATFS fs_t;
  FIL fil_t;
  FRESULT fresult_t;
  uint8_t secmin1;

  /* Infinite loop */
  for(;;)
  {
//	  status_delay = osDelay(30*SECOND);
//	  if(status_delay == osOK){
		  HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN); //current date
		  HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN); //current time
//		  if(gTime.Seconds != secmin1) printf(" EoB ");

//		  printf("Tanggal: %d-%d-%d, %2d:%2d:%2d \n", gDate.Date, gDate.Month, gDate.Year, gTime.Hours, gTime.Minutes, gTime.Seconds);

//			  if(gDate.Date == 1 && gTime.Hours == 10 && gTime.Minutes == 0 && gTime.Seconds == 0 && gTime.SubSeconds == 0)
			  if((gDate.Date == eob_date || gDate.Date == 1) && (gTime.Hours == eob_hour || gTime.Hours == 10) && (gTime.Minutes == eob_minute || gTime.Minutes == 0) && gTime.Seconds == 15)
			  {
				  printf("In: if gTime.Minutes == %d eob\n",eob_minute);
				  status_mutex = osMutexWait(myMutex01Handle, osWaitForever);
//				  printf("In: if gTime.Minutes == 10 \n");
				  if(status_mutex == osOK)
				  {
					  if(pwr_vmon==1){
				    	if(!uart2busy){
				    		uart2busy = true;
				    		//				  	  if(!usemqtt)  setup_modem();
				    		if(typemeter==10) read_eob_mk10e();
				    		else read_eob_mk6n();
				    	}
				  	  fresult_t = f_mount(&fs_t, "/", 1);
				  	  if(fresult_t != FR_OK)
				  	  {
					  sprintf((char*)tx_buffer, "Failed to mount SD Card\n");
					  tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
				  	  }
				  	  else {
				  		  if(!onlineMode){
							  if(typemeter == 6) sprintf(buffer, "eobmk6n_%d-%d-%d.txt", gDate.Date, gDate.Month, gDate.Year);
							  else sprintf(buffer, "eobmk10e_%d-%d-%d.txt", gDate.Date, gDate.Month, gDate.Year);
							    fresult_t = f_open(&fil_t, buffer, FA_OPEN_APPEND | FA_READ | FA_WRITE);
							    if(fresult_t != FR_OK)
							    {
							    	sprintf((char*)tx_buffer, "Failed to Open File\n");
							    	tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
							    }
							  // Write some text
							fresult = f_puts(mydata, &fil_t);
							  // Close file
							printf(buffer);
							printf(" created and data written\r\n");
							f_close(&fil_t);
							onlineMode = true;
				  		  }
					f_mount(0, "", 0);
				  }
				}
//			  f_mount(0, "", 0);
//				uart2busy = false;
			  osMutexRelease(myMutex01Handle);
				osSignalWait( BIT_1 | BIT_2, 100);
				osSignalSet( readEoBHandle, BIT_1);
				osDelay(900);
		  }
	  }
			  secmin1 = gTime.Seconds;
  }
  /* USER CODE END read_EoB */
}

/* USER CODE BEGIN Header_read_LP */
/**
* @brief Function implementing the readLP thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_read_LP */
void read_LP(void const * argument)
{
  /* USER CODE BEGIN read_LP */
  osStatus status_mutex;
  FATFS fs_t;
  FIL fil_t;
  FRESULT fresult_t;
  RTC_DateTypeDef gDate;
  RTC_TimeTypeDef gTime;
  uint8_t secmin1;

  /* Infinite loop */
  for(;;)
  {
//	  status_delay = osDelay(MINUTE);
//	  if(status_delay == osOK){
		  HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN); //current date
		  HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN); //current time
//		  if(gTime.Seconds != secmin1) printf(" LP ");
			  if(lp_rec!=0 || ((gTime.Minutes == 0 || gTime.Minutes == 15 || gTime.Minutes == 30 || gTime.Minutes == 45) && gTime.Seconds == 5))
//			if(lp_rec!=0 || ((gTime.Minutes == 0 || gTime.Minutes == 5 || gTime.Minutes == 10 || gTime.Minutes == 15 || gTime.Minutes == 20 | gTime.Minutes == 25 || gTime.Minutes == 30 || gTime.Minutes == 35 || gTime.Minutes == 40 || gTime.Minutes == 45 || gTime.Minutes == 50 || gTime.Minutes == 55 ) && gTime.Seconds == 10))
			  {
			  status_mutex = osMutexWait(myMutex01Handle, osWaitForever);
			  printf("In: if gTime.Minutes == 00/15/30/45 LP \n");
			  if(status_mutex == osOK)
			  {
				  if(pwr_vmon==1){
					  if(!uart2busy){
						  uart2busy = true;
						  //			  	  if(!usemqtt)  setup_modem();
						  //			  	  setup_modem();
						  //				  read_lp_mk6n();
						  if(typemeter==10) read_lp_mk10e();
						  else read_lp_mk6n();
					  }
				  // Open file to write/create if not exist
				  fresult_t = f_mount(&fs_t, "/", 1);
				  if(fresult_t != FR_OK)
				  {
					  sprintf((char*)tx_buffer, "Failed to mount SD Card\n");
					  tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
				  }
				  else {
					  if(!onlineMode){
						  if(typemeter==6) sprintf(buffer, "lpmk6n_%d-%d-%d.txt", gDate.Date, gDate.Month, gDate.Year);
						  else sprintf(buffer, "lpmk10e_%d-%d-%d.txt", gDate.Date, gDate.Month, gDate.Year);
						    fresult_t = f_open(&fil_t, buffer, FA_OPEN_APPEND | FA_READ | FA_WRITE);
						    if(fresult_t != FR_OK)
						    {
						    	sprintf((char*)tx_buffer, "Failed to Open File\n");
						    	tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
						    }
						  // Write some text
						fresult = f_puts(mydata, &fil_t);
						  // Close file
						printf(buffer);
						printf(" created and data written\r\n");
						f_close(&fil_t);
						onlineMode = true;
					  }
					f_mount(0, "", 0);
				  }
				  }
				  else cnt_lp++;
				  osMutexRelease(myMutex01Handle);
				osSignalWait( BIT_1 | BIT_2, 100);
				osSignalSet( readLPHandle, BIT_1);
				osDelay(900);
			  }
//				osDelay(MINUTES);
		  }
			  else if(cnt_lp>0){
				  status_mutex = osMutexWait(myMutex01Handle, osWaitForever);
				  printf("In: LP resend %d\n",cnt_lp);
				  if(status_mutex == osOK)
				  {
					  if(pwr_vmon==1){
						  cnt_lp--;
						  if(!uart2busy){
							  uart2busy = true;
							  //			  	  if(!usemqtt)  setup_modem();
							  //			  	  setup_modem();
							  //				  read_lp_mk6n();
							  if(typemeter==10) read_lp_mk10e();
							  else read_lp_mk6n();
						  }
					  // Open file to write/create if not exist
					  fresult_t = f_mount(&fs_t, "/", 1);
					  if(fresult_t != FR_OK)
					  {
						  sprintf((char*)tx_buffer, "Failed to mount SD Card\n");
						  tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
					  }
					  else {
						  if(!onlineMode){
							  if(typemeter == 6) sprintf(buffer, "lpmk6n_%d-%d-%d.txt", gDate.Date, gDate.Month, gDate.Year);
							  else sprintf(buffer, "lpmk10e_%d-%d-%d.txt", gDate.Date, gDate.Month, gDate.Year);
							    fresult_t = f_open(&fil_t, buffer, FA_OPEN_APPEND | FA_READ | FA_WRITE);
							    if(fresult_t != FR_OK)
							    {
							    	sprintf((char*)tx_buffer, "Failed to Open File\n");
							    	tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
							    }
							  // Write some text
							fresult = f_puts(mydata, &fil_t);
							  // Close file
							printf(buffer);
							printf(" created and data written\r\n");
							f_close(&fil_t);
							onlineMode = true;
						  }
						f_mount(0, "", 0);
					  }
					  }
					  osMutexRelease(myMutex01Handle);
					osSignalWait( BIT_1 | BIT_2, 100);
					osSignalSet( readLPHandle, BIT_1);
					osDelay(900);
				  }
			  }
			  secmin1 = gTime.Seconds;
//	  }
  }
  /* USER CODE END read_LP */
}

/* USER CODE BEGIN Header_read_instant */
/**
* @brief Function implementing the readInstant thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_read_instant */
void read_instant(void const * argument)
{
  /* USER CODE BEGIN read_instant */
  osStatus status_mutex;
  RTC_DateTypeDef gDate;
  RTC_TimeTypeDef gTime;
  FATFS fs_t;
  FIL fil_t;
  FRESULT fresult_t;

  /* Infinite loop */
  for(;;)
  {
//	  status_delay = osDelay(SECOND);
//	  if(status_delay == osOK)
//	  {
		  HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN); //current date
		  HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN); //current time
	//	  if((gTime.Hours == 1 || gTime.Hours == 13) && gTime.Minutes == 0 && gTime.Seconds == 0)
//		  if(gTime.Seconds > secmin1) printf(" Instant ");
		  if((gTime.Hours == instant_hour || gTime.Hours == 7 || gTime.Hours == 19) && (gTime.Minutes == 0 || gTime.Minutes == instant_minute ) && gTime.Seconds == 10)
		  {
			  status_mutex = osMutexWait(myMutex01Handle, osWaitForever);
			  printf("In: if gTime.Minutes == 16 Instant\n");
			  if(status_mutex == osOK)
			  {
				  HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN); //current date
				  HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN); //current time
//				  printf("InsDate: %d-%d-%d, %2d:%2d:%2d \n", gDate.Date, gDate.Month, gDate.Year, gTime.Hours, gTime.Minutes, gTime.Seconds);
				  fresult_t = f_mount(&fs_t, "/", 1);
				  if(fresult_t != FR_OK)
				  {
					  sprintf((char*)tx_buffer, "Failed to mount SD Card\n");
					  tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
				  }
				  if(pwr_vmon==1){
				    	if(!uart2busy){
				    		uart2busy = true;
					  //			  	  if(!usemqtt)  setup_modem();
					  //			  	  setup_modem();
					  //				  read_instant_mk6n();
				    		if(typemeter==10) read_instant_mk10e();
				    		else read_instant_mk6n();
				    	}
//				  timestamp_now = 12345678;
//				  timestamp_now = 12345678;
				  if(!onlineMode){
					  if(typemeter == 6) sprintf(buffer, "instantmk6n_%d-%d-%d.txt", gDate.Date, gDate.Month, gDate.Year);
					  else sprintf(buffer, "instantmk10e_%d-%d-%d.txt", gDate.Date, gDate.Month, gDate.Year);
					    fresult_t = f_open(&fil_t, buffer, FA_OPEN_APPEND | FA_READ | FA_WRITE);
					    if(fresult_t != FR_OK)
					    {
					    	sprintf((char*)tx_buffer, "Failed to Open File\n");
					    	tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
					    }
					  // Write some text
					fresult = f_puts(mydata, &fil_t);
					  // Close file
					printf(buffer);
					printf(" created and data written\r\n");
					f_close(&fil_t);
					onlineMode = true;
				}
				  }
				f_mount(0, "", 0);
//				uart2busy = false;
				osMutexRelease(myMutex01Handle);
				osSignalWait( BIT_1 | BIT_2, 100);
				osSignalSet( readInstantHandle, BIT_1);
				osDelay(900);
			  }
		  }
		  secmin1 = gTime.Seconds;
//	  }
  }
  /* USER CODE END read_instant */
}

/* USER CODE BEGIN Header_send_hard_bit */
/**
* @brief Function implementing the sendHardBit thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_send_hard_bit */
void send_hard_bit(void const * argument)
{
  /* USER CODE BEGIN send_hard_bit */
	osStatus status_delay, status_mutex;
	  RTC_DateTypeDef gDate;
	  RTC_TimeTypeDef gTime;
	  FATFS fs_t;
	  FIL fil_t;
	  FRESULT fresult_t;

	  status_mutex = osMutexWait(myMutex01Handle, osWaitForever);
	    if(status_mutex == osOK)
	    {
	  	  fresult_t = f_mount(&fs_t, "/", 1);
	  	  if(fresult_t != FR_OK)
	  	  {
	  		  sprintf((char*)tx_buffer, "Failed to mount SD Card\n");
	  		  tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
	  	  }
	  	  else
	  	  {
	  		  fresult_t = f_open(&fil_t, "config.txt", FA_READ );
	  		  if(fresult_t != FR_OK)
	  		  {
	  			  sprintf((char*)tx_buffer, "Failed to Open File\n");
	  			  tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
	  		  }
	  		  else
	  		  {
	  			  sprintf((char*)tx_buffer, "Heart Beat time configured\n");
	  			  tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
	  			  f_gets(buffer2, sizeof(buffer2), &fil_t); // Read Time of EoB Config from SD Card
	  			  eob_hour = atoi(buffer2);
	  			  printf("\nEOB HOUR: %d\r\n", eob_hour);
	  			  f_gets(buffer2, sizeof(buffer2), &fil_t);
	  			  eob_minute = atoi(buffer2);
	  			  printf("\nEOB MINUTE: %d\r\n", eob_minute);
	  			  f_close(&fil_t);
	  		  }
	  		  f_mount(0, "", 0);
	  	  }
	  	osMutexRelease(myMutex01Handle);
	    }
	  /* Infinite loop */
	  for(;;)
	  {
//		  HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN); //current date
//		  HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN); //current time
//
//		  if(gTime.Hours == eob_hour && gTime.Minutes == eob_minute && gTime.Seconds == 0 && gTime.SubSeconds < 100)
//		  {
//			sprintf((char*)tx_buffer, "Read End of Billing at %d:%d:00\n", eob_hour, eob_minute);
//			tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
//		  }
//		  osDelay(100);
	  }
  /* USER CODE END send_hard_bit */
}

/* USER CODE BEGIN Header_check_vbat */
/**
* @brief Function implementing the checkVbat thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_check_vbat */
void check_vbat(void const * argument)
{
  /* USER CODE BEGIN check_vbat */
	uint32_t adc1;
	int cnt=0;
	float vbat_percen,counter,div;
  /* Infinite loop */

  for(;;)
  {
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1,100);
	  adc1 = HAL_ADC_GetValue(&hadc1);
	  HAL_ADC_Stop(&hadc1);
	  counter = (float)(adc1 - 498);
	  div = 640 - 498;
	  vbat_percen = counter/div*100;
	  vbat_percentage = vbat_percen;
//	  if(vbat_percen<0) vbat_percen=0;
	  if(vbat_percen>100 && vbat_percen<200) vbat_percen=100;
	  else if(vbat_percen>=200) vbat_percen=0;
	  if(cnt>=5) {
		  printf("VBAT Percent: %.2f \r\n",vbat_percen);
		  cnt = 0;
	  }
	  osDelay(1000);
	  cnt++;
  }
  /* USER CODE END check_vbat */
}

/* USER CODE BEGIN Header_check_vmon */
/**
* @brief Function implementing the checkVMon thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_check_vmon */
void check_vmon(void const * argument)
{
  /* USER CODE BEGIN check_vmon */
//	  osStatus status_mutex;
	uint8_t cnt=0;
//	osDelay(3000);
  /* Infinite loop */
  for(;;)
  {
	  pwr_vmon = HAL_GPIO_ReadPin(PWR_VMON_GPIO_Port,PWR_VMON_Pin);
//	  	  pwr_vmon = pwr_vmon_rtos;
//	  status_mutex = osMutexWait(myMutex01Handle, osWaitForever);
//		if(status_mutex == osOK)
//		{
//	    		uart2busy = false;
//			osMutexRelease(myMutex01Handle);
//			osSignalWait( BIT_1 | BIT_2, 1000);
//			osSignalSet( checkVMonHandle, BIT_1);
//		}
		osDelay(1000);
		cnt++;
//	  printf("pwr_vmon: %d\r\n",pwr_vmon);
  }
  /* USER CODE END check_vmon */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
