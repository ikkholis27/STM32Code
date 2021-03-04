/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l4xx_hal.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE END Includes */

// definitions block
#define BIT_0	( 1 << 0 )
#define BIT_1	( 1 << 1 )
#define BIT_2	( 1 << 2 )

#define SerialMon  	huart1
#define SerialAT  	huart2

#define ENABLEMDM GPIO_PIN_0

#define BUFFSIZESIM	1200

uint32_t datalen;
char keyword[BUFFSIZESIM];
char datarxSIM[BUFFSIZESIM];

char ATSHSTATE[]="AT+SHSTATE?";
//char ATSHAHEAD1[]="AT+SHAHEAD=\"Accept\",\"text/html, */*\"";
char ATSHAHEAD1[]="AT+SHAHEAD=\"Accept\",\"*/*\"";
char ATSHAHEAD2[]="AT+SHAHEAD=\"User-Agent\",\"IOE Client\"";
char ATSHAHEAD3[]="AT+SHAHEAD=\"Content-Type\",\"application/x-www-form-urlencoded\"";
//char ATSHAHEAD3[]="AT+SHAHEAD=\"Content-Type\",\"application/json\"";
char ATSHAHEAD4[]="AT+SHAHEAD=\"Connection\",\"keep-alive\"";
char ATSHAHEAD5[]="AT+SHAHEAD=\"Cache-control\",\"no-cache\"";

char ATSHREQ[]="AT+SHREQ=\"http://dev-tayra.digitalisasi-project.com/api/v1/message\",3";
char ATSHBOD[]="AT+SHBOD=";

//char ATSHBOD[]="AT+SHBOD="+String(datastr.length())+",10000";

extern osThreadId sendDataToServeHandle;
extern RTC_HandleTypeDef hrtc;
/*
void set_rtc_time(){

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

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
}
*/

void set_rtc_time(char *data){

  uint8_t year, month, date;
  uint8_t hour, minute, second;

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  printf("%s\n", data);

  char *token = strtok(data, "0");
  token = strtok(NULL, "/");
  year = atoi(token);
  printf( " %d\n", year );
  token = strtok(NULL, "/");
  month = atoi(token);
  printf( " %d\n", month );
  token = strtok(NULL, ",");
  date = atoi(token);
  printf( " %d\n", date );
  token = strtok(NULL, ":");
//  hour = atoi(token)+7;
  hour = atoi(token)+7;
  printf( " %d\n", hour );
  token = strtok(NULL, ":");
  minute = atoi(token);
  printf( " %d\n", minute );
  token = strtok(NULL, "\"");
  second = atoi(token)+3;
  printf( " %d\n", second );

  if(hour > 23){
	  hour = hour - 24;
	  date++;
  }

  sTime.Hours = hour;
  printf( " %d\n", sTime.Hours );
  sTime.Minutes = minute;
  printf( " %d\n", sTime.Minutes );
  sTime.Seconds = second;
  printf( " %d\n", sTime.Seconds );
//  sTime.Hours = 0x19U;
//  sTime.Minutes = 0x30U;
//  sTime.Seconds = 0x00U;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
//  sDate.WeekDay = RTC_WEEKDAY_FRIDAY;
  sDate.Month = month;
  sDate.Date = date;
  sDate.Year = year;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
}

void set_rtc_time_cclk(char *data){

  uint8_t year, month, date;
  uint8_t hour, minute, second;

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  printf("%s\n", data);

  char *token = strtok(data, "\"");
  token = strtok(NULL, "/");
  if (atoi(token) != 0) {
	  year = atoi(token);
	  printf( " %d\n", year );
	  token = strtok(NULL, "/");
	  month = atoi(token);
	  printf( " %d\n", month );
	  token = strtok(NULL, ",");
	  date = atoi(token);
	  printf( " %d\n", date );
	  token = strtok(NULL, ":");
	  hour = atoi(token);
	  printf( " %d\n", hour );
	  token = strtok(NULL, ":");
	  minute = atoi(token);
	  printf( " %d\n", minute );
	  token = strtok(NULL, "\"");
	  second = atoi(token)+3;
	  printf( " %d\n", second );

	  if(hour > 23){
		  hour = hour - 24;
		  date++;
	  }

	  sTime.Hours = hour;
	  printf( " %d\n", sTime.Hours );
	  sTime.Minutes = minute;
	  printf( " %d\n", sTime.Minutes );
	  sTime.Seconds = second;
	  printf( " %d\n", sTime.Seconds );
	//  sTime.Hours = 0x19U;
	//  sTime.Minutes = 0x30U;
	//  sTime.Seconds = 0x00U;
	  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
	  {
		Error_Handler();
	  }
	//  sDate.WeekDay = RTC_WEEKDAY_FRIDAY;
	  sDate.Month = month;
	  sDate.Date = date;
	  sDate.Year = year;

	  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
	  {
		Error_Handler();
	  }
  }
}

/* find size of datarxSIM in buffer */
int buffsizeSIM(char buf[])
{
	int i = 0;
	while(buf[i] != '\0') i++;
	return i;
}

void buffclearSIM(void)
{
	for (int i=0; i < 1024; i++)
	{
		datarxSIM[i] = '\0';
	}
}

void SerialMonprintlnSIM(char ptr[], uint32_t len, uint32_t timeout){
	uint8_t ptr1[len+2];
	memcpy(ptr1, (uint8_t *) ptr, len);
	ptr1[len]= 0x0D;
	ptr1[len+1]= 0x0A;
	  HAL_UART_Transmit(&SerialMon, (uint8_t *) ptr1, len+2, timeout);
}

void SerialATprintln(char ptr[], uint32_t len, uint32_t timeout){
	uint8_t ptr1[len+2];
	memcpy(ptr1, ptr, len);
	ptr1[len]= '\r';
	ptr1[len+1]= '\n';
	HAL_UART_Transmit(&huart2, (uint8_t *) ptr1, len+2, timeout);
}

void SerialATreadString(char ptr[], uint32_t len, uint32_t timeout){
	  HAL_UART_Receive(&SerialAT, (uint8_t *)ptr, len, timeout);
}

// Modem Variables
int check_GPRS(){
	buffclearSIM();
	SerialATprintln("AT+CGATT?",strlen("AT+CGATT?"),0xff);
	return 0;
}
void enable_modem(){
	HAL_GPIO_WritePin(GPIOB, ENABLEMDM, GPIO_PIN_SET);
  osDelay(2500);
  HAL_GPIO_WritePin(GPIOB, ENABLEMDM, GPIO_PIN_RESET);
  osDelay(7000);
}

void get_config()
{
	char thingspeak[] = "AT+SHCONF=\"URL\",\"http://thingspeak.com\"";
	char thingspeak_api[] = "AT+SHREQ=\"http://api.thingspeak.com/apps/thinghttp/send_request?api_key=E6IYMBVXVWKDL00C\",1";

	SerialMonprintlnSIM("Get Config", sizeof("Get Config"), 0xff);

	buffclearSIM();
	SerialATprintln(thingspeak, 39, 0xff);
	osSignalWait( BIT_1 | BIT_2, 100);
	osSignalSet( sendDataToServeHandle, BIT_1);
	osDelay(500);

	buffclearSIM();
	SerialATprintln("AT+SHCONF=\"BODYLEN\",1100", 24, 0xff);
	osSignalWait( BIT_1 | BIT_2, 100);
	osSignalSet( sendDataToServeHandle, BIT_1);
	osDelay(500);

	buffclearSIM();
	SerialATprintln("AT+SHCONF=\"HEADERLEN\",350", 25, 0xff);
	osSignalWait( BIT_1 | BIT_2, 100);
	osSignalSet( sendDataToServeHandle, BIT_1);
	osDelay(500);

	buffclearSIM();
	SerialATprintln("AT+CGDCONT=1,\"IP\",\"internet\",\"0.0.0.0\",0,0,0", 44, 0xff);
	osSignalWait( BIT_1 | BIT_2, 100);
	osSignalSet( sendDataToServeHandle, BIT_1);
	osDelay(500);

	buffclearSIM();
	SerialATprintln("AT+CGDCONT?", 11, 0xff);
	osSignalWait( BIT_1 | BIT_2, 100);
	osSignalSet( sendDataToServeHandle, BIT_1);
	osDelay(500);

	buffclearSIM();
	SerialATprintln("AT+CNACT=0,1", 12, 0xff);
	osSignalWait( BIT_1 | BIT_2, 100);
	osSignalSet( sendDataToServeHandle, BIT_1);
	osDelay(500);

	buffclearSIM();
	SerialATprintln("AT+CNACT?", 9, 0xff);
	osSignalWait( BIT_1 | BIT_2, 100);
	osSignalSet( sendDataToServeHandle, BIT_1);
	osDelay(500);

	buffclearSIM();
	SerialATprintln("AT+SHCONN", 9, 0xff);
	osSignalWait( BIT_1 | BIT_2, 100);
	osSignalSet( sendDataToServeHandle, BIT_1);
	osDelay(500);

	buffclearSIM();
	SerialATprintln("AT+SHSTATE?", 11, 0xff);
	osSignalWait( BIT_1 | BIT_2, 100);
	osSignalSet( sendDataToServeHandle, BIT_1);
	osDelay(500);

	buffclearSIM();
	SerialATprintln("AT+SHAHEAD=\"User-Agent\",\"IOE Client\"", 36, 0xff);
	osSignalWait( BIT_1 | BIT_2, 100);
	osSignalSet( sendDataToServeHandle, BIT_1);
	osDelay(500);

	buffclearSIM();
	SerialATprintln("AT+SHAHEAD=\"Content-Type\",\"application/x-www-form-urlencoded\"", 61, 0xff);
	osSignalWait( BIT_1 | BIT_2, 100);
	osSignalSet( sendDataToServeHandle, BIT_1);
	osDelay(500);

	buffclearSIM();
	SerialATprintln("AT+SHAHEAD=\"Connection\",\"keep-alive\"", 36, 0xff);
	osSignalWait( BIT_1 | BIT_2, 100);
	osSignalSet( sendDataToServeHandle, BIT_1);
	osDelay(500);

	buffclearSIM();
	SerialATprintln("AT+SHAHEAD=\"Cache-control\",\"no-cache\"", 37, 0xff);
	osSignalWait( BIT_1 | BIT_2, 100);
	osSignalSet( sendDataToServeHandle, BIT_1);
	osDelay(500);

	buffclearSIM();
	SerialATprintln(thingspeak_api, 91, 0xff);
	osSignalWait( BIT_1 | BIT_2, 1000);
	osSignalSet( sendDataToServeHandle, BIT_1);
	osDelay(8000);

	buffclearSIM();
	SerialATprintln("AT+SHREAD=0,41", 14, 0xff);
	osSignalWait( BIT_1 | BIT_2, 100);
	osSignalSet( sendDataToServeHandle, BIT_1);
	osDelay(500);

	buffclearSIM();
	SerialATprintln("AT+SHDISC", 9, 0xff);
	osSignalWait( BIT_1 | BIT_2, 100);
	osSignalSet( sendDataToServeHandle, BIT_1);
	osDelay(500);

}

void setup_modem_nortos(){

	buffclearSIM();
  SerialATprintln("AT",2,0xff);
  HAL_Delay(500);

  SerialATprintln("AT+CGDCONT?",11,0xff);
  SerialATreadString(datarxSIM,BUFFSIZESIM,1000);
  SerialMonprintlnSIM(datarxSIM,300,500);

	buffclearSIM();
  SerialATprintln("AT+CGDCONT=1,\"IP\",\"internet\",\"0.0.0.0\",0,0,0",44,0xff);
  SerialATreadString(datarxSIM,BUFFSIZESIM,1000);
  SerialMonprintlnSIM(datarxSIM,300,800);

	buffclearSIM();
  SerialATprintln("AT+CGDCONT?",11,0xff);
  SerialATreadString(datarxSIM,BUFFSIZESIM,1000);
  SerialMonprintlnSIM(datarxSIM,300,1000);

	buffclearSIM();
  SerialATprintln("AT+CREG=1",9,0xff);
  SerialATreadString(datarxSIM,BUFFSIZESIM,1000);
  SerialMonprintlnSIM(datarxSIM,100,1000);

	buffclearSIM();
  SerialATprintln("AT+CGREG=1",10,0xff);
  SerialATreadString(datarxSIM,BUFFSIZESIM,1000);
  SerialMonprintlnSIM(datarxSIM,100,1000);

  SerialATprintln("AT+CGATT?",9,0xff);
  SerialATprintln("AT+CGATT=1",10,0xff);
  SerialATreadString(datarxSIM,BUFFSIZESIM,1000);
  SerialMonprintlnSIM(datarxSIM,100,1000);
}

void setup_modem_1st(){
//  enable_modem();
	 buffclearSIM();
  SerialATprintln("AT",2,0xff);
  osDelay(500);

  SerialATprintln("AT+CSQ",6,0xff);
  osDelay(1000);

  SerialATprintln("AT+CGDCONT?",11,0xff);
  osDelay(1000);

  SerialATprintln("AT+CGDCONT=1,\"IP\",\"nb1internet\",\"0.0.0.0\",0,0,0",44,0xff);
//  osSignalWait( BIT_1 | BIT_2, 100);
//  osSignalSet( sendDataToServeHandle, BIT_1);
  osDelay(1000);

  SerialATprintln("AT+CGDCONT?",11,0xff);
  osDelay(1000);

  SerialATprintln("AT+CREG=1",9,0xff);
  osDelay(1000);

  SerialATprintln("AT+CGREG=1",10,0xff);
  osDelay(1000);

  SerialATprintln("AT+CGATT?",9,0xff);
  osSignalWait( BIT_1 | BIT_2, 100);
  osSignalSet( sendDataToServeHandle, BIT_1);
  osDelay(1000);
  SerialATprintln("AT+CGATT=1",10,0xff);
  osSignalWait( BIT_1 | BIT_2, 100);
  osSignalSet( sendDataToServeHandle, BIT_1);
  osDelay(2000);

  SerialATprintln("AT+CNMP?", 8, 0xff);
  	osDelay(2000);

    SerialATprintln("AT+CNACT=0,1", 12, 0xff);
    osDelay(2000);

	SerialATprintln("AT+CNACT?", 9, 0xff);
	osDelay(500);

     SerialATprintln("AT+CNTPCID=0", 12, 0xff);
     osDelay(1000);

//  SerialATprintln("AT+CNTP=\"id.pool.ntp.org\",7,0,2", 29, 0xff);
//  SerialATprintln("AT+CNTP=\"173.249.41.186\",7,0,2", 30, 0xff);
//  SerialATprintln("AT+CNTP=\"ntppool.org\",28,1,2", 28, 0xff);
     SerialATprintln("AT+CNTP=\"173.249.41.186\",28,0,2", 35, 0xff);
  osDelay(1000);

  SerialATprintln("AT+CNTP", 7, 0xff);
  osDelay(3000);
}

void set_cntp(){
	    SerialATprintln("AT+CNACT=0,1", 12, 0xff);
	    osDelay(500);

		SerialATprintln("AT+CNACT?", 9, 0xff);
		osSignalWait( BIT_1 | BIT_2, 100);
		osSignalSet( sendDataToServeHandle, BIT_1);
		osDelay(500);

	     SerialATprintln("AT+CNTPCID=0", 12, 0xff);
	     osDelay(500);

//	  SerialATprintln("AT+CNTP=\"id.pool.ntp.org\",7,0,2", 29, 0xff);
//	  SerialATprintln("AT+CNTP=\"173.249.41.186\",7,0,2", 30, 0xff);
	//  SerialATprintln("AT+CNTP=\"ntppool.org\",28,1,2", 28, 0xff);
	     SerialATprintln("AT+CNTP=\"173.249.41.186\",28,0,2", 35, 0xff);
	     	  osDelay(1000);

	  SerialATprintln("AT+CNTP", 7, 0xff);
	  osDelay(3000);

}
void setup_RTC_periode() {
	SerialATprintln("AT+CCLK?", 8, 0xff);
	osDelay(3000);
}
void setup_modem(){
//  enable_modem();
//  SerialATreadString(datarxSIM,BUFFSIZESIM,5000);
//  SerialMonprintlnSIM(" ",1,1000);

	buffclearSIM();
  SerialATprintln("AT",2,0xff);
  osDelay(500);
//  SerialATreadString(datarxSIM,BUFFSIZESIM,2000);
//  SerialMonprintlnSIM(datarxSIM,buffsizeSIM(datarxSIM),500);
//  osDelay(500);
  buffclearSIM();
  SerialATprintln("AT+CSQ",6,0xff);
  osSignalWait( BIT_1 | BIT_2, 100);
  osSignalSet( sendDataToServeHandle, BIT_1);
  osDelay(1000);

	buffclearSIM();
  SerialATprintln("AT+CGDCONT=1,\"IP\",\"internet\",\"0.0.0.0\",0,0,0",44,0xff);
  osSignalWait( BIT_1 | BIT_2, 100);
  osSignalSet( sendDataToServeHandle, BIT_1);
  osDelay(1000);

	buffclearSIM();
  SerialATprintln("AT+CGDCONT?",11,0xff);
  osDelay(2000);

  SerialATprintln("AT+CGATT=1",10,0xff);
  osSignalWait( BIT_1 | BIT_2, 100);
  osSignalSet( sendDataToServeHandle, BIT_1);
  osDelay(2000);

  SerialATprintln("AT+SHCONF=\"URL\",\"http://dev-tayra.digitalisasi-project.com\"",59,0xff);
  osDelay(1000);

  SerialATprintln("AT+SHCONF=\"BODYLEN\",1100",24,0xff);
  osDelay(1000);

  SerialATprintln("AT+SHCONF=\"HEADERLEN\",350",25,0xff);
  osDelay(1000);
}

void sendToServer(char * datastr, int leng){

//	buffclearSIM();
	SerialMonprintlnSIM("Send To Server", 14, 500);
  SerialATprintln("AT+CGDCONT?",11,700);
  osSignalWait( BIT_1 | BIT_2, 100);
  osSignalSet( sendDataToServeHandle, BIT_1);
  osDelay(1000);

	buffclearSIM();
  SerialATprintln("AT+CGDCONT=1,\"IP\",\"internet\",\"0.0.0.0\",0,0,0",44,1000);
  osSignalWait( BIT_1 | BIT_2, 100);
  osSignalSet( sendDataToServeHandle, BIT_1);
  osDelay(1000);
//
  SerialATprintln("AT+CNACT=0,1",12,1000);
  osSignalWait( BIT_1 | BIT_2, 100);
  osSignalSet( sendDataToServeHandle, BIT_1);
  osDelay(1000);

//	buffclearSIM();
  SerialATprintln("AT+CNACT?",9,700);
  osSignalWait( BIT_1 | BIT_2, 100);
  osSignalSet( sendDataToServeHandle, BIT_1);
  osDelay(1000);

//	buffclearSIM();
  SerialATprintln("AT+SHCONN",9,700);
  osSignalWait( BIT_1 | BIT_2, 100);
  osSignalSet( sendDataToServeHandle, BIT_1);
  osDelay(1000);

//	buffclearSIM();
  SerialATprintln(ATSHSTATE,sizeof(ATSHSTATE),1000);
  osSignalWait( BIT_1 | BIT_2, 100);
  osSignalSet( sendDataToServeHandle, BIT_1);
  osDelay(1000);

//	buffclearSIM();
  SerialATprintln(ATSHAHEAD2,sizeof(ATSHAHEAD2),1000);
  osSignalWait( BIT_1 | BIT_2, 100);
  osSignalSet( sendDataToServeHandle, BIT_1);
  osDelay(1000);

	buffclearSIM();
    SerialATprintln(ATSHAHEAD3,sizeof(ATSHAHEAD3),1000);
    osSignalWait( BIT_1 | BIT_2, 100);
    osSignalSet( sendDataToServeHandle, BIT_1);
    osDelay(1000);

  int sizedatastr;
  if(leng>10 && leng<100)sizedatastr=2;
  else if(leng>100 && leng<1000)sizedatastr=3;
  else sizedatastr=1;
  char ATSHBOD1[sizeof(ATSHBOD)+sizedatastr+7];
  char lengthdatastr[sizedatastr];
  strcat(ATSHBOD1,ATSHBOD);
  sprintf(lengthdatastr,"%d",leng);
  strcat(ATSHBOD1,lengthdatastr);
  strcat(ATSHBOD1,",10000");
//  SerialMonprintlnSIM(ATSHBOD1,sizeof(ATSHBOD1),1000);

  buffclearSIM();
  SerialATprintln(ATSHBOD1,sizeof(ATSHBOD1),1000);
  osSignalWait( BIT_1 | BIT_2, 100);
  osSignalSet( sendDataToServeHandle, BIT_1);
  osDelay(1000);

// Send DataSTR
  buffclearSIM();
  SerialATprintln(datastr,leng,1000);
  osDelay(1000);

    buffclearSIM();
  SerialATprintln(ATSHREQ,sizeof(ATSHREQ),800);

  osDelay(8000);
	buffclearSIM();
  SerialATprintln("AT+SHDISC",9,0xff);
  osSignalWait( BIT_1 | BIT_2, 100);
  osSignalSet( sendDataToServeHandle, BIT_1);
  osDelay(1000);
}

void sendMQTT(char * datastr, int leng){

	SerialMonprintlnSIM("Send To MQTT", 14, 500);

  SerialATprintln("AT+CGDCONT=1,\"IP\",\"internet\"",28,500);
  osSignalWait( BIT_1 | BIT_2, 100);
  osSignalSet( sendDataToServeHandle, BIT_1);
  osDelay(1000);

  SerialATprintln("AT+CGDCONT?",11,700);
  osSignalWait( BIT_1 | BIT_2, 100);
  osSignalSet( sendDataToServeHandle, BIT_1);
  osDelay(1000);

  SerialATprintln("AT+CNACT=0,1",12,1000);
  osSignalWait( BIT_1 | BIT_2, 100);
  osSignalSet( sendDataToServeHandle, BIT_1);
  osDelay(1000);

  SerialATprintln("AT+CNACT?",9,700);
  osSignalWait( BIT_1 | BIT_2, 100);
  osSignalSet( sendDataToServeHandle, BIT_1);
  osDelay(1000);

  SerialATprintln("AT+SMDISC",9,700);
  osSignalWait( BIT_1 | BIT_2, 100);
  osSignalSet( sendDataToServeHandle, BIT_1);
  osDelay(1000);

//  SerialATprintln("AT+SMCONF=\"URL\",broker.mqttdashboard.com,1883",45,0xff);
  SerialATprintln("AT+SMCONF=\"URL\",broker.hivemq.com,1883",38,0xff);
  osSignalWait( BIT_1 | BIT_2, 100);
  osSignalSet( sendDataToServeHandle, BIT_1);
  osDelay(1000);

  SerialATprintln("AT+SMCONF=\"KEEPTIME\",60",23,0xff);
  osSignalWait( BIT_1 | BIT_2, 100);
  osSignalSet( sendDataToServeHandle, BIT_1);
  osDelay(1000);
//
//  SerialATprintln(ATSMSTATE,sizeof(ATSMSTATE),1000);
//  osSignalWait( BIT_1 | BIT_2, 100);
//  osSignalSet( sendDataToServeHandle, BIT_1);
//  osDelay(1000);

  SerialATprintln("AT+SMCONN",9,700);
  osSignalWait( BIT_1 | BIT_2, 100);
  osSignalSet( sendDataToServeHandle, BIT_1);
  osDelay(1000);
//

  buffclearSIM();
  osSignalWait( BIT_1 | BIT_2, 100);
  osSignalSet( sendDataToServeHandle, BIT_1);
  osDelay(1000);

// Send DataSTR
  osDelay(8000);
//  SerialATprintln("AT+SMDISC",9,0xff);
  osSignalWait( BIT_1 | BIT_2, 100);
  osSignalSet( sendDataToServeHandle, BIT_1);
  osDelay(1000);
}

// Subscribe to MQTT Topic
void mqtt_demo(void)
{

  SerialATprintln("AT+CGNAPN",9,0xff);
  osSignalWait( BIT_1 | BIT_2, 100);
  osSignalSet( sendDataToServeHandle, BIT_1);
  osDelay(1000);

  SerialATprintln("AT+CGDCONT?",11,0xff);
  osDelay(1000);

  buffclearSIM();
  SerialATprintln("AT+CGDCONT=1,\"IP\",\"nb1internet\",\"0.0.0.0\",0,0,0",44,0xff);
  osSignalWait( BIT_1 | BIT_2, 100);
  osSignalSet( sendDataToServeHandle, BIT_1);
  osDelay(1000);

  buffclearSIM();
  SerialATprintln("AT+CGDCONT?",11,0xff);;

  buffclearSIM();
  SerialATprintln("AT+CREG=1",9,0xff);
  osDelay(1000);

  buffclearSIM();
  SerialATprintln("AT+CGREG=1",10,0xff);
  osDelay(1000);

  SerialATprintln("AT+CGATT?",9,0xff);
  osSignalWait( BIT_1 | BIT_2, 100);
  osSignalSet( sendDataToServeHandle, BIT_1);
  osDelay(1000);

  buffclearSIM();
  SerialATprintln("AT+CNACT=0,1", 12, 0xff);
  osSignalWait( BIT_1 | BIT_2, 100);
  osSignalSet( sendDataToServeHandle, BIT_1);
  osDelay(500);

  buffclearSIM();
  SerialATprintln("AT+CNACT?",9,0xff);
  osSignalWait( BIT_1 | BIT_2, 100);
  osSignalSet( sendDataToServeHandle, BIT_1);
  osDelay(1000);

  buffclearSIM();
  SerialATprintln("AT+SMDISC",11,0xff);
  osSignalWait( BIT_1 | BIT_2, 100);
  osSignalSet( sendDataToServeHandle, BIT_1);
  osDelay(1000);

  buffclearSIM();
  SerialATprintln("AT+SMSTATE?",11,0xff);
  osSignalWait( BIT_1 | BIT_2, 100);
  osSignalSet( sendDataToServeHandle, BIT_1);
  osDelay(1000);

//  SerialATprintln("AT+SMCONF=\"URL\",\"broker.mqttdashboard.com\",1883",47,0xff);
  SerialATprintln("AT+SMCONF=\"URL\",broker.hivemq.com,1883",38,0xff);
  osSignalWait( BIT_1 | BIT_2, 100);
  osSignalSet( sendDataToServeHandle, BIT_1);
  osDelay(500);

  SerialATprintln("AT+SMCONF=\"KEEPTIME\",60",23,0xff);
  osSignalWait( BIT_1 | BIT_2, 100);
  osSignalSet( sendDataToServeHandle, BIT_1);
  osDelay(500);

  SerialATprintln("AT+SMCONF=\"CLEANSS\",1",21,0xff);
  osSignalWait( BIT_1 | BIT_2, 100);
  osSignalSet( sendDataToServeHandle, BIT_1);
  osDelay(1000);

//  buffclearSIM();
//  SerialATprintln("AT+SMCONF=\"CLIENTID\",\"clientId-RjE651foVf\"",42,0xff);
//  osSignalWait( BIT_1 | BIT_2, 100);
//  osSignalSet( sendDataToServeHandle, BIT_1);
//  osDelay(2000);

  buffclearSIM();
  SerialATprintln("AT+SMCONN",9,0xff);
  osSignalWait( BIT_1 | BIT_2, 100);
  osSignalSet( sendDataToServeHandle, BIT_1);
  osDelay(10000);

  buffclearSIM();
  SerialATprintln("AT+SMSTATE?",11,0xff);
  osSignalWait( BIT_1 | BIT_2, 100);
  osSignalSet( sendDataToServeHandle, BIT_1);
  osDelay(1000);

  buffclearSIM();
//  SerialATprintln("AT+SMSUB=\"testtopic/Flooerp\",1", 30, 0xff);
//  SerialATprintln("AT+SMSUB=\"tayra/message\",1", 26, 0xff); //tayra/message
//  SerialATprintln("AT+SMSUB=\"tayra/incoming\",1", 27, 0xff); //tayra/message
  SerialATprintln("AT+SMSUB=\"tayra/config\",1", 25, 0xff); //tayra/message
  osSignalWait( BIT_1 | BIT_2, 100);
  osSignalSet( sendDataToServeHandle, BIT_1);
  osDelay(3000);
}
