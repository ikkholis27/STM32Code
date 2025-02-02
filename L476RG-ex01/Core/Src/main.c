/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h> //strlen
#include <stdio.h> //printf
#include <stdlib.h> //printf
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t debug = 1;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
int _write(int file, char *ptr, int len)
 {
	 int DataIdx;
	 for (DataIdx = 0; DataIdx < len; DataIdx++)
	 {
		 ITM_SendChar(*ptr++);
	 }
	 HAL_Delay(500);
	 return len;
 }
//int _write(int file, char *ptr, int len){
//  HAL_UART_Transmit(&huart4, (uint8_t *)ptr, len, 0xff);
//  return len;
//}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int i=0,j=0,k=0,l=0;
int cnt1=0, cnt2=0, cnt3=0, cnt4=0, cnt5=0;
bool uart3done, uart4busy, uart3disc;
bool usemqtt,uart4done;
int cntpok=0;
int bugint;
char uart4_at[20];

char csq[20],cgatt[27];
uint8_t csqint,cgattint,smstateint,secmin1;
int signal;
char *networkstr;
char *smstatestr;
char *ATSMSTATE="AT+SMSTATE?\r\n";
char *ATSMPUB;
char * temp_data;
char * temp_data2;
uint8_t * uart_uint8t;

uint8_t rxavailable=0;

char uart1_temp[1100],uart1_buf[0],uart1_data[1100];
char uart3_temp[1100],uart3_buf[0],uart3_data[1100];
char uart4_temp[1100],uart4_buf[0],uart4_data[1100];
char uart5_temp[1100],uart5_buf[0],uart5_data[1100];
char buffer[1100], buffer_mdm[1100];
//char buffer1[1100];
//char buffer3[1100];
//char buffer4[1100];
//char buffer5[1100];

void SerialATprintln(char ptr[], uint32_t len, uint32_t timeout){
	uint8_t ptr1[len+2];
	memcpy(ptr1, ptr, len);
	ptr1[len]= '\r';
	ptr1[len+1]= '\n';
	HAL_UART_Transmit(&huart1, (uint8_t *) ptr1, len+2, timeout);
}

void hard_reset_mdm(){
	  HAL_GPIO_WritePin(PWRKEY_GPIO_Port, PWRKEY_Pin, GPIO_PIN_RESET);
	  HAL_Delay(1000);
	  HAL_GPIO_WritePin(PWRKEY_GPIO_Port, PWRKEY_Pin, GPIO_PIN_SET);
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

//void bufclear(void)
//{
//	for (int i=0; i < 1024; i++)
//	{
//		buffer[i] = '\0';
//		mydata[i] = '\0';
//	}
//}

///////////////////// MODEM FUNCTION
void ATCOMMAND(char *command, char* res)
{
//	char respond[120] = {0};
//	char *ptr= strstr(respond,res);
//	while(ptr == NULL){
	memset(buffer,0,1100);
	HAL_UART_Transmit(&huart1,(uint8_t *)command,strlen(command),0xff);
	HAL_UART_Receive(&huart1,(uint8_t *)buffer,1100,500);
//	ptr = strstr(respond,res);
//	}
//	printf("%s\r\n",respond);
	printf(buffer);
	printf("\r\n");

}

void ATCOMMANDLong(char *command, char* res)
{

//	char *ptr= strstr(respond,res);
//	while(ptr == NULL){
	memset(buffer,0,1100);
	HAL_UART_Transmit(&huart1,(uint8_t *)command,strlen(command),0xff);
	HAL_UART_Receive(&huart1,(uint8_t *)buffer,1100,2000);
//	ptr = strstr(respond,res);
//	}
	printf(buffer);
	printf("\r\n");
}

void ATCOMMAND_IT(char *command)
{
	HAL_UART_Transmit(&huart1,(uint8_t *)command,strlen(command),0xff);
}

void modem_reset_rtos()
{
	printf("\r\nModem Reboot\r\n");

	SerialATprintln("AT+CREBOOT", 10, 0xff);
	osDelay(4000);
//	for(int y=0;y<40;y++){
//		osDelay(200);
//		if(uart2done)break;
//	}

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

//	char ate[]="ATE0\r\n";
//	char ate_cmp[] = "OK";
//	ATCOMMAND(ate,ate_cmp);

	char cpin[]="AT+CPIN?\r\n";
	char cpin_cmp[] = "+CPIN:";
	ATCOMMAND(cpin,cpin_cmp);

	char csq[]="AT+CSQ\r\n";
	char csq_cmp[] = "+CSQ:";
	ATCOMMAND(csq,csq_cmp);

	char creg[]="AT+CREG?\r\n";
	char creg_cmp[] = "+CREG";
	ATCOMMAND(creg,creg_cmp);

	char cgreg[]="AT+CGREG?\r\n";
	char cgreg_cmp[] = "+CGREG";
	ATCOMMAND(cgreg,cgreg_cmp);

//	char cgatt1[]="AT+CGATT=1\r\n";
//	char cgatt1_cmp[] = "+CGATT";
//	ATCOMMAND(cgatt1,cgatt1_cmp);

	char cgatt[]="AT+CGATT?\r\n";
	char cgatt_cmp[] = "+CGATT:";
	ATCOMMAND(cgatt,cgatt_cmp);

	char cops[]="AT+COPS?\r\n";
	char cops_cmp[] = "+COPS:";
	ATCOMMAND(cops,cops_cmp);

	bearer_config();

	http_request();

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
/*
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
*/
	printf("NBIOT done\r\n");
}

void bearer_config(void)
{
	printf("\r\nBearer setup\r\n");

	memset(uart4_at,0,20);
//	char at[]="AT\r\n";
////	ATCOMMAND_IT(at);
//	ATCOMMAND(at,at);
//	HAL_Delay(1000);

	char cgatt[]="AT+CGATT?\r\n";
//	ATCOMMAND_IT(cgatt);
	ATCOMMAND(cgatt,cgatt);
	HAL_Delay(1000);

	char csq[]="AT+SAPBR=0,1\r\n";
//	ATCOMMAND_IT(csq);
	ATCOMMAND(csq,csq);
	HAL_Delay(500);

	char ate[]="AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r\n";
//	ATCOMMAND_IT(ate);
	ATCOMMAND(ate,ate);
	HAL_Delay(500);

	char cpin[]="AT+SAPBR=3,1,\"APN\",\"nb1internet\"\r\n";
//	ATCOMMAND_IT(cpin);
	ATCOMMAND(cpin,cpin);
	HAL_Delay(500);

	char creg[]="AT+SAPBR=1,1\r\n";
//	ATCOMMAND_IT(creg);
//	ATCOMMAND(creg,creg);
	ATCOMMANDLong(creg,creg);
	HAL_Delay(500);

	char cgreg[]="AT+SAPBR=2,1\r\n";
//	ATCOMMAND_IT(cgreg);
//	ATCOMMAND(cgreg,cgreg);
	ATCOMMANDLong(cgreg,cgreg);
	HAL_Delay(500);

	printf("Bearer Config done\r\n");
}

void http_request(void)
{
	printf("\r\nHTTP Request\r\n");

	memset(uart4_at,0,20);
//	char at[]="AT\r\n";
//	char at_cmp[] = "OK";
////	ATCOMMAND_IT(at);
//	ATCOMMAND(at,at);
//	HAL_Delay(1000);

	char cgatt[]="AT+HTTPINIT\r\n";
	char cgatt_cmp[] = "+CGATT:";
//	ATCOMMAND_IT(cgatt);
	ATCOMMAND(cgatt,cgatt);
	HAL_Delay(500);

	char ate[]="AT+HTTPPARA=\"CID\",1\r\n";
//	ATCOMMAND_IT(ate);
	ATCOMMAND(ate,ate);
	HAL_Delay(1000);

	char cpin[]="AT+HTTPPARA=\"URL\",\"www.flexisolve.com/bs/index.php/sensor/testing\"\r\n";
//	ATCOMMAND_IT(cpin);
	ATCOMMANDLong(cpin,cpin);
//	HAL_Delay(500);

	char csq[]="AT+HTTPACTION=0\r\n";
//	ATCOMMAND_IT(csq);
	ATCOMMANDLong(csq,csq);
	HAL_Delay(3000);

//	sprintf(uart4_at,"AT+HTTPREAD");
	char creg[]="AT+HTTPREAD\r\n";
//	ATCOMMAND_IT(creg);
	ATCOMMANDLong(creg,creg);
	HAL_Delay(2000);

	char cgreg[]="AT+HTTPTERM\r\n";
	//	ATCOMMAND_IT(cgreg);
	ATCOMMAND(cgreg,cgreg);
	HAL_Delay(1000);

	printf("HTTP REQ done\r\n");
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//	int x;
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
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  if(debug==1) printf("START\r\n please wait Modem Starting in 10s\r\n");
//  HAL_UART_Transmit(&huart4, "START\r\n",7,10);
  hard_reset_mdm();
  HAL_UART_Receive(&huart1,buffer_mdm,1100,10000);
  printf(buffer_mdm);
  HAL_Delay(1000);
  memset(buffer_mdm,0,1100);
//  NBIOT_setup();
  //enable uart interrupt
//  HAL_UART_Receive_IT(&huart1,(uint8_t *)uart1_buf,1);
//  HAL_UART_Receive_IT(&huart3,(uint8_t *)uart3_buf,1);
//  HAL_UART_Receive_IT(&huart4,(uint8_t *)uart4_buf,1);
//  HAL_UART_Receive_IT(&huart5,(uint8_t *)uart5_buf,1);
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  HAL_Delay(1000);
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  HAL_Delay(1000);
//  HAL_UART_Transmit(&huart1, "AT\r\n",4,10);
//  HAL_UART_Transmit(&huart3, "AT\r\n",4,10);
//  HAL_UART_Transmit(&huart4, "AT\r\n",4,10);
//  HAL_UART_Transmit(&huart5, "A5\r\n",4,10);
//  if(debug==1) printf("GO\r\n");
//  hard_reset_mdm();
//  NBIOT_setup_IT();
//  MQTT_setup();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//	  if(rxavailable>0){
//		  printf(buffer);
//		  memset(buffer,0,1100);
//		  rxavailable=0;
//	  }
//	  else {
	  memset(uart4_at,0,20);
	  sprintf(uart4_at,"AT+CPSI?\r\n");
//	  printf(uart4_at);
//	  HAL_Delay(500);
//	  printf((uint8_t *)uart4_at);
//	  HAL_Delay(500);
//	  sprintf(uart4_at,"AT+CPSI?");
//	  HAL_UART_Transmit(&huart1, "AT+CPSI?\r\n",10,10);
	  memset(buffer,0,1100);
	  if(HAL_UART_Transmit(&huart1, uart4_at,10,100)==HAL_OK){
		  bugint = HAL_UART_Receive(&huart1,buffer,1100,2000);
	  }
	  printf(buffer);
	  HAL_Delay(1000);
	  memset(buffer,0,1100);
	  if(cnt1==0) NBIOT_setup();
	  else {
		  printf("%d",cnt1);
		 http_request();
		 if(cnt1>=5) cnt1=-1;
	  }
	  cnt1++;
//	  bearer_config();
//	  HAL_Delay(500);
	  HAL_UART_Transmit(&huart3, "AT\r\n",4,10);
	  HAL_UART_Receive(&huart3,buffer,1100,3000);
	  printf(buffer);
	  HAL_Delay(500);
	  memset(buffer,0,1100);
//	  HAL_UART_Transmit(&huart4, "AT\r\n",4,10);
//	  HAL_UART_Receive(&huart4,buffer,1100,1000);
//	  printf(buffer);
//	  HAL_Delay(500);
//	  memset(buffer,0,1100);
	  HAL_Delay(500);
//	  }
//	  switch(rxavailable){
//	  case 1 :	HAL_UART_Transmit(&huart5, (uint8_t *)uart1_data,strlen(uart1_data),500);
//		  	  	memset(uart1_data,0,1100);
//		  	  	rxavailable=0;
//		  	  	break;
//	  case 3 :	HAL_UART_Transmit(&huart5, (uint8_t *)uart3_data,strlen(uart3_data),500);
//		  	  	memset(uart3_data,0,1100);
//		  	  	rxavailable=0;
//		  	  	break;
//	  case 4 :	HAL_UART_Transmit(&huart5, (uint8_t *)uart4_data,strlen(uart4_data),500);
//		  	  	memset(uart4_data,0,1100);
//		  	  	rxavailable=0;
//		  	  	break;
//	  case 5 :	HAL_UART_Transmit(&huart3, (uint8_t *)uart5_data,strlen(uart5_data),500);
//		  	  	memset(uart5_data,0,1100);
//		  	  	rxavailable=0;
//		  	  	break;
//	  default : rxavailable=0;
//	  	  	  	break;
//	  }
//	  printf("%d Hello World!\r\n", ++x);
//	  HAL_UART_Transmit(&huart1, "AT1\r\n",5,10);
////	  HAL_UART_Transmit(&huart2, "AT2\r\n",5,10);
//	  HAL_UART_Transmit(&huart3, "AT\r\n",4,10);
//	  HAL_UART_Transmit(&huart4, "AT4\r\n",5,10);
//	  HAL_UART_Transmit(&huart5, "AT5\r\n",5,10);
//	  HAL_UART_Transmit(&hlpuart1, "ATLP\r\n",6,10);
	  HAL_Delay(1000);
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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 36;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_UART4;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
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
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart1.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  huart1.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart3.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  huart3.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
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
  HAL_GPIO_WritePin(GPIOA, LED_Pin|PWRKEY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin PWRKEY_Pin */
  GPIO_InitStruct.Pin = LED_Pin|PWRKEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
	if(huart == &huart4)
	{
		uart4_temp[k]=uart4_buf[0]; //store byte in temp buffer
		if(k>6){
			if(uart4_temp[k]=='\n' && (uart4_temp[k-3] == 'O' && uart4_temp[k-2] == 'K') || (uart4_temp[k-3] == 'O' && uart4_temp[k-2] == 'R')){
				if(debug==1) printf(uart4_temp);
				rxavailable=4;
				uart4done = true;
				uart4busy = false;
				int cmpint=strcmp(uart4_at,"AT+CPSI?");
				/*
				if(cmpint==0){
					uart4_temp[k] = 0;
					uart4_temp[k-1] = 0;
					uart4_temp[k-2] = 0;
					uart4_temp[k-3] = 0;
					uart4_temp[k-4] = 0;
					uart4_temp[k-5] = 0;
					k=k-5;
					char *token = strtok(uart4_temp, "\n");
					token = strtok(NULL, " ");
					token = strtok(NULL, ",");
					strcpy(networkstr,token);
					strcpy(temp_data2,token);
//					printf("net: %s\n",networkstr);
					if(debug==1)printf(token);
					if(debug==1)printf("\r\n");

					if(k<82 && k>60){
						for(int y=0;y<6;y++){
							token = strtok(NULL, ",");
//								printf(" %s ",token);
						}
						if(debug==1)printf(token);
						if(debug==1)printf("\r\n");
						signal = atoi(token);
//						printf("sig: %d\n",signal);
					}
					else {
						for(int y=0;y<11;y++){
							token = strtok(NULL, ",");
//								printf(" %s ",token);
						}
						if(debug==1)printf(token);
						if(debug==1)printf("\r\n");
						signal = atoi(token);
//						printf("sig: %d\n",signal);
					}
					memset(uart4_at,0,20);
				}*/
				memcpy(buffer_mdm,uart4_temp,strlen(uart4_temp));
				memset(uart4_temp,0,1100);
				k=0;
			}
			/* Receive Network Time Protocol and Sync RTC Time with NTP */
			else if(k>29){
				if(uart4_temp[k-29] == 'C' && uart4_temp[k-28] == 'N' && uart4_temp[k-27] == 'T' && uart4_temp[k-26] == 'P' && uart4_temp[k-25] == ':'  && uart4_temp[k-23] == '1')
					{
					if(debug==1) printf(uart4_temp);
					cntpok=2; // cntp berhasil
						printf("insert rtc date\n");
						printf("%s\n", uart4_temp);
//						set_rtc_time(uart4_temp);
						uart4done = true;
						uart4busy = false;
						memset(uart4_temp,0,1100);
						j=0;
					}
				if(uart4_temp[j-28] == 'C' && uart4_temp[k-27] == 'C' && uart4_temp[k-26] == 'L' && uart4_temp[k-25] == 'K' && uart4_temp[k-24] == ':')
					{
					if(debug==1) printf(uart4_temp);
						cntpok=2; // cntp berhasil
						printf("insert rtc date CCLK\r\n");
						printf("%s\n", uart4_temp);
//						set_rtc_time_cclk(uart4_temp);
						uart4done = true;
						uart4busy = false;
						memset(uart4_temp,0,1100);
						k=0;
					}
				else if(j>1099){
					if(debug==1)printf("%s\r\n",uart4_temp);
					memset(uart4_temp,0,1100);
					k=0;
				}
				else
					k++;
			}
			else if(k>1099){
				if(debug==1)printf("%s\r\n",uart4_temp);
				memset(uart4_temp,0,1100);
				k=0;
			}
			else {
				k++;
			}
		}
		else if(uart4_temp[k] == '>')
		{
//						HAL_UART_Transmit(&huart1, (uint8_t *)uart4_temp, strlen(uart4_temp),0xff);
			if(debug==1)printf("%s\r\n",uart4_temp);
			uart4done = true;
			uart4busy = false;
			memset(uart4_temp,0,1100);
			k=0;
		}
		else if(k>1099){
			if(debug==1)printf("%s\r\n",uart4_temp);
			memset(uart4_temp,0,1100);
			k=0;
		}
		else {
			k++;
		}
		HAL_UART_Receive_IT(&huart4,(uint8_t *)uart4_buf,1);
	}
//	if(huart == &huart5)
//		{
//			uart5_temp[l]=uart5_buf[0]; //store byte in temp buffer
//			if(uart5_temp[l]=='\n'){
//				if(debug==1) printf(uart5_temp);
//				HAL_Delay(10);
//				rxavailable=5;
//				memcpy(uart5_data,uart5_temp,strlen(uart5_temp));
//				HAL_UART_Transmit(&huart4,(uint8_t *)uart5_temp,l+1,200);
//				memset(uart5_temp,0,strlen(uart5_temp));
//				l=0;
//			}
//			else {
//				l++;
//			}
//			HAL_UART_Receive_IT(&huart5,(uint8_t *)uart5_buf,1);
//		}
	/*
	if(huart == &huart3)
	{
		uart3_temp[j]=uart3_buf[0]; //store byte in temp buffer
		if(uart3_temp[j]=='\n'){
			if(debug==1) printf(uart3_temp);
			rxavailable=3;
			HAL_Delay(10);
			memcpy(uart3_data,uart3_temp,strlen(uart3_temp));
//			HAL_UART_Transmit(&huart5,(uint8_t *)uart3_temp,j+1,200);
			memset(uart3_temp,0,strlen(uart3_temp));
			j=0;
		}
		else {
			j++;
		}
		HAL_UART_Receive_IT(&huart3,(uint8_t *)uart3_buf,1);
	}
	if(huart == &huart1)
	{
		uart1_temp[i]=uart1_buf[0]; //store byte in temp buffer
		if(uart1_temp[i]=='\n'){
			if(debug==1) printf(uart1_temp);
			rxavailable=1;
			memcpy(uart1_data,uart1_temp,strlen(uart1_temp));
			HAL_UART_Transmit(&huart1,(uint8_t *)uart1_temp,i+1,20);
			memset(uart1_temp,0,strlen(uart1_temp));
			i=0;
		}
		else {
			i++;
		}
		HAL_UART_Receive_IT(&huart1,(uint8_t *)uart1_buf,1);
	}
*/
	/* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_RxCpltCallback can be implemented in the user file.
   */
}

/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
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
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
