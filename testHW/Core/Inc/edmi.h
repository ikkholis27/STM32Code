/*
 * edmi.h
 *
 *  Created on: Sep 25, 2020
 *      Author: Faizal
 */

#ifndef INC_EDMI_H_
#define INC_EDMI_H_


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l4xx_hal.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
//#include <mdm_sim7070g.h>

/* UART Handler ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

// definitions block
#define SerialMon  	huart1
#define SerialAT  	huart2
#define SerialMtr  	huart3

#define BUFFSIZE  20
#define MK6N      6
#define MK10E     10

// -------------------------------------- EDMI Common Variables ---------------------------------
int net_err_cnt, send_err_cnt, state, int1, int2, int3;
unsigned int data_len;
bool sent;
uint8_t *datastr[BUFFSIZE];
char data[BUFFSIZE];
uint8_t data2[BUFFSIZE];

bool SerialMtrFlag;
unsigned short valuecrc;
uint8_t loginreq[19];
//uint8_t loginreq[] = {0x02,0x4C,0x45,0x44,0x4D,0x49,0x2C,0x49,0x4D,0x44,0x45,0x49,0x4D,0x44,0x45,0x00,0xD9,0x69,0x03};

//// variable for instant
bool onlineMode;
uint8_t metertype;
//// Variable Meter Serial Number
uint8_t meterreq[8];
uint8_t meterSN[11];

//extern osThreadId readEoBHandle;
//extern osThreadId readLPHandle;
//extern osThreadId readInstantHandle;

union ulf
{
    long long ul;
    double f;
} ulf;

union myFloat {
    unsigned long ul;
    long l;
    float f;
} myFloat;

union myInt {
    unsigned int ui;
    int i;
} myInt;

int _write(int file, char *ptr, int len);

int datasize(unsigned char buf[]);
int datasizeChar(char buf[]);
void dataclear(void);

// Convert Hex 8 uint8_ts to Double
double hex2Double(uint8_t myhex[], int from);

// Convert Hex 4 uint8_ts to Float
float hex2Float(uint8_t myhex[], int from);
long hex2Long(uint8_t myhex[], int from);
unsigned long hex2ULong(uint8_t myhex[], int from);
int hex2Int(uint8_t myhex[], int from);
int hex2IntInv(uint8_t myhex[], int from);
unsigned int hex2UInt(uint8_t myhex[], int from);
unsigned int hex2UIntInv(uint8_t myhex[], int from);

void SerialMonprintln(char * ptr);
void SerialMtrprint(uint8_t * ptr, uint32_t len, uint32_t timeout);
void SerialMtrread(uint8_t ptr[], uint32_t len, uint32_t timeout);

void printData(unsigned char mydata[], int length);
void printDataChar(char mydata[], int length);

int loginToMeter();
void loginToMeterRTOS();

void readMeterAllFunctions(int type, int state);


#endif /* INC_EDMI_H_ */
