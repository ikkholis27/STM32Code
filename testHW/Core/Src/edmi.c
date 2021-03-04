#include "edmi.h"
#include "edmi_mk6n.h"
#include "edmi_mk10e.h"

//int _write(int file, char *ptr, int len){
//  HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 0xFFFF);
//  return len;
//}

//int net_err_cnt = 0;
//int send_err_cnt = 0;
int state = 0;
bool sent = false;

bool SerialMtrFlag = false;
uint8_t loginreq[] = {0x02,0x4C,0x45,0x44,0x4D,0x49,0x2C,0x49,0x4D,0x44,0x45,0x49,0x4D,0x44,0x45,0x00,0xD9,0x69,0x03};

//// variable for instant
bool onlineMode = true;
uint8_t metertype = 6;
//// Variable Meter Serial Number
uint8_t meterreq[] = {0x02,0x52,0xF0,0x10,0x42,0xEE,0x45,0x03};
uint8_t meterSN[] = {'0','0','0','0','0','0','0','0','0','0'};

int datasize(unsigned char buf[])
{
	int i = 0;
	while(buf[i] != 0x03) i++;
	return i+1;
}

int datasizeChar(char buf[])
{
	int i = 0;
	while(buf[i] != 0x03) i++;
	return i+1;
}

void dataclear(void)
{
	for (int i=0; i < BUFFSIZE; i++)
	{
		data[i] = '\0';
	}
	for (int i=0; i < BUFFSIZE; i++)
	{
		data2[i] = '\0';
	}
	for (int i=0; i < BUFFSIZE; i++)
	{
		datastr[i] = '\0';
	}
}

// Convert Hex 8 uint8_ts to Double
double hex2Double(uint8_t myhex[], int from){
//  double temp;
  ulf.ul = myhex[from];
  ulf.ul = ulf.ul<<8;
  ulf.ul = ulf.ul | myhex[from+1];
  ulf.ul = ulf.ul<<8;
  ulf.ul = ulf.ul | myhex[from+2];
  ulf.ul = ulf.ul<<8;
  ulf.ul = ulf.ul | myhex[from+3];
  ulf.ul = ulf.ul<<8;
  ulf.ul = ulf.ul | myhex[from+4];
  ulf.ul = ulf.ul<<8;
  ulf.ul = ulf.ul | myhex[from+5];
  ulf.ul = ulf.ul<<8;
  ulf.ul = ulf.ul | myhex[from+6];
  ulf.ul = ulf.ul<<8;
  ulf.ul = ulf.ul | myhex[from+7];
  return ulf.f;
}

// Convert Hex 4 uint8_ts to Float
float hex2Float(uint8_t myhex[], int from){
//  double temp;
  myFloat.l = myhex[from];
  myFloat.l = myFloat.l<<8;
  myFloat.l = myFloat.l | myhex[from+1];
  myFloat.l = myFloat.l<<8;
  myFloat.l = myFloat.l | myhex[from+2];
  myFloat.l = myFloat.l<<8;
  myFloat.l = myFloat.l | myhex[from+3];
  return myFloat.f;
}

long hex2Long(uint8_t myhex[], int from){
//  double temp;
  myFloat.l = myhex[from];
  myFloat.l = myFloat.l<<8;
  myFloat.l = myFloat.l | myhex[from+1];
  myFloat.l = myFloat.l<<8;
  myFloat.l = myFloat.l | myhex[from+2];
  myFloat.l = myFloat.l<<8;
  myFloat.l = myFloat.l | myhex[from+3];
  return myFloat.l;
}

unsigned long hex2ULong(uint8_t myhex[], int from){
//  double temp;
  myFloat.l = myhex[from];
  myFloat.l = myFloat.l<<8;
  myFloat.l = myFloat.l | myhex[from+1];
  myFloat.l = myFloat.l<<8;
  myFloat.l = myFloat.l | myhex[from+2];
  myFloat.l = myFloat.l<<8;
  myFloat.l = myFloat.l | myhex[from+3];
  return myFloat.ul;
}

int hex2Int(uint8_t myhex[], int from){
  myInt.i = myhex[from];
  myInt.i = myInt.i<<8;
  myInt.i = myInt.i | myhex[from+1];
  return myInt.i;
}

int hex2IntInv(uint8_t myhex[], int from){
  myInt.i = myhex[from+1];
  myInt.i = myInt.i<<8;
  myInt.i = myInt.i | myhex[from];
  return myInt.i;
}

unsigned int hex2UInt(uint8_t myhex[], int from){
  myInt.i = myhex[from];
  myInt.i = myInt.i<<8;
  myInt.i = myInt.i | myhex[from+1];
  return myInt.ui;
}

unsigned int hex2UIntInv(uint8_t myhex[], int from){
  myInt.i = myhex[from+1];
  myInt.i = myInt.i<<8;
  myInt.i = myInt.i | myhex[from];
  return myInt.ui;
}

void SerialMonprintln(char * ptr){
	uint32_t len=strlen(ptr);
	uint8_t ptr1[len+2];
	memcpy(ptr1, (uint8_t *)ptr, len);
	ptr1[len]= 0x0D;
	ptr1[len+1]= 0x0A;
	  HAL_UART_Transmit(&SerialMon, ptr1, len+2, 3000);
}

void SerialMtrprint(uint8_t * ptr, uint32_t len, uint32_t timeout){
	HAL_UART_Transmit(&SerialMtr, ptr, len, timeout);
}

void SerialMtrread(uint8_t ptr[], uint32_t len, uint32_t timeout){
	  HAL_UART_Receive(&SerialMtr, (uint8_t * )ptr, len, timeout);
}

void printData(unsigned char mydata[], int length){
	for(int i=0;i<length;i++){
		printf("%X ",mydata[i]);
	}
	printf("\r\n");
}

void printDataChar(char mydata[], int length){
	for(int i=0;i<length;i++){
		printf("%X ",mydata[i]);
	}
	printf("\r\n");
}

int loginToMeter(){
	dataclear();
//	SerialMtrprint((uint8_t *)loginreq,sizeof(loginreq),100);
	HAL_UART_Transmit(&SerialMtr, (uint8_t * )loginreq, sizeof(loginreq), 100);
	HAL_UART_Receive(&SerialMtr, (uint8_t * )data, 15, 1000);
	data_len=0;
	data_len=datasize(data);
	printf("datasize: %d\r\n",data_len);
//	printf("0x%X 0x%X 0x%X 0x%X 0x%X ",data[0],data[1],data[2],data[3],data[4]);
//	osDelay(500);
    if(data_len>4){
    	SerialMtrFlag=true;
//    	printf("0x%X 0x%X 0x%X 0x%X 0x%X \r\n",data[0],data[1],data[2],data[3],data[4]);
//    	SerialMonprintln((char *)data);
        return 1;
    }
    return 0;
}

void loginToMeterRTOS(){
	HAL_UART_Transmit(&SerialMtr, (uint8_t * )loginreq, sizeof(loginreq), 100);
	osDelay(1000);
}

void readMeterAllFunctions(int type, int state) {
      switch(type){
        case MK6N:
//            readMeterAllFunctionsMK6N(state);
            break;
        case MK10E:
            //readMeterAllFunctionsMK10E(state);
            break;
        default: break;
      }
}

// ------------------  Functions of MK6N ---------------------------------
