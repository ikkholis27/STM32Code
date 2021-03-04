#include "stm32l4xx_hal.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include "Crc16.h"
//#include "edmi.h"

#define BUFFSIZEMK6N  1100

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
//
///* USER CODE END Includes */
//// definitions block
#define SerialMon  	huart1
#define SerialAT  	huart2
#define SerialMtr  	huart3

#define MK6N      6
#define MK10E     10

// -------------------------------------- Meter Variables ---------------------------------
//char * d;
char datastrMK6N[1500];
unsigned char dataMK6N[BUFFSIZEMK6N];
char data2MK6N[100];

void dataallclearMK6N(void)
{
	for (int i=0; i < BUFFSIZEMK6N; i++)
	{
		dataMK6N[i] = '\0';
	}
	for (int i=0; i < 100; i++)
	{
		data2MK6N[i] = '\0';
	}
	for (int i=0; i < 1500; i++)
	{
		datastrMK6N[i] = '\0';
	}
}

void datastrclearMK6N(void)
{
	for (int i=0; i < 1500; i++)
	{
		datastrMK6N[i] = '\0';
	}
}

int datasizestrMK6N(char buf[])
{
	int i = 0;
	while(buf[i] != '\0') i++;
	return i+1;
}

void data2clearMK6N(void)
{
	for (int i=0; i < 100; i++)
	{
		data2MK6N[i] = '\0';
	}
}

void SerialMonprintlnMK6N(char * ptr){
	uint32_t len=strlen(ptr);
	uint8_t ptr1[len+2];
	memcpy(ptr1, (uint8_t *)ptr, len);
	ptr1[len]= 0x0D;
	ptr1[len+1]= 0x0A;
	  HAL_UART_Transmit(&SerialMon, ptr1, len+2, 3000);
}

bool SerialMtrFlagMK6N;
unsigned short valuecrcMK6N;

int data_lenMK6N, eoblen;


//uint8_t meterreqMK6N[] = {0x02,0x52,0xF0,0x10,0x42,0xEE,0x45,0x03};
uint8_t typereq[] = {0x02,0x52,0xF0,0x00,0xCE,0x07,0x03};
char timereqMK6N[] = {0x02,0x52,0xF0,0x3D};
//// Variable Meter Serial Number
char meterSNMK6N[10]={'\0','\0','\0','\0','\0','\0','\0','\0','\0','\0'};

// 0xF78x (max. F788)
char eobinit1[]={0x02,0x4D,0x00,0x00,0xFF,0xF0,0x00,0x00,0xF7,0x80,0x00,0x00,0xF7,0x81,0x00,0x00,0xF7,0x82,0x00,0x00,0xF7,0x83,0x00,0x00,0xF7,0x84,0x00,0x00,0xF7,0x85,0x00,0x00,0xF7,0x86,0x00,0x00,0xF7,0x87,0x00,0x00,0xF7,0x88};
char eobinit1x[]={0x02,0x4D,0x00,0x00,0xFF,0xF0,0x00,0x00,0xF7,0x89,0x00,0x00,0xF7,0x8A};
char eobinit2[]={0x02,0x4D,0x00,0x00,0xFF,0xF0,0x00,0x00,0xF7,0xD0,0x00,0x00,0xF7,0xD1,0x00,0x00,0xF7,0xD2,0x00,0x00,0xF7,0xD3,0x00,0x00,0xF7,0xD4,0x00,0x00,0xF7,0xD5,0x00,0x00,0xF7,0xD6,0x00,0x00,0xF7,0xD7,0x00,0x00,0xF7,0xD8};
char eobinit2x[]={0x02,0x4D,0x00,0x00,0xFF,0xF0,0x00,0x00,0xF7,0xD9,0x00,0x00,0xF7,0xDA,0x00,0x00,0xF7,0xDB};
char eobuser0[]={0x02,0x4D,0x00,0x00,0xFF,0xF0,0x00,0x00,0x00,0x40,0x00,0x00,0x00,0x41,0x00,0x00,0x00,0x42,0x00,0x00,0x00,0x43,0x00,0x00,0x00,0x44,0x00,0x00,0x00,0x49};
char instant[]={0x02,0x4D,0x00,0x00,0xFF,0xF0,0x00,0x00,0xE0,0x00,0x00,0x00,0xE0,0x10,0x00,0x00,0xE0,0x20,0x00,0x00,0xE0,0x30,0x00,0x00,0xE0,0x40,0x00,0x00,0xE0,0x50,0x00,0x00,0xE0,0x60,0x00,0x00,0xE0,0x25};
char lpinit1[]={0x03,0x05,0xF0,0x20,0x03,0x05,0xF0,0x14,0x03,0x05,0xF0,0x21,0x03,0x05,0xF0,0x13,0x03,0x05,0xF0,0x12};
char lpinit2[]={0x03,0x05,0xE0,0x00};
char lpinit2a[]={0x03,0x05,0xE8,0x00,0x03,0x05,0xE2,0x00};
//uint8_t lpinit2ori[]={0x03,0x05,0xE4,0x00,0x03,0x05,0xE8,0x00,0x03,0x05,0xE2,0x00,0x03,0x05,0xE1,0x00};

float instantA[8],instantB[8],instantC[8];

// variable for eob
uint8_t regeobinit1[11];
uint8_t eoblenMK6N=0;
double eob0MK6N[11],eob1MK6N[11],eob2MK6N[11],eob3MK6N[11],eob4MK6N[11],eob9MK6N[11];

// variable for lp
uint8_t reglpinit1[11];
//uint8_t reglpinit1[11];
float lpscaleMK6N[15];
uint8_t jmlchannelMK6N;
unsigned long lpintervalMK6N;
uint8_t lplenMK6N=0, jmlchannelMK6N, dateThn,dateBln,dateTgl,dateJam,dateMnt,dateDtk;
long lpfirstentrydateMK6N;
long lpfirstentryMK6N, lplastentryMK6N,lpmaxentryMK6N,lprecordMK6N;

uint8_t lptype[15];
double lpdata[14];
long lpdatalong[14];

unsigned long lpdataulong[14];

//extern union myFloat myFloat;
//extern union myInt myint;

// Function Prototype
bool getRawDataMK6N();
void removeheadcrctailMK6N(int headcmd);
void removePaddingMK6N();
void setPaddingMK6N();

bool decodeEoB();
bool decodeInstant();
int decodelpdata();
int decodelpinit1MK6N(unsigned char datalpinit[]);

const char * checkregEoBInit1(uint8_t reg);
const char * checkregEoBRate1(uint8_t reg);
const char * checkregEoBRate2(uint8_t reg);
const char * checkregEoBRate3(uint8_t reg);
const char * checkregEoBRate4(uint8_t reg);
const char * checkregEoBRate5(uint8_t reg);
const char * checkregEoBRate9(uint8_t reg);

const char * checkregInstant(uint8_t reg);
const char * checkregLPMK6N(uint8_t reg);

void lp2JsonMK6N();
void saveEoBInit1();

int readlpInit1MK6N();

//time_t time2TimestampNowMK6N(int dtk, int mnt, int jam, int tgl, int bln, int thn){
//  tmElements_t myElements = {dtk, mnt, jam, weekday(), tgl, bln, thn-1970 };
//  time_t myTime = makeTime(myElements);
//  return myTime;
//}
//
//time_t time2TimestampMK6N(uint8_t dtk, uint8_t mnt, uint8_t jam, uint8_t tgl, uint8_t bln, uint8_t thn){
//  tmElements_t myElements = {dtk, mnt, jam, weekday(), tgl, bln, thn+30 };
//  time_t myTime = makeTime(myElements);
//  return myTime;
//}
//
//void setTimeMdmMK6N(uint8_t dtk, uint8_t mnt, uint8_t jam, uint8_t tgl, uint8_t bln, uint8_t thn){
//  tmElements_t myElements = {dtk, mnt, jam, weekday(), tgl, bln, thn+30 };
//  time_t pctime = makeTime(myElements);
//  setTime(pctime); // Sync Arduino clock to the time received on the serial port
//}


void setPaddingMK6N(){
  unsigned int lentemp=data_lenMK6N, idx=0;
  uint8_t datatemp[data_lenMK6N];
  for(unsigned int i=0;i<data_lenMK6N;i++){
    datatemp[i]=data2MK6N[i];
  }
  for(unsigned int i=0;i<lentemp;i++){
    if(i==0){
      data2MK6N[idx]=0x02;
    }
    else if(i==lentemp-1){
      data2MK6N[idx]=0x03;
    }
    else if((datatemp[i]==0x13 && (i<lentemp-1 && i>0)) || datatemp[i]==0x02 || datatemp[i]==0x10 || datatemp[i]==0x11){
      data2MK6N[idx++]=0x10;
      data2MK6N[idx]=datatemp[i]|0x40;
    }
    else if(i<lentemp-1 && datatemp[i]==0x03){
      data2MK6N[idx++]=0x10;
      data2MK6N[idx]=datatemp[i]|0x40;
    }
    else    data2MK6N[idx]=datatemp[i];
    idx++;
  }
  data_lenMK6N = idx;
}

void removePaddingMK6N(){
  unsigned int lentemp=data_lenMK6N;
//  char mydata[lentemp];
//  memcpy(mydata,dataMK6N,data_lenMK6N);
//  dataallclearMK6N();
  for(unsigned int i=0;i<data_lenMK6N;i++){
    if(dataMK6N[i]==0x10){
      dataMK6N[i]=dataMK6N[i+1]^0x40;
      for(unsigned int j=i+1;j<data_lenMK6N-1;j++){
    	  dataMK6N[j]=dataMK6N[j+1];
      }
      lentemp--;
    }
  }
  data_lenMK6N = lentemp;
}

bool getRawDataMK6N(int headcmd){
    clearCrc();
    data_lenMK6N = datasize(dataMK6N);
    removePaddingMK6N();
    removeheadcrctailMK6N(headcmd);
//    valuecrc = XModemCrc(dataMK10E,0,data_lenMK10E-3);
//    if(checkcrcMK10E()){
//      SerialMonprintln("RemoveHeadCRCTail");
//      printData(dataMK10E,data_lenMK10E);
//      osDelay(100);
      return true;
//    }
    return true;
}

int setdatastr(uint8_t * ptr, int len){
	for (int i=data_lenMK6N-1; i < sizeof(ptr); i++)
	{
		datastr[i] = ptr++;
	}
	return len+sizeof(ptr);
}

void removeheadcrctailMK6N(int sizeheadcmd){
  // remove CRC and Tail
  dataMK6N[data_lenMK6N-1]='\0';
  dataMK6N[data_lenMK6N-2]='\0';
  dataMK6N[data_lenMK6N-3]='\0';
  data_lenMK6N=data_lenMK6N-3;
  //remove head
  for(unsigned int i=0;i<data_lenMK6N;i++){
    if(i+sizeheadcmd<=BUFFSIZEMK6N){
      dataMK6N[i]=dataMK6N[i+sizeheadcmd];
    }
  }
  // update data_lenMK6N
  data_lenMK6N=data_lenMK6N-sizeheadcmd;
}

void saveSNMK6N(){
  if(data_lenMK6N>11) getRawDataMK6N(4);
//  printData(dataMK6N,data_lenMK6N);
  for(int i=0;i<data_lenMK6N;i++){
    meterSNMK6N[i] = dataMK6N[i];
  }
//  printf("SN: %s\r\n",meterSNMK6N);
}

const char * readSNMK6N(){
	dataallclearMK6N();
	HAL_UART_Transmit(&SerialMtr, (uint8_t * )meterreq, sizeof(meterreq), 100);
	HAL_UART_Receive(&SerialMtr, (uint8_t * )dataMK6N, 100, 1500);
	data_lenMK6N=0;
	data_lenMK6N=datasize(dataMK6N);
//	printf("datasize: %d 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X\r\n",data_lenMK6N,dataMK6N[0],dataMK6N[1],dataMK6N[2],dataMK6N[3],dataMK6N[4],dataMK6N[5],dataMK6N[6],dataMK6N[7]);
//	printf("0x%X 0x%X 0x%X 0x%X 0x%X ",dataMK6N[5],dataMK6N[6],dataMK6N[7],dataMK6N[8],dataMK6N[9]);
    if(data_lenMK6N>4){
    	SerialMtrFlag=true;
    	saveSNMK6N();
//        return 1;
    }
    return meterSNMK6N;
}

void setSNMK6N(char * datain, int len){
	dataallclearMK6N();
	data_lenMK6N=len;
	memcpy(meterSNMK6N,datain, len);
}

void setCommandMK6N(char array[], uint16_t sizearr){
  for(int i=0;i<sizearr;i++){
    data2MK6N[i]=array[i];
  }
  data_lenMK6N = sizearr;
  clearCrc();
   valuecrc = XModemCrc(array,0,sizearr);
   data2MK6N[sizearr+2]=0x03;
   data2MK6N[sizearr+1]= (uint8_t) valuecrc;
   valuecrc  = valuecrc >> 8;
   data2MK6N[sizearr]  = (uint8_t)valuecrc;
   data_lenMK6N = sizearr+3;
   setPaddingMK6N();
}

int readTimeMK6N(){
    setCommandMK6N(timereqMK6N, sizeof(timereqMK6N));
//    osDelay(500);
	HAL_UART_Transmit(&huart3, (uint8_t * )data2MK6N, data_lenMK6N, 200);
	HAL_UART_Receive(&huart3, (uint8_t * )dataMK6N, BUFFSIZEMK6N, 3000);
    if(getRawDataMK6N(4)){
      dateTgl = dataMK6N[0];
      dateBln = dataMK6N[1];
      dateThn = dataMK6N[2];
      dateJam = dataMK6N[3];
      dateMnt = dataMK6N[4];
      dateDtk = dataMK6N[5];
//      setTimeMdmMK6N(dateDtk, dateMnt, dateJam, dateTgl, dateBln, dateThn);
      return 1;
    }
    return 0;
}

// -------------------------- Instant ------------------------
void readInstantMK6N(char* outstr){
  //reading 00000400 eob rate1,2,3,4,5,unified for user0-8
	dataallclearMK6N();
  char mychar[sizeof(instant)];
  for(int i=0;i<3;i++){
    for(int j=0;j<sizeof(instant);j++){
      mychar[j]=instant[j];
      if(j==9 || j==13 || j==17 || j==21 || j==25 || j==29 || j==33){
        mychar[j]=instant[j] | i;      
      }
    }
    setCommandMK6N(mychar, sizeof(mychar));
//    printData(data2MK6N, data_lenMK6N);
//	printf("datasize: %d 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X\r\n",data_lenMK6N,data2MK6N[0],data2MK6N[1],data2MK6N[2],data2MK6N[3],data2MK6N[4],data2MK6N[5],data2MK6N[6],data2MK6N[7]);
	HAL_UART_Transmit(&huart3, (uint8_t * )data2MK6N, data_lenMK6N, 200);
	HAL_UART_Receive(&huart3, (uint8_t * )dataMK6N, BUFFSIZEMK6N, 2000);

	data_lenMK6N=0;
	data_lenMK6N=datasize(dataMK6N);
    if(getRawDataMK6N(6)){
      eoblenMK6N=9;
      for(int k=0;k<8;k++){
        hex2Float(dataMK6N,4*k);
        if(i==0) instantA[k]=myFloat.f;
        else if(i==1) instantB[k]=myFloat.f;
        else instantC[k]=myFloat.f;
    	printf(" %.5f ",myFloat.f);
      }
      printf(" \r\n");
    }
//    osDelay(100);
  }
  if(decodeInstant()) {
	  for(int i=0;i<datasizestrMK6N(datastrMK6N);i++){
		  *outstr++=datastrMK6N[i];
	  }
  }
}

void txInstantMK6N(int i){
  //reading 00000400 eob rate1,2,3,4,5,unified for user0-8
	dataallclearMK6N();
  char mychar[sizeof(instant)];
//  for(int i=0;i<3;i++){
    for(int j=0;j<sizeof(instant);j++){
      mychar[j]=instant[j];
      if(j==9 || j==13 || j==17 || j==21 || j==25 || j==29 || j==33){
        mychar[j]=instant[j] | i;
      }
    }
    setCommandMK6N(mychar, sizeof(mychar));
	HAL_UART_Transmit(&huart3, (uint8_t * )data2MK6N, data_lenMK6N, 200);
}

void parsingDataRawInstantMK6N(unsigned char buf[],int len,int i){
  //reading 00000400 eob rate1,2,3,4,5,unified for user0-8
	dataallclearMK6N();
	memcpy(dataMK6N,buf,len);
	data_lenMK6N=0;
	data_lenMK6N=datasize(dataMK6N);
    if(getRawDataMK6N(6)){
//      eoblenMK6N=9;
      for(int k=0;k<8;k++){
        hex2Float(dataMK6N,4*k);
        if(i==0) instantA[k]=myFloat.f;
        else if(i==1) instantB[k]=myFloat.f;
        else instantC[k]=myFloat.f;
//    	printf(" %.5f ",myFloat.f);
      }
//      printf(" \r\n");
    }
}

void decodeJsonInstantMK6N(char * outstr){
	  if(decodeInstant()) {
		  for(int i=0;i<datasizestrMK6N(datastrMK6N);i++){
			  *outstr++=datastrMK6N[i];
		  }
	  }
}

const char * checkregInstant(uint8_t reg){
  switch(reg){
    case 0x00:
            return "\"3P59\"";
            break;
    case 0x01:
            return "\"3P60\"";
            break;
    case 0x02:
            return "\"3P61\"";
            break;
    case 0x10:
            return "\"3P09\"";
            break;
    case 0x11:
            return "\"3P10\"";
            break;
    case 0x12:
            return "\"3P11\"";
            break;
    case 0x20:
            return "\"3P62\""; // phase angle A
            break;
    case 0x21:
            return "\"3P63\""; // phase angle B
            break;
    case 0x22:
            return "\"3P64\""; // phase angle C
            break;
    case 0x30:
            return "\"3P01\""; // phase A Watts
            break;
    case 0x31:
            return "\"3P02\""; // phase B Watts
            break;
    case 0x32:
            return "\"3P03\""; // phase C Watts
            break;
    case 0x40:
            return "\"3P43\""; // phase A Vars
            break;
    case 0x41:
            return "\"3P44\""; // phase B Vars
            break;
    case 0x42:
            return "\"3P45\""; // phase C Vars
            break;
    case 0x50:
            return "\"3P05\""; // Total Apparent Power of A
            break;
    case 0x51:
            return "\"3P06\""; // Total Apparent Power of B
            break;
    case 0x52:
            return "\"3P07\""; // Total Apparent Power of C
            break;
    case 0x60:
            return "\"3P13\""; // Freq
            break;
    case 0x25:
            return "\"3P42\""; 	// PF
            break;
    default:
    		return "\"3P42\""; 	// PF
            break;
  }
}


bool decodeInstant(){
//  printData(data, data_lenMK6N);
	data_lenMK6N = 0;
	datastrclearMK6N();
//	printf("decodeInstant begin \r\n");

	strcpy(datastrMK6N,"{\"mid\":");
	strcat(datastrMK6N, meterSNMK6N);
	strcat(datastrMK6N, ",\"pid\":\"InstantMK6N\",");

//	  data2clearMK10E();
//	  sprintf(data2MK10E,"%d,",timestamp_now);
//	  strcat(datastrMK6N, data2MK10E);
//  datastr+=String(time2TimestampNowMK6N(second(),minute(),hour(),day(),month(),year()));
//  datastr+="123456789,";
  for(int i=0;i<8;i++){
    if(i==7){
    	strcat(datastrMK6N, checkregInstant(0x25));
    	strcat(datastrMK6N, ":");
    	data2clearMK6N();
    	sprintf(data2MK6N,"%.5f,",instantA[i]);
    	strcat(datastrMK6N, data2MK6N);
    	strcat(datastrMK6N, checkregInstant(0x25));
    	strcat(datastrMK6N, ":");
    	data2clearMK6N();
    	sprintf(data2MK6N,"%.5f,",instantB[i]);
    	strcat(datastrMK6N, data2MK6N);
    	strcat(datastrMK6N, checkregInstant(0x25));
    	strcat(datastrMK6N, ":");
    	data2clearMK6N();
    	sprintf(data2MK6N,"%.5f}",instantC[i]);
    	strcat(datastrMK6N, data2MK6N);
//      datastr+="}";
    }
    else {
    	strcat(datastrMK6N, checkregInstant((uint8_t)i<<4));
    	strcat(datastrMK6N, ":");
    	data2clearMK6N();
    	sprintf(data2MK6N,"%.5f,",instantA[i]);
    	strcat(datastrMK6N, data2MK6N);
    	strcat(datastrMK6N, checkregInstant((uint8_t)i<<4 | 0x01));
    	strcat(datastrMK6N, ":");
    	data2clearMK6N();
    	sprintf(data2MK6N,"%.5f,",instantB[i]);
    	strcat(datastrMK6N, data2MK6N);
    	strcat(datastrMK6N, checkregInstant((uint8_t)i<<4 | 0x02));
    	strcat(datastrMK6N, ":");
    	data2clearMK6N();
    	sprintf(data2MK6N,"%.5f,",instantC[i]);
    	strcat(datastrMK6N, data2MK6N);
    }
  }
//  SerialMonprintlnMK6N(datastrMK6N);
    return true;
}

// ----------------------- LP --------------------------------
const char * checkregLPMK6N(uint8_t reg){
  switch(reg){
    case 0x00:
            return "\"3P59\"";
            break;
    case 0x01:
            return "\"3P60\"";
            break;
    case 0x02:
            return "\"3P61\"";
            break;
    case 0x03:
            return "\"3P09\"";
            break;
    case 0x04:
            return "\"3P10\"";
            break;
    case 0x05:
            return "\"3P11\"";
            break;
    case 0x06:
            return "\"3P62\""; // phase angle A
            break;
    case 0x07:
            return "\"3P63\""; // phase angle B
            break;
    case 0x08:
            return "\"3P64\""; // phase angle C
            break;
    case 0x09:
            return "\"3P01\""; // phase A Watts
            break;
    case 0x0A:
            return "\"3P02\""; // phase B Watts
            break;
    case 0x0B:
            return "\"3P03\""; // phase C Watts
            break;
    case 0x0C:
            return "\"3P43\""; // phase A Vars
            break;
    case 0x0D:
            return "\"3P44\""; // phase B Vars
            break;
    case 0x0E:
            return "\"3P45\""; // phase C Vars
            break;
    case 0x0F:
            return "\"3P05\""; // Total Apparent Power of A
            break;
    case 0x51:
            return "\"3P06\""; // Total Apparent Power of B
            break;
    case 0x52:
            return "\"3P07\""; // Total Apparent Power of C
            break;
    case 0x60:
            return "\"3P13\""; // Freq
            break;
    case 0x25:
            return "\"3P42\"";   // PF
            break;
    default:
            return "\"3P42\"";   // PF
            break;
  }
}

int decodelpdata(){
//    uint8_t mylpdata[data_lenMK6N+1];

  for(int i=0;i<jmlchannelMK6N;i++){
    hex2Long(dataMK6N,i*4+2);
    lpdatalong[i]=myFloat.l;
    lpdataulong[i]=myFloat.ul;
    if(lptype[i+1]==0x4F){
      lpdata[i]=(double) lpdatalong[i]*lpscaleMK6N[i+1];    
    }
    else if(lptype[i+1]==0x46){
      hex2Float(dataMK6N,i*4+2);
      lpdata[i]=(double) myFloat.f*lpscaleMK6N[i+1];
    }
//    printf("%d %.5f %.5f\r\n",lpdatalong[i],lpscaleMK6N[i+1],lpdata[i]);
  }
  return 1;
}

void readlpMK6N(int indexmin1, char* outstr){
	  dataallclearMK6N();
  //reading 00000400 lp rate1,2,3,4,5,unified for user0-8
  unsigned long lpindex = lplastentryMK6N-indexmin1;
  char mydata[17];
  mydata[0]=0x02;
  mydata[1]=0x46;
  mydata[2]=0x52;
  mydata[3]=0x03;
  mydata[4]=0x05;
  mydata[5]=0xF0;
  mydata[6]=0x08;
  if(lpindex<65536){
  mydata[7]=0;
  mydata[8]=0;
  mydata[9]=lpindex>>8;	
  mydata[10]=(uint8_t)lpindex;
  }
  else {
  mydata[7]=lpindex>>24;
  mydata[8]=lpindex>>16;
  mydata[9]=lpindex>>8;
  mydata[10]=(uint8_t)lpindex;
  }
  mydata[11]=0x00;
  mydata[12]=0x01;
  mydata[13]=0x00;
  mydata[14]=0x00;
  mydata[15]=0x05;
  mydata[16]=0xDC;
  data_lenMK6N = 17;
  setCommandMK6N(mydata, data_lenMK6N);
//      printData(data2MK6N,data_lenMK6N);
  	HAL_UART_Transmit(&huart3, (uint8_t * )data2MK6N, data_lenMK6N, 500);
  	HAL_UART_Receive(&huart3, (uint8_t * )dataMK6N, BUFFSIZEMK6N, 3000);
  	data_lenMK6N=datasize(dataMK6N);
//      printData(dataMK6N,data_lenMK6N);
  if(getRawDataMK6N(17)){
//      printData(dataMK6N,data_lenMK6N);
      if(decodelpdata())   {
    	  lp2JsonMK6N();
//    	  return (char * )(datastrMK6N);
//    	  char *d = datastrMK6N;
    	  for(int i=0;i<datasizestrMK6N(datastrMK6N);i++){
    		  *outstr++=datastrMK6N[i];
    	  }
//    	  return d;
      }
  }
//  return "{\"nodata\":0}";
}

void txlpMK6NRec(int lp_index){
	dataallclearMK6N();
  //reading 00000400 lp rate1,2,3,4,5,unified for user0-8
  unsigned long lpindex = lp_index;
  lprecordMK6N = lpindex;
  char mydata[17];
  mydata[0]=0x02;
  mydata[1]=0x46;
  mydata[2]=0x52;
  mydata[3]=0x03;
  mydata[4]=0x05;
  mydata[5]=0xF0;
  mydata[6]=0x08;
  if(lpindex<65536){
  mydata[7]=0;
  mydata[8]=0;
  mydata[9]=lpindex>>8;
  mydata[10]=(uint8_t)lpindex;
  }
  else {
  mydata[7]=lpindex>>24;
  mydata[8]=lpindex>>16;
  mydata[9]=lpindex>>8;
  mydata[10]=(uint8_t)lpindex;
  }
  mydata[11]=0x00;
  mydata[12]=0x01;
  mydata[13]=0x00;
  mydata[14]=0x00;
  mydata[15]=0x05;
  mydata[16]=0xDC;
  data_lenMK6N = 17;
  setCommandMK6N(mydata, data_lenMK6N);
  	HAL_UART_Transmit(&huart3, (uint8_t * )data2MK6N, data_lenMK6N, 500);
}

void txlpMK6N(int indexmin1){
	dataallclearMK6N();
  //reading 00000400 lp rate1,2,3,4,5,unified for user0-8
  unsigned long lpindex = lplastentryMK6N-indexmin1;
  lprecordMK6N = lpindex;
  char mydata[17];
  mydata[0]=0x02;
  mydata[1]=0x46;
  mydata[2]=0x52;
  mydata[3]=0x03;
  mydata[4]=0x05;
  mydata[5]=0xF0;
  mydata[6]=0x08;
  if(lpindex<65536){
  mydata[7]=0;
  mydata[8]=0;
  mydata[9]=lpindex>>8;
  mydata[10]=(uint8_t)lpindex;
  }
  else {
  mydata[7]=lpindex>>24;
  mydata[8]=lpindex>>16;
  mydata[9]=lpindex>>8;
  mydata[10]=(uint8_t)lpindex;
  }
  mydata[11]=0x00;
  mydata[12]=0x01;
  mydata[13]=0x00;
  mydata[14]=0x00;
  mydata[15]=0x05;
  mydata[16]=0xDC;
  data_lenMK6N = 17;
  setCommandMK6N(mydata, data_lenMK6N);
  	HAL_UART_Transmit(&huart3, (uint8_t * )data2MK6N, data_lenMK6N, 500);
}

void rxlpMK6N(char* outstr,char* datain, int len){
	memcpy(dataMK6N,datain,len);
  	data_lenMK6N=datasize(dataMK6N);
  if(getRawDataMK6N(17)){
      if(decodelpdata())   {
    	  lp2JsonMK6N();
    	  for(int i=0;i<datasizestrMK6N(datastrMK6N);i++){
    		  *outstr++=datastrMK6N[i];
    	  }
      }
  }
}

void lp2JsonMK6N(){
//  printData(data, data_lenMK6N);
	data_lenMK6N = 0;
	datastrclearMK6N();
	strcpy(datastrMK6N,"{\"mid\":");
	  strcat(datastrMK6N, meterSNMK6N);
	  strcat(datastrMK6N, ",\"pid\":\"LPMK6N\",");
//  data_lenMK6N = setdatastr(",\"pid\":\"LPMK6N\",\"readdate\":",data_lenMK6N);
//  datastr+=String(time2TimestampNowMK6N(second(),minute(),hour(),day(),month(),year()));
	  strcat(datastrMK6N, "\"3P9B\":");

  	data2clearMK6N();
  	sprintf(data2MK6N,"%ld,",lprecordMK6N);
  	strcat(datastrMK6N, data2MK6N);
//  datastr+=",\"3P9B\":";
//  datastr+=String(lplastentry-1);
//  datastr+=",";
  for(int i=0;i<jmlchannelMK6N;i++){
	strcat(datastrMK6N, checkregLPMK6N(reglpinit1[i]));
	strcat(datastrMK6N, ":");
    data2clearMK6N();
    if(i<jmlchannelMK6N-1){
      	sprintf(data2MK6N,"%.5f,",lpdata[i]);
      	strcat(datastrMK6N, data2MK6N);
    }
    else {
      	sprintf(data2MK6N,"%.5f}",lpdata[i]);
      	strcat(datastrMK6N, data2MK6N);
    }
  }
//  SerialMonprintlnMK6N(datastrMK6N);
}

int readlpInit1MK6N(){
	dataallclearMK6N();
  data2MK6N[0]=0x02;
  data2MK6N[1]=0x4D;
  data2MK6N[2]=0x00;
  data2MK6N[3]=0x00;
  data2MK6N[4]=0xFF;
  data2MK6N[5]=0xF0;
    for(int j=0;j<sizeof(lpinit1);j++){
      data2MK6N[j+6]=lpinit1[j];
    }
    data_lenMK6N=sizeof(lpinit1)+6;
    setCommandMK6N(data2MK6N, data_lenMK6N);
//    printData(data2MK6N,data_lenMK6N);
	HAL_UART_Transmit(&huart3, (uint8_t * )data2MK6N, data_lenMK6N, 500);
	HAL_UART_Receive(&huart3, (uint8_t * )dataMK6N, BUFFSIZEMK6N, 2000);
	data_lenMK6N=datasize(dataMK6N);
//    printData(dataMK6N,data_lenMK6N);
    if(getRawDataMK6N(6)){
//    	printf("datalen: %d \r\n",data_lenMK6N);
//      printData(dataMK6N,data_lenMK6N);
      decodelpinit1MK6N(dataMK6N);
//      return 1;
    }
    return 0;
}

void txlpInit1MK6N(){
	dataallclearMK6N();
  data2MK6N[0]=0x02;
  data2MK6N[1]=0x4D;
  data2MK6N[2]=0x00;
  data2MK6N[3]=0x00;
  data2MK6N[4]=0xFF;
  data2MK6N[5]=0xF0;
    for(int j=0;j<sizeof(lpinit1);j++){
      data2MK6N[j+6]=lpinit1[j];
    }
    data_lenMK6N=sizeof(lpinit1)+6;
    setCommandMK6N(data2MK6N, data_lenMK6N);
	HAL_UART_Transmit(&huart3, (uint8_t * )data2MK6N, data_lenMK6N, 500);
}

void rxlpInit1MK6N(char* datain, int len){
	memcpy(dataMK6N,datain,len);
	data_lenMK6N=0;
	data_lenMK6N=datasize(dataMK6N);
    if(getRawDataMK6N(6)){
      decodelpinit1MK6N(dataMK6N);
    }
}

int decodelpinit1MK6N(unsigned char datalpinit[]){
  dateTgl = datalpinit[0];
  dateBln = datalpinit[1];
  dateThn = datalpinit[2];
  dateJam = datalpinit[3];
  dateMnt = datalpinit[4];
  dateDtk = datalpinit[5];
  lpintervalMK6N = hex2ULong(dataMK6N,6);
  lplastentryMK6N = hex2Long(dataMK6N,10);
  lpmaxentryMK6N = hex2Long(dataMK6N,14);
  jmlchannelMK6N = dataMK6N[data_lenMK6N-1];
  printf("%d %d %d %d %d %d\r\n", dateTgl,dateBln,dateThn,dateJam,dateMnt,dateDtk);
  printf("%ld %ld %ld %d\r\n", lpintervalMK6N,lplastentryMK6N,lpmaxentryMK6N,jmlchannelMK6N);
  return 1;
}

int readlpMK6NInit2(){
	int idxnow;
	data2MK6N[0]=0x02;
  data2MK6N[1]=0x4D;
  data2MK6N[2]=0x00;
  data2MK6N[3]=0x00;
  data2MK6N[4]=0xFF;
  data2MK6N[5]=0xF0;
  idxnow = 6;
  for(int i=0;i<8;i++){
    for(int j=0;j<sizeof(lpinit2);j++){
      data2MK6N[idxnow]=lpinit2[j];
      if(j==3) data2MK6N[idxnow]=lpinit2[j] | i;
	  idxnow++;
    }
  }
    data_lenMK6N=idxnow;
    setCommandMK6N(data2MK6N, data_lenMK6N);
//	printData(data2MK6N,data_lenMK6N);
	HAL_UART_Transmit(&huart3, (uint8_t * )data2MK6N, data_lenMK6N, 500);
	HAL_UART_Receive(&huart3, (uint8_t * )dataMK6N, BUFFSIZEMK6N, 2000);
	data_lenMK6N=datasize(dataMK6N);
//    printData(dataMK6N,data_lenMK6N);
    if(getRawDataMK6N(6)){
//		printData(dataMK6N,data_lenMK6N);
    	for(int i=0;i<8;i++){
    		reglpinit1[i]=dataMK6N[4*i+3];
    		printf("0x%X ",reglpinit1[i]);
    	}
    	printf("\r\n");
    }
	
  return 1;
}

void txlpMK6NInit2(){
	dataallclearMK6N();
	int idxnow;
	data2MK6N[0]=0x02;
  data2MK6N[1]=0x4D;
  data2MK6N[2]=0x00;
  data2MK6N[3]=0x00;
  data2MK6N[4]=0xFF;
  data2MK6N[5]=0xF0;
  idxnow = 6;
  for(int i=0;i<8;i++){
    for(int j=0;j<sizeof(lpinit2);j++){
      data2MK6N[idxnow]=lpinit2[j];
      if(j==3) data2MK6N[idxnow]=lpinit2[j] | i;
	  idxnow++;
    }
  }
    data_lenMK6N=idxnow;
    setCommandMK6N(data2MK6N, data_lenMK6N);
	HAL_UART_Transmit(&huart3, (uint8_t * )data2MK6N, data_lenMK6N, 500);
}

void rxlpMK6NInit2(char* datain, int len){
	memcpy(dataMK6N,datain,len);
	data_lenMK6N=0;
	data_lenMK6N=datasize(dataMK6N);
//    printData(dataMK6N,data_lenMK6N);
    if(getRawDataMK6N(6)){
//		printData(dataMK6N,data_lenMK6N);
    	for(int i=0;i<8;i++){
    		reglpinit1[i]=dataMK6N[4*i+3];
//    		printf("0x%X ",reglpinit1[i]);
    	}
//    	printf("\r\n");
    }
}

// Scaling and Type of Data LP
int readlpMK6NInit2a(){
  int idxnow;
  for(int i=0;i<jmlchannelMK6N;i++){
  data[0]=0x02;
  data[1]=0x4D;
  data[2]=0x00;
  data[3]=0x00;
  data[4]=0xFF;
  data[5]=0xF0;
    for(int j=0;j<sizeof(lpinit2a);j++){
      idxnow = j+6;
      data[idxnow]=lpinit2a[j];
      if(j==3 || j==7){        
        data[idxnow]=lpinit2a[j] | i;
      }
    }
    data_lenMK6N=sizeof(lpinit2a)+6;
    setCommandMK6N(data, data_lenMK6N);
//	printData(data2MK6N,data_lenMK6N);
	HAL_UART_Transmit(&huart3, (uint8_t * )data2MK6N, data_lenMK6N, 500);
	HAL_UART_Receive(&huart3, (uint8_t * )dataMK6N, BUFFSIZEMK6N, 2500);
	data_lenMK6N=datasize(dataMK6N);
//    printData(dataMK6N,data_lenMK6N);
    if(getRawDataMK6N(6)){
//    printData(dataMK6N,data_lenMK6N);
      hex2Float(dataMK6N,0);
      lpscaleMK6N[i]=myFloat.f;
      lptype[i]=dataMK6N[data_lenMK6N-1];
    }
  }
//  for(int i=0;i<jmlchannelMK6N;i++){
//      printf("0x%X %.7f\r\n",lptype[i],lpscaleMK6N[i]);
//  }
  return 1;
}
int getJumlahChannelMK6N(){
	return jmlchannelMK6N;
}
void txlpMK6NInit2a(int i){
  int idxnow;
  data[0]=0x02;
  data[1]=0x4D;
  data[2]=0x00;
  data[3]=0x00;
  data[4]=0xFF;
  data[5]=0xF0;
    for(int j=0;j<sizeof(lpinit2a);j++){
      idxnow = j+6;
      data[idxnow]=lpinit2a[j];
      if(j==3 || j==7){
        data[idxnow]=lpinit2a[j] | i;
      }
    }
    data_lenMK6N=sizeof(lpinit2a)+6;
    setCommandMK6N(data, data_lenMK6N);
	HAL_UART_Transmit(&huart3, (uint8_t * )data2MK6N, data_lenMK6N, 500);
}

void rxlpMK6NInit2a(char* datain, int len, int i){
	memcpy(dataMK6N,datain,len);
	data_lenMK6N=datasize(dataMK6N);
    if(getRawDataMK6N(6)){
      hex2Float(dataMK6N,0);
      lpscaleMK6N[i]=myFloat.f;
      lptype[i]=dataMK6N[data_lenMK6N-1];
    }
}

// ----------------------- EoB -------------------------------
const char * checkregEoBInit1(uint8_t reg){
  switch(reg){
    case 0x93:
            return "\"3P37\""; //"Total Import Wh";
            break;
    case 0x97:
            return "\"3P33\""; //"Total Export Wh";
            break;
    case 0x9B:
            return "\"3P29\""; //"Total Import varh";
            break;
    case 0x9F:
            return "\"3P25\""; //"Total Export varh";
            break;
    case 0xE3:
            return "\"3P21\""; //"Total Import VAh";
            break;
    case 0xE7:
            return "\"3P21\""; //"Total Export VAh";
            break;
    case 0x00:
            return "\"3P66\""; //"Total ABS Wh";
            break;
    case 0x01:
            return "\"3P67\""; //"Total ABS varh";
            break;
    case 0x02:
            return "\"3P68\""; //"Total ABS VAh";
            break;
    default:
    		return "\"3P68\""; //"Total ABS VAh";
            break;
  }
}

const char * checkregEoBRate9(uint8_t reg){
  switch(reg){
    case 0x93:
            return "\"3P76\""; //"Total Import Wh";
            break;
    case 0x97:
            return "\"3P70\""; //"Total Export Wh";
            break;
    case 0x9B:
            return "\"3P88\""; //"Total Import varh";
            break;
    case 0x9F:
            return "\"3P82\""; //"Total Export varh";
            break;
    case 0xE3:
            return "\"3P0A\""; //"Total Import VAh";
            break;
    case 0xE7:
            return "\"3P94\""; //"Total Export VAh";
            break;
    case 0x00:
            return "\"3P6A\""; //"Total ABS Wh";
            break;
    case 0x01:
            return "\"3PCA\""; //"Total ABS varh";
            break;
    case 0x02:
            return "\"3P2B\""; //"Total ABS VAh";
            break;
    default:
    		return "\"3P2B\""; //"Total ABS VAh";
            break;
  }
}

const char * checkregEoBRate1(uint8_t reg){
  switch(reg){
    case 0x93:
            return "\"3P77\""; //"Total Import Wh";
            break;
    case 0x97:
            return "\"3P71\""; //"Total Export Wh";
            break;
    case 0x9B:
            return "\"3P89\""; //"Total Import varh";
            break;
    case 0x9F:
            return "\"3P83\""; //"Total Export varh";
            break;
    case 0xE3:
            return "\"3P1A\""; //"Total Import VAh";
            break;
    case 0xE7:
            return "\"3P95\""; //"Total Export VAh";
            break;
    case 0x00:
            return "\"3P7A\""; //"Total ABS Wh";
            break;
    case 0x01:
            return "\"3PDA\""; //"Total ABS varh";
            break;
    case 0x02:
            return "\"3P3B\""; //"Total ABS VAh";
            break;
    default:
    		return "\"3P3B\""; //"Total ABS VAh";
            break;
  }
}

const char * checkregEoBRate2(uint8_t reg){
  switch(reg){
    case 0x93:
            return "\"3P78\""; //"Total Import Wh";
            break;
    case 0x97:
            return "\"3P72\""; //"Total Export Wh";
            break;
    case 0x9B:
            return "\"3P90\""; //"Total Import varh";
            break;
    case 0x9F:
            return "\"3P84\""; //"Total Export varh";
            break;
    case 0xE3:
            return "\"3P2A\""; //"Total Import VAh";
            break;
    case 0xE7:
            return "\"3P96\""; //"Total Export VAh";
            break;
    case 0x00:
            return "\"3P8A\""; //"Total ABS Wh";
            break;
    case 0x01:
            return "\"3PEA\""; //"Total ABS varh";
            break;
    case 0x02:
            return "\"3P4B\""; //"Total ABS VAh";
            break;
    default:
    		return "\"3P4B\""; //"Total ABS VAh";
            break;
  }
}

const char * checkregEoBRate3(uint8_t reg){
  switch(reg){
    case 0x93:
            return "\"3P79\""; //"Total Import Wh";
            break;
    case 0x97:
            return "\"3P73\""; //"Total Export Wh";
            break;
    case 0x9B:
            return "\"3P91\""; //"Total Import varh";
            break;
    case 0x9F:
            return "\"3P85\""; //"Total Export varh";
            break;
    case 0xE3:
            return "\"3P3A\""; //"Total Import VAh";
            break;
    case 0xE7:
            return "\"3P97\""; //"Total Export VAh";
            break;
    case 0x00:
            return "\"3P9A\""; //"Total ABS Wh";
            break;
    case 0x01:
            return "\"3PFA\""; //"Total ABS varh";
            break;
    case 0x02:
            return "\"3P5B\""; //"Total ABS VAh";
            break;
    default:
    		return "\"3P5B\""; //"Total ABS VAh";
            break;
  }
}

const char * checkregEoBRate4(uint8_t reg){
  switch(reg){
    case 0x93:
            return "\"3P80\""; //"Total Import Wh";
            break;
    case 0x97:
            return "\"3P74\""; //"Total Export Wh";
            break;
    case 0x9B:
            return "\"3P92\""; //"Total Import varh";
            break;
    case 0x9F:
            return "\"3P86\""; //"Total Export varh";
            break;
    case 0xE3:
            return "\"3P4A\""; //"Total Import VAh";
            break;
    case 0xE7:
            return "\"3P98\""; //"Total Export VAh";
            break;
    case 0x00:
            return "\"3PAA\""; //"Total ABS Wh";
            break;
    case 0x01:
            return "\"3P0B\""; //"Total ABS varh";
            break;
    case 0x02:
            return "\"3P6B\""; //"Total ABS VAh";
            break;
    default:
    		return "\"3P6B\""; //"Total ABS VAh";
            break;
  }
}

const char * checkregEoBRate5(uint8_t reg){
  switch(reg){
    case 0x93:
            return "\"3P85\""; //"Total Import Wh";
            break;
    case 0x97:
            return "\"3P75\""; //"Total Export Wh";
            break;
    case 0x9B:
            return "\"3P93\""; //"Total Import varh";
            break;
    case 0x9F:
            return "\"3P87\""; //"Total Export varh";
            break;
    case 0xE3:
            return "\"3P5A\""; //"Total Import VAh";
            break;
    case 0xE7:
            return "\"3P99\""; //"Total Export VAh";
            break;
    case 0x00:
            return "\"3PBA\""; //"Total ABS Wh";
            break;
    case 0x01:
            return "\"3P1B\""; //"Total ABS varh";
            break;
    case 0x02:
            return "\"3P7B\""; //"Total ABS VAh";
            break;
    default:
    		return "\"3P7B\""; //"Total ABS VAh";
            break;
  }
}

void readEoB(char* outstr){
  //reading 00000400 eob rate1,2,3,4,5,unified for user0-8
	dataallclearMK6N();
  char mychar[sizeof(eobuser0)];
  for(int i=0;i<9;i++){
    for(int j=0;j<sizeof(eobuser0);j++){
      mychar[j]=eobuser0[j];
      if(j==8 || j==12 || j==16 || j==20 || j==24 || j==28){
        mychar[j]=eobuser0[j] | i;      
      }
    }
    setCommandMK6N(mychar, sizeof(mychar));
//	HAL_UART_Transmit(&huart3, (uint8_t * )data2MK6N, data_lenMK6N, 200);
//	HAL_UART_Receive(&huart3, (uint8_t * )dataMK6N, BUFFSIZEMK6N, 1000);
	HAL_UART_Transmit(&huart3, (uint8_t * )data2MK6N, data_lenMK6N, 500);
	HAL_UART_Receive(&huart3, (uint8_t * )dataMK6N, BUFFSIZEMK6N, 1000);
	data_lenMK6N=0;
	data_lenMK6N=datasize(dataMK6N);

//	data_lenMK6N=datasize(dataMK6N);
    if(getRawDataMK6N(6)){
//    	printData(dataMK6N, data_lenMK6N);
      eoblen=9;
//      printf("datasize: %d,readEoB: ",data_lenMK6N);
      for(int k=0;k<6;k++){
        hex2Double(dataMK6N,8*k);
        if(k==0) eob0MK6N[i]=ulf.f;
        else if(k==1) eob1MK6N[i]=ulf.f;
        else if(k==2) eob2MK6N[i]=ulf.f;
        else if(k==3) eob3MK6N[i]=ulf.f;
        else if(k==4) eob4MK6N[i]=ulf.f;
        else eob9MK6N[i]=ulf.f;
//        printf("%.5f ",ulf.f);
      }
//      printf("\r\n");
    }
  }
  if(decodeEoB()) {
	  for(int i=0;i<datasizestrMK6N(datastrMK6N);i++){
		  *outstr++=datastrMK6N[i];
	  }
	  printf("%d\r\n",datasizestrMK6N(datastrMK6N));
//	  return d;
  }
//  return "{\"nodata\":0}";
//    return 0;
}

void readEoB2(int i){
  //reading 00000400 eob rate1,2,3,4,5,unified for user0-8
  char mychar[sizeof(eobuser0)];
  memcpy(mychar,eobuser0,sizeof(eobuser0));
  mychar[8] = eobuser0[8] | i;
  mychar[12] = eobuser0[12] | i;
  mychar[16] = eobuser0[16] | i;
  mychar[20] = eobuser0[20] | i;
  mychar[24] = eobuser0[24] | i;
  mychar[28] = eobuser0[28] | i;
//    for(int j=0;j<sizeof(eobuser0);j++){
////      mychar[j]=eobuser0[j];
//      if(j==8 || j==12 || j==16 || j==20 || j==24 || j==28){
//    	eobuser0[j]=eobuser0[j] | i;
//      }
//    }

    setCommandMK6N(mychar, sizeof(mychar));
	HAL_UART_Transmit(&huart3, (uint8_t * )data2MK6N, data_lenMK6N, 200);
}

void readEoB3(char* datain, int len, int i){
	memcpy(dataMK6N,datain,len);
	data_lenMK6N=datasize(dataMK6N);
    if(getRawDataMK6N(6)){
//    	printData(dataMK6N, data_lenMK6N);
      eoblen=9;
//      printf("datasize: %d,readEoB: ",data_lenMK6N);
      for(int k=0;k<6;k++){
        hex2Double(dataMK6N,8*k);
        if(k==0) eob0MK6N[i]=ulf.f;
        else if(k==1) eob1MK6N[i]=ulf.f;
        else if(k==2) eob2MK6N[i]=ulf.f;
        else if(k==3) eob3MK6N[i]=ulf.f;
        else if(k==4) eob4MK6N[i]=ulf.f;
        else eob9MK6N[i]=ulf.f;
//        printf("%.5f ",ulf.f);
      }
//      printf("\r\n");
    }
}

void decodeEoB3(char* outstr){

  if(decodeEoB()) {
	  for(int i=0;i<datasizestrMK6N(datastrMK6N);i++){
		  *outstr++=datastrMK6N[i];
	  }
	  printf("%d\r\n",datasizestrMK6N(datastrMK6N));
//	  return d;
  }
}
int readEoBInit1(){
	dataallclearMK6N();
    setCommandMK6N(eobinit1, sizeof(eobinit1));
//    printData(data2MK6N, data_lenMK6N);
	HAL_UART_Transmit(&huart3, (uint8_t * )data2MK6N, data_lenMK6N, 500);
	HAL_UART_Receive(&huart3, (uint8_t * )dataMK6N, BUFFSIZEMK6N, 1000);
	data_lenMK6N=0;
	data_lenMK6N=datasize(dataMK6N);
//	printf("datasize: %d 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X\r\n",data_lenMK6N,dataMK6N[3],dataMK6N[7],dataMK6N[11],dataMK6N[15],dataMK6N[19],dataMK6N[23],dataMK6N[27],dataMK6N[31]);
    if(getRawDataMK6N(6)){
      eoblen=9;
//      printData(dataMK6N,data_lenMK6N);
      saveEoBInit1();
      return 1;
    }
    return 0;
}
void txEoBInit1(){
	dataallclearMK6N();
    setCommandMK6N(eobinit1, sizeof(eobinit1));
//    printData(data2MK6N, data_lenMK6N);
	HAL_UART_Transmit(&huart3, (uint8_t * )data2MK6N, data_lenMK6N, 500);
}
void rxEoBInit1(char* datain, int len){
	memcpy(dataMK6N,datain,len);
	data_lenMK6N=0;
	data_lenMK6N=datasize(dataMK6N);
//	printf("datasize: %d 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X\r\n",data_lenMK6N,dataMK6N[3],dataMK6N[7],dataMK6N[11],dataMK6N[15],dataMK6N[19],dataMK6N[23],dataMK6N[27],dataMK6N[31]);
    if(getRawDataMK6N(6)){
      eoblen=9;
//      printData(dataMK6N,data_lenMK6N);
      saveEoBInit1();
    }
}

int readEoBInit2(){
    setCommandMK6N(eobinit2, sizeof(eobinit2));
    if(getRawDataMK6N(6)){
      printData(dataMK6N,data_lenMK6N);
      return 1;
    }
    return 0;
}

void saveEoBInit1(){
	for(int i=0;i<9;i++){
	    regeobinit1[i]=dataMK6N[i*4+3];
	    printf("0x%X ",regeobinit1[i]);
//	    temp=i*4+3;
	  }
}

bool decodeEoB(){
//  printData(data, data_lenMK6N);
	int maxloop=8;
	data_lenMK6N = 0;
	datastrclearMK6N();

	strcpy(datastrMK6N,"{\"mid\":");
	strcat(datastrMK6N, meterSNMK6N);
	strcat(datastrMK6N, ",\"pid\":\"EoBMK6N\",");
//	SerialMonprintlnMK6N(datastrMK6N);
//  data_lenMK6N = setdatastr(",\"pid\":\"EOBMK6N\",\"readdate\":",data_lenMK6N);
//  datastr+=String(time2TimestampNowMK6N(second(),minute(),hour(),day(),month(),year()));
//  datastr+=",";
  for(int i=0;i<maxloop;i++){
//	  printf("%d\r\n",i);
  	strcat(datastrMK6N, checkregEoBRate1(regeobinit1[i]));
  	strcat(datastrMK6N, ":");
  	data2clearMK6N();
  	sprintf(data2MK6N,"%.5f,",eob0MK6N[i]);
  	strcat(datastrMK6N, data2MK6N);
  	strcat(datastrMK6N, checkregEoBRate2(regeobinit1[i]));
  	strcat(datastrMK6N, ":");
  	data2clearMK6N();
  	sprintf(data2MK6N,"%.5f,",eob1MK6N[i]);
  	strcat(datastrMK6N, data2MK6N);
  	strcat(datastrMK6N, checkregEoBRate3(regeobinit1[i]));
  	strcat(datastrMK6N, ":");
  	data2clearMK6N();
  	sprintf(data2MK6N,"%.5f,",eob2MK6N[i]);
  	strcat(datastrMK6N, data2MK6N);
  	strcat(datastrMK6N, checkregEoBRate4(regeobinit1[i]));
  	strcat(datastrMK6N, ":");
  	data2clearMK6N();
  	sprintf(data2MK6N,"%.5f,",eob3MK6N[i]);
  	strcat(datastrMK6N, data2MK6N);
  	strcat(datastrMK6N, checkregEoBRate5(regeobinit1[i]));
  	strcat(datastrMK6N, ":");
  	data2clearMK6N();
  	sprintf(data2MK6N,"%.5f,",eob4MK6N[i]);
  	strcat(datastrMK6N, data2MK6N);
  	strcat(datastrMK6N, checkregEoBRate9(regeobinit1[i]));
  	strcat(datastrMK6N, ":");
  	data2clearMK6N();
  	sprintf(data2MK6N,"%.5f",eob9MK6N[i]);
  	strcat(datastrMK6N, data2MK6N);
    if(i<maxloop-1){
      	strcat(datastrMK6N,",");
    }
    else if(i==maxloop-1){
      	strcat(datastrMK6N,"}");
    }
  }
//    SerialMonprintlnMK6N(datastrMK6N);
    return true;
}
