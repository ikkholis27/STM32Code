#include "stm32l4xx_hal.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include "Crc16.h"
//#include "edmi.h"

#define VOSCALEIDX  0
#define IISCALEIDX  1
#define PWSCALEIDX  2
#define PFSCALEIDX  3
#define FRSCALEIDX  4

#define OFFSETTIMESTAMP 	820454400

#define BUFFSIZESTRMK10E  	1500
#define BUFFSIZEMK10E  		1100

// definitions block
//#define SerialMon  	huart1
//#define SerialAT  	huart2
//#define SerialMtr  	huart3

#define MK6N      6
#define MK10E     10

char datastrMK10E[BUFFSIZESTRMK10E];
unsigned char dataMK10E[BUFFSIZEMK10E];
char data2MK10E[20];

bool SerialMtrFlagMK10E;
unsigned short valuecrcMK10E;

int data_lenMK10E;
long timestamp_now;
uint8_t lplen=0, jmlchannel;
long lpfirstentrydate;
long lpfirstentry, lplastentry,lpmaxentry,lprecord;

char meterSNMK10E[10]={'0','0','0','0','0','0','0','0','0','0'};

uint8_t timereq[] = {0x02,0x52,0xF0,0x3D,0x29,0xF9,0x03};
// 0xF78x (max. F788)
//uint8_t instantMK10E[]={0x02,0x52,0xF5,0x30,0x07,0xA1,0x03};
uint8_t instantPFAMK10E[]={0x02,0x52,0xE0,0x23};
uint8_t eobbilltot[]={};
//uint8_t eobenergy[]={0x02,0x52,0xF5,0x4F,0x88,0xD9,0x03};
//uint8_t eobcurrent[]={0x02,0x52,0xF5,0x40,0x79,0x36,0x03};
//uint8_t eobstan[]={0x02,0x52,0xF5,0x4E,0x98,0xF8,0x03};
//uint8_t eobmin1[]={0x02,0x52,0xF5,0x41,0x69,0x17,0x03};
//uint8_t eoblpinit[]={0x02,0x52,0xF5,0x32,0x27,0xE3,0x03};

// variable for eob
double eob0[7],eob1[7],eob2[7],eob3[7],eob4[7],eob9[7];
double eobmd0[7],eobmd1[7],eobmd2[7],eobmd3[7],eobmd4[7],eobmd9[7];
int eobreg[7];

// variable for lp
//uint8_t reglpinit1[11];
int lpdataint[14];
unsigned int lpdatauint[14];
float lpdataf[14];
double lpdatad[14];
int lpreg[14];

extern int lpregm[14];

float commonscale[5];
double lpscaleMK10E[5];

long timestamp;
uint8_t lpintervalMK10E;
// variable for instant
float instantF530[25],instantPF[4];

//extern union myFloat myfloat;
//extern union myInt myint;

bool checkcrcMK10E();
const char* checkregEoBInit1MK10E(int reg);
const char* checkregEoBRate1MK10E(int reg);
const char* checkregEoBRate2MK10E(int reg);
const char* checkregEoBRate3MK10E(int reg);
const char* checkregEoBRate4MK10E(int reg);
const char* checkregEoBRate5MK10E(int reg);
const char* checkregEoBUnifiedMK10E(int reg);

void decodeEoBLpinitMK10E();
void decodeEoBMK10E();

bool decodeInstantMK10E();
void decodeInstantPFMK10E();

void decodelpinitMK10E();
void decodelpscaleMK10E(int a, uint8_t index);
int decodelpdataMK10E();

void lp2JsonMK10E();

void saveEoBDataMK10E();
void saveSNMK10E();

int datasizestrMK10E(char buf[])
{
	int i = 0;
	while(buf[i] != '\0') i++;
	return i+1;
}

void SerialMonprintlnMK10E(char * ptr){
	uint32_t len=strlen(ptr);
	uint8_t ptr1[len+2];
	memcpy(ptr1, (uint8_t *)ptr, len);
	ptr1[len]= 0x0D;
	ptr1[len+1]= 0x0A;
	  HAL_UART_Transmit(&SerialMon, ptr1, len+2, 3000);
}

void SerialMtrprintMK10E(uint8_t * ptr, uint32_t len, uint32_t timeout){
	HAL_UART_Transmit(&SerialMtr, ptr, len, timeout);
}

void SerialMtrreadMK10E(uint8_t ptr[], uint32_t len, uint32_t timeout){
	  HAL_UART_Receive(&SerialMtr, (uint8_t * )ptr, len, timeout);
}

int bufsizeMK10E(unsigned char *buf)
{
	int i = 0;
	while(*buf++ != '\0') i++;
	return i;
}

void dataallclearMK10E(void)
{
	for (int i=0; i < BUFFSIZEMK10E; i++)
	{
		dataMK10E[i] = '\0';
	}
	for (int i=0; i < BUFFSIZEMK10E; i++)
	{
		data2MK10E[i] = '\0';
	}
	for (int i=0; i < BUFFSIZEMK10E; i++)
	{
		datastrMK10E[i] = '\0';
	}
}

void dataclearMK10E(void)
{
	for (int i=0; i < BUFFSIZEMK10E; i++)
	{
		dataMK10E[i] = '\0';
	}
}

void data2clearMK10E(void)
{
	for (int i=0; i < 20; i++)
	{
		data2MK10E[i] = '\0';
	}
}

void datastrclearMK10E(void)
{
	for (int i=0; i < BUFFSIZEMK10E; i++)
	{
		datastrMK10E[i] = '\0';
	}
}

void setPaddingMK10E(){
  unsigned int lentemp=data_lenMK10E, idx=0;
  uint8_t datatemp[data_lenMK10E];
  for(unsigned int i=0;i<data_lenMK10E;i++){
    datatemp[i]=dataMK10E[i];
  }
  for(unsigned int i=0;i<lentemp;i++){
    if(i==0){
      dataMK10E[idx]=0x02;
    }
    else if(i==lentemp-1){
      dataMK10E[idx]=0x03;
    }
    else if((datatemp[i]==0x13 && (i<lentemp-1 && i>0)) || datatemp[i]==0x02 || datatemp[i]==0x10 || datatemp[i]==0x11){
      dataMK10E[idx++]=0x10;
      dataMK10E[idx]=datatemp[i]|0x40;
    }
    else if(i<lentemp-1 && datatemp[i]==0x03){
      dataMK10E[idx++]=0x10;
      dataMK10E[idx]=datatemp[i]|0x40;
    }
    else    dataMK10E[idx]=datatemp[i];
    idx++;
  }
  data_lenMK10E = idx;
}

void setPaddingMK10ELP(){
  unsigned int lentemp=data_lenMK10E, idx=0;
  uint8_t datatemp[data_lenMK10E];
  for(unsigned int i=0;i<data_lenMK10E;i++){
    datatemp[i]=data2MK10E[i];
  }
  for(unsigned int i=0;i<lentemp;i++){
    if(i==0){
      data2MK10E[idx]=0x02;
    }
    else if(i==lentemp-1){
      data2MK10E[idx]=0x03;
    }
    else if((datatemp[i]==0x13 && (i<lentemp-1 && i>0)) || datatemp[i]==0x02 || datatemp[i]==0x10 || datatemp[i]==0x11){
      data2MK10E[idx++]=0x10;
      data2MK10E[idx]=datatemp[i]|0x40;
    }
    else if(i<lentemp-1 && datatemp[i]==0x03){
      data2MK10E[idx++]=0x10;
      data2MK10E[idx]=datatemp[i]|0x40;
    }
    else    data2MK10E[idx]=datatemp[i];
    idx++;
  }
  data_lenMK10E = idx;
}

void setCommandMK10E(uint8_t array[], unsigned int sizearr){
  for(int i=0;i<sizearr;i++){
    dataMK10E[i]=array[i];
  }
  data_lenMK10E = sizearr;
  clearCrc();
   valuecrc = XModemCrc(array,0,sizearr);
   dataMK10E[sizearr+2]=0x03;
   dataMK10E[sizearr+1]= (uint8_t) valuecrc;
   valuecrc  = valuecrc >> 8;
   dataMK10E[sizearr]  = (uint8_t)valuecrc;
   data_lenMK10E = sizearr+3;
   setPaddingMK10E();
}

void setCommandLPMK10E(char array[], unsigned int sizearr){
  for(int i=0;i<sizearr;i++){
    data2MK10E[i]=array[i];
  }
  data_lenMK10E = sizearr;
  clearCrc();
   valuecrc = XModemCrc(array,0,sizearr);
   data2MK10E[sizearr+2]=0x03;
   data2MK10E[sizearr+1]= (uint8_t) valuecrc;
   valuecrc  = valuecrc >> 8;
   data2MK10E[sizearr]  = (uint8_t)valuecrc;
   data_lenMK10E = sizearr+3;
   setPaddingMK10ELP();
}

void removePaddingMK10E(){
  unsigned int lentemp=data_lenMK10E;
  for(unsigned int i=0;i<data_lenMK10E;i++){
    if(dataMK10E[i]==0x10){
    	dataMK10E[i]=dataMK10E[i+1]^0x40;
      for(unsigned int j=i+1;j<data_lenMK10E-1;j++){
    	  dataMK10E[j]=dataMK10E[j+1];
      }
      lentemp--;
    }
  }
  data_lenMK10E = lentemp;
}


void removeheadcrctailMK10E(int sizeheadcmd){
  // remove CRC and Tail
  dataMK10E[data_lenMK10E-1]=0;
  dataMK10E[data_lenMK10E-2]=0;
  dataMK10E[data_lenMK10E-3]=0;
  data_lenMK10E=data_lenMK10E-3;
  //remove head
  for(unsigned int i=0;i<data_lenMK10E;i++){
    if(i+sizeheadcmd<=BUFFSIZEMK10E){
      dataMK10E[i]=dataMK10E[i+sizeheadcmd];
    }
  }
  // update data_lenMK10E
  data_lenMK10E=data_lenMK10E-sizeheadcmd;
}

bool checkcrcMK10E(){
    valuecrc = XModemCrc(data,0,data_len-3);
    unsigned short datacrcfromrx;
    datacrcfromrx = (unsigned short) (data[data_len-3])<<8;
    datacrcfromrx = datacrcfromrx | data[data_len-2];
    if(valuecrc == datacrcfromrx){
      return true;
    }
    else return false;
}

bool getRawDataMK10E(int headcmd){
    clearCrc();
    data_lenMK10E = datasize(dataMK10E);
    removePaddingMK10E();
//    valuecrc = XModemCrc(dataMK10E,0,data_lenMK10E-3);
//    if(checkcrcMK10E()){
//      SerialMonprintln("RemoveHeadCRCTail");
//      printData(dataMK10E,data_lenMK10E);
      removeheadcrctailMK10E(headcmd);
      return true;
//    }
    return true;
}

void saveSNMK10E(){
  if(data_lenMK10E>11) removeheadcrctailMK10E(5);
  for(int i=0;i<data_lenMK10E;i++){
    meterSNMK10E[i] = dataMK10E[i];
  }
//  printf("SN: %s\r\n",meterSNMK10E);
//  HAL_Delay(100);
}

const char* readSNMK10E(){
	dataallclearMK10E();
	HAL_UART_Transmit(&SerialMtr, (uint8_t * )meterreq, sizeof(meterreq), 100);
	HAL_UART_Receive(&SerialMtr, (uint8_t * )dataMK10E, 100, 1000);
	data_lenMK10E=0;
	data_lenMK10E=datasize(dataMK10E);
    if(data_lenMK10E>4){
    	SerialMtrFlag=true;
    	saveSNMK10E();
        return meterSNMK10E;
//    	printf("0x%X 0x%X 0x%X 0x%X 0x%X ",dataMK10E[0],dataMK10E[1],dataMK10E[2],dataMK10E[3],dataMK10E[4]);
//    	printf("0x%X 0x%X 0x%X 0x%X 0x%X \r\n",dataMK10E[5],dataMK10E[6],dataMK10E[7],dataMK10E[8],dataMK10E[9]);
//    	SerialMonprintln((char *)dataMK10E);
    }
    return meterSNMK10E;
}

void txSNMK10E_rtos(){
	dataallclearMK10E();
	HAL_UART_Transmit(&SerialMtr, (uint8_t * )meterreq, sizeof(meterreq), 100);
	osDelay(2000);
}

const char* rxSNMK10E_rtos(unsigned char buf[],int len){
	dataallclearMK10E();
	memcpy(dataMK10E,buf,len);
  data_lenMK10E=0;
  data_lenMK10E=datasize(dataMK10E);
    if(data_lenMK10E>4){
    	SerialMtrFlag=true;
    	saveSNMK10E();
        return meterSNMK10E;
//    	printf("0x%X 0x%X 0x%X 0x%X 0x%X ",dataMK10E[0],dataMK10E[1],dataMK10E[2],dataMK10E[3],dataMK10E[4]);
//    	printf("0x%X 0x%X 0x%X 0x%X 0x%X \r\n",dataMK10E[5],dataMK10E[6],dataMK10E[7],dataMK10E[8],dataMK10E[9]);
//    	SerialMonprintln((char *)dataMK10E);
    }
    return meterSNMK10E;
}

long readTimeMK10E(){
	dataallclearMK10E();
	data_lenMK10E=0;
	data2MK10E[0]=0x02;
	data2MK10E[1]=0x52;
	data2MK10E[2]=0xF0;
	data2MK10E[3]=0x3E;
	data2MK10E[4]=0x19;
	data2MK10E[5]=0x9A;
	data2MK10E[6]=0x03;
	data_lenMK10E=7;
    setCommandMK10E((uint8_t *)data2MK10E, data_lenMK10E);
//    printData(data2MK10E,data_lenMK10E);
    HAL_Delay(500);
	HAL_UART_Transmit(&SerialMtr, (uint8_t * )data2MK10E, data_lenMK10E, 100);
	HAL_UART_Receive(&SerialMtr, (uint8_t * )dataMK10E, 100, 2000);
	data_lenMK10E=0;
	data_lenMK10E=datasize(dataMK10E);
	printf("datalen readtime: %d\n",data_lenMK10E);
    printData(dataMK10E,data_lenMK10E);
    if(getRawDataMK10E(4)){
    	printf("rawdatalen readtime: %d\n",data_lenMK10E);
        printData(dataMK10E,data_lenMK10E);
    timestamp_now = hex2Long(dataMK10E,0);
      timestamp_now = timestamp_now + OFFSETTIMESTAMP;
//      return 1;
    }
    return timestamp_now;
}
// ---------- Decoding and Formating Functions

const char * formatInstantMK10E(uint8_t reg){
  switch(reg){
    case 0:
            return "\"3P59\""; // volt A
            break;
    case 1:
    		return  "\"3P60\""; // volt B
            break;
    case 2:
    		return "\"3P61\""; // volt C
            break;
    case 3:
    		return "\"3P09\""; // current A
            break;
    case 4:
    		return "\"3P10\""; // current B
            break;
    case 5:
    		return "\"3P11\""; // current C
            break;
    case 6:
    		return "\"3P62\""; // phase angle A
            break;
    case 7:
			return "\"3P63\""; // phase angle B
            break;
    case 8:
			return "\"3P64\""; // phase angle C
            break;
    case 15:
			return "\"3P01\""; // phase A Watts
            break;
    case 16:
			return "\"3P02\""; // Phase B Watts
            break;
    case 17:
            return "\"3P03\""; // phase C Watts
            break;
    case 18:
            return "\"3P43\""; // phase A Vars
            break;
    case 19:
            return "\"3P44\""; // phase B Vars
            break;
    case 20:
            return "\"3P45\""; // phase C Vars
            break;
    case 21:
            return "\"3P05\""; // phase A VA
            break;
    case 22:
            return "\"3P06\""; // phase B VA
            break;
    case 23:
            return "\"3P07\""; // phase C VA
            break;
    case 24:
            return "\"3P13\""; // Freq
            break;
    case 25:
            return "PF";
            break;
    default:
            return "\"3P13\""; // Freq
            break;
  }
}

const char * formatInstantPFMK10E(uint8_t reg){
  switch(reg){
    case 0x23:
            return "\"3P39\"";
            break;
    case 0x24:
            return "\"3P40\"";
            break;
    case 0x25:
            return "\"3P41\"";
            break;
    case 0x26:
            return "\"3P42\"";
            break;
    default:
            return "\"3P42\"";
            break;
  }
}

const char * formatLPRegMK10E(int a){
  switch (a){
    case 13315:  return "\"3P66\""; // abs total wh
                break;
    case 13331:  return "\"3P33\""; // total export wh
                break;
    case 13347:  return "\"3P37\""; // total import wh
                break;
    case 13319:  return "\"3P67\""; // abs total varh 0x3407
                break;
    case 13335:  return "\"3P25\""; // total export wh 0x3417
                break;
    case 13351:  return "\"3P29\""; // total import wh 0x3427
                break;
    case 46211:  return "\"3P59\""; // tegangan R
                break;
    case 46212:  return "\"3P60\""; // tegangan S
                break;
    case 46213:  return "\"3P61\""; // tegangan T
                break;
    case 46208:  return "\"3P09\""; // Arus R
                break;
    case 46209:  return "\"3P10\""; // Arus S
                break;
    case 46210:  return "\"3P11\""; // Arus T
                break;
    case 46323  :  return "\"3P42\""; // PF
                break;
    default   :  return "\"3P42\""; // PF
                break;
    
  }
}

const char * formatLPMK10E(uint8_t reg){
  switch(reg){
    case 0x23:
            return "\"3P39\"";
            break;
    case 0x24:
            return "\"3P40\"";
            break;
    case 0x25:
            return "\"3P41\"";
            break;
    case 0x26:
            return "\"3P42\"";
            break;
    default:
        return "\"3P42\"";
            break;
  }
}

void decodeEobLpinitMK10E(){
  int index = 0;
  if(getRawDataMK10E(4)){
//    SerialMonprintlnMK10E();
  }
  hex2Long(dataMK10E,0);
  lpfirstentrydate = myFloat.l;
  lpfirstentrydate += 820454400L;
//  printf("%ld ",lpfirstentrydate);
  index += 8;
  hex2Long(dataMK10E,index);
  lpfirstentry = myFloat.l;
//  printf("%ld ",lpfirstentry);
  index += 8;
  hex2Long(dataMK10E,index);
  lplastentry = myFloat.l;
//  printf("%ld ",lplastentry);
  index += 8;
//  hex2Long(dataMK10E,index);
  uint8_t temp = dataMK10E[index+1];
//  printf("\ntemp:%x ",temp);
  temp = temp & 0x3F;
//  printf("%x ",temp);
  jmlchannel = (uint8_t)temp;
//  printf("jmlch:%d ",jmlchannel);
  temp = dataMK10E[index] << 4;
//  printf("\ntemp:%x ",temp);
  temp = temp >> 2;
//  printf("%x ",temp);
  uint8_t temp1 = dataMK10E[index+1] >> 6;
//  printf("temp1:%x ",temp1);
  temp = temp | temp1;
//  SerialMonprintlnMK10E(temp);
  lpintervalMK10E = (uint8_t)temp;
//  printf("%d \n",lpintervalMK10E);
  index += 4;
  hex2Long(dataMK10E,index);
  commonscale[0]=myFloat.f;
  index += 4;
  hex2Long(dataMK10E,index);
  commonscale[1]=myFloat.f;
  index += 4;
  hex2Long(dataMK10E,index);
  commonscale[2]=myFloat.f;
  index += 4;
  hex2Long(dataMK10E,index);
  commonscale[3]=myFloat.f;
  index += 4;
  hex2Long(dataMK10E,index);
  commonscale[4]=myFloat.f;
  for(int i=0;i<5;i++){
//	  printf("%.5f ",commonscale[i]);
  }
//  printf("\r\n");
  index = 118;
  for(int i=0;i<jmlchannel;i++){
    lpreg[i]=dataMK10E[2*i+index];
    lpreg[i]=lpreg[i]<<8;
    lpreg[i]=lpreg[i] | dataMK10E[2*i+index+1];
    lpregm[i]=lpreg[i];
//	  printf("0x%X ",lpreg[i]);
//    SerialMon.print(" ");
//    SerialMon.print(lpreg[i]);
  }
//  printf("\r\n");
//  SerialMonprintlnMK10E(" ");
  index = 246;
  for(int i=0;i<7;i++){
    eobreg[i]=dataMK10E[2*i+index];
    eobreg[i]=eobreg[i]<<8;
    eobreg[i]=eobreg[i] | dataMK10E[2*i+index+1];
//    printf("0x%X ",eobreg[i]);
//    datastrMK10E=" ";
//    checkregEoBInit1MK10E(eobreg[i]);
//    SerialMonprint(datastrMK10E);
  }
//  printf("\r\n");
//  SerialMonprintlnMK10E();
  decodelpinitMK10E();
}

// -------------------------- Instant ------------------------
void decodeInstantPFMK10E(){
  dataallclearMK10E();
  int3=0;
  uint8_t regPFA=0x23;
  strcpy(datastrMK10E,"{\"mid\":");
  strcat(datastrMK10E, meterSNMK10E);
//  strcat(datastrMK10E, ",\"pid\":\"InstantPFMK10E\",\"readdate\":");
  strcat(datastrMK10E, ",\"pid\":\"InstantPFMK10E\",");
//  data2clearMK10E();
//  sprintf(data2MK10E,"%d,",timestamp_now);
//  strcat(datastrMK10E, data2MK10E);
  for(int i=0; i<3;i++){
    	strcat(datastrMK10E, formatInstantPFMK10E(regPFA+i));
    	strcat(datastrMK10E, ":");
    	data2clearMK10E();
    	sprintf(data2MK10E,"%.5f",instantPF[int3]);
    	int2 = bufsizeMK10E(data2);
    	strcat(datastrMK10E, data2MK10E);
    	strcat(datastrMK10E, ",");
        int3++;
    }
//      formatInstantPFMK10E(regPFA+1);
  	strcat(datastrMK10E, formatInstantPFMK10E(regPFA+1));
  	strcat(datastrMK10E, ":");
  	data2clearMK10E();
  	sprintf(data2MK10E,"%.5f",instantPF[int3]);
  	int2 = bufsizeMK10E(data2);
  	strcat(datastrMK10E, data2MK10E);
  	strcat(datastrMK10E, "}");
        int3++;
    SerialMonprintlnMK10E(datastrMK10E);
}

bool decodeInstantMK10E(){
	//int index=0;
  myFloat.f = 0;
  printf("decode Instant ");
  for(int i=0;i<25;i++){    
	  instantF530[i]=hex2Float(dataMK10E,i*4);
      printf("%.5f ",instantF530[i]);
  }
  printf("%.5f \r\n",instantF530[0]);
  HAL_Delay(1000);
  int1=0;
  int2=0;
  int3=0;
  datastrclearMK10E();
  strcpy(datastrMK10E,"{\"mid\":");
  strcat(datastrMK10E, meterSNMK10E);
  strcat(datastrMK10E, ",\"pid\":\"InstantMK10E\",");
//  strcat(datastrMK10E, ",\"pid\":\"InstantMK10E\",\"readdate\":");
//  data2clearMK10E();
//  sprintf(data2MK10E,"%d,",timestamp_now);
//  strcat(datastrMK10E, data2MK10E);
  for(int i=0; i<9;i++){
	strcat(datastrMK10E, formatInstantMK10E(i));
	strcat(datastrMK10E, ":");
	data2clearMK10E();
	sprintf(data2MK10E,"%.5f",instantF530[int3]);
//	int2 = bufsizeMK10E(data2MK10E);
	strcat(datastrMK10E, data2MK10E);
	strcat(datastrMK10E, ",");
    int1+=4;
    int3++;
  }
  for(int i=0;i<6;i++){
    int1+=4;
    int3++;
  }
  int2 = int1;
  for(int i=0; i<9;i++){
	strcat(datastrMK10E, formatInstantMK10E(int3));
	strcat(datastrMK10E, ":");
	data2clearMK10E();
	sprintf(data2MK10E,"%.5f",instantF530[int3]);
//	int2 = bufsizeMK10E(data2);
	strcat(datastrMK10E, data2MK10E);
	strcat(datastrMK10E, ",");
    int1+=4;
    int3++;
  }
  strcat(datastrMK10E, formatInstantMK10E(int3));
  strcat(datastrMK10E, ":");
  data2clearMK10E();
  sprintf(data2MK10E,"%.5f",instantF530[int3]);
//  int2 = bufsizeMK10E(data2);
  strcat(datastrMK10E, data2MK10E);
  strcat(datastrMK10E, "}");
  int1+=4;
  int3++;
  return true;
//  SerialMonprintln(datastrMK10E);
}

void readInstantMK10E(char* outstr){
  dataallclearMK10E();
  dataMK10E[0]=0x02;
  dataMK10E[1]=0x52;
  dataMK10E[2]=0xF5;
  dataMK10E[3]=0x30;
  dataMK10E[4]=0x07;
  dataMK10E[5]=0xA1;
  dataMK10E[6]=0x03;
  HAL_UART_Transmit(&SerialMtr, (uint8_t * )dataMK10E, 7, 100);
  HAL_UART_Receive(&SerialMtr, (uint8_t * )dataMK10E, BUFFSIZEMK10E, 5000);
  data_lenMK10E=0;
  data_lenMK10E=datasize(dataMK10E);
  printf("datasize: %d\r\n",data_lenMK10E);
  HAL_Delay(500);
  if(data_lenMK10E>5){
//    return 1;
//	  getRawDataMK10E(0);
	  removeheadcrctailMK10E(4);
	  printData(dataMK10E,data_lenMK10E);
	  HAL_Delay(1000);
	  printf("Decode InstantMK10E\r\n");
	  HAL_Delay(1000);
	  if(decodeInstantMK10E()){
		  printf("datasizestrMK10E: %d \r\n",datasizestrMK10E(datastrMK10E));
		  for(int i=0;i<datasizestrMK10E(datastrMK10E);i++){
			  *outstr++=datastrMK10E[i];
		  }
		  printf(outstr);
	  }
  }
//  return "0";
}

void txInstantMK10E(){
  dataallclearMK10E();
  dataMK10E[0]=0x02;
  dataMK10E[1]=0x52;
  dataMK10E[2]=0xF5;
  dataMK10E[3]=0x30;
  dataMK10E[4]=0x07;
  dataMK10E[5]=0xA1;
  dataMK10E[6]=0x03;
  HAL_UART_Transmit(&SerialMtr, (uint8_t * )dataMK10E, 7, 200);
}

void rxInstantMK10E(char* outstr, unsigned char buf[],int len){
	dataallclearMK10E();
	memcpy(dataMK10E,buf,len);
  data_lenMK10E=0;
  data_lenMK10E=datasize(dataMK10E);
  if(data_lenMK10E>5){
	  removeheadcrctailMK10E(4);
	  if(decodeInstantMK10E()){
//		  printf("datasizestrMK10E: %d \r\n",datasizestrMK10E(datastrMK10E));
		  for(int i=0;i<datasizestrMK10E(datastrMK10E);i++){
			  *outstr++=datastrMK10E[i];
		  }
//		  printf(outstr);
	  }
  }
//  return "0";
}

void readInstantPFMK10E(){
  uint8_t mydata[4];
  for(int j=0;j<4;j++){
      mydata[j]=instantPFAMK10E[j];
  }
  for(int i=0;i<4;i++){
    mydata[3] = instantPFAMK10E[3]+i;
    datastrclearMK10E();
    data_lenMK10E=sizeof(instantPFAMK10E);
    setCommandMK10E(mydata, 4);
	HAL_UART_Transmit(&SerialMtr, (uint8_t * )data2, data_lenMK10E, 100);
//	SerialMtrread(dataMK10E,BUFFSIZE,5000);
	HAL_UART_Receive(&SerialMtr, (uint8_t * )data2, 100, 5000);
//    readSerialMtrLP(2000);
    getRawDataMK10E(0);
	removeheadcrctailMK10E(4);
    hex2Float(dataMK10E,0);
    instantPF[i]=myFloat.f;
    osDelay(1000);
  }
  decodeInstantPFMK10E(); 
}


// ----------------------- LP --------------------------------

void readLPMK10E(int indexmin1, char* outstr){
	dataallclearMK10E();
	data_lenMK10E = 0;
  //reading 00000400 lp rate1,2,3,4,5,unified for user0-8
  long lpindex = (long)lplastentry-indexmin1;
  lprecord = lpindex;
  data2MK10E[0]=0x02;
  data2MK10E[1]=0x46;
  data2MK10E[2]=0x30;
  data2MK10E[3]=0x00;
  data2MK10E[4]=lpindex>>24;
  data2MK10E[5]=lpindex>>16;
  data2MK10E[6]=lpindex>>8;
  data2MK10E[7]=(uint8_t) lpindex;
  data2MK10E[8]=0x00;
  data2MK10E[9]=0x01;
  data_lenMK10E = 10;
  setCommandLPMK10E(data2MK10E, data_lenMK10E);
//  printf("datalenMK10E: %d\n",data_lenMK10E);
//  printData(data2MK10E, data_lenMK10E);
//  HAL_UART_Transmit(&SerialMon, (uint8_t *)dataMK10E, 10, 100);
  printf("Read LP MK10E \n");
  HAL_Delay(200);
//  HAL_UART_Transmit(&SerialMon, (uint8_t *)data2MK10E, data_lenMK10E, 100);
  HAL_Delay(200);
  HAL_UART_Transmit(&SerialMtr, (uint8_t *)data2MK10E, data_lenMK10E, 100);
  HAL_UART_Receive(&SerialMtr, (uint8_t *)dataMK10E, 100, 17000);
  data_lenMK10E = datasize(dataMK10E);
//  printData(dataMK10E,40);
  printf("\r\ndatasize LPMK10E: %d\n",data_lenMK10E);
  HAL_Delay(500);
  getRawDataMK10E(10);
//  printData(dataMK10E,20);
  decodelpdataMK10E();
  lp2JsonMK10E();
  for(int i=0;i<datasizestrMK10E(datastrMK10E);i++){
	  *outstr++=datastrMK10E[i];
  }
}

void txLPMK10ERec(int lp_index){
	dataallclearMK10E();
	data_lenMK10E = 0;
  //reading 00000400 lp rate1,2,3,4,5,unified for user0-8
  long lpindex = lp_index;
  lprecord = lpindex;
  data2MK10E[0]=0x02;
  data2MK10E[1]=0x46;
  data2MK10E[2]=0x30;
  data2MK10E[3]=0x00;
  data2MK10E[4]=lpindex>>24;
  data2MK10E[5]=lpindex>>16;
  data2MK10E[6]=lpindex>>8;
  data2MK10E[7]=(uint8_t) lpindex;
  data2MK10E[8]=0x00;
  data2MK10E[9]=0x01;
  data_lenMK10E = 10;
  setCommandLPMK10E(data2MK10E, data_lenMK10E);
  HAL_UART_Transmit(&SerialMtr, (uint8_t *)data2MK10E, data_lenMK10E, 100);
}

void txLPMK10E(int indexmin1){
	dataallclearMK10E();
	data_lenMK10E = 0;
  //reading 00000400 lp rate1,2,3,4,5,unified for user0-8
  long lpindex = (long)lplastentry-indexmin1;
  lprecord = lpindex;
  data2MK10E[0]=0x02;
  data2MK10E[1]=0x46;
  data2MK10E[2]=0x30;
  data2MK10E[3]=0x00;
  data2MK10E[4]=lpindex>>24;
  data2MK10E[5]=lpindex>>16;
  data2MK10E[6]=lpindex>>8;
  data2MK10E[7]=(uint8_t) lpindex;
  data2MK10E[8]=0x00;
  data2MK10E[9]=0x01;
  data_lenMK10E = 10;
  setCommandLPMK10E(data2MK10E, data_lenMK10E);
  HAL_UART_Transmit(&SerialMtr, (uint8_t *)data2MK10E, data_lenMK10E, 100);
}

void rxLPMK10E(char* outstr,char* datain, int len){
	memcpy(dataMK10E,datain,len);
  data_lenMK10E = datasize(dataMK10E);
  if(getRawDataMK10E(10)){
	  decodelpdataMK10E();
	  lp2JsonMK10E();
	  for(int i=0;i<datasizestrMK10E(datastrMK10E);i++){
		  *outstr++=datastrMK10E[i];
	  }
  }
}

void decodelpscaleMK10E(int a, uint8_t index){
  switch (a){
    case 13315:  lpscaleMK10E[index]=1;
                break;
    case 13331:  lpscaleMK10E[index]=1;
                break;
    case 13347:  lpscaleMK10E[index]=1;
                break;
    case 13319:  lpscaleMK10E[index]=1;
                break;
    case 32027:  lpscaleMK10E[index]=1;
                break;
    case 13351:  lpscaleMK10E[index]=1;
                break;
    case 46211:  lpscaleMK10E[index]=(double)commonscale[VOSCALEIDX];
                break;
    case 46212:  lpscaleMK10E[index]=(double)commonscale[VOSCALEIDX];
                break;
    case 46213:  lpscaleMK10E[index]=(double)commonscale[VOSCALEIDX];
                break;
    case 46208:  lpscaleMK10E[index]=(double)commonscale[IISCALEIDX];
                break;
    case 46209:  lpscaleMK10E[index]=(double)commonscale[IISCALEIDX];
                break;
    case 46210:  lpscaleMK10E[index]=(double)commonscale[IISCALEIDX];
                break;
    case 46323  :  lpscaleMK10E[index]=(double)commonscale[PFSCALEIDX];
                break;
    default:  lpscaleMK10E[index]=1;
                break;    
  }
}

void decodelpinitMK10E(){
//	printf("\nDecode LPInit MK10E\n");
  for(int i=0;i<jmlchannel;i++){
    // scale decode
    decodelpscaleMK10E(lpregm[i],i);
//    decodelpscaleMK10E(lpreg[i],i);
//    printf("0x%X %.5f\n",lpreg[i],lpscaleMK10E[i]);
//    SerialMon.print(lpreg[i]);
//    SerialMon.print(" ");
//    SerialMonprintlnMK10E(lpscaleMK10E[i],7);
  }
//  printf("\r\n");
}

int decodelpdataMK10E(){
  double temp;//, temp1;
  //int dataint;
  for(int i=0;i<jmlchannel;i++){
//    decodelpscaleMK10E(lpreg[i],i);
    decodelpscaleMK10E(lpregm[i],i);
    myInt.i = 0;
    myInt.i = (int)(dataMK10E[i*2+1]);
    myInt.i = myInt.i<<8;
    myInt.i = myInt.i | (int)(dataMK10E[i*2]);
    lpdataint[i] = myInt.i;
    lpdatauint[i] = myInt.ui;
    lpdataf[i]=(float) lpdatauint[i];
    temp=(double)lpdataf[i];
    lpdatad[i]=temp*lpscaleMK10E[i];
//    printf("0x%X %.5f %d %.5f %.5f\r\n",lpreg[i],lpscaleMK10E[i],lpdataint[i],lpdataf[i],lpdatad[i]);
  }
  return 1;
}

void lp2JsonMK10E(){
//  printData(dataMK10E, data_lenMK10E);
	datastrclearMK10E();
    strcpy(datastrMK10E,"{\"mid\":");
    strcat(datastrMK10E, meterSNMK10E);
    strcat(datastrMK10E, ",\"pid\":\"LPMK10E\",");
//    strcat(datastrMK10E, ",\"pid\":\"LPMK10E\",\"readdate\":");
//      data2clearMK10E();
//      sprintf(data2MK10E,"%d,",timestamp_now);
//      strcat(datastrMK10E, data2MK10E);
      strcat(datastrMK10E, "\"3P65\":");
      data2clearMK10E();
	sprintf(data2MK10E,"%ld,",lprecord);
    strcat(datastrMK10E, data2MK10E);
  for(int i=0;i<jmlchannel;i++){
    if(i<jmlchannel-1){
//      strcat(datastrMK10E, formatLPRegMK10E(lpreg[i]));
      strcat(datastrMK10E, formatLPRegMK10E(lpregm[i]));
      data2clearMK10E();
      sprintf(data2MK10E,":%.5f,",lpdatad[i]);
      strcat(datastrMK10E, data2MK10E);
    }
    else {
//        strcat(datastrMK10E, formatLPRegMK10E(lpreg[i]));
        strcat(datastrMK10E, formatLPRegMK10E(lpregm[i]));
        data2clearMK10E();
        sprintf(data2MK10E,":%.5f}",lpdatad[i]);
        strcat(datastrMK10E, data2MK10E);
    }
  }
}

int readEobLpInitMK10E(){
	dataallclearMK10E();
	data_lenMK10E = 0;
	dataMK10E[0]=0x02;
	dataMK10E[1]=0x52;
	dataMK10E[2]=0xF5;
	dataMK10E[3]=0x32;
	dataMK10E[4]=0x27;
	dataMK10E[5]=0xE3;
	dataMK10E[6]=0x03;
  HAL_UART_Transmit(&SerialMtr, (uint8_t *)dataMK10E, 7, 100);
  HAL_UART_Receive(&SerialMtr, (uint8_t *)dataMK10E, BUFFSIZEMK10E, 7000);
  printData(dataMK10E,datasize(dataMK10E));
  data_lenMK10E = datasize(dataMK10E);
  printf("\r\ndatasizeEOBINIT: %d\n",data_lenMK10E);
  HAL_Delay(500);
  decodeEobLpinitMK10E();
  //  getRawDataMK10E(0);
//  removeheadcrctailMK10E(4);
//  printf("datasizeEOBINIT: %d\n",data_lenMK10E);
//  HAL_Delay(1000);
//  printData(dataMK10E,data_lenMK10E);
//  HAL_Delay(1000);
  return 0;
}

void txEobLpInitMK10E(){
	dataallclearMK10E();
	data_lenMK10E = 0;
	dataMK10E[0]=0x02;
	dataMK10E[1]=0x52;
	dataMK10E[2]=0xF5;
	dataMK10E[3]=0x32;
	dataMK10E[4]=0x27;
	dataMK10E[5]=0xE3;
	dataMK10E[6]=0x03;
  HAL_UART_Transmit(&SerialMtr, (uint8_t *)dataMK10E, 7, 100);
}

void rxEobLpInitMK10E(char* datain, int len){
  memcpy(dataMK10E,datain,len);
  data_lenMK10E = datasize(dataMK10E);
  decodeEobLpinitMK10E();
}

const char * checkregEoBInit1MK10E(int reg){
  switch(reg){
    case 26899: // 0x6913 export wh tot
    case 27155: // 0x6A13 export wh tot
            return "\"3P33\"";

            break;
    case 26915: //0x6923 import wh tot
    case 27171: // 0x6A23 import wh tot
            return "\"3P37\"";

            break;
    case 26903: // 0x6917 export varh tot
    case 26647: // 0x6817 export varh tot
    case 27159: // 0x6A17 export varh tot
            return "\"3P25\"";

            break;
    case 26919: //0x6927 import varh tot
    case 26663: // 0x6827 import varh tot
    case 27175: // 0x6A27 import varh tot
            return "\"3P29\"";

            break;
    case 26907: // 0x691B export VAh tot
    case 27163: // 0x6A1B export VAh tot
            return "\"3P17\"";

            break;
    case 26923: // 0x692B import VAh tot
    case 27179: // 0x6A2B import VAh tot
            return "\"3P21\"";

            break;
    case 26883: // 0x6903 abs wh tot
    case 27139: // 0x6A03 abs wh tot
            return "\"3P66\"";

            break;
    case 27143: // 0x6A07 abs varh tot
            return "\"3P67\"";

            break;
    case 26724: // 0x6864 varh penalty
            return "\"3P69\"";

            break;
    case 255:
            return "\"00FF\"";

            break;
    default:
            return 0;
            break;
  }
}

const char *checkregEoBUnifiedMK10E(int reg){
  switch(reg){
    case 26899: // 0x6913 export wh tot
    case 27155: // 0x6A13 export wh tot
            return "\"3P70\"";

            break;
    case 26915: //0x6923 import wh tot
    case 27171: // 0x6A23 import wh tot
            return "\"3P76\"";

            break;
    case 26903: // 0x6917 export varh tot
    case 26647: // 0x6817 export varh tot
    case 27159: // 0x6A17 export varh tot
            return "\"3P82\"";

            break;
    case 26919: //0x6927 import varh tot
    case 26663: // 0x6827 import varh tot
    case 27175: // 0x6A27 import varh tot
            return "\"3P88\"";

            break;
    case 26907: // 0x691B export VAh tot
    case 27163: // 0x6A1B export VAh tot
            return "\"3P94\"";

            break;
    case 26923: // 0x692B import VAh tot
    case 27179: // 0x6A2B import VAh tot
            return "\"3P0A\"";

            break;
    case 26883: // 0x6903 abs wh tot
    case 27139: // 0x6A03 abs wh tot
            return "\"3P6A\"";

            break;
    case 27143: // 0x6A07 abs varh tot
            return "\"3PCA\"";

            break;
    case 26724: // 0x6864 varh penalty
            return "\"3PAB\"";

            break;
    case 255:
            return "\"00FF\"";

            break;
    default:
            return "\"00FF\"";
            return 0;
            break;
  }
}

const char * checkregEoBRate1MK10E(int reg){
  switch(reg){
    case 26899: // 0x6913 export wh tot
    case 27155: // 0x6A13 export wh tot
            return "\"3P71\"";

            break;
    case 26915: //0x6923 import wh tot
    case 27171: // 0x6A23 import wh tot
            return "\"3P77\"";

            break;
    case 26903: // 0x6917 export varh tot
    case 26647: // 0x6817 export varh tot
    case 27159: // 0x6A17 export varh tot
            return "\"3P83\"";

            break;
    case 26919: //0x6927 import varh tot
    case 26663: // 0x6827 import varh tot
    case 27175: // 0x6A27 import varh tot
            return "\"3P89\"";

            break;
    case 26907: // 0x691B export VAh tot
    case 27163: // 0x6A1B export VAh tot
            return "\"3P95\"";

            break;
    case 26923: // 0x692B import VAh tot
    case 27179: // 0x6A2B import VAh tot
            return "\"3P1A\"";

            break;
    case 26883: // 0x6903 abs wh tot
    case 27139: // 0x6A03 abs wh tot
            return "\"3P7A\"";

            break;
    case 27143: // 0x6A07 abs varh tot
            return "\"3PDA\"";

            break;
    case 26724: // 0x6864 varh penalty
            return "\"3PAB\"";

            break;
    case 255:
            return "\"00FF\"";

            break;
    default:
            return "\"00FF\"";
            return 0;
            break;
  }
}

const char * checkregEoBRate2MK10E(int reg){
  switch(reg){
    case 26899: // 0x6913 export wh tot
    case 27155: // 0x6A13 export wh tot
            return "\"3P72\"";

            break;
    case 26915: //0x6923 import wh tot
    case 27171: // 0x6A23 import wh tot
            return "\"3P78\"";

            break;
    case 26903: // 0x6917 export varh tot
    case 26647: // 0x6817 export varh tot
    case 27159: // 0x6A17 export varh tot
            return "\"3P84\"";

            break;
    case 26919: //0x6927 import varh tot
    case 26663: // 0x6827 import varh tot
    case 27175: // 0x6A27 import varh tot
            return "\"3P90\"";

            break;
    case 26907: // 0x691B export VAh tot
    case 27163: // 0x6A1B export VAh tot
            return "\"3P96\"";

            break;
    case 26923: // 0x692B import VAh tot
    case 27179: // 0x6A2B import VAh tot
            return "\"3P2A\"";

            break;
    case 26883: // 0x6903 abs wh tot
    case 27139: // 0x6A03 abs wh tot
            return "\"3P8A\"";

            break;
    case 27143: // 0x6A07 abs varh tot
            return "\"3PEA\"";

            break;
    case 26724: // 0x6864 varh penalty
            return "\"3PAB\"";

            break;
    case 255:
            return "\"00FF\"";

            break;
    default:
            return "\"00FF\"";
            return 0;
            break;
  }
}

const char * checkregEoBRate3MK10E(int reg){
  switch(reg){
    case 26899: // 0x6913 export wh tot
    case 27155: // 0x6A13 export wh tot
            return "\"3P73\"";

            break;
    case 26915: //0x6923 import wh tot
    case 27171: // 0x6A23 import wh tot
            return "\"3P79\"";

            break;
    case 26903: // 0x6917 export varh tot
    case 26647: // 0x6817 export varh tot
    case 27159: // 0x6A17 export varh tot
            return "\"3P85\"";

            break;
    case 26919: //0x6927 import varh tot
    case 26663: // 0x6827 import varh tot
    case 27175: // 0x6A27 import varh tot
            return "\"3P91\"";

            break;
    case 26907: // 0x691B export VAh tot
    case 27163: // 0x6A1B export VAh tot
            return "\"3P97\"";

            break;
    case 26923: // 0x692B import VAh tot
    case 27179: // 0x6A2B import VAh tot
            return "\"3P3A\"";

            break;
    case 26883: // 0x6903 abs wh tot
    case 27139: // 0x6A03 abs wh tot
            return "\"3P9A\"";

            break;
    case 27143: // 0x6A07 abs varh tot
            return "\"3PFA\"";

            break;
    case 26724: // 0x6864 varh penalty
            return "\"3PAB\"";

            break;
    case 255:
            return "\"00FF\"";

            break;
    default:
            return "\"00FF\"";
            return 0;
            break;
  }
}

const char * checkregEoBRate4MK10E(int reg){
  switch(reg){
    case 26899: // 0x6913 export wh tot
    case 27155: // 0x6A13 export wh tot
            return "\"3P74\"";

            break;
    case 26915: //0x6923 import wh tot
    case 27171: // 0x6A23 import wh tot
            return "\"3P80\"";

            break;
    case 26903: // 0x6917 export varh tot
    case 26647: // 0x6817 export varh tot
    case 27159: // 0x6A17 export varh tot
            return "\"3P86\"";

            break;
    case 26919: //0x6927 import varh tot
    case 26663: // 0x6827 import varh tot
    case 27175: // 0x6A27 import varh tot
            return "\"3P92\"";

            break;
    case 26907: // 0x691B export VAh tot
    case 27163: // 0x6A1B export VAh tot
            return "\"3P98\"";

            break;
    case 26923: // 0x692B import VAh tot
    case 27179: // 0x6A2B import VAh tot
            return "\"3P4A\"";

            break;
    case 26883: // 0x6903 abs wh tot
    case 27139: // 0x6A03 abs wh tot
            return "\"3PAA\"";

            break;
    case 27143: // 0x6A07 abs varh tot
            return "\"3P0B\"";

            break;
    case 26724: // 0x6864 varh penalty
            return "\"3PAB\"";

            break;
    case 255:
            return "\"00FF\"";

            break;
    default:
            return "\"00FF\"";
            return 0;
            break;
  }
}

const char * checkregEoBRate5MK10E(int reg){
  switch(reg){
    case 26899: // 0x6913 export wh tot
    case 27155: // 0x6A13 export wh tot
            return "\"3P75\"";

            break;
    case 26915: //0x6923 import wh tot
    case 27171: // 0x6A23 import wh tot
            return "\"3P81\"";

            break;
    case 26903: // 0x6917 export varh tot
    case 26647: // 0x6817 export varh tot
    case 27159: // 0x6A17 export varh tot
            return "\"3P87\"";

            break;
    case 26919: //0x6927 import varh tot
    case 26663: // 0x6827 import varh tot
    case 27175: // 0x6A27 import varh tot
            return "\"3P93\"";

            break;
    case 26907: // 0x691B export VAh tot
    case 27163: // 0x6A1B export VAh tot
            return "\"3P99\"";

            break;
    case 26923: // 0x692B import VAh tot
    case 27179: // 0x6A2B import VAh tot
            return "\"3P5A\"";

            break;
    case 26883: // 0x6903 abs wh tot
    case 27139: // 0x6A03 abs wh tot
            return "\"3PBA\"";

            break;
    case 27143: // 0x6A07 abs varh tot
            return "\"3P1B\"";

            break;
    case 26724: // 0x6864 varh penalty
            return "\"3PAB\"";

            break;
    case 255:
            return "\"00FF\"";

            break;
    default:
            return "\"00FF\"";
            return 0;
            break;
  }
}

void decodeEoBMK10E(){
	int1=7;
	datastrclearMK10E();
    strcpy(datastrMK10E,"{\"mid\":");
    strcat(datastrMK10E, meterSNMK10E);
    strcat(datastrMK10E, ",\"pid\":\"EoBMK10E\",");
//    strcat(datastrMK10E, ",\"pid\":\"EoBMK10E\",\"readdate\":");
//  data2clearMK10E();
//  sprintf(data2MK10E,"%d,",timestamp_now);
//  strcat(datastrMK10E, data2MK10E);
  for(int i=0;i<int1;i++){
      int3=0;
    if(i==int1-1){
    	strcat(datastrMK10E, checkregEoBRate1MK10E(eobreg[int1-1]));
    	strcat(datastrMK10E, ":");
    	sprintf(data2MK10E,"%.5f",eob0[int1-1]);
    	strcat(datastrMK10E, data2MK10E);
    	strcat(datastrMK10E, ",");
    	strcat(datastrMK10E, checkregEoBRate2MK10E(eobreg[int1-1]));
    	strcat(datastrMK10E, ":");
    	sprintf(data2MK10E,"%.5f",eob1[int1-1]);
    	strcat(datastrMK10E, data2MK10E);
    	strcat(datastrMK10E, ",");
    	strcat(datastrMK10E, checkregEoBRate3MK10E(eobreg[int1-1]));
    	strcat(datastrMK10E, ":");
    	sprintf(data2MK10E,"%.5f",eob2[int1-1]);
    	strcat(datastrMK10E, data2MK10E);
    	strcat(datastrMK10E, ",");
    	strcat(datastrMK10E, checkregEoBRate4MK10E(eobreg[int1-1]));
    	strcat(datastrMK10E, ":");
    	sprintf(data2MK10E,"%.5f",eob3[int1-1]);
    	strcat(datastrMK10E, data2MK10E);
    	strcat(datastrMK10E, ",");
    	strcat(datastrMK10E, checkregEoBRate5MK10E(eobreg[int1-1]));
    	strcat(datastrMK10E, ":");
    	sprintf(data2MK10E,"%.5f",eob4[int1-1]);
    	strcat(datastrMK10E, data2MK10E);
    	strcat(datastrMK10E, ",");
    	strcat(datastrMK10E, checkregEoBUnifiedMK10E(eobreg[int1-1]));
    	strcat(datastrMK10E, ":");
    	sprintf(data2MK10E,"%.5f",eob9[int1-1]);
    	strcat(datastrMK10E, data2MK10E);
    	strcat(datastrMK10E, "}");
    }
    else {
    	strcat(datastrMK10E, checkregEoBRate1MK10E(eobreg[i]));
    	strcat(datastrMK10E, ":");
    	sprintf(data2MK10E,"%.5f",eob0[i]);
    	strcat(datastrMK10E, data2MK10E);
    	strcat(datastrMK10E, ",");
    	strcat(datastrMK10E, checkregEoBRate2MK10E(eobreg[i]));
    	strcat(datastrMK10E, ":");
    	sprintf(data2MK10E,"%.5f",eob1[i]);
    	strcat(datastrMK10E, data2MK10E);
    	strcat(datastrMK10E, ",");
    	strcat(datastrMK10E, checkregEoBRate3MK10E(eobreg[i]));
    	strcat(datastrMK10E, ":");
    	sprintf(data2MK10E,"%.5f",eob2[i]);
    	strcat(datastrMK10E, data2MK10E);
    	strcat(datastrMK10E, ",");
    	strcat(datastrMK10E, checkregEoBRate4MK10E(eobreg[i]));
    	strcat(datastrMK10E, ":");
    	sprintf(data2MK10E,"%.5f",eob3[i]);
    	strcat(datastrMK10E, data2MK10E);
    	strcat(datastrMK10E, ",");
    	strcat(datastrMK10E, checkregEoBRate5MK10E(eobreg[i]));
    	strcat(datastrMK10E, ":");
    	sprintf(data2MK10E,"%.5f",eob4[i]);
    	strcat(datastrMK10E, data2MK10E);
    	strcat(datastrMK10E, ",");
    	strcat(datastrMK10E, checkregEoBUnifiedMK10E(eobreg[i]));
    	strcat(datastrMK10E, ":");
    	sprintf(data2MK10E,"%.5f",eob9[i]);
    	strcat(datastrMK10E, data2MK10E);
    	strcat(datastrMK10E, ",");
    }
  }
    SerialMonprintlnMK10E(datastrMK10E);
}

// ----------------------- EoB -------------------------------

const char * formatRateMK10E(uint8_t reg){
  switch(reg){
    case 0: // 0x6913 export wh tot
            return "\"3P70\"";

            break;
    case 1: // 0x6A23 import wh tot
            return "\"3P71\"";

            break;
    case 2: // 0x6A17 export varh tot
            return "\"3P72\"";

            break;
    case 3: // 0x6A27 import varh tot
            return "\"3P73\"";

            break;
    case 4: // 0x6A1B export VAh tot
            return "\"3P74\"";

            break;
    case 5: // 0x6A2B import VAh tot
            return "\"3P75\"";

            break;
    default:
        	return "\"3P75\"";
            break;
  }
}


void readEoBCurrentMK10E(char* outstr){
	dataallclearMK10E();
	dataMK10E[0]=0x02;
	dataMK10E[1]=0x52;
	dataMK10E[2]=0xF5;
	dataMK10E[3]=0x40;
	dataMK10E[4]=0x79;
	dataMK10E[5]=0x36;
	dataMK10E[6]=0x03;
	HAL_UART_Transmit(&SerialMtr, (uint8_t * )dataMK10E, 7, 100);
	HAL_UART_Receive(&SerialMtr, (uint8_t * )dataMK10E, BUFFSIZEMK10E, 7000);
	data_lenMK10E=0;
	data_lenMK10E=datasize(dataMK10E);
	printf("datasize: %d\r\n",data_lenMK10E);
	HAL_Delay(500);
	if(data_lenMK10E>5){
	//    return 1;
		getRawDataMK10E(4);
//		removeheadcrctailMK10E(4);
		printData(dataMK10E,data_lenMK10E);
		HAL_Delay(500);
		printf("Save Data EoBCurrentMK10E\r\n");
		saveEoBDataMK10E();
		HAL_Delay(500);
		printf("Decode EoBCurrentMK10E\r\n");
		HAL_Delay(500);
		decodeEoBMK10E();
//		decodeInstantMK10E();
		  for(int i=0;i<datasizestrMK10E(datastrMK10E);i++){
			  *outstr++=datastrMK10E[i];
		  }
	  }
//	  return "0";
}

void txEoBCurrentMK10E(){
	dataallclearMK10E();
	dataMK10E[0]=0x02;
	dataMK10E[1]=0x52;
	dataMK10E[2]=0xF5;
	dataMK10E[3]=0x40;
	dataMK10E[4]=0x79;
	dataMK10E[5]=0x36;
	dataMK10E[6]=0x03;
	HAL_UART_Transmit(&SerialMtr, (uint8_t * )dataMK10E, 7, 100);
}

void rxEoBCurrentMK10E(char* outstr,char* datain, int len){
	  memcpy(dataMK10E,datain,len);
	data_lenMK10E=0;
	data_lenMK10E=datasize(dataMK10E);
	if(data_lenMK10E>5){
		getRawDataMK10E(4);
		saveEoBDataMK10E();
		decodeEoBMK10E();
		  for(int i=0;i<datasizestrMK10E(datastrMK10E);i++){
			  *outstr++=datastrMK10E[i];
		  }
	  }
}

const char* readEoBEnergyMK10E(){
	dataallclearMK10E();
	dataMK10E[0]=0x02;
	dataMK10E[1]=0x52;
	dataMK10E[2]=0xF5;
	dataMK10E[3]=0x4F;
	dataMK10E[4]=0x88;
	dataMK10E[5]=0xD9;
	dataMK10E[6]=0x03;
	HAL_UART_Transmit(&SerialMtr, (uint8_t * )dataMK10E, 7, 100);
	HAL_UART_Receive(&SerialMtr, (uint8_t * )dataMK10E, BUFFSIZEMK10E, 5000);
	data_lenMK10E=0;
	data_lenMK10E=datasize(dataMK10E);
	printf("datasize: %d\r\n",data_lenMK10E);
	HAL_Delay(500);
	if(data_lenMK10E>5){
	//    return 1;
		getRawDataMK10E(4);
//		removeheadcrctailMK10E(4);
		printData(dataMK10E,data_lenMK10E);
		HAL_Delay(500);
		printf("Save Data EoBEnergyMK10E\r\n");
		saveEoBDataMK10E();
		HAL_Delay(500);
		printf("Decode EoBEnergyMK10E\r\n");
		HAL_Delay(500);
		decodeEoBMK10E();
//		decodeInstantMK10E();
		return datastrMK10E;
	  }
	  return "0";
}

const char* readEoBStanMK10E(){
	dataallclearMK10E();
	dataMK10E[0]=0x02;
	dataMK10E[1]=0x52;
	dataMK10E[2]=0xF5;
	dataMK10E[3]=0x4E;
	dataMK10E[4]=0x98;
	dataMK10E[5]=0xF8;
	dataMK10E[6]=0x03;
	HAL_UART_Transmit(&SerialMtr, (uint8_t * )dataMK10E, 7, 100);
	HAL_UART_Receive(&SerialMtr, (uint8_t * )dataMK10E, BUFFSIZEMK10E, 5000);//  SerialMonprintlnMK10E(datastrMK10E.length());
//  SerialMonprintlnMK10E(datastrMK10E);
	data_lenMK10E=0;
	data_lenMK10E=datasize(dataMK10E);
	printf("datasize: %d\r\n",data_lenMK10E);
	HAL_Delay(500);
	if(data_lenMK10E>5){
	//    return 1;
		getRawDataMK10E(4);
//		removeheadcrctailMK10E(4);
		printData(dataMK10E,data_lenMK10E);
		HAL_Delay(500);
		printf("Save Data EoBStanMK10E\r\n");
		saveEoBDataMK10E();
		HAL_Delay(500);
		printf("Decode EoBStanMK10E\r\n");
		HAL_Delay(500);
		decodeEoBMK10E();
//		decodeInstantMK10E();
		return datastrMK10E;
	  }
	  return "0";

}

void saveEoBDataMK10E(){
//  removeheadcrctailMK10E(4);
//  printData(dataMK10E, data_lenMK10E);
  SerialMonprintlnMK10E(" ");
  // exp wh
  hex2Float(dataMK10E,0);
  eob9[0]=(double)(myFloat.f);
  hex2Float(dataMK10E,4);
  eob0[0]=(double)(myFloat.f);
  hex2Float(dataMK10E,8);
  eob1[0]=(double)(myFloat.f);
  hex2Float(dataMK10E,12);
  eob2[0]=(double)(myFloat.f);
  // imp wh
  hex2Float(dataMK10E,16);
  eob9[1]=(double)(myFloat.f);
  hex2Float(dataMK10E,20);
  eob0[1]=(double)(myFloat.f);
  hex2Float(dataMK10E,24);
  eob1[1]=(double)(myFloat.f);
  hex2Float(dataMK10E,28);
  eob2[1]=(double)(myFloat.f);
  // exp varh
  hex2Float(dataMK10E,32);
  eob9[2]=(double)(myFloat.f);
  hex2Float(dataMK10E,36);
  eob0[2]=(double)(myFloat.f);
  // imp varh
  hex2Float(dataMK10E,40);
  eob9[3]=(double)(myFloat.f);
  hex2Float(dataMK10E,44);
  eob0[3]=(double)(myFloat.f);
  // abs wh
  hex2Float(dataMK10E,48);
  eob9[4]=(double)(myFloat.f);
  hex2Float(dataMK10E,52);
  eob0[4]=(double)(myFloat.f);
  hex2Float(dataMK10E,56);
  eob1[4]=(double)(myFloat.f);
  hex2Float(dataMK10E,60);
  eob2[4]=(double)(myFloat.f);
  // abs varh
  hex2Float(dataMK10E,64);
  eob9[5]=(double)(myFloat.f);
  hex2Float(dataMK10E,68);
  eob0[5]=(double)(myFloat.f);
  hex2Float(dataMK10E,72);
  eob1[5]=(double)(myFloat.f);
  hex2Float(dataMK10E,76);
  eob2[5]=(double)(myFloat.f);
  // varh penalty
  hex2Float(dataMK10E,80);
  eob9[6]=(double)(myFloat.f);
  hex2Float(dataMK10E,84);
  eob0[6]=(double)(myFloat.f);

//  for(int i=0;i<7;i++){
//    Serial.print(eob9[i], 7);
//    Serial.print(" ");
//    Serial.print(eob0[i], 7);
//    Serial.print(" ");
//    Serial.print(eob1[i], 7);
//    Serial.print(" ");
//    Serial.println(eob2[i], 7);
//  }
}

/*
void readMeterAllFunctionsMK10E() {
      switch(state){
        case 0:
                SerialMonprintlnMK10E("Login to meter");
                if(!loginToMeter()){
                    SerialMonprintlnMK10E("Error Login Meter");
                    return;
                }
//                printData(dataMK10E, data_lenMK10E);
                state++;
                break;
        case 1:
              SerialMonprintlnMK10E("Serial Number");
            if(readSN()){
              saveSN();
//              SerialMonprintlnMK10E();
              printData(meterSNMK10E,sizeof(meterSNMK10E));
              SerialMonprintlnMK10E();
            }
//            sendToServer();
              state++;
              break;
        case 2:
              SerialMonprintlnMK10E("Read Instant");
            if(readInstantMK10E()){
//              decodeInstant();
            }
            if(onlineMode)sendToServer();
              readInstantPFMK10E();
            if(onlineMode)sendToServer();
              state++;
//              state=200;
              break;
        case 3:
              SerialMonprintlnMK10E("Read F532");
            if(readEobLpInitMK10E()){
              decodeEobLpinitMK10E();
            }
              state++;
              state++;
              break;
        case 4:
//         read EoB STAN
              SerialMonprintlnMK10E("Read EoB Stan Billing");
            if(readEoBStanMK10E()){
            }
              state++;
              break;
        case 5:
              SerialMonprintlnMK10E("Read LP");
            if(readLPMK10E(1)){
              // printData(dataMK10E,data_lenMK10E);
              Serial.println();
              decodelpdataMK10E();
              lp2JsonMK10E();
            }
            if(onlineMode)sendToServer();
              state++;
//              state=0;
              break;
        case 6:
//         read EoB Energy
              SerialMonprintlnMK10E("Read EoB Energy F54F");
            if(readEoBEnergyMK10E()){
              saveEoBDataMK10E();
              decodeEoBMK10E();
            }
            if(onlineMode)sendToServer();
              state++;
              break;
        case 7:
//         read EoB Current
              SerialMonprintlnMK10E("Read EoB Current F540");
            if(readEoBCurrentMK10E()){
              saveEoBDataMK10E();
              decodeEoBMK10E();
            }
            if(onlineMode)sendToServer();
              state++;
              state=200;
              break;
        case 8:
            if(readLPMK10E(4)){
            }
              state++;
              break;
        case 9:
            if(readLPMK10E(5)){
            }
              state++;
              break;
         default:
                 state=0;
                 break;        
      }
      delay(500);
}
*/
