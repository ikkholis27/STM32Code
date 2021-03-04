//-------------------------------------------------------------------------------------
// CRC16 support class
// Based on various examples found on the web
// Copyright (C) 2014 Vincenzo Mennella (see license.txt)
// History
//  0.1.0 31/05/2014:   First public code release
//  0.1.1 17/12/2014:   Minor revision and commented code
//  0.1.2 06/06/2019:   Fix reflect routine for 16 bit data
//                      Added ModBus and Mcrf4XX inline functions
//
// License
// "MIT Open Source Software License":
// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this software and associated documentation files (the "Software"), to deal in the
// Software without restriction, including without limitation the rights to use, copy,
// modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
// and to permit persons to whom the Software is furnished to do so, subject to
// the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
// IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//-------------------------------------------------------------------------------------
#ifndef CRC16_H
#define CRC16_H

#include <stdint.h>
#include <stdbool.h>

//Crc parameters
//uint16_t _msbMask = 0x8000;
//uint16_t _mask = 0xFFFF;
//uint16_t _xorIn= 0x0000;
//uint16_t _xorOut= 0x0000;
//uint16_t _polynomial= 0x1021;
//uint8_t _reflectIn = false;
//uint8_t _reflectOut = false;

#define _msbMask 0x8000
#define _mask 0xFFFF
#define _xorIn 0x0000
#define _xorOut 0x0000
#define _polynomial 0x1021
#define _reflectIn false
#define _reflectOut false

//Crc value
//uint16_t _crc= 0x0000;
uint8_t reflect(uint8_t data);
uint16_t reflect16(uint16_t data);

void clearCrc();
void updateCrc(uint8_t data);
uint16_t getCrc();
unsigned int fastCrc(uint8_t data[], uint8_t start, uint16_t length, uint8_t reflectIn, uint8_t reflectOut, uint16_t polynomial, uint16_t xorIn, uint16_t xorOut, uint16_t msbMask, uint16_t mask);
unsigned int XModemCrc(uint8_t data[], uint8_t start, uint16_t length);
unsigned int Mcrf4XX(uint8_t data[], uint8_t start, uint16_t length);
unsigned int Modbus(uint8_t data[], uint8_t start, uint16_t length);

#endif
