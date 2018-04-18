/*
 * ufocus.h
 *
 *  Created on: Nov 30, 2013
 *      Author: dev
 */

#ifndef UFOCUS_H_
#define UFOCUS_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include "avr_mcu_section.h"
#include <stdlib.h>
#include <avr/eeprom.h>

#define TxBufSize 32
volatile unsigned char RxBuf;
volatile unsigned char TxBuf[TxBufSize];
volatile unsigned char TxBufIdx;
volatile unsigned char TxBufEnd;
volatile unsigned char SerStatus;

volatile unsigned char MotStatus;
#define IdleOff 0
#define IdleOffInProg 1

volatile unsigned char ADCStatus;
#define ADCDone 0

unsigned char CmdType;
/* Bottom nibble is the received command */

unsigned int TempADC;
signed int Temperature;
signed int BaseTemp;
signed int BasePosition;
signed int TempCoeff;

unsigned char lPinD;
unsigned char Buttons;
#define Button0 0
#define Button1 1
unsigned char ButtonCnt0;
unsigned char ButtonCnt1;

volatile __int24 Position;
volatile signed int SetPosition;
volatile signed int EndDist;

volatile unsigned char step;

volatile __int24 Vel;
volatile __int24 lVel;
volatile signed char Accel;

unsigned int IdleTimer;
volatile unsigned int SafeStopVel;
#define StepTimerFreq 19531L * 100L
volatile unsigned int MaxVel;
volatile __uint24 MaxVel100;
volatile signed char MaxAccel;

unsigned char CmdData[16];
unsigned char CmdDataLen;
unsigned char CmdDataRec;
unsigned int CmdTimer;

unsigned int LEDTimer;
volatile unsigned char LEDStatus;
#define LEDRed 0
#define LEDGreen 1
#define LEDTimed 2

volatile unsigned int Tmr1ms;

unsigned int saveMaxVel;
unsigned char saveAccel;
unsigned char saveIdle;

unsigned int CalcDelay(__uint24 AbsVel);
void CalcMaxVel100();

static inline uint8_t friden_sqrt16(uint16_t val)
{
	uint16_t rem = 0;
	uint16_t root = 0;
	uint8_t i;

	for(i = 0; i < 8; i++)
	{
		root <<= 1;
		rem = ((rem << 2) + (val >> 14));
		val <<= 2;
		root++;
		if (root <= rem)
		{
			rem -=root;
			root++;
		}
		else
		{
			root--;
		}
	}
	return (uint8_t)(root >> 1);
}

__uint24 CalcSafeStopVel(unsigned char accel);

unsigned char AddToTx(unsigned char cmd, unsigned char* data, unsigned char datalen);
void ProcessRx(void);
void CheckSwitches(void);
void Step(unsigned char phase);
void LED(void);

void InitSerial(void);
void InitAD(void);
int main(void);

#endif /* UFOCUS_H_ */
