/*
 * comms.c
 *
 *  Created on: Dec 3, 2013
 *      Author: dev
 */

#include "ufocus.h"

/* Assembles and processes commands from the serial port
 *
 * High nibble of the command byte is the length of the data associated with the command
 * Low nibble is the command value
 *
 * Commands:
 * 00 -
 * 01 - Returns the position, 16 bit signed
 * 02 - Move, 16 bit signed position parameter absolute
 * 03 - Halt, position is saved when the command is received.  Motor will return to that position after deceleration.
 * 04 - Enable temperature compensation with the specified 16 bit signed coefficient.  Coefficient of zero disables temp comp.
 * 05 - Returns the temp comp state and motor velocity, acceleration, and idle off settings
 * 06 - Set motor velocity, acceleration, and idle off settings
 * 07 - Zero the current Position and SetPosition
 * 08 - Return the home switch state
 * 09 - Set a temporary Max Vel
 * 10 - Return the temperature
 * 11 - Return motion status
 * 12 -
 * 13 -
 * 14 -
 * 15 -
 */
void ProcessRx(void)
{
	cli();

	/* Pull the first byte out of the buffer */
	unsigned char RxByte = RxBuf;

	if(SerStatus == 1)
	{
		/* Is there a command in progress? */
		if(CmdDataRec)
		{
			/* Save the byte in our command buffer */
			CmdData[CmdDataRec - 1] = RxByte;

			/* Increment the command data counter */
			CmdDataRec ++;

			/* Reset the command timeout */
			CmdTimer = Tmr1ms;
		}
		else
		{
			/* Pull our command data length out of the command byte */
			CmdDataLen = (RxByte & 0xF0)>>4;
			/* Pull the command value out of the byte and indicate that there's a command in progress */
			CmdType = (RxByte & 0x0F);

			CmdDataRec ++;

			/* Reset the command timeout */
			CmdTimer = Tmr1ms;
		}

		/* Have we received all of the bytes for this command yet? */
		if(CmdDataRec == CmdDataLen + 1)
		{
			/* Save a local Position */
			signed int Pos = (signed int)Position;

			/* Sort out what command we're processing */
			switch(CmdType)
			{
			/* Return Position */
			case 0x01:
				CmdData[0] = *(char *)&Pos;
				CmdData[1] = *((char *)&Pos + 1);
				AddToTx(0x21, CmdData, 2);
				break;
			/* Set Position */
			case 0x02:
				*(char *)&SetPosition = CmdData[0];
				*((char *)&SetPosition + 1) = CmdData[1];
				AddToTx(0x22, CmdData, 2);
				break;
			/* Halt */
			case 0x03:
				SetPosition = Pos;
				AddToTx(0x03, 0, 0);
				break;
			/* Enable temperature compensation */
			case 0x04:
				BaseTemp = Temperature;
				BasePosition = Pos;
				*(char *)&TempCoeff = CmdData[0];
				*((char *)&TempCoeff + 1) = CmdData[1];
				AddToTx(0x24, CmdData, 2);
				break;
			/* Return temp coefficient, motor velocity, position, and idle off setting */
			case 0x05:
				CmdData[0] = *(char *)&TempCoeff;
				CmdData[1] = *((char *)&TempCoeff + 1);
				CmdData[2] = MotStatus & (1<<IdleOff);
				CmdData[3] = MaxAccel;
				CmdData[4] = *(char *)&MaxVel;
				CmdData[5] = *((char *)&MaxVel + 1);
				AddToTx(0x65, CmdData, 6);
				break;
			/* Set motor velocity, acceleration, and idle off settings*/
			case 0x06:
				if(CmdData[3]) MotStatus |= 1<<IdleOff;
				else MotStatus &= ~(1<<IdleOff);

				// Make sure accel is non-zero
				MaxAccel = CmdData[2];
				if(MaxAccel > 127) MaxAccel = 127;
				if(MaxAccel == 0) MaxAccel = 1;
				SafeStopVel = CalcSafeStopVel(MaxAccel);

				*(char *)&MaxVel = CmdData[0];
				*((char *)&MaxVel + 1) = CmdData[1];
				//Limit max velocity to 1-2000
				if(MaxVel > 2000) MaxVel = 2000;
				if(MaxVel == 0) MaxVel = 1;
				CalcMaxVel100();

				eeprom_update_word(&saveMaxVel, MaxVel);
				eeprom_update_byte(&saveAccel, MaxAccel);
				eeprom_update_byte(&saveIdle, CmdData[3]);
				AddToTx(0x46, CmdData, 4);
				break;
			/* Set the current position and set point */
			case 0x07:
				*(char *)&SetPosition = CmdData[0];
				*((char *)&SetPosition + 1) = CmdData[1];
				Position = (__int24)SetPosition;
				AddToTx(0x27, CmdData, 2);
				break;
			/* Return the home switch state */
			case 0x08:
				CmdData[0] = PINB & 0x01;
				AddToTx(0x18, CmdData, 1);
				break;
			/* Set a temporary velocity limit*/
			case 0x09:
				*(char *)&MaxVel = CmdData[0];
				*((char *)&MaxVel + 1) = CmdData[1];
				CalcMaxVel100();

				AddToTx(0x29, CmdData, 2);
				break;
			/* Return the measured temperature */
			case 0x0A:
				CmdData[0] = *(char *)&Temperature;
				CmdData[1] = *((char *)&Temperature + 1);
				AddToTx(0x2A, CmdData, 2);
				break;
			/* Return the moving status */
			case 0x0B:
				CmdData[0] = TCCR1B & 0b00000101;
				AddToTx(0x1B, CmdData, 1);
				break;
			/* Return unknown command */
			default:
				AddToTx(0x0F, 0, 0);
				break;
			}

			/* Indicate that we're done with this command */
			CmdDataRec = 0;
		}
	}
	else
	{
		/* Clear the command status and send back an serial receive error */
		CmdDataRec = 0;

		AddToTx(CmdDataRec*16 + 0x0E, &CmdData[0], CmdDataRec);
	}

	sei();

	/* Flash the red LED */
	LEDStatus |= 1<<LEDRed;
	SerStatus = 0;
}

/* Add a command to the TX buffer */
unsigned char AddToTx(unsigned char cmd, unsigned char* data, unsigned char datalen)
{
	while(TxBufEnd) { };

	cli();

	/* Add the packet to the TX buffer */
	for(unsigned char i = 0; i <= datalen; i++)
	{
		/* Add a byte */
		if(i)
		{
			TxBuf[TxBufEnd] = *data;
			data ++;
		}
		else TxBuf[TxBufEnd] = cmd;

		if(i < datalen)
		{
			/* Increment the local buffer end pointer */
			TxBufEnd ++;

			/* If we've run out of buffer, return an error and forget we added anything */
			if(TxBufEnd == TxBufSize)
			{
				sei();
				return(1);
			}
		}
	}

	/* Enable UDRIE interrupt */
	UCSR0B |= 1<<UDRIE0;
	sei();

	/* Flash the green LED */
	LEDStatus |= 1<<LEDGreen;

	/* Return OK */
	return(0);
}
