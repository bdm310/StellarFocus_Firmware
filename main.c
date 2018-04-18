/*
 * main.c
 *
 *  Created on: Nov 30, 2013
 *      Author: dev
 */

#include "ufocus.h"

unsigned int EEMEM saveMaxVel = 500;
unsigned char EEMEM saveAccel = 10;
unsigned char EEMEM saveIdle = 1;

__uint24 CalcSafeStopVel(unsigned char accel)
{
	unsigned int trueaccel = (unsigned int)accel * 100;

	return friden_sqrt16(trueaccel) * 200;
}

void CalcMaxVel100()
{
	MaxVel100 = 100 * (__uint24)MaxVel;
}

/* Set up the serial system */
void InitSerial(void)
{
	/* Initialize TX/RX ring buffers */
	TxBufIdx = 0;
	TxBufEnd = 0;

	/*
	 * Configure the USART for:
	 * 115200 Baud
	 * 8 bit data
	 */
	UBRR0 = 10;
	UCSR0A = 0;
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
}

/* Set up the AD converter */
void InitAD(void)
{
	/* Select 5 Vref, right adjust, ADC7 */
	ADMUX = 0b01000111;
	/* Enable ADC, ADC interrupt, select 128 clock prescaler */
	ADCSRA = 0b10001111;
	/* Disable digital input buffers on ADC0 and ADC1 */
	DIDR0 = 0b00000011;
}

/* Handle the button state */
void CheckSwitches(void)
{
	unsigned char TempPIND = PIND;
	/* Check Port C for any changed switches */
	lPinD ^= TempPIND;

	cli();
	/* Is button 1 released? */
	if(TempPIND & 0b00001000)
	{
		/* Was it pressed the last time we were here? */
		if(lPinD & 0b00001000)
		{
			/* Was it pressed for less than 200ms? */
			if(ButtonCnt1 < 10)
			{
				/* Jog 1 step negative */
				if(SetPosition) SetPosition --;
			}
			else
			{
				SetPosition = (signed int)Position;
			}
		}

		/* Clear our timer */
		ButtonCnt1 = 0;
	}
	else
	{
		/* If it's been pressed for more than 200ms, declare it */
		if(ButtonCnt1 == 10)
		{
			/* Jog negative */
			SetPosition = -32768;
		}
		else
		{
			/* Increment our timer */
			ButtonCnt1 ++;
		}
	}

	/* Is button 0 released? */
	if(TempPIND & 0b00000100)
	{
		/* Was it pressed the last time we were here? */
		if(lPinD & 0b00000100)
		{
			/* Was it pressed for less than 200ms? */
			if(ButtonCnt0 < 10)
			{
				/* Jog 1 step positive */
				if(~SetPosition) SetPosition ++;
			}
			else
			{
				SetPosition = (signed int)Position;
			}
		}

		/* Clear our timer */
		ButtonCnt0 = 0;
	}
	else
	{
		/* If it's been pressed for more than 200ms, declare it */
		if(ButtonCnt0 == 10)
		{
			/* Jog positive */
			SetPosition = 32767;
		}
		else
		{
			/* Increment our timer */
			ButtonCnt0 ++;
		}
	}
	sei();

	/* Save Port C */
	lPinD = TempPIND & 0b00001100;
}

/* Handles flashing the LED */
void LED(void)
{
	/* Are we currently waiting to turn the LED off? */
	if(LEDStatus & (1<<LEDTimed))
	{
		/* Has the LED been on for more than 100ms? */
		if(Tmr1ms - LEDTimer > 100)
		{
			/* Turn it off and kill the timing flag */
			LEDStatus &= ~(1<<LEDTimed);
			PORTD &= 0b01101111;
		}
	}
	else
	{
		/* Has someone requested something? */
		if(LEDStatus)
		{
			/* Set up the timer */
			LEDTimer = Tmr1ms;
			LEDStatus |= 1<<LEDTimed;

			/* Has someone requested Green? */
			if(LEDStatus & (1<<LEDGreen))
			{
				/* Set Green and kill the flag */
				PORTD &= 0b11101111;
				PORTD |= 0b10000000;
				LEDStatus &= ~(1<<LEDGreen);
			}
			/* If not, someone has requested Red */
			else
			{
				/* Set Red and kill the flag */
				PORTD &= 0b01111111;
				PORTD |= 0b00010000;
				LEDStatus &= ~(1<<LEDRed);
			}
		}
	}
}

int main(void)
{
	unsigned int TempTmr;
	unsigned int DebounceTimer;

	/* Initialize the serial system */
	InitSerial();

	/* Initialize the AD converter */
	InitAD();

	/* Set up 1ms timer */
	OCR2A = 156;
	TCCR2A = 0b00000010;
	TCCR2B = 0b00000101;
	TIMSK2 = 0b00000010;

	/* Set up motor control timer and variables */
	TCCR1A = 0b00000000;
	TCCR1B = 0b00001000;
	TIMSK1 = 0b00000010;

	Temperature = 0xEFFF;
	MaxVel = eeprom_read_word(&saveMaxVel);
	CalcMaxVel100();
	MaxAccel = eeprom_read_byte(&saveAccel);
	if(eeprom_read_byte(&saveIdle))	MotStatus |= 1<<IdleOff;
	SafeStopVel = CalcSafeStopVel(MaxAccel);

	/* Set up port pins
	 * B1, B2 Motor control outputs
	 * D0 TX
	 * D4, D5 Motor control outputs
	 * D6, D7 LED outputs
	 */
	DDRB = 0b00000110;
	DDRC = 0b00000000;
	DDRD = 0b11110001;
	/* Enable pull ups on D2, D3 Buttons */
	PORTD |= 0b00001100;
	/* Set last button pin state */
	lPinD = PIND & 0b00001100;

	/* Initialize the button handler */
	CheckSwitches();

	/* Initialize main loop timers */
	TempTmr = Tmr1ms;
	DebounceTimer = Tmr1ms;

	/* Start the show */
	sei();

	/* Indicate that we've started up by flashing both colors */
	LEDStatus = (1<<LEDRed) | (1<<LEDGreen);

	/* Main loop */
	while(1)
	{
		/* Process any received serial commands, if they exist */
		if(SerStatus) ProcessRx();

		/* Get the current time */
		cli();
		unsigned int localTmr1ms = Tmr1ms;
		sei();

		/* Has our command timer timed out while a command was in progress? */
		if((localTmr1ms - CmdTimer > 1000) && CmdDataRec)
		{
			/* Clear the command status and send back an invalid command byte */
			CmdDataRec = 0;

			AddToTx(CmdDataRec*16, CmdData, CmdDataRec);
		}

		/* Handle the LED */
		LED();

		/* Are we waiting to turn off the motor? */
		if(MotStatus & 1<<IdleOffInProg)
		{
			/* Has a second elapsed since we stopped moving? */
			if(localTmr1ms - IdleTimer > 1000)
			{
				/* Keep the ISR out of our business */
				cli();
				/* Is the step timer disabled? */
				if((TCCR1B & 0b00000101) == 0)
				{
					/* Turn off the motor */
					Step(8);
				}
				/* Clear the flag */
				MotStatus &= ~(1<<IdleOffInProg);
				sei();
			}
		}
		else if(MotStatus & 1<<IdleOff)
		{
			/* Is the motor stopped? */
			if((TCCR1B & 0b00000101) == 0)
			{
				/* Indicate that we're waiting to kill power */
				MotStatus |= 1<<IdleOffInProg;
				/* Save off the current time */
				IdleTimer = localTmr1ms;
			}
		}

		/* Is it time to check for a button press? */
		if(localTmr1ms - DebounceTimer >= 20)
		{
			/* Save the time */
			DebounceTimer = localTmr1ms;

			/* Update the button state */
			CheckSwitches();
		}

		/* Is it time to attempt a temperature measurement? */
		if(localTmr1ms - TempTmr >= 500) //TempTimeout)
		{
			/* Save off the time */
			TempTmr = localTmr1ms;

			/* Initiate a Temp conversion if we aren't waiting to process a previous conversion
			 * or are in the middle of one
			 */
			if((~ADCStatus & 0x01) && (~ADCSRA & 0b01000000))
			{
				/* Select ADC7 */
				ADMUX &= 0b01110000;
				ADMUX |= 0b00000111;
				/* Start conversion */
				ADCSRA |= 0b01000000;
			}
		}

		/* Do we have an AD conversion to process? */
		if(ADCStatus & 0x01)
		{
			/* Kill the flag */
			ADCStatus &= ~0x01;

			/* Store a local copy of ADC */
			unsigned int lADC = ADC;

			/* Is the ADC value unreasonable? */
			if(lADC > 0x3F0 || lADC < 0x010)
			{
				/* Indicate that something is wrong */
				Temperature = 0xEFFF;
			}
			else
			{
				if(Temperature == 0xEFFF)
				{
					TempADC = lADC<<4;
				}
				else
				{
					TempADC = (signed int)TempADC + ((long)(lADC<<4) - (signed int)TempADC)/16;
				}

				lADC = TempADC/16;

				unsigned int slope;
				signed int intercept;
				unsigned int offset;

				/* Do linear interpolation to find the temperature *10 */
				if(lADC < 277)
				{
					slope = 279;
					intercept = 800;
					offset = 276;
				}
				else if (lADC < 876)
				{
					slope = 150;
					intercept = 100;
					offset = 875;
				}
				else
				{
					slope = 335;
					intercept = -200;
					offset = 989;
				}

				Temperature = (signed int)((((__int24)offset - (__int24)lADC) * slope)>>7) + intercept;

				/* Is temp comp enabled? */
				if(TempCoeff)
				{
					/* Update the set point based on the temperature change since comp was enabled */
					SetPosition = BasePosition + (int)(((__int24)(Temperature - BaseTemp) * (__int24)TempCoeff)/16);
				}
			}
		}
	}

	return(1);
}

