/*
 * motor.c
 *
 *  Created on: Dec 3, 2013
 *      Author: dev
 */

#include "ufocus.h"

/* Calculates what our step delay is for a given velocity */
unsigned int CalcDelay(__uint24 AbsVel)
{
	if(AbsVel > SafeStopVel)
	{
		return (unsigned int)(StepTimerFreq / AbsVel);
	}
	else
	{
		return (unsigned int)(StepTimerFreq / SafeStopVel);
	}
}

/* Sets the port pins to what they should be for each of the 8 phases
 * anything above 7 simply turns both coils off
 *
 * PD6 - M11
 * PD5 - M12
 *
 * PB1 - M21
 * PB2 - M22
 */
void Step(unsigned char phase)
{
	switch(phase)
	{
	case 0:
		/*
		 * Out11 - H
		 * Out12 - L
		 * Out21 - Off
		 * Out22 - Off
		 */
		PORTD &= 0b11011111;
		PORTD |= 0b01000000;
		PORTB &= 0b11111001;
		PORTB |= 0b00000000;
		break;
	case 1:
		/*
		 * Out11 - H
		 * Out12 - L
		 * Out21 - L
		 * Out22 - H
		 */
		PORTD &= 0b11011111;
		PORTD |= 0b01000000;
		PORTB &= 0b11111101;
		PORTB |= 0b00000100;
		break;
	case 2:
		/*
		 * Out11 - Off
		 * Out12 - Off
		 * Out21 - L
		 * Out22 - H
		 */
		PORTD &= 0b10011111;
		PORTD |= 0b00000000;
		PORTB &= 0b11111101;
		PORTB |= 0b00000100;
		break;
	case 3:
		/*
		 * Out11 - L
		 * Out12 - H
		 * Out21 - L
		 * Out22 - H
		 */
		PORTD &= 0b10111111;
		PORTD |= 0b00100000;
		PORTB &= 0b11111101;
		PORTB |= 0b00000100;
		break;
	case 4:
		/*
		 * Out11 - L
		 * Out12 - H
		 * Out21 - Off
		 * Out22 - Off
		 */
		PORTD &= 0b10111111;
		PORTD |= 0b00100000;
		PORTB &= 0b11111001;
		PORTB |= 0b00000000;
		break;
	case 5:
		/*
		 * Out11 - L
		 * Out12 - H
		 * Out21 - H
		 * Out22 - L
		 */
		PORTD &= 0b10111111;
		PORTD |= 0b00100000;
		PORTB &= 0b11111011;
		PORTB |= 0b00000010;
		break;
	case 6:
		/*
		 * Out11 - Off
		 * Out12 - Off
		 * Out21 - H
		 * Out22 - L
		 */
		PORTD &= 0b10011111;
		PORTD |= 0b00000000;
		PORTB &= 0b11111011;
		PORTB |= 0b00000010;
		break;
	case 7:
		/*
		 * Out11 - H
		 * Out12 - L
		 * Out21 - H
		 * Out22 - L
		 */
		PORTD &= 0b11011111;
		PORTD |= 0b01000000;
		PORTB &= 0b11111011;
		PORTB |= 0b00000010;
		break;
	default:
		/*
		 * Out11 - Off
		 * Out12 - Off
		 * Out21 - Off
		 * Out22 - Off
		 */
		PORTD &= 0b10011111;
		PORTB &= 0b11111001;
		break;
	}
}
