/*
 * isr_c.c
 *
 *  Created on: Nov 30, 2013
 *      Author: dev
 */

#include "ufocus.h"
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

/* ADC */
ISR(ADC_vect)
{
	/* Notify the main loop that we're done converting */
	ADCStatus |= 0x01;
}

/* Stops the motor if the velocity is below a safe level */
void StopIfSafe(__uint24 AbsVel)
{
	/* Is it safe to stop? */
	if(AbsVel < SafeStopVel)
	{
		/* Turn off clock to T1 */
		TCCR1B &= 0b11111000;
		/* Set the timeout to the slowest */
		OCR1A = CalcDelay(1);
		/* Zero out movement variables */
		TCNT1 = 0;
		Vel = 0;
		Accel = 0;
		EndDist = 0;
	}
}

/* 1ms Timer */
ISR(TIMER2_COMPA_vect)
{
	/* We use the sign of Vel and lVel a lot, so precalculate them.
	 * Also precalc the absolute velocity.
	 */
	unsigned char VelSign = Vel > 0;
	unsigned char lVelSign = lVel > 0;
	__uint24 AbsVel = (__uint24)labs((signed long)Vel);

	/* Increment the global 1ms clock */
	Tmr1ms ++;

	/* Remember our last velocity and update our current velocity */
	lVel = Vel;
	Vel += ((signed int)Accel) * 10;

	/* Saturate velocity to +/- MaxVel */
	if(AbsVel > MaxVel100)
	{
		if(VelSign) Vel = MaxVel100;
		else Vel = -MaxVel100;
	}

	/* If there's a zero crossing in velocity, reset our ending distance counter */
	if(lVelSign ^ VelSign) EndDist = 0;

	/* Are we on position and going slow enough to stop? */
	if(Position == (__int24)SetPosition)
	{
		StopIfSafe(AbsVel);
	}

	/* Is the step timer running? */
	if(TCCR1B & 0b00000101)
	{
		/* Get what our OC timeout should be for our current velocity */
		unsigned int TempOCR1A = CalcDelay(AbsVel);

		/* Is the next timeout short enough that we should just do it now? */
		if(TempOCR1A <= TCNT1)
		{
			/* Turn off clock to T1 */
			TCCR1B &= 0b11111000;
			/* Set our new timeout and set the timer so that it'll timeout immediately */
			OCR1A = TempOCR1A;
			TCNT1 = TempOCR1A - 2;
			/* Enable T1 clock */
			TCCR1B |= 0b00000101;
		}
	}
	else
	{
		/* Do we need to move? */
		if(Position != SetPosition)
		{
			/* Re-enable the motor and set a slow timeout */
			Step(step);
			OCR1A = CalcDelay(0);
			TCNT1 = 0;

			/* Enable T1 clock */
			TCCR1B |= 0b00000101;
		}
	}
}

/* Step timer */
ISR(TIMER1_COMPA_vect)
{
	/* We use the sign of Vel and Accel a lot, so precalculate them.
	 * Also precalc the absolute velocity and distance to the set position.
	 */
	unsigned char VelSign = Vel >= 0;
	unsigned char AccelSign = Accel >= 0;
	__int24 Distance = (__int24)SetPosition - Position;
	__uint24 AbsVel = (__uint24)labs((signed long)Vel);

	/* Look up what our timeout value should be for our current velocity and set it */
	OCR1A = CalcDelay(AbsVel);

	/* Are we moving? */
	if(AbsVel)
	{
		/* Is our velocity positive? */
		if(VelSign)
		{
			/* Increment and wrap the step counter */
			step ++;
			if(step == 8) step = 0;

			/* Increment position */
			Position ++;

			/* Are we accelerating? */
			if(Accel)
			{
				/* If we're increasing velocity, increment the ending distance.
				 * If we're decreasing velocity, decrement the ending distance until it zeros out.
				 */
				if(AccelSign)
				{
					EndDist ++;
				}
				else
				{
					if(EndDist) EndDist --;
				}
			}
		}
		/* Negative velocity */
		else
		{
			/* Decrement and wrap the step counter */
			step --;
			if(step == 0xFF) step = 7;

			/* Decrement position */
			Position --;

			/* Are we accelerating? */
			if(Accel)
			{
				/* If we're decreasing velocity, increment the ending distance.
				 * If we're increasing velocity, decrement the ending distance until it zeros out.
				 */
				if(AccelSign)
				{
					if(EndDist) EndDist --;
				}
				else
				{
					EndDist ++;
				}
			}
		}
	}
	/* Not moving */
	else
	{
		/* Zero out the ending distance */
		EndDist = 0;
	}

	/* Perform the actual step */
	Step(step);

	/* Do we need to move? */
	if(Distance)
	{
		/* If we're already at the maximum velocity, don't accelerate */
		if(AbsVel >= MaxVel100)
		{
			Accel = 0;
		}
		else
		{
			/* Accelerate in the direction of the distance */
			if(Distance > 0)
			{
				Accel = MaxAccel;
			}
			else
			{
				Accel = -MaxAccel;
			}
		}

		/* Should we be slowing down? */
		if(abs(Distance) <= EndDist)
		{
			/* Accelerate opposite our velocity */
			if(VelSign)
			{
				Accel = -MaxAccel;
			}
			else
			{
				Accel = MaxAccel;
			}
		}
	}
	/* Zero distance */
	else
	{
		StopIfSafe(AbsVel);
	}
}

ISR(USART_RX_vect)
{
	/* Read the USART data */
	unsigned char RXErr = UCSR0A & 0b00011100;
	unsigned char RX = UDR0;

	if(!RXErr)
	{
		/* Start filling our buffer and let the main loop know there's something going on */
		RxBuf = RX;
		SerStatus = 1;
	}
	else
	{
		SerStatus |= RXErr;
		SerStatus &= 0b00000000;
	}
}

ISR(USART_UDRE_vect)
{
	/* Pull the next byte out and send it */
	UDR0 = TxBuf[TxBufIdx];

	/* Is the buffer empty? */
	if(TxBufIdx == TxBufEnd)
	{
		/* Turn off the UDR0 interrupt*/
		UCSR0B &= ~(1<<UDRIE0);
		TxBufIdx = 0;
		TxBufEnd = 0;
	}
	else
	{
		/* Increment our start pointer */
		TxBufIdx ++;
	}
}
