/*
	tg_motors.c
	TinyG Motor Driver - Generic file
	Written by: Alden Hart
	Revision: 03/20/10
*/

#include <stdio.h>
#include <avr\io.h>
#define F_CPU 32000000UL
#include <util\delay.h>

#include <tg_motors.h>

void initMotors()
{
};


void run_motor (void)
{
  while(1)
  {
	PORTA.OUT = st_step;
	_delay_us(1);
	PORTA.OUT = 0;
	_delay_us(1000);
  }
}
