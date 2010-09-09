/*
	tg_motors.h
	TinyG Motor Driver - Include file
	Written by: Alden Hart
	Revision: 03/20/10
*/

#define st_uStep1 (1<<4);		// stepper controller bits
#define st_uStep0 (1<<3);
#define st_enable (1<<2);
#define st_dir  (1<<1);
#define st_step (1<<0);


void initMotors(void);
