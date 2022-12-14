/*
 * timer.c
 *
 *  Created on: Nov 30, 2022
 *      Author: HELLO SON
 */

#include "main.h"
#include "timer.h"

int reset_flag=0;
int reset_counter=0;
int run_flag=0;
int run_counter=0;
int pes_flag=0;
int pes_counter=0;
int blink_flag=0;
int blink_counter=0;
int buzzer_flag=0;
int buzzer_counter=0;
void setTimerReset(int duration)
{
	reset_counter=duration/10;
	reset_flag=0;
}
void setTimerRun(int duration)
{
	run_counter=duration/10;
	run_flag=0;
}
void setTimerPesBut(int duration)
{
	pes_counter=duration/10;
	pes_flag=0;
}
void setTimerBlink(int duration)
{
	blink_counter=duration/10;
	blink_flag=0;
}
void setTimerBuzzer(int duration)
{
	buzzer_counter=duration/10;
	buzzer_flag=0;
}
void timerRun()
{
	if(reset_counter>0)
	{
		reset_counter--;
		if(reset_counter<=0)
		{
			reset_flag=1;
		}
	}
	if(pes_counter>0)
	{
		pes_counter--;
		if(pes_counter<=0)
		{
			pes_flag=1;
		}
	}
	if(run_counter>0)
	{
		run_counter--;
		if(run_counter<=0)
		{
			run_flag=1;
		}
	}
	if(blink_counter>0)
	{
		blink_counter--;
		if(blink_counter<=0)
		{
			blink_flag=1;
		}
	}
	if(buzzer_counter>0)
	{
		buzzer_counter--;
		if(buzzer_counter<=0)
		{
			buzzer_flag=1;
		}
	}
}
void HAL_TIM_PeriodElapsedCallback ( TIM_HandleTypeDef * htim )
{
if(htim->Instance == TIM2 ) {
button_reading() ;
timerRun();
}
}
