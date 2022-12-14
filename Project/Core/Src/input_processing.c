/*
 * input_processing.c
 *
 *  Created on: Nov 30, 2022
 *      Author: HELLO SON
 */
#include "input_processing.h"
#include "input_reading.h"
#include "timer.h"
#include "main.h"
#include "stdio.h"

#define RED 	0
#define YELLOW 	1
#define GREEN 	2

#define MAN_RED 	11
#define MAN_YELLOW 	12
#define MAN_GREEN	13

#define button1 0
#define button2 1
#define button3 2
#define pes_but 3

#define AUTO_RED 	0
#define AUTO_YELLOW 1
#define AUTO_GREEN 	2

static uint8_t lightbufferMod[3]={1,1,1};
static uint8_t lightbufferRun[3]={5,2,3};
int status_1;
int status_2;
int time1;
int time2;
int mode;
int initial_flag=1;

void initial()
{
	status_1=AUTO_RED;
	status_2=AUTO_GREEN;
	mode=1;
	time1=lightbufferRun[AUTO_RED];
	time2=lightbufferRun[AUTO_GREEN];
	setTimerRun(1000);
}

void traffic()
{
	if(status_1==AUTO_RED)
	{
		HAL_GPIO_WritePin(Traffic_1_RED_GPIO_Port, Traffic_1_RED_Pin, SET);
		HAL_GPIO_WritePin(Traffic_1_GREEN_GPIO_Port, Traffic_1_GREEN_Pin, RESET);
		if(time1==0)
		{
			status_1=AUTO_GREEN;
			HAL_GPIO_WritePin(Traffic_1_RED_GPIO_Port, Traffic_1_RED_Pin, RESET);
			HAL_GPIO_WritePin(Traffic_1_GREEN_GPIO_Port, Traffic_1_GREEN_Pin, SET);
			time1=lightbufferRun[AUTO_GREEN];
		}
	}
	else if(status_1==AUTO_GREEN)
	{
		HAL_GPIO_WritePin(Traffic_1_RED_GPIO_Port, Traffic_1_RED_Pin, RESET);
		HAL_GPIO_WritePin(Traffic_1_GREEN_GPIO_Port, Traffic_1_GREEN_Pin, SET);
		if(time1==0)
		{
			status_1=AUTO_YELLOW;
			HAL_GPIO_WritePin(Traffic_1_RED_GPIO_Port, Traffic_1_RED_Pin, SET);
			HAL_GPIO_WritePin(Traffic_1_GREEN_GPIO_Port, Traffic_1_GREEN_Pin, SET);
			time1=lightbufferRun[AUTO_YELLOW];
		}
	}
	else if(status_1==AUTO_YELLOW)
	{
		HAL_GPIO_WritePin(Traffic_1_RED_GPIO_Port, Traffic_1_RED_Pin, SET);
		HAL_GPIO_WritePin(Traffic_1_GREEN_GPIO_Port, Traffic_1_GREEN_Pin, SET);
		if(time1==0)
		{
			status_1=AUTO_RED;
			HAL_GPIO_WritePin(Traffic_1_RED_GPIO_Port, Traffic_1_RED_Pin, SET);
			HAL_GPIO_WritePin(Traffic_1_GREEN_GPIO_Port, Traffic_1_GREEN_Pin, RESET);
			time1=lightbufferRun[AUTO_RED];
		}
	}
	if(status_2==AUTO_RED)
	{
		HAL_GPIO_WritePin(Traffic_2_RED_GPIO_Port, Traffic_2_RED_Pin, SET);
		HAL_GPIO_WritePin(Traffic_2_GREEN_GPIO_Port, Traffic_2_GREEN_Pin, RESET);
		if(time2==0)
		{
			status_2=AUTO_GREEN;
			HAL_GPIO_WritePin(Traffic_2_RED_GPIO_Port, Traffic_2_RED_Pin, RESET);
			HAL_GPIO_WritePin(Traffic_2_GREEN_GPIO_Port, Traffic_2_GREEN_Pin, SET);
			time2=lightbufferRun[AUTO_GREEN];
		}
	}
	else if(status_2==AUTO_GREEN)
	{
		HAL_GPIO_WritePin(Traffic_2_RED_GPIO_Port, Traffic_2_RED_Pin, RESET);
		HAL_GPIO_WritePin(Traffic_2_GREEN_GPIO_Port, Traffic_2_GREEN_Pin, SET);
		if(time2==0)
		{
			status_2=AUTO_YELLOW;
			HAL_GPIO_WritePin(Traffic_2_RED_GPIO_Port, Traffic_2_RED_Pin, SET);
			HAL_GPIO_WritePin(Traffic_2_GREEN_GPIO_Port, Traffic_2_GREEN_Pin, SET);
			time2=lightbufferRun[AUTO_YELLOW];
		}
	}
	else if(status_2==AUTO_YELLOW)
	{
		HAL_GPIO_WritePin(Traffic_2_RED_GPIO_Port, Traffic_2_RED_Pin, SET);
		HAL_GPIO_WritePin(Traffic_2_GREEN_GPIO_Port, Traffic_2_GREEN_Pin, SET);
		if(time2==0)
		{
			status_2=AUTO_RED;
			HAL_GPIO_WritePin(Traffic_2_RED_GPIO_Port, Traffic_2_RED_Pin, SET);
			HAL_GPIO_WritePin(Traffic_2_GREEN_GPIO_Port, Traffic_2_GREEN_Pin, RESET);
			time2=lightbufferRun[AUTO_RED];
		}
	}
	if(mode==1 && run_flag==1)
	{
		send_uart(time1, time2);
		time1--;
		time2--;
		setTimerRun(1000);
	}
}
void updateMode()
{
	mode++;
	if(mode>4)
	{
		mode=1;
	}
}
void increase(int index)
{
	lightbufferMod[index]++;
	if(lightbufferMod[index]>99)
	{
		lightbufferMod[index]=1;
	}
}
void copyModtoRun()
{
	for(int i=0;i<2;i++)
	{
		lightbufferRun[i]=lightbufferMod[i];
	}
}
void blinkLed(int led)
{
	if((mode!=1) && (blink_flag==1))
	{
		if(led==RED)
		{
			HAL_GPIO_TogglePin(Traffic_1_RED_GPIO_Port, Traffic_1_RED_Pin);
			HAL_GPIO_TogglePin(Traffic_2_RED_GPIO_Port, Traffic_2_RED_Pin);
		}
		else if(led==YELLOW)
		{
			HAL_GPIO_TogglePin(Traffic_1_RED_GPIO_Port, Traffic_1_RED_Pin);
			HAL_GPIO_TogglePin(Traffic_2_RED_GPIO_Port, Traffic_2_RED_Pin);
			HAL_GPIO_TogglePin(Traffic_1_GREEN_GPIO_Port, Traffic_1_GREEN_Pin);
			HAL_GPIO_TogglePin(Traffic_2_GREEN_GPIO_Port, Traffic_2_GREEN_Pin);
		}
		else if(led==GREEN)
		{
			HAL_GPIO_TogglePin(Traffic_1_GREEN_GPIO_Port, Traffic_1_GREEN_Pin);
			HAL_GPIO_TogglePin(Traffic_2_GREEN_GPIO_Port, Traffic_2_GREEN_Pin);
		}
		setTimerBlink(500);
	}
}
void checkpes()
{
	if(status_1==AUTO_RED || status_1==MAN_RED)
	{
		HAL_GPIO_WritePin(PES_RED_GPIO_Port, PES_RED_Pin, 0);
		HAL_GPIO_WritePin(PES_GREEN_GPIO_Port, PES_GREEN_Pin, 1);
	}
	else if(status_1==AUTO_GREEN || status_1==MAN_GREEN)
	{
		HAL_GPIO_WritePin(PES_RED_GPIO_Port, PES_RED_Pin, 1);
		HAL_GPIO_WritePin(PES_GREEN_GPIO_Port, PES_GREEN_Pin, 0);
	}
	else if(status_1==AUTO_YELLOW || status_1==MAN_YELLOW)
	{
		HAL_GPIO_WritePin(PES_RED_GPIO_Port, PES_RED_Pin, 1);
		HAL_GPIO_WritePin(PES_GREEN_GPIO_Port, PES_GREEN_Pin, 1);
	}
}
uint8_t str[30]="\0";
int init_for_pes=0;
uint8_t buzzer=0;
int pes_start=0;
void fsm_automatic()
{
	if(is_button_pressed(button1))
	{
		updateMode();
		initial_flag=1;
	}
	if(is_button_pressed(pes_but))
	{
		int all_time=(lightbufferRun[0]+lightbufferRun[1]+lightbufferRun[2])*2000;
		setTimerPesBut(all_time);
		pes_start=1;
		buzzer_flag=1;
		buzzer=0;
	}
	if(pes_start==1)
	{
		checkpes();
		buzzer_boi(buzzer);
		if(buzzer_flag==1)
		{
			buzzer+=30;
			if(buzzer>=99)
			{
				buzzer=99;
			}
			setTimerBuzzer(1000);
		}
	}
	if(pes_flag==1)
	{
		HAL_GPIO_WritePin(PES_RED_GPIO_Port, PES_RED_Pin, RESET);
		HAL_GPIO_WritePin(PES_GREEN_GPIO_Port, PES_GREEN_Pin, RESET);
		buzzer=0;
		pes_start=0;
		buzzer_boi(0);
	}
	switch(mode)
	{
	case 1:
		if(initial_flag==1)
		{
			initial_flag=0;
			status_1=AUTO_RED;
			status_2=AUTO_GREEN;
			time1=lightbufferRun[AUTO_RED];
			time2=lightbufferRun[AUTO_GREEN];
			setTimerRun(1000);
		}
		traffic();
		//send signal

		break;
	case 2:
		if(initial_flag==1)
		{
			initial_flag=0;
			setTimerReset(5000);
			status_1=MAN_RED;
			status_2=MAN_RED;
			HAL_GPIO_WritePin(Traffic_1_RED_GPIO_Port, Traffic_1_RED_Pin, RESET);
			HAL_GPIO_WritePin(Traffic_2_RED_GPIO_Port, Traffic_2_RED_Pin, RESET);
			HAL_GPIO_WritePin(Traffic_1_GREEN_GPIO_Port, Traffic_1_GREEN_Pin, RESET);
			HAL_GPIO_WritePin(Traffic_2_GREEN_GPIO_Port, Traffic_2_GREEN_Pin, RESET);
			blink_flag=1;
		}
		blinkLed(RED);
		if(is_button_pressed(button2))
		{
			status_1=MAN_RED;
			status_2=MAN_RED;
			setTimerReset(5000);
			increase(RED);
		}
		if(is_button_pressed(button3))
		{
			copyModtoRun();
			mode=1;
			initial_flag=1;
		}
		if(reset_flag==1)
		{
			mode=1;
			initial_flag=1;
		}

		//send signal
		break;
	case 3:
		if(initial_flag==1)
		{
			initial_flag=0;
			setTimerReset(5000);
			status_1=MAN_YELLOW;
			status_2=MAN_YELLOW;
			HAL_GPIO_WritePin(Traffic_1_RED_GPIO_Port, Traffic_1_RED_Pin, RESET);
			HAL_GPIO_WritePin(Traffic_2_RED_GPIO_Port, Traffic_2_RED_Pin, RESET);
			HAL_GPIO_WritePin(Traffic_1_GREEN_GPIO_Port, Traffic_1_GREEN_Pin, RESET);
			HAL_GPIO_WritePin(Traffic_2_GREEN_GPIO_Port, Traffic_2_GREEN_Pin, RESET);
			blink_flag=1;
		}
		blinkLed(YELLOW);
		if(is_button_pressed(button2))
		{
			status_1=MAN_YELLOW;
			status_2=MAN_YELLOW;
			setTimerReset(5000);
			increase(YELLOW);
		}
		if(is_button_pressed(button3))
		{
			copyModtoRun();
			mode=1;
			initial_flag=1;
		}
		if(reset_flag==1)
		{
			mode=1;
			initial_flag=1;
		}
		//send signal

		break;
	case 4:
		if(initial_flag==1)
		{
			initial_flag=0;
			setTimerReset(5000);
			status_1=MAN_GREEN;
			status_2=MAN_GREEN;
			HAL_GPIO_WritePin(Traffic_1_RED_GPIO_Port, Traffic_1_RED_Pin, RESET);
			HAL_GPIO_WritePin(Traffic_2_RED_GPIO_Port, Traffic_2_RED_Pin, RESET);
			HAL_GPIO_WritePin(Traffic_1_GREEN_GPIO_Port, Traffic_1_GREEN_Pin, RESET);
			HAL_GPIO_WritePin(Traffic_2_GREEN_GPIO_Port, Traffic_2_GREEN_Pin, RESET);
			blink_flag=1;
		}
		blinkLed(GREEN);
		if(is_button_pressed(button2))
		{
			status_1=MAN_GREEN;
			status_2=MAN_GREEN;
			setTimerReset(5000);
			increase(GREEN);
		}
		if(is_button_pressed(button3))
		{
			copyModtoRun();
			mode=1;
			initial_flag=1;
		}
		if(reset_flag==1)
		{
			mode=1;
			initial_flag=1;
		}
		//send signal

		break;
	default:
		break;
	}

}
