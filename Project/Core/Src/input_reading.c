/*
 * input_reading.c
 *
 *  Created on: Nov 30, 2022
 *      Author: HELLO SON
 */
#include "input_reading.h"
#include "main.h"
#define NO_OF_BUTTONS 4
#define BUTTON_IS_PRESSED  GPIO_PIN_RESET
#define BUTTON_IS_RELEASED GPIO_PIN_SET
static GPIO_PinState buttonBuffer[NO_OF_BUTTONS]={BUTTON_IS_RELEASED,BUTTON_IS_RELEASED,BUTTON_IS_RELEASED,BUTTON_IS_RELEASED};
static GPIO_PinState debounceButtonBuffer1[NO_OF_BUTTONS];
static GPIO_PinState debounceButtonBuffer2[NO_OF_BUTTONS];
static GPIO_PinState debounceButtonBuffer3[NO_OF_BUTTONS];

static uint16_t buttonPin[NO_OF_BUTTONS]={BUT_1_Pin,BUT_2_Pin,BUT_3_Pin,PES_BUT_Pin};
static GPIO_TypeDef* buttonPort[NO_OF_BUTTONS]={BUT_1_GPIO_Port,BUT_2_GPIO_Port,BUT_3_GPIO_Port,PES_BUT_GPIO_Port};
int duration_for_buttons;
void button_reading()
{
	for(uint8_t i=0;i<NO_OF_BUTTONS;i++)
	{
		debounceButtonBuffer2[i]=debounceButtonBuffer1[i];
		debounceButtonBuffer1[i]=HAL_GPIO_ReadPin(buttonPort[i], buttonPin[i]);
		if(debounceButtonBuffer1[i]==debounceButtonBuffer2[i])
		{
			if(debounceButtonBuffer3[i]!=debounceButtonBuffer1[i])
			{
				debounceButtonBuffer3[i]=debounceButtonBuffer1[i];
				if(debounceButtonBuffer1[i]==BUTTON_IS_PRESSED)
				{
					buttonBuffer[i]=BUTTON_IS_PRESSED;
					duration_for_buttons=100;
				}
			}
			else
			{
				duration_for_buttons--;
				if(duration_for_buttons<=0)
				{
					debounceButtonBuffer3[i]=BUTTON_IS_RELEASED;
				}
			}
		}
	}
}

int is_button_pressed(uint8_t index)
{
	if(buttonBuffer[index]==BUTTON_IS_PRESSED)
	{
		buttonBuffer[index]=BUTTON_IS_RELEASED;
		return 1;
	}
	else return 0;
}
