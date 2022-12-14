/*
 * timer.h
 *
 *  Created on: Nov 30, 2022
 *      Author: HELLO SON
 */

#ifndef INC_TIMER_H_
#define INC_TIMER_H_
#include "main.h"
#include "input_reading.h"
extern int reset_flag;
extern int run_flag;
extern int pes_flag;
extern int blink_flag;
extern int buzzer_flag;
void setTimerReset(int duration);
void setTimerRun(int duration);
void setTimerPesBut(int duration);
void setTimerBlink(int duration);
void setTimerBuzzer(int duration);

#endif /* INC_TIMER_H_ */
