/*
 * Timer.h
 *
 *  Created on: 20. 12. 2016
 *      Author: pbures
 */

#ifndef TIMER_H_
#define TIMER_H_

#include <avr/interrupt.h>
#include <inttypes.h>
#include <stdint.h>

#define TIMER_MAX 65535

class TimerClass {
public:
	TimerClass();
	static uint32_t millis();
	static uint16_t timer1OverflowCnt;
};

extern TimerClass Timer;

#endif /* TIMER_H_ */
