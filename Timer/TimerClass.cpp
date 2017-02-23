/*
 * Timer.cpp
 *
 *  Created on: 20. 12. 2016
 *      Author: pbures
 */

#include "TimerClass.h"
#include <stdio.h>

TimerClass Timer;

uint16_t TimerClass::timer1OverflowCnt = 0;

ISR(TIMER1_OVF_vect) {
	TimerClass::timer1OverflowCnt++;
	TCNT1 = 0;
}

uint32_t TimerClass::millis() {

	uint32_t overflows = timer1OverflowCnt;
	uint16_t tim1val = TCNT1;

	if (TIFR1 & (1 << TOV1)) { /* We have just encountered timer overflow. Overflows counters are not updated however. */
		overflows++;
		tim1val = TCNT1;
	}

	uint32_t ticks = (TIMER_MAX / 1000) * timer1OverflowCnt + tim1val / 1000;
	return ticks;
}

TimerClass::TimerClass() {
	TCCR1B |= (1 << CS10); /* Prescaler set to 1, each ms. */
	TIMSK1 |= (1 << TOIE1);
	timer1OverflowCnt = 0;
	TCNT1 = 0;
}
