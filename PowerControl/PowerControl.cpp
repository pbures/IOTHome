/* 
* PowerControl.cpp
*
* Created: 21.2.2017 13:42:58
* Author: pbures
*/
#include "PowerControl.h"

volatile uint8_t PowerControl::counter = 0;

ISR(WDT_vect) {
	PowerControl::interruptHandler();
}

// default constructor
PowerControl::PowerControl()
{
} //PowerControl


void PowerControl::sleepNow(uint8_t seconds) {
	
	PowerControl::counter = 0;	
	while(counter < (seconds/8)) {
		/* Clear, is set to 1 after wdt System reset occurs */
		MCUSR &= ~( 1 << WDRF); 
	
		/* WDCE to 1 so we are in Interrupt Mode (no reset after interrupt routine) */
		WDTCSR |= ( 1 << WDCE );
		WDTCSR &= ~(1 << WDE);
	
		/* Enable interrupt for watchdog */
		WDTCSR |= ( 1 << WDIE );
	
		set_sleep_mode(SLEEP_MODE_PWR_DOWN);
		sleep_enable();
		sei();
		sleep_mode();
	}
	
	sleep_disable();
	
	/* Disable interrupt, in other places we want watchdog to reset the chip */
	WDTCSR &= ~(1 << WDIE );
	WDTCSR |= (1 << WDE);
}

void PowerControl::interruptHandler()
{
	PowerControl::counter++;
}
