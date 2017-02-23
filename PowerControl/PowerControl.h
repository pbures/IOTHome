/* 
* PowerControl.h
*
* Created: 21.2.2017 13:42:58
* Author: pbures
*/


#ifndef __POWERCONTROL_H__
#define __POWERCONTROL_H__

#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

class PowerControl
{
public:
	PowerControl();
	~PowerControl();
	static void interruptHandler();
	
	static void sleepNow(uint8_t seconds);

private:
	static volatile uint8_t counter;

};

#endif //__POWERCONTROL_H__
