/* 
* BatteryVoltage.h
*
* Created: 24.2.2017 17:03:25
* Author: pbures
*/


#ifndef __BATTERYVOLTAGE_H__
#define __BATTERYVOLTAGE_H__
#include <util/delay.h>
#include <avr/power.h>
#include <math.h>

class BatteryVoltage
{
public: 
	long getVoltage();
	
	/* maxVoltage in milivolts */
	uint8_t getVoltagePercentage(long maxVoltage);

public:
	BatteryVoltage();
	
private:
	long readVcc();
}; //BatteryVoltage

#endif //__BATTERYVOLTAGE_H__
