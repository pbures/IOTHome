/* 
* BatteryVoltage.cpp
*
* Created: 24.2.2017 17:03:25
* Author: pbures
*/


#include "BatteryVoltage.h"

long BatteryVoltage::getVoltage()
{
	//return 3300;
	return readVcc();
}

uint8_t BatteryVoltage::getVoltagePercentage(long maxVoltage)
{
	return (floor(getVoltage() * 100 / maxVoltage));
}

// default constructor
BatteryVoltage::BatteryVoltage()
{
} //BatteryVoltage

long BatteryVoltage::readVcc() {
	ADCSRA |= (1 << ADEN);
	// Read 1.1V reference against AVcc
	// set the reference to Vcc and the measurement to the internal 1.1V reference
	ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);

	_delay_ms(2); // Wait for Vref to settle
	ADCSRA |= _BV(ADSC); // Start conversion
	while (bit_is_set(ADCSRA,ADSC)); // measuring

	uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
	uint8_t high = ADCH; // unlocks both

	long result = (high<<8) | low;
	
	ADCSRA &= ~(1 << ADEN);
	result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
	return result; // Vcc in millivolts
}