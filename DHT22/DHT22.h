/* DHT library
MIT license
written by Adafruit Industries
*/
#ifndef DHT22_H
#define DHT22_H

#include <stdio.h>
#include <inttypes.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <TimerClass.h>
#include <IOPin.h>

// Uncomment to enable printing out nice debug messages.
//#define DHT_DEBUG

// Define where debug output will be printed.
#define LOW false
#define HIGH true

class DHT {
	public:
	DHT(volatile uint8_t *ddr, volatile uint8_t *port, volatile uint8_t *pin, uint8_t bit);
	
	DHT(IOPin *dataPin) : dataPin(dataPin) {
		_ddr = dataPin->ddr;
		_port = dataPin->port;
		_pin = dataPin->pin;
		_bit = dataPin->bit;
	};
		
	void begin(void);
	float getTemperature(bool force=false);
	float getHumidity(bool force=false);
	bool readSensor(bool force=false);

	private:
	IOPin *dataPin;
	
	uint8_t bytes[5];
	volatile uint8_t *_ddr, *_port, *_pin;
	uint8_t _bit;
	volatile uint32_t lastReadTimestamp;
	bool lastReading;
	
	 void setLow() {
		 (*_port) &= ~(1<<_bit);
		 };
		 
	 void setHigh() {
		 (*_port) |= (1<<_bit);
		 };
		
	void setToOutput() {(*_ddr) |= (1<<_bit);};
	void setToInput() {(*_ddr) &= ~(1<<_bit);};
};

class InterruptLock {
	public:
	InterruptLock() {
		cli();
	}
	~InterruptLock() {
		sei();
	}
};

#endif //DHT22_H