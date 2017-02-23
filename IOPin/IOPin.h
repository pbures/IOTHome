/*
* IOPin.h
*
* Created: 5.2.2017 17:10:32
* Author: pbures
*/


#ifndef __IOPIN_H__
#define __IOPIN_H__


class IOPin
{
	public:
	IOPin(volatile uint8_t* ddr, volatile uint8_t *port,volatile uint8_t *pin, uint8_t bit):
	ddr(ddr), port(port), pin(pin), bit(bit) {};
		
	void setLow() { (*port) &= ~(1<<bit);	};
	void setHigh() { (*port) |= (1<<bit); };
	
	void setToOutput() {(*ddr) |= (1<<bit);};
	void setToInput() {(*ddr) &= ~(1<<bit);};
		
	volatile uint8_t *ddr;
	volatile uint8_t *port;
	volatile uint8_t *pin;
	uint8_t bit;
	
}; //IOPin

#endif //__IOPIN_H__
