/* DHT library
MIT license
written by Adafruit Industries
*/

#include "DHT22.h"

#define MIN_INTERVAL 2000


DHT::DHT(volatile uint8_t *ddr, volatile uint8_t *port, volatile uint8_t *pin, uint8_t bit) :
_ddr(ddr), _port(port), _pin(pin), _bit(bit)
{
}

void DHT::begin() {
	
	setToInput();
	setHigh();
	
	// Using this value makes sure that millis() - lastreadtime will be
	// >= MIN_INTERVAL right away. Note that this assignment wraps around,
	// but so will the subtraction.
	lastReadTimestamp = -MIN_INTERVAL;
}

float DHT::getTemperature(bool force) {
	float temp = NAN;

	if (readSensor(force)) {
		temp = bytes[2] & 0x7F;
		temp *= 256;
		temp += bytes[3];
		temp *= 0.1;
		if (bytes[2] & 0x80) {
			temp *= -1;
		}
	}
	return temp;
}

float DHT::getHumidity(bool force) {
	float hum = NAN;
	
	if (readSensor(force)) {
		hum = bytes[0];
		hum *= 256;
		hum += bytes[1];
		hum *= 0.1;
	}
	return hum;
}

bool DHT::readSensor(bool force) {
	uint32_t currenttime = Timer.millis();
	if (!force && ((currenttime - lastReadTimestamp) < 2000)) {
		return lastReading; // return last correct measurement
	}
	lastReadTimestamp = currenttime;

	/* 5 bytes will be received from sensor (4 + 1 CRC).
	 * https://cdn-shop.adafruit.com/datasheets/DHT22.pdf
	 */
	for (uint8_t i=0; i<5; i++) bytes[i] = 0;
	
	/* let pull-up raise data line level and start the reading process. */
	setHigh();
	_delay_ms(250);

	/* Start the reading by setting line low to 20ms */
	setToOutput();
	setLow();
	_delay_ms(20);

	uint16_t data[84];
	
	{
		cli();

		/* Set the line high for 40us, starting the measurement */
		setHigh();
		_delay_us(40);
		setToInput();

		/* Start recording Timer1 values at changing values. Timer1 values tick in microseconds. */
		//TODO: So far I have no idea why I can't refernce the PINB using *_pin value;
		uint8_t myPinb = PINB;
		uint16_t myTcnt=TCNT1;
		TCNT1 = 0;

		/* This is extremely time critical, running on the edge. Do not make it even a bit more complex,
		* or the edges are not caught. Interrupt based handlinng does not work either.
		*/
		uint8_t c=0;
		for(c=0;c<84;c++){
			while(!(PINB ^ myPinb)&(0x1)) {;} //TODO: Here this may get stuck forever! Need to add some protection.
//			while(!(((*_pin) ^ myPinb)&(0x1))) {;} //TODO: Here this may get stuck forever! Need to add some protection.
			myPinb = *_pin;
			data[c] = TCNT1;
		}
		
		//TODO: We may have overflow!, check on that.
		TCNT1 += myTcnt;
		sei();
	}
	
	for(uint8_t c=0; c<83;c++) {
		data[c] = (data[c+1] - data[c]);
	}
	#ifdef DHT_DEBUG
	printf("\r\nT:");
	for(uint8_t c=0; c<83;c++){
		printf(" %i",data[c]);
	}
	printf("\r\nB:      ");
	#endif
	
	// Inspect pulses and determine which ones are 0 (high state cycle count < low
	// state cycle count), or 1 (high state cycle count > low state cycle count).
	for (int i=0; i<40; ++i) {
		
		bytes[i/8] <<= 1;
		if (data[i*2 + 3] > data[i*2 + 2]) {
			// Ones are cycles are longer than 50us low gap. Zeros are shorter than the gap.
			bytes[i/8] |= 1;
			#ifdef DHT_DEBUG
			printf("     1");
			#endif
			} else {
			#ifdef DHT_DEBUG
			printf("     0");
			#endif
		}
	}
	#ifdef DHT_DEBUG
	printf("\r\nReceived: %x, %x, %x, %x. %x == %x ?\n\r",
	bytes[0], bytes[1], bytes[2], bytes[3], bytes[4],
	(bytes[0] + bytes[1] + bytes[2] + bytes[3]) & 0xFF);
	
	printf("\r\n");
	#endif

	// Check we read 40 bits and that the checksum matches.
	if (bytes[4] == ((bytes[0] + bytes[1] + bytes[2] + bytes[3]) & 0xFF)) {
		lastReading = true;
		return lastReading;
	}
	else {
		#ifdef DHT_DEBUG
		printf_P(PSTR("Checksum failure!"));
		#endif
		lastReading = false;
		return lastReading;
	}
}