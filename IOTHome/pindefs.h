#ifndef PINDEFS_H_
#define PINDEFS_H_

#include <IOPin.h>

IOPin rf69SpiCsPin (&DDRB, &PORTB, &PINB, PINB2);
IOPin rf69DIO0Pin  (&DDRD, &PORTD, &PIND, PIND2);
IOPin dht22Pin     (&DDRB, &PORTB, &PINB, PINB0);
IOPin redLed       (&DDRB, &PORTB, &PINB, PINB1);
IOPin grnLed       (&DDRB, &PORTB, &PINB, PINB6);
IOPin pwrPin	   (&DDRB, &PORTB, &PINB, PINB7);

#endif /* PINDEFS_H_ */
