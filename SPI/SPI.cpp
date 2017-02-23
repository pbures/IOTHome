/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@arduino.cc>
 * Copyright (c) 2014 by Paul Stoffregen <paul@pjrc.com> (Transaction API)
 * Copyright (c) 2014 by Matthijs Kooijman <matthijs@stdin.nl> (SPISettings AVR)
 * Copyright (c) 2014 by Andrew J. Kroll <xxxajk@gmail.com> (atomicity fixes)
 * SPI Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#include "SPI.h"

SPIClass SPI;
uint8_t SPIClass::initialized = 0;

void SPIClass::begin()
{
  uint8_t sreg = SREG;
  cli();

  if (!initialized) {
	SPI_MOSI_DDR |= (1 << SPI_MOSI);  /* output on MOSI */
	SPI_MISO_PORT |= (1 << SPI_MISO); /* pullup on MISO */
	SPI_SCK_DDR |= (1 << SPI_SCK);    /* output on SCK */

	/* Keep default mode (phase and polarity) at 0 */
	SPCR |= (1 << SPR1); /* div 16, safer for breadboards */
	SPCR |= (1 << MSTR); /* clockmaster */
	SPCR |= (1 << SPE);  /* enable */
  }
  initialized=1; // reference count
  SREG = sreg;
}

void SPIClass::end() {
  uint8_t sreg = SREG;
  cli();

  initialized=0;
  SPCR &= ~_BV(SPE);

  SREG = sreg;
}
