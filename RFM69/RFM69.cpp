// **********************************************************************************
// Driver definition for HopeRF RFM69W/RFM69HW/RFM69CW/RFM69HCW, Semtech SX1231/1231H
// **********************************************************************************
// Copyright Felix Rusu 2016, http://www.LowPowerLab.com/contact
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it
// and/or modify it under the terms of the GNU General
// Public License as published by the Free Software
// Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will
// be useful, but WITHOUT ANY WARRANTY; without even the
// implied warranty of MERCHANTABILITY or FITNESS FOR A
// PARTICULAR PURPOSE. See the GNU General Public
// License for more details.
//
// Licence can be viewed at
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************
#include "RFM69.h"
#include "RFM69registers.h"
#include "SPI.h"
#include "TimerClass.h"
//#include "config.h"

volatile uint8_t RFM69::DATA[RF69_MAX_DATA_LEN];
volatile uint8_t RFM69::_mode;        // current transceiver state
volatile uint8_t RFM69::DATALEN;
volatile uint8_t RFM69::SENDERID;
volatile uint8_t RFM69::TARGETID;     // should match _address
volatile uint8_t RFM69::PAYLOADLEN;
volatile uint8_t RFM69::ACK_REQUESTED;
volatile uint8_t RFM69::ACK_RECEIVED; // should be polled immediately after sending a packet with ACK request
volatile int16_t RFM69::RSSI; // most accurate RSSI during reception (closest to the reception)
volatile bool RFM69::_inISR;
RFM69* RFM69::selfPointer;

bool RFM69::initialize(uint8_t freqBand, uint8_t nodeID, uint8_t networkID) {
	const uint8_t CONFIG[][2] =
	{
		/* 0x01 */{ REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },
		/* 0x02 */{ REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET	| RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00 }, // no shaping
		/* 0x03 */{ REG_BITRATEMSB, RF_BITRATEMSB_55555 }, // default: 4.8 KBPS
		/* 0x04 */{ REG_BITRATELSB, RF_BITRATELSB_55555 },
		/* 0x05 */{ REG_FDEVMSB, RF_FDEVMSB_50000 }, // default: 5KHz, (FDEV + BitRate / 2 <= 500KHz)
		/* 0x06 */{ REG_FDEVLSB, RF_FDEVLSB_50000 },

		/* 0x07 */{ REG_FRFMSB, (uint8_t) (
			freqBand == RF69_315MHZ ?
			RF_FRFMSB_315 :
			(freqBand == RF69_433MHZ ?
			RF_FRFMSB_433 :
			(freqBand == RF69_868MHZ ?
			RF_FRFMSB_868 :
		RF_FRFMSB_915))) },
		/* 0x08 */{ REG_FRFMID, (uint8_t) (
			freqBand == RF69_315MHZ ?
			RF_FRFMID_315 :
			(freqBand == RF69_433MHZ ?
			RF_FRFMID_433 :
			(freqBand == RF69_868MHZ ?
			RF_FRFMID_868 :
		RF_FRFMID_915))) },
		/* 0x09 */{ REG_FRFLSB, (uint8_t) (
			freqBand == RF69_315MHZ ?
			RF_FRFLSB_315 :
			(freqBand == RF69_433MHZ ?
			RF_FRFLSB_433 :
			(freqBand == RF69_868MHZ ?
			RF_FRFLSB_868 :
		RF_FRFLSB_915))) },

		// looks like PA1 and PA2 are not implemented on RFM69W, hence the max output power is 13dBm
		// +17dBm and +20dBm are possible on RFM69HW
		// +13dBm formula: Pout = -18 + OutputPower (with PA0 or PA1**)
		// +17dBm formula: Pout = -14 + OutputPower (with PA1 and PA2)**
		// +20dBm formula: Pout = -11 + OutputPower (with PA1 and PA2)** and high power PA settings (section 3.3.7 in datasheet)
		///* 0x11 */ { REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111},
		///* 0x13 */ { REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95 }, // over current protection (default is 95mA)

		// RXBW defaults are { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_5} (RxBw: 10.4KHz)
		//for BR-19200: /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_3 },
		
		/* 0x19 */{ REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16	| RF_RXBW_EXP_2 }, // (BitRate < 2 * RxBw)
		/* 0x25 */{ REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 }, // DIO0 is the only IRQ we're using
		/* 0x26 */{ REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF }, // DIO5 ClkOut disable for power saving
		/* 0x28 */{ REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN }, // writing to this bit ensures that the FIFO & status flags are reset
		/* 0x29 */{ REG_RSSITHRESH, 220 }, // must be set to dBm = (-Sensitivity / 2), default is 0xE4 = 228 so -114dBm
		///* 0x2D */ { REG_PREAMBLELSB, RF_PREAMBLESIZE_LSB_VALUE } // default 3 preamble bytes 0xAAAAAA
		
		/* 0x2E */{ REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2	| RF_SYNC_TOL_0 },
		
		//TODO: Why this has to be set to 0x2D??? Does not work for me with 0x00.
		/* 0x2F */{ REG_SYNCVALUE1, 0x2D }, // attempt to make this compatible with sync1 byte of RFM12B lib

		/* 0x30 */{ REG_SYNCVALUE2, networkID }, // NETWORK ID
		/* 0x37 */{ REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE  | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_NODE },
		
		/* 0x38 */{ REG_PAYLOADLENGTH, 66 }, // in variable length mode: the max frame size, not used in TX
		
		/* 0x39 */{ REG_NODEADRS, nodeID},
		
		/* 0x3C */{ REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE }, // TX on FIFO not empty
		/* 0x3D */{ REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
		
		//for BR-19200: /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_NONE | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
		/* 0x6F */{ REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 }, // run DAGC continuously in RX mode for Fading Margin Improvement, recommended default for AfcLowBetaOn=0
		{ 255, 0 } };

		SPI.begin();

		//SET_OUTPUT_MODE(RF69_SPI_CS_DDR, RF69_SPI_CS_BIT);
		spiCsPin->setToOutput();
		//SET_HIGH(RF69_SPI_CS_PORT, RF69_SPI_CS_BIT);
		spiCsPin->setHigh();
		
		printf("RFM69 Init started.\r\n");
		uint32_t start = Timer.millis();
		uint8_t timeout = 50;
		do
		writeReg(REG_SYNCVALUE1, 0xAA);
		while (readReg(REG_SYNCVALUE1) != 0xaa && Timer.millis() - start < timeout);
		start = Timer.millis();
		do
		writeReg(REG_SYNCVALUE1, 0x55);
		while (readReg(REG_SYNCVALUE1) != 0x55 && Timer.millis() - start < timeout);

		for (uint8_t i = 0; CONFIG[i][0] != 255; i++)
		writeReg(CONFIG[i][0], CONFIG[i][1]);

		encrypt(0);
		setHighPower(_isRFM69HW); // called regardless if it's a RFM69W or RFM69HW
		setMode(RF69_MODE_STANDBY);
		
		start = Timer.millis();
		while (((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00) && Timer.millis() - start < timeout) {;} // wait for ModeReady
		if (Timer.millis() - start >= 1000){
			return false;
		}

		_inISR = false;
		selfPointer = this;
		_address = nodeID;

		/* INT0 on rising edge */
		//SET_INPUT_MODE(RF69_DIO0, RF69_DIO0_BIT);
		DIO0Pin->setToInput();

		EIMSK |= (1 << INT0);
		EICRA |= ((1 << ISC01) | (1 << ISC00));
		sei();

		printf("Config or RFM69 done.\r\n");
		return true;
	}

	ISR(INT0_vect) {
		RFM69::isr0();
	}

	uint32_t RFM69::getFrequencyHz() {
		return RF69_FSTEP * (((uint32_t) readReg(REG_FRFMSB) << 16) + ((uint16_t) readReg(REG_FRFMID) << 8)	+ readReg(REG_FRFLSB));
	}

	void RFM69::setFrequencyHz(uint32_t freqHz) {
		uint8_t oldMode = _mode;
		if (oldMode == RF69_MODE_TX) {
			setMode(RF69_MODE_RX);
		}
		
		freqHz /= RF69_FSTEP; // divide down by FSTEP to get FRF
		writeReg(REG_FRFMSB, freqHz >> 16);
		writeReg(REG_FRFMID, freqHz >> 8);
		writeReg(REG_FRFLSB, freqHz);
		if (oldMode == RF69_MODE_RX) {
			setMode(RF69_MODE_SYNTH);
		}
		
		setMode(oldMode);
	}

	void RFM69::setMode(uint8_t newMode) {
		if (newMode == _mode)
		return;

		switch (newMode) {
			case RF69_MODE_TX:
				writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_TRANSMITTER);
				if (_isRFM69HW) {
					setHighPowerRegs(true);
				}
				break;
			
			case RF69_MODE_RX:
				writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_RECEIVER);
				if (_isRFM69HW){
					setHighPowerRegs(false);
				}
				break;
			
			case RF69_MODE_SYNTH:
				writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SYNTHESIZER);
				break;
				
			case RF69_MODE_STANDBY:
				writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
				break;
				
			case RF69_MODE_SLEEP:
				writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SLEEP);
				break;
				
			default:
			return;
		}

		while (_mode == RF69_MODE_SLEEP && (readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00) {;} //Wait for more ready.
			
		_mode = newMode;
	}

	void RFM69::sleep() {
		setMode(RF69_MODE_SLEEP);
	}

	void RFM69::setNodeAddress(uint8_t addr) {
		_address = addr;
		writeReg(REG_NODEADRS, _address);
	}

	void RFM69::setNetworkId(uint8_t networkID) {
		writeReg(REG_SYNCVALUE2, networkID);
	}

	// set *transmit/TX* output power: 0=min, 31=max
	// this results in a "weaker" transmitted signal, and directly results in a lower RSSI at the receiver
	// the power configurations are explained in the SX1231H datasheet (Table 10 on p21; RegPaLevel p66): http://www.semtech.com/images/datasheet/sx1231h.pdf
	// valid powerLevel parameter values are 0-31 and result in a directly proportional effect on the output/transmission power
	// this function implements 2 modes as follows:
	//       - for RFM69W the range is from 0-31 [-18dBm to 13dBm] (PA0 only on RFIO pin)
	//       - for RFM69HW the range is from 0-31 [5dBm to 20dBm]  (PA1 & PA2 on PA_BOOST pin & high Power PA settings - see section 3.3.7 in datasheet, p22)
	void RFM69::setPowerLevel(uint8_t powerLevel) {
		_powerLevel = (powerLevel > 31 ? 31 : powerLevel);
		
		if (_isRFM69HW) {
			_powerLevel /= 2;
		}
		
		writeReg(REG_PALEVEL, (readReg(REG_PALEVEL) & 0xE0) | _powerLevel);
	}

	bool RFM69::canSend() {
		if (_mode == RF69_MODE_RX && PAYLOADLEN == 0 && readRSSI() < CSMA_LIMIT) // if signal stronger than -100dBm is detected assume channel activity
		{
			setMode(RF69_MODE_STANDBY);
			return true;
		}
		return false;
	}

	void RFM69::send(uint8_t toAddress, const void* buffer, uint8_t bufferSize,	bool requestACK) {
		
		writeReg(REG_PACKETCONFIG2,	(readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // Avoid RX deadlock

		uint32_t now = Timer.millis();
		uint8_t count = 0;

		while (true){
			if (canSend()) { break; }
			if (Timer.millis() - now >= RF69_CSMA_LIMIT_MS) { break; }
			if (count++ > 100) { break; }

			receiveDone();
		}

		sendFrame(toAddress, buffer, bufferSize, requestACK, false);
	}

	bool RFM69::sendWithRetry(uint8_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t retries, uint16_t retryWaitTime) {
		
		for (uint8_t i = 0; i <= retries; i++) {
			send(toAddress, buffer, bufferSize, true);
			
			uint32_t sentTime = Timer.millis();
			while (Timer.millis() - sentTime < retryWaitTime) {
				if (ACKReceived(toAddress)) {
					return true;
				}
			}
		}
		
		return false;
	}

	/* Must immediately after sending a packet with ACK request */
	bool RFM69::ACKReceived(uint8_t fromNodeID) {

		if (receiveDone()){
			return (SENDERID == fromNodeID || fromNodeID == RF69_BROADCAST_ADDR) && ACK_RECEIVED;
		}
		
		return false;
	}

	/* Check whether an ACK was requested in the last received packet (non-broadcasted packet) */
	bool RFM69::ACKRequested() {
		return ACK_REQUESTED && (TARGETID != RF69_BROADCAST_ADDR);
	}

	/* Must be called immediately after reception in case sender wants ACK */
	void RFM69::sendACK(const void* buffer, uint8_t bufferSize) {
		ACK_REQUESTED = 0; // TWS added to make sure we don't end up in a timing race and infinite loop sending Acks
		uint8_t sender = SENDERID;
		int16_t _RSSI = RSSI; // save payload received RSSI value
		
		writeReg(REG_PACKETCONFIG2, (readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // Avoid RX deadlock
		
		uint32_t now = Timer.millis();
		while (!canSend() && Timer.millis() - now < RF69_CSMA_LIMIT_MS) {
			receiveDone();
		}
		
		SENDERID = sender; // Restore SenderID after it gets wiped out by receiveDone()
		sendFrame(sender, buffer, bufferSize, false, true);
		
		RSSI = _RSSI; // restore payload RSSI
	}

	inline void RFM69::sendFrame(uint8_t toAddress, const void* buffer, uint8_t bufferSize,	bool requestACK, bool sendACK) {

		setMode(RF69_MODE_STANDBY); // turn off receiver to prevent reception while filling fifo

		while ((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00) {;} // wait for ModeReady
		writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00); // DIO0 is "Packet Sent"
		
		bufferSize = ((bufferSize > RF69_MAX_DATA_LEN) ? RF69_MAX_DATA_LEN : bufferSize);

		// Control byte
		uint8_t CTLbyte = 0x00;
		if (sendACK) {
			CTLbyte = RFM69_CTL_SENDACK;
		} else if (requestACK) {
			CTLbyte = RFM69_CTL_REQACK;
		}
		
		select();
		SPI.transfer(REG_FIFO | 0x80);
		SPI.transfer(bufferSize + 3);
		SPI.transfer(toAddress);
		SPI.transfer(_address);
		SPI.transfer(CTLbyte);

		for (uint8_t i = 0; i < bufferSize; i++){
			SPI.transfer(((uint8_t*) buffer)[i]);
		}
		unselect();

		/* Module starts transmitting immediately after it is set to TX */
		setMode(RF69_MODE_TX);
		
		uint32_t txStart = Timer.millis();
		
		/* Wait for DIO0 goes up after transmit. Interrupt handler is called but does
		 * nothing as we are in TX mode, not in RX mode 
		 */
		while (true) {
			//if ((RF69_DIO0_PIN & (1<<RF69_DIO0_BIT)) != 0 ) {
			if ( ((*(DIO0Pin->pin)) & (1 << DIO0Pin->bit)) != 0 ) {
				break;
			}
			
			if (Timer.millis() - txStart > RF69_TX_LIMIT_MS) {
				printf("\r\nTimeout during waiting for DIO0 send TX success\r\n");
				break;
			}
		}
		
		/* I don't know why, but when the delay is here the RSSI signal on receiver is much stronger */
		_delay_ms(20);
		
		setMode(RF69_MODE_STANDBY);
	}

	/* Called when DIO0 is raised. */
	void RFM69::interruptHandler() {
		
		/* If we are in receive mode and payload available, switch to standby and read the payload */
		if (_mode == RF69_MODE_RX && (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY)) {
			RSSI = readRSSI();
			setMode(RF69_MODE_STANDBY);
			
			select();
			SPI.transfer(REG_FIFO & 0x7F);
			PAYLOADLEN = SPI.transfer(0);
			PAYLOADLEN = PAYLOADLEN > 66 ? 66 : PAYLOADLEN; // precaution
			TARGETID = SPI.transfer(0);
			
			/* TODO: May not be needed as the address should be filtered out by the module. Payload size still needs to be checked */
			if (!(_promiscuousMode || TARGETID == _address || TARGETID == RF69_BROADCAST_ADDR) || PAYLOADLEN < 3)
			{
				PAYLOADLEN = 0;
				unselect();
				receiveBegin();
				return;
			}

			DATALEN = PAYLOADLEN - 3;
			SENDERID = SPI.transfer(0);
			uint8_t CTLbyte = SPI.transfer(0);

			ACK_RECEIVED = CTLbyte & RFM69_CTL_SENDACK; // extract ACK-received flag
			ACK_REQUESTED = CTLbyte & RFM69_CTL_REQACK; // extract ACK-requested flag
	
			/* TODO: need to check if needed */		
			interruptHook(CTLbyte); // TWS: hook to derived class interrupt function

			for (uint8_t i = 0; i < DATALEN; i++) {
				DATA[i] = SPI.transfer(0);
			}
			
			if (DATALEN < RF69_MAX_DATA_LEN) {
				DATA[DATALEN] = 0;
			}
			
			unselect();
			setMode(RF69_MODE_RX);
		}
		RSSI = readRSSI();
	}

	void RFM69::isr0() {
		_inISR = true;
		selfPointer->interruptHandler();
		_inISR = false;
	}

	void RFM69::receiveBegin() {
		DATALEN = 0;
		SENDERID = 0;
		TARGETID = 0;
		PAYLOADLEN = 0;
		ACK_REQUESTED = 0;
		ACK_RECEIVED = 0;
		RSSI = 0;

		if (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY){
				writeReg(REG_PACKETCONFIG2,	(readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // Avoid RX deadlock
		}
		
		writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01); // set DIO0 to "PAYLOADREADY" in receive mode
		setMode(RF69_MODE_RX);
	}

	// checks if a packet was received and/or puts transceiver in receive (ie RX or listen) mode
	bool RFM69::receiveDone() {
		volatile uint8_t sreg = SREG;

		cli(); /* re-enabled in unselect() via setMode() or via receiveBegin().. I don't like this. Will deal with it later */
		if (_mode == RF69_MODE_RX && PAYLOADLEN > 0) {
			setMode(RF69_MODE_STANDBY); // enables interrupts
			return true;
		} else if (_mode == RF69_MODE_RX) { // already in RX no payload yet
			SREG = sreg;
			return false;
		}
		
		receiveBegin();
		return false;
	}

	// To enable encryption: radio.encrypt("ABCDEFGHIJKLMNOP");
	// To disable encryption: radio.encrypt(null) or radio.encrypt(0)
	// KEY HAS TO BE 16 bytes !!!
	void RFM69::encrypt(const char* key) {
		setMode(RF69_MODE_STANDBY);
		if (key != 0) {
			select();
			SPI.transfer(REG_AESKEY1 | 0x80);
			for (uint8_t i = 0; i < 16; i++)
			SPI.transfer(key[i]);
			unselect();
		}
		writeReg(REG_PACKETCONFIG2,
		(readReg(REG_PACKETCONFIG2) & 0xFE) | (key ? 1 : 0));
	}

	/* Get the received signal strength indicator (RSSI) */
	int16_t RFM69::readRSSI(bool forceTrigger) {
		int16_t rssi = 0;
		
		if (forceTrigger) {
			// RSSI trigger not needed if DAGC is in continuous mode
			writeReg(REG_RSSICONFIG, RF_RSSI_START);
			while ((readReg(REG_RSSICONFIG) & RF_RSSI_DONE) == 0x00) {;} // wait for RSSI_Ready
		}
		rssi = -readReg(REG_RSSIVALUE);
		rssi >>= 1;
		return rssi;
	}

	uint8_t RFM69::readReg(uint8_t addr) {
		select();
		SPI.transfer(addr & 0x7F);
		uint8_t regval = SPI.transfer(0);
		unselect();
		return regval;
	}

	void RFM69::writeReg(uint8_t addr, uint8_t value) {
		select();
		SPI.transfer(addr | 0x80);
		SPI.transfer(value);
		unselect();
	}

	// select the RFM69 transceiver (save SPI settings, set CS low)
	void RFM69::select() {
		_SREG = SREG;
		cli();
		#if defined (SPCR) && defined (SPSR)
		// save current SPI settings
		_SPCR = SPCR;
		_SPSR = SPSR;
		#endif
		// set RFM69 SPI settings
		SPI.setDataMode(SPI_MODE0);
		SPI.setBitOrder(MSBFIRST);
		SPI.setClockDivider(SPI_CLOCK_DIV4); // decided to slow down from DIV2 after SPI stalling in some instances, especially visible on mega1284p when RFM69 and FLASH chip both present
		
		//SET_LOW(RF69_SPI_CS_PORT, RF69_SPI_CS_BIT);
		spiCsPin->setLow();
	}

	void RFM69::unselect() {
		//SET_HIGH(RF69_SPI_CS_PORT, RF69_SPI_CS_BIT);
		spiCsPin->setHigh();

		#if defined (SPCR) && defined (SPSR)
		SPCR = _SPCR;
		SPSR = _SPSR;
		#endif

		enaleInterrupts();
	}

	/* The true value disables node address filtering */
	void RFM69::promiscuous(bool onOff) {
		_promiscuousMode = onOff;
		if(onOff) {
			writeReg(REG_PACKETCONFIG1, (readReg(REG_PACKETCONFIG1) & 0xF9) | (onOff ? RF_PACKET1_ADRSFILTERING_OFF : RF_PACKET1_ADRSFILTERING_NODEBROADCAST));
		}
	}

	/* For RFM69HW only: you must call setHighPower(true) after initialize() or else transmission won't work */
	void RFM69::setHighPower(bool onOff) {
		_isRFM69HW = onOff;
		
		writeReg(REG_OCP, _isRFM69HW ? RF_OCP_OFF : RF_OCP_ON);
		
		if (_isRFM69HW) {
			writeReg(REG_PALEVEL, (readReg(REG_PALEVEL) & 0x1F) | RF_PALEVEL_PA1_ON	| RF_PALEVEL_PA2_ON); // Enables P1 & P2 amplifier stages
		} else {
			writeReg(REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | _powerLevel); // Enables P0 only
		}
	}

	void RFM69::setHighPowerRegs(bool onOff) {
		writeReg(REG_TESTPA1, onOff ? 0x5D : 0x55);
		writeReg(REG_TESTPA2, onOff ? 0x7C : 0x70);
	}

	//For debugging, prints out detailed registeres information. May take a big part of data segment on the chip */
	#define REGISTER_DETAIL 1
	void RFM69::readAllRegs() {

		#if REGISTER_DETAIL
		int capVal;

		volatile uint8_t regVal;
		uint8_t modeFSK = 0;
		int bitRate = 0;
		int freqDev = 0;
		long freqCenter = 0;
		#endif

		printf_P(PSTR("Address - HEX - BIN\n\r"));
		for (uint8_t regAddr = 1; regAddr <= 0x4F; regAddr++) {
			select();
			SPI.transfer(regAddr & 0x7F); // send address + r/w bit
			regVal = SPI.transfer(0);
			unselect();

			printf("%#04x - %#04x\r\n", regAddr, regVal);

			#if REGISTER_DETAIL
			switch (regAddr) {
				case 0x1: {
					printf_P(
					PSTR(
					"Controls the automatic Sequencer ( see section 4.2 )\n\rSequencerOff:"));
					if (0x80 & regVal) {
						printf_P(PSTR("1 -> Mode is forced by the user\n\r"));
						} else {
						printf_P(
						PSTR(
						"0 -> Operating mode as selected with Mode bits in RegOpMode is automatically reached with the Sequencer\n\r"));
					}

					printf_P(
					PSTR(
					"\nEnables Listen mode, should be enabled whilst in Standby mode:\n\rListenOn:"));
					if (0x40 & regVal) {
						printf_P(PSTR("1 -> On\n\r"));
						} else {
						printf_P(PSTR("0 -> Off ( see section 4.3)\n\r"));
					}

					printf_P(
					PSTR(
					"\n\rAborts Listen mode when set together with ListenOn=0 See section 4.3.4 for details (Always reads 0.)\n\r"));
					if (0x20 & regVal) {
						printf_P(
						PSTR(
						"ERROR - ListenAbort should NEVER return 1 this is a write only register\n\r"));
					}

					printf_P(PSTR("\r\nTransceiver's operating modes:\n\rMode : "));
					capVal = (regVal >> 2) & 0x7;
					if (capVal == 0b000) {
						printf_P(PSTR("000 -> Sleep mode (SLEEP)\n\r"));
						} else if (capVal == 0b001) {
						printf_P(PSTR("001 -> Standby mode (STDBY)\n\r"));
						} else if (capVal == 0b010) {
						printf_P(PSTR("010 -> Frequency Synthesizer mode (FS)\n\r"));
						} else if (capVal == 0b011) {
						printf_P(PSTR("011 -> Transmitter mode (TX)\n\r"));
						} else if (capVal == 0b100) {
						printf_P(PSTR("100 -> Receiver Mode (RX)\n\r"));
						} else {
						printf("%#02x", capVal);
						printf_P(PSTR(" -> RESERVED\n\r"));
					}
					printf_P(PSTR("\n\r"));
					break;
				}

				case 0x2: {

					printf_P(PSTR("Data Processing mode:\r\nDataMode : "));
					capVal = (regVal >> 5) & 0x3;
					if (capVal == 0b00) {
						printf_P(PSTR("00 -> Packet mode\n\r"));
						} else if (capVal == 0b01) {
						printf_P(PSTR("01 -> reserved\n\r"));
						} else if (capVal == 0b10) {
						printf_P(
						PSTR(
						"10 -> Continuous mode with bit synchronizer\n\r"));
						} else if (capVal == 0b11) {
						printf_P(
						PSTR(
						"11 -> Continuous mode without bit synchronizer\n\r"));
					}

					printf_P(PSTR("\n\rModulation scheme:\n\rModulation Type : "));
					capVal = (regVal >> 3) & 0x3;
					if (capVal == 0b00) {
						printf_P(PSTR("00 -> FSK\n\r"));
						modeFSK = 1;
						} else if (capVal == 0b01) {
						printf_P(PSTR("01 -> OOK\n\r"));
						} else if (capVal == 0b10) {
						printf_P(PSTR("10 -> reserved\n\r"));
						} else if (capVal == 0b11) {
						printf_P(PSTR("11 -> reserved\n\r"));
					}

					printf_P(PSTR("\n\rData shaping: "));
					if (modeFSK) {
						printf_P(PSTR("in FSK:\n\r"));
						} else {
						printf_P(PSTR("in OOK:\n\r"));
					}
					printf_P(PSTR("ModulationShaping : "));
					capVal = regVal & 0x3;
					if (modeFSK) {
						if (capVal == 0b00) {
							printf_P(PSTR("00 -> no shaping\n\r"));
							} else if (capVal == 0b01) {
							printf_P(PSTR("01 -> Gaussian filter, BT = 1.0\n\r"));
							} else if (capVal == 0b10) {
							printf_P(PSTR("10 -> Gaussian filter, BT = 0.5\n\r"));
							} else if (capVal == 0b11) {
							printf_P(PSTR("11 -> Gaussian filter, BT = 0.3\n\r"));
						}
						} else {
						if (capVal == 0b00) {
							printf_P(PSTR("00 -> no shaping\n\r"));
							} else if (capVal == 0b01) {
							printf_P(PSTR("01 -> filtering with f(cutoff) = BR\n\r"));
							} else if (capVal == 0b10) {
							printf_P(PSTR("10 -> filtering with f(cutoff) = 2*BR\n\r"));
							} else if (capVal == 0b11) {
							printf_P(PSTR("ERROR - 11 is reserved\n\r"));
						}
					}

					printf_P(PSTR("\n\r"));
					break;
				}

				case 0x3: {
					bitRate = (regVal << 8);
					break;
				}

				case 0x4: {
					bitRate |= regVal;
					printf_P(
					PSTR(
					"Bit Rate (Chip Rate when Manchester encoding is enabled)\n\rBitRate : "));
					unsigned long val = 32UL * 1000UL * 1000UL / bitRate;
					printf("%lu\n\n\r", val);
					break;
				}

				case 0x5: {
					freqDev = ((regVal & 0x3f) << 8);
					break;
				}

				case 0x6: {
					freqDev |= regVal;
					printf_P(PSTR("Frequency deviation\n\rFdev : "));
					unsigned long val = 61UL * freqDev;
					printf("%lu\n\n\r", val);
					break;
				}

				case 0x7: {
					unsigned long tempVal = regVal;
					freqCenter = (tempVal << 16);
					break;
				}

				case 0x8: {
					unsigned long tempVal = regVal;
					freqCenter = freqCenter | (tempVal << 8);
					break;
				}

				case 0x9: {
					freqCenter = freqCenter | regVal;
					printf_P(PSTR("RF Carrier frequency\n\rFRF : "));
					unsigned long val = 61UL * freqCenter;
					printf("%lu\r\n\n", val);
					break;
				}

				case 0xa: {
					printf_P(PSTR("RC calibration control & status\n\rRcCalDone : "));
					if (0x40 & regVal) {
						printf_P(PSTR("1 -> RC calibration is over\n\r"));
						} else {
						printf_P(PSTR("0 -> RC calibration is in progress\n\r"));
					}

					printf_P(PSTR("\n\r"));
					break;
				}

				case 0xb: {
					printf(
					"Improved AFC routine for signals with modulation index lower than 2.  Refer to section 3.4.16 for details\n\rAfcLowBetaOn : ");
					if (0x20 & regVal) {
						printf_P(PSTR("1 -> Improved AFC routine\n\r"));
						} else {
						printf_P(PSTR("0 -> Standard AFC routine\n\r"));
					}
					printf_P(PSTR("\n\r"));
					break;
				}

				case 0xc: {
					printf_P(PSTR("Reserved\n\n\r"));
					break;
				}

				case 0xd: {
					uint8_t val;
					printf_P(PSTR("Resolution of Listen mode Idle time (calibrated RC osc):\n\rListenResolIdle : "));
					val = regVal >> 6;
					if (val == 0b00) {
						printf_P(PSTR("00 -> reserved\n\r"));
						} else if (val == 0b01) {
						printf_P(PSTR("01 -> 64 us\n\r"));
						} else if (val == 0b10) {
						printf_P(PSTR("10 -> 4.1 ms\n\r"));
						} else if (val == 0b11) {
						printf_P(PSTR("11 -> 262 ms\n\r"));
					}

					printf_P(PSTR("\nResolution of Listen mode Rx time (calibrated RC osc):\n\rListenResolRx : "));
					val = (regVal >> 4) & 0x3;
					if (val == 0b00) {
						printf_P(PSTR("00 -> reserved\n\r"));
						} else if (val == 0b01) {
						printf_P(PSTR("01 -> 64 us\n\r"));
						} else if (val == 0b10) {
						printf_P(PSTR("10 -> 4.1 ms\n\r"));
						} else if (val == 0b11) {
						printf_P(PSTR("11 -> 262 ms\n\r"));
					}

					printf_P(PSTR("\n\rCriteria for packet acceptance in Listen mode:\n\rListenCriteria : "));
					if (0x8 & regVal) {
						printf_P(
						PSTR(
						"1 -> signal strength is above RssiThreshold and SyncAddress matched\n\r"));
						} else {
						printf_P(
						PSTR(
						"0 -> signal strength is above RssiThreshold\n\r"));
					}

					printf_P(
					PSTR(
					"\n\rAction taken after acceptance of a packet in Listen mode:\n\rListenEnd : "));
					val = (regVal >> 1) & 0x3;
					if (val == 0b00) {
						printf_P(
						PSTR(
						"00 -> chip stays in Rx mode. Listen mode stops and must be disabled (see section 4.3)\n\r"));
						} else if (val == 0b01) {
						printf_P(
						PSTR(
						"01 -> chip stays in Rx mode until PayloadReady or Timeout interrupt occurs.  It then goes to the mode defined by Mode. Listen mode stops and must be disabled (see section 4.3)\n\r"));
						} else if (val == 0b10) {
						printf_P(
						PSTR(
						"10 -> chip stays in Rx mode until PayloadReady or Timeout occurs.  Listen mode then resumes in Idle state.  FIFO content is lost at next Rx wakeup.\n\r"));
						} else if (val == 0b11) {
						printf_P(PSTR("11 -> Reserved\n\r"));
					}

					printf_P(PSTR("\n\r"));
					break;
				}

				default: {
				}
			}
			#endif
		}
		unselect();
	}

	uint8_t RFM69::readTemperature(uint8_t calFactor) // returns centigrade
	{
		setMode(RF69_MODE_STANDBY);
		writeReg(REG_TEMP1, RF_TEMP1_MEAS_START);
		while ((readReg(REG_TEMP1) & RF_TEMP1_MEAS_RUNNING)) {;}
		return ~readReg(REG_TEMP2) + COURSE_TEMP_COEF + calFactor; // 'complement' corrects the slope, rising temp = rising val
	} // COURSE_TEMP_COEF puts reading in the ballpark, user can add additional correction

	void RFM69::rcCalibration() {
		writeReg(REG_OSC1, RF_OSC1_RCCAL_START);
		while ((readReg(REG_OSC1) & RF_OSC1_RCCAL_DONE) == 0x00){;}
	}

	inline void RFM69::enaleInterrupts() {
		if (!_inISR){
			sei();
		}
	}
