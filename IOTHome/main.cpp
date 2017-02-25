 /*
 RF69Communication.cpp

 License
 **********************************************************************************
 This program is free software; you can redistribute it
 and/or modify it under the terms of the GNU General
 Public License as published by the Free Software
 Foundation; either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will
 be useful, but WITHOUT ANY WARRANTY; without even the
 implied warranty of MERCHANTABILITY or FITNESS FOR A
 PARTICULAR PURPOSE. See the GNU General Public
 License for more details.

 Licence can be viewed at
 http://www.gnu.org/licenses/gpl-3.0.txt

 Please maintain this license information along with authorship
 and copyright notices in any redistribution of this code
 **********************************************************************************
 */
 
 #include <inttypes.h>
 #include <avr/io.h>
 #include <avr/pgmspace.h>
 #include <stdio.h>
 #include <util/delay.h>
 #include <string.h>
 #include <avr/wdt.h>
 
 #include "RFM69.h"
 #include "SPI.h"
 #include "uart.h"
 #include "TimerClass.h"
 #include "pindefs.h"
 #include <PowerControl.h>
 #include <DHT22.h>
 #include <IOPin.h>
 #include <BatteryVoltage.h>
 
 static int uart_putchar(char c, FILE *stream);
 static FILE mystdout = {0};

 static int uart_putchar(char c, FILE *stream) {
	 txByte(c);
	 return 0;
 }



 int main(void)
 {
	 wdt_reset();
	 wdt_enable(WDTO_8S);
	 grnLed.setToOutput();
	 grnLed.setLow();
	 redLed.setToOutput();
	 redLed.setLow();
	 pwrPin.setToOutput();
	 pwrPin.setLow();
	 
	 rf69SpiCsPin.setToOutput();
	 rf69SpiCsPin.setHigh();
	 
	 initUart();
	 fdev_setup_stream(&mystdout, uart_putchar, NULL, _FDEV_SETUP_WRITE);
	 stdout = &mystdout;
	 
	 #ifdef TRANSCIEVER
	 printf_P(PSTR("\n\r===========Transciever started=========\n\r"));
	 #else
	 printf_P(PSTR("\n\r===========Receiver started=========\n\r"));
	 #endif
	 
	 printf("...Initializing RFM.\r\n");
	 sei();
	 wdt_reset();
	 for(uint8_t i=0; i<5; i++) {
		 grnLed.setHigh();
		 _delay_ms(200);
		 grnLed.setLow();
		 _delay_ms(200);
	 }
	 
	 DHT myDht(&dht22Pin);
	 myDht.begin();
	 
	 RFM69 radio(true, &rf69SpiCsPin, &rf69DIO0Pin);
	 
	 BatteryVoltage batteryVoltage;
	 
	 #ifdef TRANSCIEVER
	 bool res = radio.initialize(RF69_868MHZ, RFM_TRANSCIEVER_DEVICE_ID, RFM_NETWORK_ID);
	 #else
	 bool res = radio.initialize(RF69_868MHZ, RFM_RECEIVER_DEVICE_ID, RFM_NETWORK_ID);
	 #endif
	 wdt_reset();
	 
	 if (res) {
		 printf("...RF69 Initialization successful.\r\n");
		 } else {
		 printf("...RF69 Failed.\r\n");
	 }
	 
	 radio.setHighPower();
	 //radio.encrypt(ENCRYPTKEY);
	 //radio.enableAutoPower(ATC_RSSI);
	 
	 #ifdef TRANSCIEVER
	 char buffer[50];
	 
	 while(true) {
		 
		 //for(uint8_t i=0;i<10;i++){
		 //wdt_reset();
		 //_delay_ms(1000);
		 //}
		 
		 /* Power on the DHT22 sensor and git it 500ms to start (got this time by experimenting) */
		 pwrPin.setHigh();
		 _delay_ms(500);

		 float temperature = 0.0f;
		 float humidity = 0.0f;
		 
		 for(uint8_t c=0; c < 3; c++) {
			 wdt_reset();
			 temperature = myDht.getTemperature(true);
			 humidity = myDht.getHumidity();
			 if (!isnan(temperature) && !isnan(humidity)) break; 
		 };

		 /* Power off the DHT by disconnecting it */
		 pwrPin.setLow();
		 
		 uint8_t voltage = batteryVoltage.getVoltagePercentage(3300);

		 snprintf(buffer,50,"T[%+07.2f] H[%+07.2f] B[%i]",(double)temperature, (double)humidity, voltage);
		 printf("%s", buffer);

		 wdt_reset();
		 /*Retries set to 1 as we do not have ACK on arduino side */
		 bool sent = radio.sendWithRetry(RFM_RECEIVER_DEVICE_ID, buffer, strlen(buffer)+1, 1);
		 radio.sleep();
		 
		 printf_P(PSTR("...%s\r\n"), sent ? " success" : " failure");
		 
		 wdt_reset();
		 if (sent) {
			 grnLed.setHigh();
			 _delay_ms(100);
			 grnLed.setLow();
			 } else {
			 redLed.setHigh();
			 _delay_ms(100);
			 redLed.setLow();
		 }
		 
		 PowerControl::sleepNow(10);
	 }
	 #endif
	 
	 #ifdef RECEIVER
	 radio.promiscuous(false);
	 
	 #ifdef DEBUG
	 printf("Checking if radio received signal...\r\n");
	 #endif
	 
	 while(true){
		 if (radio.receiveDone())
		 {
			 
			 printf("[sender:%i]", radio.SENDERID);
			 printf(" to [%i/(me:%i)l:%i]", radio.TARGETID, RFM_RECEIVER_DEVICE_ID, radio.DATALEN);
			 
			 for (uint8_t i = 0; i < radio.DATALEN; i++)
			 printf("%c",(char)radio.DATA[i]);
			 printf("[RX_RSSI: %d]", radio.RSSI);
			 
			 if (radio.ACKRequested())
			 {
				 radio.sendACK();
				 printf_P(PSTR(" ACK"));
			 }
			 
			 printf("\r\n");
			 _delay_ms(500);
		 }
	 }
	 #endif
	 
	 #ifndef TRANSCIEVER
	 #ifndef RECEIVER
	 #error "TRANSCIEVER or RECEIVER must be defined";
	 #endif
	 #endif
 }

