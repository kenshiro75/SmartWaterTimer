/* RFM69 library and code by Felix Rusu - felix@lowpowerlab.com
// Get libraries at: https://github.com/LowPowerLab/
// Make sure you adjust the settings in the configuration section below !!!
// **********************************************************************************
// Copyright Felix Rusu, LowPowerLab.com
// Library and code by Felix Rusu - felix@lowpowerlab.com
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
// You should have received a copy of the GNU General    
// Public License along with this program.
// If not, see <http://www.gnu.org/licenses></http:>.
//                                                        
// Licence can be viewed at                               
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************/

#include <Wire.h>
#include <SPI.h>



#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69



// Only used for sprintf
#include <stdio.h>

#include "LowPower.h"

#include <ArduinoJson.h>


#define SERIAL_DEBUG 2

//*********************************************************************************************
// *********** IMPORTANT SETTINGS - YOU MUST CHANGE/ONFIGURE TO FIT YOUR HARDWARE *************
//*********************************************************************************************
#define NETWORKID     201  // The same on all nodes that talk to each other
#define NODEID        10    // The unique identifier of this node
#define RECEIVER      1    // The recipient of packets

//Match frequency to the hardware version of the radio on your Feather
//#define FREQUENCY     RF69_433MHZ
#define FREQUENCY     RF69_868MHZ
//#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HCW   true // set to 'true' if you are using an RFM69HCW module

//*********************************************************************************************
#define SERIAL_BAUD   115200

/*#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega88) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega88__)*/
#define RFM69_CS      10
#define RFM69_IRQ     2
#define RFM69_IRQN    digitalPinToInterrupt(RFM69_IRQ)
#define RFM69_RST     9
#define LED           9  // onboard blinky
#elif defined(__arm__)//Use pin 10 or any pin you want
// Arduino Zero works
#define RFM69_CS      10
#define RFM69_IRQ     5
#define RFM69_IRQN    digitalPinToInterrupt(RFM69_IRQ)
#define RFM69_RST     6
#define LED           13  // onboard blinky
#elif defined(ESP8266)
// ESP8266
#define RFM69_CS      15  // GPIO15/HCS/D8
#define RFM69_IRQ     4   // GPIO04/D2
#define RFM69_IRQN    digitalPinToInterrupt(RFM69_IRQ)
#define RFM69_RST     2   // GPIO02/D4
#define LED           0   // GPIO00/D3, onboard blinky for Adafruit Huzzah
#else
#define RFM69_CS      10
#define RFM69_IRQ     2
#define RFM69_IRQN    digitalPinToInterrupt(RFM69_IRQ)
#define RFM69_RST     9
#define LED           9  // onboard blinky
#endif


#define E1 7
#define E2 8




bool bRadioInit = false; // handle radio initialiation

int  iRetries = 10; 		// max tries to send
int  iRetriesWait = 10;  	// try to send every 10ms

uint32_t packetnum = 0;  // packet counter, we increment per xmission

RFM69 radio = RFM69(RFM69_CS, RFM69_IRQ, IS_RFM69HCW, RFM69_IRQN);

long ActiveTimeMs = 6000;

void setup() {
	//while (!Serial); // wait until serial console is open, remove if not tethered to computer
	Serial.begin(SERIAL_BAUD);
	Serial.println();

#if SERIAL_DEBUG==2
		Serial.println("Arduino RFM69HCW Transmitter");
#endif


	// Hard Reset the RFM module
	pinMode(RFM69_RST, OUTPUT);	
	digitalWrite(RFM69_RST, HIGH);
	delay(100);
	digitalWrite(RFM69_RST, LOW);
	delay(100);

	// Initialize radio
	if (!radio.initialize(FREQUENCY,NODEID,NETWORKID)) {
#if SERIAL_DEBUG>0 && SERIAL_DEBUG<=2
            Serial.println("[error] radio.initialize failed!");
#endif
	}
	else {
		bRadioInit = true;

		if (IS_RFM69HCW) {
			radio.setHighPower();    // Only for RFM69HCW & HW!
		}
		radio.setPowerLevel(31); // power output ranges from 0 (5dBm) to 31 (20dBm)

		radio.encrypt(ENCRYPTKEY);

		pinMode(LED, OUTPUT);


#if SERIAL_DEBUG == 2

		Serial.print("[info] Transmitting at ");
		Serial.print(FREQUENCY == RF69_433MHZ ? 433 : FREQUENCY == RF69_868MHZ ? 868 : 915);
		Serial.println(" MHz");
		Serial.print("Network ");
		Serial.print(NETWORKID);
		Serial.print(" Node ");
		Serial.println(NODEID);
		Serial.println();
#endif

	}

  // init E pins
  pinMode(E1, OUTPUT);
  pinMode(E2, OUTPUT);
}


void loop() {
	


	if (bRadioInit)
	{


        int checksum=0;


        char szActiveMessage[50];
        szActiveMessage[0]=0;

        sprintf(szActiveMessage,"{\"s\":\"ON\",\"to\":%ld}",ActiveTimeMs);

        //sprintf(szActiveMessage,"O|%ld",ActiveTimeMs);



        // send active message
        //radioSendPayload(addChecksum(szActiveMessage));
        radioSendPayload(szActiveMessage);

        // loop for ActiveTimeMs
        unsigned long startMillis = millis();
        while((millis() - startMillis)<ActiveTimeMs) {

            //unsigned long deltaMillis = millis() - startMillis;
            //if (deltaMillis>=ActiveTimeMs){ break; }

            //check if something was received (could be an interrupt from the radio)
            if (radio.receiveDone()) {
                bool ackReq = false;
                //check if sender asked for ACK
                if (radio.ACKRequested()) {

                    radio.sendACK();
                    ackReq = true;

                }

#if SERIAL_DEBUG==2
        Serial.print('[');
        Serial.print(radio.SENDERID);
        Serial.print("] ");
        Serial.print((char *) radio.DATA);
        Serial.print("   [RX_RSSI:");
        Serial.print(radio.RSSI);
        Serial.print("]");
        //if (ackReq) {
        if (ackReq) { Serial.print(" - ACK sent"); }
        //}
        Serial.println();
#endif

                char *pszPayload = (char *) radio.DATA;

                StaticJsonBuffer<200> jsonBuffer;
                JsonObject& root = jsonBuffer.parseObject(pszPayload);

                // Test if parsing succeeds.
                if (!root.success()) {
                    Serial.println("[error] parseObject() failed ###");                    
                }
                else {
                  const char* szCMD = root["message"];
  
                  int iCMD = 0;
					        
  
                  // turn on and off led
                  digitalWrite(LED, HIGH);
                  delay(500);
                  digitalWrite(LED, LOW);
  
                  // manage cmd received
                  if (strlen(szCMD)) {
                     
                     
                      iCMD = atoi(szCMD);
                      char *payload = NULL;
                      int iCID = root["cid"];
  
                      if (iCMD==1) { // send "Test"
                              // read datas
                              payload = "test OK"; 
                      }       
                      else if (iCMD==3){ // VCC read
                          payload = longToChar(VCCread(),10);
                      }                      
                      else if (iCMD==5){ // Elettrovalve ON
                          digitalWrite(E1, HIGH);
                          digitalWrite(E2, LOW);
                      } 
                      else if (iCMD==6){ // Elettrovalve OFF
                          digitalWrite(E1, LOW);
                          digitalWrite(E2, HIGH);
                      }                       
                      else {
  #if SERIAL_DEBUG>0 && SERIAL_DEBUG<=2
                          Serial.println("[error] command not available");                          
  #endif
                          //payload="error: command not available";
                      }

                      // send data to the gateway
                      if (payload!=NULL) {

  #if SERIAL_DEBUG == 2
  
                              Serial.print("[info] payload: ");
                              Serial.println(payload);
  #endif

                          szActiveMessage[0]=0;

                          sprintf(szActiveMessage,"{\"s\":\"A\",\"cid\":%d,\"m\":\"%s\"}",iCID,payload);
                          //sprintf(szActiveMessage,"%d|A|%s*%s",iCID,payload,checksum);
                          //radioSendPayload(addChecksum(szActiveMessage));
                          radioSendPayload(szActiveMessage);
                      }

                  }

                }

            }

            radio.receiveDone(); //put radio in RX mode
        }
	}
	else
	{  // problem with radio initialization!!
		blinkAdv(LED,3,500);
	} 

  #if SERIAL_DEBUG>0 && SERIAL_DEBUG<=2
    Serial.println("[info] going to sleep!");
    delay(500);
    Serial.flush();
  #endif

  
	// put radio in sleep mode
    radio.sleep();
	
    // Enter power down state with ADC and BOD module disabled.
    // Wake up after 8s
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); 
	
	
}

int radioSendPayload(char *payload)
{
    int iRet=0;
    if (radio.sendWithRetry(RECEIVER, payload, strlen(payload), iRetries, iRetriesWait)) {
        iRet++;

       blinkAdv(LED,1,200);

#if SERIAL_DEBUG==2

        Serial.print("[info] sent payload: ");
        Serial.print(payload);
        Serial.print(" to gatewayid: ");
        Serial.print(RECEIVER);
        //if (radio.ACKReceived(RECEIVER)) { Serial.print(" - ACK received");}
        Serial.println();

#endif


    } else {
#if SERIAL_DEBUG>0 && SERIAL_DEBUG<=2
        Serial.print("[error] cannot send payload: ");
        Serial.print(payload);
        Serial.print(" to gatewayid: ");
        Serial.println(RECEIVER);
#endif
    }
    return iRet;
}

char *addChecksum(char *buffer){

    if (strlen(buffer)){

        int checksum=0;
        char sz[5];
        char *p=buffer;

        for (;*p;p++){
            checksum^=*p;
        }
        sz[0]=0;
        sprintf(sz,"*%d",checksum);
        strcat(buffer,sz);

    }

    return buffer;

}


char *ftoa(char *buffer, double d, int precision) {

	long wholePart = (long) d;

	// Deposit the whole part of the number.

	itoa(wholePart,buffer,10);

	// Now work on the faction if we need one.

	if (precision > 0) {

		// We do, so locate the end of the string and insert
		// a decimal point.

		char *endOfString = buffer;
		while (*endOfString != '\0') endOfString++;
		*endOfString++ = '.';

		// Now work on the fraction, be sure to turn any negative
		// values positive.

		if (d < 0) {
			d *= -1;
			wholePart *= -1;
		}
		
		double fraction = d - wholePart;
		while (precision > 0) {

			// Multipleby ten and pull out the digit.

			fraction *= 10;
			wholePart = (long) fraction;
			*endOfString++ = '0' + wholePart;

			// Update the fraction and move on to the
			// next digit.

			fraction -= wholePart;
			precision--;
		}

		// Terminate the string.

		*endOfString = '\0';
	}

	return buffer;
}


void wakeUp()
{
    // Just a handler for the pin interrupt.
}


void blinkAdv(int pin, int n,int t)
{
	// init pin with LOW
	digitalWrite(pin, LOW);	
	
	for (int x=0;x<n;x++){
		digitalWrite(pin, HIGH);             
		delay(t);
		digitalWrite(pin, LOW);	
		delay(t);		
	}

}


//
// readVcc()
// lettura volts 
// ritorna il valore in mV
//
long VCCread() {

  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
 
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
 
}


//
// longToChar()
// converte un tipo long in un tipo char
//
char *longToChar(long l,int base){
  char buf[6];  
  ltoa(l,buf,base);
  return buf;

}


