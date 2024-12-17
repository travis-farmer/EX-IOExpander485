/*
 *  © 2023, Peter Cole. All rights reserved.
 *  
 *  This file is part of EX-IOExpander.
 *
 *  This is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  It is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <Arduino.h>
#include "globals.h"
#include "rs485_functions.h"
#include "display_functions.h"
#include "pin_io_functions.h"
static const byte PAYLOAD_FALSE = 0;
static const byte PAYLOAD_NORMAL = 1;
static const byte PAYLOAD_STRING = 2;
#define COMMAND_BUFFER_SIZE 25
byte tmpBuffer[COMMAND_BUFFER_SIZE]; 
uint8_t numAnaloguePins = 0;  // Init with 0, will be overridden by config
uint8_t numDigitalPins = 0;   // Init with 0, will be overridden by config
uint8_t numPWMPins = 0;  // Number of PWM capable pins
bool setupComplete = false;   // Flag when initial configuration/setup has been received
uint8_t outboundFlag;   // Used to determine what data to send back to the CommandStation
char commandBuffer[3];    // Command buffer to interact with device driver
byte responseBuffer[1];   // Buffer to send single response back to device driver
uint8_t numReceivedPins = 0;
int rxBufferLen = 0;
bool rxEnd = false;
char rxStart[] = {'<'};
char rxTerm[] = {'>'};
byte bufferLength;
byte inCommandPayload = PAYLOAD_FALSE;
/*
* Function triggered when CommandStation is sending data to this device.
*/
void receiveEvent() {
  char buffer[25];
  //unsigned long startMicros = micros();
  byte ch[1];
  while (RS485_SERIAL.available()) {
    RS485_SERIAL.readBytes(ch,1);
    //RS485_SERIAL.write(ch,1); // send round the ring in case not ours
    if (!inCommandPayload) {
      if (ch[0] == '<') {
        inCommandPayload = PAYLOAD_NORMAL;
        bufferLength = 0;
        USB_SERIAL.println();
        USB_SERIAL.print("S");
      }
    } else { // if (inCommandPayload)

      //if (bufferLength <  (COMMAND_BUFFER_SIZE-1)) {
        tmpBuffer[bufferLength++] = ch[0];
        if (ch[0] != '>') {
          USB_SERIAL.print((char)ch[0]);
          USB_SERIAL.print(":");
        }
      
      //}
        
      if (inCommandPayload > PAYLOAD_NORMAL) {
        if (inCommandPayload > 32 + 2) {    // String way too long
          ch[0] = '>';                         // we end this nonsense
          inCommandPayload = PAYLOAD_NORMAL;
          USB_SERIAL.print("U");
          // fall through to ending parsing below
        } else
          inCommandPayload++;
      }
      if (inCommandPayload == PAYLOAD_NORMAL) {
        if (ch[0] == '>') {
          for (int i = 0; i < bufferLength; i++) {
            buffer[i] = tmpBuffer[i];
            
          }
          rxBufferLen = bufferLength;
          bufferLength = 0;
          USB_SERIAL.println("X");
          buffer[bufferLength] = 0x00;
          inCommandPayload = PAYLOAD_FALSE;
          break;
        }
      }
    }
  }
  if (rxBufferLen < 1) {
    //if (inCommandPayload >1) USB_SERIAL.println("P");
    return;
  }
  char curBuff[25];
  int outArray[25];
  int byteCntr = 0;
  int counter = 0;
  for (int i = 0; i< 25; i++) {
    if (i > rxBufferLen) {
      free(curBuff);
    }
    if (buffer[i] == 0x20){
      //outArray[counter] = 0x00;
      outArray[byteCntr] = atoi(curBuff);
      byteCntr++;
      counter = 0;
    }else if (buffer[i] == 0x00) {
      //do nothing
    } else {
      curBuff[counter] = buffer[i];
      counter++;
    }
  }
  rxBufferLen = 0;
  for (int k = 0; k < byteCntr; k++) {
    USB_SERIAL.print(outArray[k]);
    USB_SERIAL.print(":");
  }
  USB_SERIAL.println();
  if (outArray[0] != RS485_NODE) return;
  switch(outArray[0]) {
    // Initial configuration start, must be 2 bytes
    case EXIOINIT:
        {initialisePins();
        numReceivedPins = outArray[2];
        firstVpin = (outArray[4] << 8) + outArray[3];
        if (numReceivedPins == numPins) {
          displayEventFlag = 0;
          setupComplete = true;
        } else {
          displayEventFlag = 1;
          setupComplete = false;
        }
        outboundFlag = EXIOINIT;
        displayEvent = EXIOINIT;
        requestEvent();
      break;}
    case EXIOINITA:
        {outboundFlag = EXIOINITA;
      requestEvent();
      break;}
    // Flag to set digital pin pullups, 0 disabled, 1 enabled
    case EXIODPUP:
      {outboundFlag = EXIODPUP;
        uint8_t pin = outArray[1];
        bool pullup = outArray[2];
        bool response = enableDigitalInput(pin, pullup);
        if (response) {
          responseBuffer[0] = EXIORDY;
        } else {
          responseBuffer[0] = EXIOERR;
        }
      requestEvent();
      break;}
    case EXIORDAN:
        {outboundFlag = EXIORDAN;
      requestEvent();
      break;}
    case EXIOWRD:
      {outboundFlag = EXIOWRD;
        uint8_t pin = outArray[1];
        bool state = outArray[2];
        bool response = writeDigitalOutput(pin, state);
        if (response) {
          responseBuffer[0] = EXIORDY;
        } else {
          responseBuffer[0] = EXIOERR;
        }
      requestEvent();
      break;}
    case EXIORDD:
        {outboundFlag = EXIORDD;
      requestEvent();
      break;}
    case EXIOVER:
        {outboundFlag = EXIOVER;
      requestEvent();
      break;}
    case EXIOENAN:
      {outboundFlag = EXIOENAN;
        uint8_t pin = outArray[3];
        bool response = enableAnalogue(pin);
        if (response) {
          responseBuffer[0] = EXIORDY;
        } else {
          responseBuffer[0] = EXIOERR;
        }
      requestEvent();
      break;}
    case EXIOWRAN:
      {outboundFlag = EXIOWRAN;
        uint8_t pin = outArray[3];
        uint16_t value = outArray[4];
        uint8_t profile = outArray[5];
        uint16_t duration = outArray[6];
        bool response = writeAnalogue(pin, value, profile, duration);
        if (response) {
          responseBuffer[0] = EXIORDY;
        } else {
          responseBuffer[0] = EXIOERR;
        }
      requestEvent();
      break;}
  }
}

/*
* Function triggered when CommandStation polls for inputs on this device.
*/
void requestEvent() {
  
  switch(outboundFlag) {
    case EXIOINIT:
      {
      char buff[25];
      sprintf(buff, "<%i %i %i %i %i>", 0, RS485_NODE, EXIOPINS, numDigitalPins, numAnaloguePins);
      digitalWrite(RS485_DEPIN, HIGH);

      RS485_SERIAL.print(buff);
      //RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;} 
    case EXIOINITA: {
      char buff[25];
      
      int cntr = 0;
      char tmpStr[25];
      char tmpTotal[200];
      for (int i = 0; i <= numAnaloguePins; i++) {
        sprintf(tmpStr, "%i ", analoguePinMap[i]);
        for (int j = 0; j < 25; j++) {
          tmpTotal[cntr] = tmpStr[j];
          if (tmpStr[j] == ' ') break;
        }
      }
      sprintf(buff, "<%i %i %i %s>", 0, RS485_NODE, EXIOINITA, tmpTotal);
      digitalWrite(RS485_DEPIN, HIGH);
      RS485_SERIAL.print(buff);
      //RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;}
    case EXIORDAN: {
      char buff[25];
      
      int cntr = 0;
      char tmpStrB[25];
      char tmpTotalB[200];
      for (int i = 0; i <= analoguePinBytes; i++) {
        sprintf(tmpStrB, "%i ", analoguePinStates[i]);
        for (int j = 0; j < 25; j++) {
          tmpTotalB[cntr] = tmpStrB[j];
          if (tmpStrB[j] == ' ') break;
        }
      }
      sprintf(buff, "<%i %i %i %s>", 0, RS485_NODE, EXIORDAN, tmpTotalB);
      digitalWrite(RS485_DEPIN, HIGH);
      RS485_SERIAL.print(buff);
      //RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;}
    case EXIORDD:
      {
      char buff[25];
      
      int cntr = 0;
      char tmpStrB[25];
      char tmpTotalB[200];
      for (int i = 0; i <= digitalPinBytes; i++) {
        sprintf(tmpStrB, "%i ", digitalPinStates[i]);
        for (int j = 0; j < 25; j++) {
          tmpTotalB[cntr] = tmpStrB[j];
          if (tmpStrB[j] == ' ') break;
        }
      }
      sprintf(buff, "<%i %i %i %s>", 0, RS485_NODE, EXIORDD, tmpTotalB);
      digitalWrite(RS485_DEPIN, HIGH);
      RS485_SERIAL.print(buff);
      //RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;}
    case EXIOVER: {
      char buff[25];
      sprintf(buff, "<%i %i %i %i %i %i>", 0, RS485_NODE, EXIOVER, versionBuffer[0], versionBuffer[1], versionBuffer[2]);
      digitalWrite(RS485_DEPIN, HIGH);
      RS485_SERIAL.print(buff);
      //RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;}
    case EXIODPUP:
      {char buff[25];
      sprintf(buff, "<%i %i %i>", 0, RS485_NODE, responseBuffer);
      digitalWrite(RS485_DEPIN, HIGH);
      RS485_SERIAL.print(buff);
      //RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;}
    case EXIOENAN:
      {char buff[25];
      sprintf(buff, "<%i %i %i>", 0, RS485_NODE, responseBuffer);
      digitalWrite(RS485_DEPIN, HIGH);
      RS485_SERIAL.print(buff);
      //RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;}
    case EXIOWRAN:
      {char buff[25];
      sprintf(buff, "<%i %i %i>", 0, RS485_NODE, responseBuffer);
      digitalWrite(RS485_DEPIN, HIGH);
      RS485_SERIAL.print(buff);
      //RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;}
    case EXIOWRD:
      {char buff[25];
      sprintf(buff, "<%i %i %i>", 0, RS485_NODE, responseBuffer);
      digitalWrite(RS485_DEPIN, HIGH);
      RS485_SERIAL.print(buff);
      //RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;}
  }
  outboundFlag = 0;
}

