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

uint8_t numAnaloguePins = 0;  // Init with 0, will be overridden by config
uint8_t numDigitalPins = 0;   // Init with 0, will be overridden by config
uint8_t numPWMPins = 0;  // Number of PWM capable pins
bool setupComplete = false;   // Flag when initial configuration/setup has been received
uint8_t outboundFlag;   // Used to determine what data to send back to the CommandStation
byte commandBuffer[3];    // Command buffer to interact with device driver
byte responseBuffer[1];   // Buffer to send single response back to device driver
uint8_t numReceivedPins = 0;
int rxBufferLen = 0;
bool rxEnd = false;
byte rxStart[] = {0xFD};
byte rxTerm[] = {0xFE};

/*
* Function triggered when CommandStation is sending data to this device.
*/
void receiveEvent() {
  byte buffer[25];
  //unsigned long startMicros = micros();
  if (RS485_SERIAL.available()) {
    rxBufferLen = RS485_SERIAL.readBytesUntil(rxTerm[0], buffer, 25);
  }
  
  if (buffer[0] != rxStart[0]) return; // bad packet
  if (buffer[1] != RS485_NODE) return; // not for us
  for (int i = 3; i < rxBufferLen-3; i++) buffer[i-3] = buffer[i]; // reorder buffer[]
  int numBytes = rxBufferLen-4;
  rxBufferLen = 0;
  switch(buffer[1]) {
    // Initial configuration start, must be 2 bytes
    case EXIOINIT:
        {initialisePins();
        numReceivedPins = buffer[2];
        firstVpin = (buffer[4] << 8) + buffer[3];
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
        uint8_t pin = buffer[2];
        bool pullup = buffer[3];
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
        uint8_t pin = buffer[2];
        bool state = buffer[3];
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
        uint8_t pin = buffer[2];
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
        uint8_t pin = buffer[2];
        uint16_t value = (buffer[4] << 8) + buffer[3];
        uint8_t profile = buffer[5];
        uint16_t duration = (buffer[7] << 8) + buffer[6];
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
      {if (setupComplete) {
        commandBuffer[0] = EXIOPINS;
        commandBuffer[1] = numDigitalPins;
        commandBuffer[2] = numAnaloguePins;
      } else {
        commandBuffer[0] = 0;
        commandBuffer[1] = 0;
        commandBuffer[2] = 0;
      }
      uint8_t tmpHeadA[] = {0, RS485_NODE};
      digitalWrite(RS485_DEPIN, HIGH);
      RS485_SERIAL.write(rxStart,1);
      RS485_SERIAL.write(tmpHeadA,2);
      RS485_SERIAL.write(commandBuffer, sizeof(commandBuffer));
      RS485_SERIAL.write(rxTerm,1);
      RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;}
    case EXIOINITA:
      {uint8_t tmpHeadB[] = {0, RS485_NODE, EXIOINITA};
      digitalWrite(RS485_DEPIN, HIGH);
      RS485_SERIAL.write(rxStart,1);
      RS485_SERIAL.write(tmpHeadB,3);
      RS485_SERIAL.write(analoguePinMap, numAnaloguePins);
      RS485_SERIAL.write(rxTerm,1);
      RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;}
    case EXIORDAN:
      {uint8_t tmpHeadC[] = {0, RS485_NODE, EXIORDAN};
      digitalWrite(RS485_DEPIN, HIGH);
      RS485_SERIAL.write(rxStart,1);
      RS485_SERIAL.write(tmpHeadC,3);
      RS485_SERIAL.write(analoguePinStates, analoguePinBytes);
      RS485_SERIAL.write(rxTerm,1);
      RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;}
    case EXIORDD:
      {uint8_t tmpHeadD[] = {0, RS485_NODE, EXIORDD};
      digitalWrite(RS485_DEPIN, HIGH);
      RS485_SERIAL.write(rxStart,1);
      RS485_SERIAL.write(tmpHeadD,3);
      RS485_SERIAL.write(digitalPinStates, digitalPinBytes);
      RS485_SERIAL.write(rxTerm,1);
      RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;}
    case EXIOVER:
      {uint8_t tmpHeadE[] = {0, RS485_NODE, EXIOVER};
      digitalWrite(RS485_DEPIN, HIGH);
      RS485_SERIAL.write(rxStart,1);
      RS485_SERIAL.write(tmpHeadE,3);
      RS485_SERIAL.write(versionBuffer, sizeof(versionBuffer));
      RS485_SERIAL.write(rxTerm,1);
      RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;}
    case EXIODPUP:
      {uint8_t tmpHeadF[] = {0, RS485_NODE};
      digitalWrite(RS485_DEPIN, HIGH);
      RS485_SERIAL.write(rxStart,1);
      RS485_SERIAL.write(tmpHeadF,2);
      RS485_SERIAL.write(responseBuffer, sizeof(responseBuffer));
      RS485_SERIAL.write(rxTerm,1);
      RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;}
    case EXIOENAN:
      {uint8_t tmpHeadG[] = {0, RS485_NODE};
      digitalWrite(RS485_DEPIN, HIGH);
      RS485_SERIAL.write(rxStart,1);
      RS485_SERIAL.write(tmpHeadG,2);
      RS485_SERIAL.write(responseBuffer, sizeof(responseBuffer));
      RS485_SERIAL.write(rxTerm,1);
      RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;}
    case EXIOWRAN:
      {uint8_t tmpHeadH[] = {0, RS485_NODE};
      digitalWrite(RS485_DEPIN, HIGH);
      RS485_SERIAL.write(rxStart,1);
      RS485_SERIAL.write(tmpHeadH,2);
      RS485_SERIAL.write(responseBuffer, sizeof(responseBuffer));
      RS485_SERIAL.write(rxTerm,1);
      RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;}
    case EXIOWRD:
      {uint8_t tmpHeadI[] = {0, RS485_NODE};
      digitalWrite(RS485_DEPIN, HIGH);
      RS485_SERIAL.write(rxStart,1);
      RS485_SERIAL.write(tmpHeadI,2);
      RS485_SERIAL.write(responseBuffer, sizeof(responseBuffer));
      RS485_SERIAL.write(rxTerm,1);
      RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;}
  }
  outboundFlag = 0;
}

