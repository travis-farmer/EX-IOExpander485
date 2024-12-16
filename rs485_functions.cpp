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
byte commandBuffer[3];    // Command buffer to interact with device driver
uint8_t responseBuffer[1];   // Buffer to send single response back to device driver
uint8_t numReceivedPins = 0;
int rxBufferLen = 0;
bool rxEnd = false;
byte rxStart[] = {0xFD};
byte rxTerm[] = {0xFE};
byte bufferLength;
byte inCommandPayload = PAYLOAD_NORMAL;
/*
* Function triggered when CommandStation is sending data to this device.
*/
void receiveEvent() {
  byte buffer[25];
  //unsigned long startMicros = micros();
  while (RS485_SERIAL.available()) {
    char ch = RS485_SERIAL.read();
    if (!inCommandPayload) {
      if (ch == rxStart[0]) {
        inCommandPayload = PAYLOAD_NORMAL;
        bufferLength = 0;
      }
    } else { // if (inCommandPayload)
      if (bufferLength <  (24))
        tmpBuffer[bufferLength] = ch;
        bufferLength++;
      if (inCommandPayload > PAYLOAD_NORMAL) {
        if (inCommandPayload > 22) {    // String way too long
          inCommandPayload = PAYLOAD_NORMAL;
          // fall through to ending parsing below
        } else if (ch == '"') {               // String end
          inCommandPayload = PAYLOAD_NORMAL;
          continue; // do not fall through
        } else
          inCommandPayload++;
      }
      if (inCommandPayload == PAYLOAD_NORMAL) {
        if (ch == rxTerm[0]) {
          for (int i = 0; i < bufferLength; i++) {
            buffer[i] = tmpBuffer[i];
          } 
          inCommandPayload = PAYLOAD_FALSE;
          break;
        } else if (ch == '"') {
          inCommandPayload = PAYLOAD_STRING;
        }
      }
    }
  }
  
  if (buffer[0] != RS485_NODE) {
    RS485_SERIAL.write(buffer,rxBufferLen);
    //USB_SERIAL.println("not for us");
    return;
  }
  for (int i = 2; i < rxBufferLen-2; i++) buffer[i-2] = buffer[i]; // reorder buffer[]
  rxBufferLen = 0;
  switch(buffer[2]) {
    // Initial configuration start, must be 2 bytes
    case EXIOINIT:
        {initialisePins();
        numReceivedPins = buffer[3];
        firstVpin = (buffer[4] << 8) + buffer[4];
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
        uint8_t pin = buffer[3];
        bool pullup = buffer[4];
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
        uint8_t pin = buffer[3];
        bool state = buffer[4];
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
        uint8_t pin = buffer[3];
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
        uint8_t pin = buffer[3];
        uint16_t value = (buffer[5] << 8) + buffer[4];
        uint8_t profile = buffer[6];
        uint16_t duration = (buffer[8] << 8) + buffer[7];
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
        commandBuffer[0] = rxStart[0];
        commandBuffer[1] = 0;
        commandBuffer[2] = RS485_NODE;
        commandBuffer[3] = EXIOPINS;
        commandBuffer[4] = numDigitalPins;
        commandBuffer[5] = numAnaloguePins;
        commandBuffer[6] = rxTerm[0];
      } else {
        commandBuffer[0] = rxStart[0];
        commandBuffer[1] = 0;
        commandBuffer[2] = RS485_NODE;
        commandBuffer[3] = EXIOPINS;
        commandBuffer[4] = 0;
        commandBuffer[5] = 0;
        commandBuffer[6] = rxTerm[0];
      }
      uint8_t tmpHeadA[] = {0, RS485_NODE};
      digitalWrite(RS485_DEPIN, HIGH);
      RS485_SERIAL.write(commandBuffer, sizeof(commandBuffer));
      //RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;} 
    case EXIOINITA: {
      commandBuffer[0] = rxStart[0];
        commandBuffer[1] = 0;
        commandBuffer[2] = RS485_NODE;
        commandBuffer[3] = EXIOINITA;
        for (int i = 4; i <= numAnaloguePins+4; i++) {
          if (i < numAnaloguePins+4) commandBuffer[i] = analoguePinMap[i-4];
          else commandBuffer[i] = rxTerm[0];
        }
      digitalWrite(RS485_DEPIN, HIGH);
      RS485_SERIAL.write(commandBuffer, numAnaloguePins+5);
      //RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;}
    case EXIORDAN: {
      commandBuffer[0] = rxStart[0];
        commandBuffer[1] = 0;
        commandBuffer[2] = RS485_NODE;
        commandBuffer[3] = EXIORDAN;
        for (int i = 4; i <= analoguePinBytes+4; i++) {
          if (i < analoguePinBytes+4) commandBuffer[i] = analoguePinStates[i-4];
          else commandBuffer[i] = rxTerm[0];
        }
      digitalWrite(RS485_DEPIN, HIGH);
      RS485_SERIAL.write(commandBuffer, analoguePinBytes+5);
      //RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;}
    case EXIORDD:
      {
      commandBuffer[0] = rxStart[0];
        commandBuffer[1] = 0;
        commandBuffer[2] = RS485_NODE;
        commandBuffer[3] = EXIORDD;
        for (int i = 4; i <= digitalPinBytes+4; i++) {
          if (i < digitalPinBytes+4) commandBuffer[i] = digitalPinStates[i-4];
          else commandBuffer[i] = rxTerm[0];
        }
      digitalWrite(RS485_DEPIN, HIGH);
      RS485_SERIAL.write(digitalPinStates, digitalPinBytes+5);
      //RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;}
    case EXIOVER: {
      digitalWrite(RS485_DEPIN, HIGH);
      RS485_SERIAL.write(versionBuffer, sizeof(versionBuffer));
      //RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;}
    case EXIODPUP:
      {uint8_t tmpHeadF[] = {rxStart[0], 0, RS485_NODE, (uint8_t)responseBuffer, rxTerm[0]};
      digitalWrite(RS485_DEPIN, HIGH);
      RS485_SERIAL.write(tmpHeadF,5);
      //RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;}
    case EXIOENAN:
      {uint8_t tmpHeadG[] = {rxStart[0], 0, RS485_NODE, (uint8_t)responseBuffer, rxTerm[0]};
      digitalWrite(RS485_DEPIN, HIGH);
      RS485_SERIAL.write(tmpHeadG,5);
      //RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;}
    case EXIOWRAN:
      {uint8_t tmpHeadH[] = {rxStart[0], 0, RS485_NODE, (uint8_t)responseBuffer, rxTerm[0]};
      digitalWrite(RS485_DEPIN, HIGH);
      RS485_SERIAL.write(tmpHeadH,5);
      //RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;}
    case EXIOWRD:
      {uint8_t tmpHeadI[] = {rxStart[0], 0, RS485_NODE, (uint8_t)responseBuffer, rxTerm[0]};
      digitalWrite(RS485_DEPIN, HIGH);
      RS485_SERIAL.write(tmpHeadI,5);
      //RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;}
  }
  outboundFlag = 0;
}

