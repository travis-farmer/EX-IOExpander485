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

void updateCrc(uint8_t *buf, uint16_t len) {
  uint16_t crc = calculateCrc(buf, len);
  buf[len] = lowByte(crc);
  buf[len + 1] = highByte(crc);
}

bool crcGood(uint8_t *buf, uint16_t len) {
  uint16_t aduCrc = buf[len] | (buf[len + 1] << 8);
  uint16_t calculatedCrc = calculateCrc(buf, len);
  if (aduCrc == calculatedCrc) return true;
  else return false;
}

uint16_t calculateCrc(uint8_t *buf, uint16_t len) {
  uint16_t value = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    value ^= (uint16_t)buf[i];
    for (uint8_t j = 0; j < 8; j++) {
      bool lsb = value & 1;
      value >>= 1;
      if (lsb == true) value ^= 0xA001;
    }
  }
  return value;
}

/*
* Function triggered when CommandStation is sending data to this device.
*/
void receiveEvent() {
  byte buffer[25];
  if (!RS485_SERIAL.available()) {
    return;
  }
  uint16_t len = 0;
  unsigned long startMicros = micros();
  do {
    if (RS485_SERIAL.available()) {
      startMicros = micros();
      buffer[len] = RS485_SERIAL.read();
      len++;
    }
  } while (micros() - startMicros <= 500 && len < 256);
  if (!crcGood(responseBuffer,sizeof(responseBuffer)-2)) return;
  int numBytes = len-2;
  if (buffer[0] == 0xFF) return; // other nodes sending to master
  if (buffer[1] != RS485_NODE) return; // not for us
  for (int i = 2; i < numBytes-1; i++) buffer[i-1] = buffer[i]; // reorder buffer[]
  switch(buffer[0]) {
    // Initial configuration start, must be 2 bytes
    case EXIOINIT:
      if (numBytes == 4) {
        initialisePins();
        numReceivedPins = buffer[1];
        firstVpin = (buffer[3] << 8) + buffer[2];
        if (numReceivedPins == numPins) {
          displayEventFlag = 0;
          setupComplete = true;
        } else {
          displayEventFlag = 1;
          setupComplete = false;
        }
        outboundFlag = EXIOINIT;
        displayEvent = EXIOINIT;
      } else {
        displayEventFlag = 2;
      }
      break;
    case EXIOINITA:
      if (numBytes == 1) {
        outboundFlag = EXIOINITA;
      } else {
        displayEvent = EXIOINITA;
      }
      break;
    // Flag to set digital pin pullups, 0 disabled, 1 enabled
    case EXIODPUP:
      outboundFlag = EXIODPUP;
      if (numBytes == 3) {
        uint8_t pin = buffer[1];
        bool pullup = buffer[2];
        bool response = enableDigitalInput(pin, pullup);
        if (response) {
          responseBuffer[0] = EXIORDY;
        } else {
          responseBuffer[0] = EXIOERR;
        }
      } else {
        displayEvent = EXIODPUP;
        responseBuffer[0] = EXIOERR;
      }
      break;
    case EXIORDAN:
      if (numBytes == 1) {
        outboundFlag = EXIORDAN;
      }
      break;
    case EXIOWRD:
      outboundFlag = EXIOWRD;
      if (numBytes == 3) {
        uint8_t pin = buffer[1];
        bool state = buffer[2];
        bool response = writeDigitalOutput(pin, state);
        if (response) {
          responseBuffer[0] = EXIORDY;
        } else {
          responseBuffer[0] = EXIOERR;
        }
      } else {
        displayEvent = EXIOWRD;
        responseBuffer[0] = EXIOERR;        
      }
      break;
    case EXIORDD:
      if (numBytes == 1) {
        outboundFlag = EXIORDD;
      }
      break;
    case EXIOVER:
      if (numBytes == 1) {
        outboundFlag = EXIOVER;
      }
      break;
    case EXIOENAN:
      outboundFlag = EXIOENAN;
      if (numBytes == 2) {
        uint8_t pin = buffer[1];
        bool response = enableAnalogue(pin);
        if (response) {
          responseBuffer[0] = EXIORDY;
        } else {
          responseBuffer[0] = EXIOERR;
        }
      } else {
        responseBuffer[0] = EXIOERR;
      }
      break;
    case EXIOWRAN:
      outboundFlag = EXIOWRAN;
      if (numBytes == 7) {
        uint8_t pin = buffer[1];
        uint16_t value = (buffer[3] << 8) + buffer[2];
        uint8_t profile = buffer[4];
        uint16_t duration = (buffer[6] << 8) + buffer[5];
        bool response = writeAnalogue(pin, value, profile, duration);
        if (response) {
          responseBuffer[0] = EXIORDY;
        } else {
          responseBuffer[0] = EXIOERR;
        }
      } else {
        displayEvent = EXIOWRAN;
        responseBuffer[0] = EXIOERR;
      }
      break;
    default:
      break;
  }
}

void addMasterFlag(uint8_t *buf) {
  //streach buffer and add master flag
  for (int i = sizeof(buf)-2; i > 0; i--) buf[i] = buf[i-1];
  buf[0] = 0xFF;
}

/*
* Function triggered when CommandStation polls for inputs on this device.
*/
void requestEvent() {
  
  switch(outboundFlag) {
    case EXIOINIT:
      if (setupComplete) {
        commandBuffer[0] = EXIOPINS;
        commandBuffer[1] = numDigitalPins;
        commandBuffer[2] = numAnaloguePins;
      } else {
        commandBuffer[0] = 0;
        commandBuffer[1] = 0;
        commandBuffer[2] = 0;
      }
      uint8_t newBufferA[5];
      memcpy(newBufferA, commandBuffer, sizeof(commandBuffer));
      calloc(*newBufferA, sizeof(newBufferA)+1);
      addMasterFlag(newBufferA);
      updateCrc(newBufferA, sizeof(newBufferA)-2);
      digitalWrite(RS485_DEPIN, HIGH);
      RS485_SERIAL.write(commandBuffer, sizeof(commandBuffer));
      RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;
    case EXIOINITA:
      uint8_t newBufferB[5];
      calloc(*newBufferB, numAnaloguePins+2);
      memcpy(newBufferB, analoguePinMap, numAnaloguePins);
      calloc(*newBufferB, sizeof(newBufferA)+1);
      addMasterFlag(newBufferB);
      updateCrc(newBufferB, sizeof(newBufferB)-2);
      digitalWrite(RS485_DEPIN, HIGH);
      RS485_SERIAL.write(analoguePinMap, numAnaloguePins+2);
      RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;
    case EXIORDAN:
      uint8_t newBufferC[5];
      calloc(*newBufferC, analoguePinBytes+2);
      memcpy(newBufferC, analoguePinStates, analoguePinBytes);
      calloc(*newBufferC, sizeof(newBufferA)+1);
      addMasterFlag(newBufferC);
      updateCrc(newBufferC, sizeof(newBufferC)-2);
      digitalWrite(RS485_DEPIN, HIGH);
      RS485_SERIAL.write(analoguePinStates, analoguePinBytes+2);
      RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;
    case EXIORDD:
      uint8_t newBufferD[5];
      calloc(*newBufferD, digitalPinBytes+2);
      memcpy(newBufferD, digitalPinStates, digitalPinBytes);
      calloc(*newBufferD, sizeof(newBufferA)+1);
      addMasterFlag(newBufferD);
      updateCrc(newBufferD, sizeof(newBufferD)-2);
      digitalWrite(RS485_DEPIN, HIGH);
      RS485_SERIAL.write(digitalPinStates, digitalPinBytes+2);
      RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;
    case EXIOVER:
      uint8_t newBufferE[5];
      memcpy(newBufferE, versionBuffer,sizeof(versionBuffer));
      calloc(*newBufferE, sizeof(newBufferA)+1);
      addMasterFlag(newBufferE);
      updateCrc(newBufferE, sizeof(newBufferE)-2);
      digitalWrite(RS485_DEPIN, HIGH);
      RS485_SERIAL.write(versionBuffer, sizeof(newBufferE));
      RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;
    case EXIODPUP:
      uint8_t newBufferF[5];
      calloc(*newBufferF, 3);
      memcpy(newBufferF, responseBuffer, sizeof(responseBuffer));
      calloc(*newBufferF, sizeof(newBufferA)+1);
      addMasterFlag(newBufferF);
      updateCrc(newBufferF, sizeof(newBufferF)-2);
      digitalWrite(RS485_DEPIN, HIGH);
      RS485_SERIAL.write(responseBuffer, sizeof(newBufferF));
      RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;
    case EXIOENAN:
      uint8_t newBufferG[5];
      calloc(*newBufferG, 3);
      memcpy(newBufferG, responseBuffer, sizeof(responseBuffer));
      calloc(*newBufferG, sizeof(newBufferA)+1);
      addMasterFlag(newBufferG);
      updateCrc(newBufferG, sizeof(newBufferG)-2);
      digitalWrite(RS485_DEPIN, HIGH);
      RS485_SERIAL.write(responseBuffer, sizeof(newBufferG));
      RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;
    case EXIOWRAN:
      uint8_t newBufferH[5];
      calloc(*newBufferH, 3);
      memcpy(newBufferH, responseBuffer, sizeof(responseBuffer));
      calloc(*newBufferH, sizeof(newBufferA)+1);
      addMasterFlag(newBufferH);
      updateCrc(newBufferH, sizeof(newBufferH)-2);
      digitalWrite(RS485_DEPIN, HIGH);
      RS485_SERIAL.write(responseBuffer, sizeof(newBufferH));
      RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;
    case EXIOWRD:
      uint8_t newBufferI[5];
      calloc(*newBufferI, 3);
      memcpy(newBufferI, responseBuffer, sizeof(responseBuffer));
      calloc(*newBufferI, sizeof(newBufferA)+1);
      addMasterFlag(newBufferI);
      updateCrc(newBufferI, sizeof(newBufferI)-2);
      digitalWrite(RS485_DEPIN, HIGH);
      RS485_SERIAL.write(responseBuffer, sizeof(newBufferI));
      RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;
    default:
      break;
  }
}

void disableWire() {
#ifdef WIRE_HAS_END
  Wire.end();
#else
#if defined(USB_SERIAL)
  USB_SERIAL.println(F("WARNING! The Wire.h library has no end() function, ensure EX-IOExpander is disconnected from your CommandStation"));
#endif
#endif
}