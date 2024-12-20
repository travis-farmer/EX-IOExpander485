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
char commandBuffer[3];    // Command buffer to interact with device driver
uint8_t responseBuffer[1];   // Buffer to send single response back to device driver
uint8_t numReceivedPins = 0;
int byteCounter = 0;
#define CRC16_POLYNOME              0x8001 // x15 + 1 =  1000 0000 0000 0001 = 0x8001
#define ARRAY_SIZE 250
bool flagEnd = false;
bool flagEnded = false;
bool flagStart = false;
bool flagStarted = false;
bool rxStart = false;
bool rxEnd = false;
bool crcPass = false;
uint16_t received_crc;
uint8_t crc[2];
// CRC-16 implementation (replace with your preferred CRC library if needed)

uint16_t crc16(uint8_t *data, uint16_t length) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      bool bit = ((crc & 0x0001) != 0);
      crc >>= 1;
      if (bit) {
        crc ^= 0xA001;
      }
    }
  }
  return crc;
}

void SendResponce(uint8_t *Buffer, int byteCount) {
      // Calculate CRC for response data
      uint16_t response_crc = crc16((uint8_t*)Buffer, byteCount);
      digitalWrite(RS485_DEPIN,HIGH);
      // Send response data with CRC
      RS485_SERIAL.write(response_crc >> 8);
      RS485_SERIAL.write(response_crc & 0xFF);
      RS485_SERIAL.write(byteCount);
      for (int i = 0; i < byteCount; i++) {
        RS485_SERIAL.write(Buffer[i]);
      }
      digitalWrite(RS485_DEPIN,LOW);
}

void serialLoopRx() {
  int numBytes = 0;
  // Check if data is available
  if (RS485_SERIAL.available()) {
    uint8_t received_data[ARRAY_SIZE];
    
    uint16_t calculated_crc;
    int byteCount = 100;
    
    uint8_t byte_array[byteCount];
    int curByte = RS485_SERIAL.read();
    
    if (curByte == 0xFE && flagStart == false) flagStart = true;
    else if ( curByte == 0xFE && flagStart == true) {
      byteCounter = 0;
      flagStarted = true;
      flagStart = false;
      flagEnded = false;
      rxStart = true;
      rxEnd = false;
      crcPass = false;
    }else if (flagStarted) {
      crc[0] = curByte;
      byteCounter++;
      flagStarted = false;
    } else if (byteCounter == 1) {
      crc[1] = curByte;
      received_crc  = (crc[0] << 8) | crc[1];
      USB_SERIAL.print("crc ");
      USB_SERIAL.print(received_crc, HEX);
      USB_SERIAL.print(":");
      byteCounter++;
    } else if (byteCounter == 2) {
      byteCount = curByte;
      byteCounter++;
    } else if (flagEnded == false && byteCounter >= 3) {
      received_data[byteCounter-3] = curByte;
      byteCounter++;
    }
    if (curByte == 0xFD && flagEnd == false) flagEnd = true;
    else if ( curByte == 0xFD && flagEnd == true) {
      flagEnded = true;
      flagEnd = false;
      rxEnd = true;
      byteCount = byteCounter;
      byteCounter = 0;
    }
    if (flagEnded) {
      calculated_crc = crc16((uint8_t*)received_data, byteCount-6);
      USB_SERIAL.print("CRC ");
      USB_SERIAL.print(calculated_crc, HEX);
      USB_SERIAL.println(":");
      if (received_crc == calculated_crc) {
        USB_SERIAL.println("CRC PASS");
        crcPass = true;
      }
      flagEnded = false;
    }
    // Check CRC validity
    if (crcPass) {
      int nodeTo = received_data[0];
      if (nodeTo != RS485_NODE) {
        // retransmit and exit, if not ours
        //SendResponce(received_data);
       return;
      }
      int nodeFr = received_data[1];
      int AddrCode = received_data[2];
      USB_SERIAL.println(AddrCode, HEX);
      switch (AddrCode) {
        case EXIOINIT:
          {
          initialisePins();
          numReceivedPins = received_data[3];
          firstVpin = (received_data[5] << 8) + received_data[4];
          USB_SERIAL.print(numReceivedPins);
          USB_SERIAL.print(":");
          USB_SERIAL.println(firstVpin);
          if (numReceivedPins == numPins) {
            displayEventFlag = 0;
            setupComplete = true;
          } else {
            displayEventFlag = 1;
            setupComplete = false;
          }
          uint8_t resArrayA[ARRAY_SIZE];
          resArrayA[0] = (0);
          resArrayA[1] = (RS485_NODE);
          resArrayA[2] = (EXIOPINS);
          resArrayA[3] = (numDigitalPins);
          resArrayA[4] = (numAnaloguePins);
          SendResponce(resArrayA,5);
          break;}
        case EXIOINITA:
          {
          uint8_t resArrayB[ARRAY_SIZE];
          resArrayB[0] = (0);
          resArrayB[1] = (RS485_NODE);
          resArrayB[2] = (EXIOINITA);
          int j = 0;
          for (j = 0; j < numAnaloguePins; j++) {
            resArrayB[j+3] = highByte(analoguePinMap[j]);
          }
          SendResponce(resArrayB,j+3);
          break;}
        // Flag to set digital pin pullups, 0 disabled, 1 enabled
        case EXIODPUP:
          {
            uint8_t pin = received_data[3];
            bool pullup = received_data[4];
            bool response = enableDigitalInput(pin, pullup);
            uint8_t resArrayC[ARRAY_SIZE];
            resArrayC[0] = (0);
            resArrayC[1] = (RS485_NODE);
            if (response) {
              resArrayC[2] = (EXIORDY);
            } else {
              resArrayC[2] = (EXIOERR);
            }
          SendResponce(resArrayC,3);
          break;}
          case EXIORDAN:
            {
            uint8_t resArrayD[ARRAY_SIZE];
            resArrayD[0] = (0);
            resArrayD[1] = (RS485_NODE);
            resArrayD[2] = (EXIORDAN);
            int j;
            for (j = 0; j < analoguePinBytes; j++) {
              resArrayD[j+3] = (analoguePinStates[j]);
            }
            SendResponce(resArrayD, j+3);
            break;}
          case EXIOWRD:
            {
            uint8_t pin = received_data[3];
            bool state = ((received_data[4]) == 1)? true:false;
            bool response = writeDigitalOutput(pin, state);
            uint8_t resArrayE[ARRAY_SIZE];
            resArrayE[0] = (0);
            resArrayE[1] = (RS485_NODE);
            if (response) {
              resArrayE[2] = (EXIORDY);
            } else {
              resArrayE[2] = (EXIOERR);
            }
            SendResponce(resArrayE,3);
            break;}
          case EXIORDD:
            {
            uint8_t resArrayF[ARRAY_SIZE];
            resArrayF[0] = (0);
            resArrayF[1] = (RS485_NODE);
            resArrayF[2] = (EXIORDD);
            int j;
            for (j = 0; j < digitalPinBytes; j++) {
              resArrayF[j+3] = (digitalPinStates[j]);
            }
            SendResponce(resArrayF,j+3);
            break;}
          case EXIOVER:
            {
            uint8_t resArrayG[ARRAY_SIZE];
            resArrayG[0] = (0);
            resArrayG[1] = (RS485_NODE);
            resArrayG[2] = (EXIOVER);
            resArrayG[3] = (versionBuffer[0]);
            resArrayG[4] = (versionBuffer[1]);
            resArrayG[5] = (versionBuffer[2]);
            SendResponce(resArrayG,6);
            break;}
          case EXIOENAN:
            {
            uint8_t pin = received_data[3];
            bool response = enableAnalogue(pin);
            uint8_t resArrayH[ARRAY_SIZE];
            resArrayH[0] = (0);
            resArrayH[1] = (RS485_NODE);
            if (response) {
              resArrayH[2] = (EXIORDY);
            } else {
              resArrayH[2] = (EXIOERR);
            }
            SendResponce(resArrayH,3);
            break;}
          case EXIOWRAN:
            {
            uint8_t pin = received_data[3];
            uint16_t value = (received_data[5] << 8) + received_data[4];
            uint8_t profile = received_data[6];
            uint16_t duration = (received_data[8] << 8) + received_data[7];
            bool response = writeAnalogue(pin, value, profile, duration);
            uint8_t resArrayH[ARRAY_SIZE];
            resArrayH[0] = (0);
            resArrayH[1] = (RS485_NODE);
            if (response) {
              resArrayH[2] = (EXIORDY);
            } else {
              resArrayH[2] = (EXIOERR);
            }
            SendResponce(resArrayH,3);
            break;}
      }

    } else {
      ///USB_SERIAL.println("Error: CRC mismatch!");
    }
  } else {
    //USB_SERIAL.print(RS485_SERIAL.available());
    //USB_SERIAL.print(":");
  }
}
