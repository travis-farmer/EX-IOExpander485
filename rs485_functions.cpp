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
byte responseBuffer[1];   // Buffer to send single response back to device driver
uint8_t numReceivedPins = 0;

#define ARRAY_SIZE 250
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

void SendResponce(int *Buffer) {
      // Calculate CRC for response data
      uint16_t response_crc = crc16((uint8_t*)Buffer, ARRAY_SIZE * sizeof(int));
      digitalWrite(RS485_DEPIN,HIGH);
      // Send response data with CRC
      for (int i = 0; i < ARRAY_SIZE; i++) {
        RS485_SERIAL.write(Buffer[i]);
      }
      RS485_SERIAL.write(response_crc >> 8);
      RS485_SERIAL.write(response_crc & 0xFF);
      digitalWrite(RS485_DEPIN,LOW);
}

void serialLoopRx() {
  // Check if data is available
  //if (RS485_SERIAL.available() >= ARRAY_SIZE) {
    int received_data[ARRAY_SIZE];

    // Read data and CRC
    for (int i = 0; i < ARRAY_SIZE; i++) {
      received_data[i] = RS485_SERIAL.read();
    }
    uint16_t received_crc = (RS485_SERIAL.read() << 8) | RS485_SERIAL.read();

    // Calculate CRC for received data
    uint16_t calculated_crc = crc16((uint8_t*)received_data, ARRAY_SIZE);

    // Check CRC validity
    if (calculated_crc == received_crc) {
      // Data received successfully, process it 
      USB_SERIAL.println("CRC pass");
      int nodeTo = (received_data[1] << 8) | received_data[0];
      //if (nodeTo != RS485_NODE) {
        // retransmit and exit, if not ours
        //SendResponce(received_data);
       // return;
      //}
      int nodeFr = (received_data[3] << 8) | received_data[2];
      int AddrCode = (received_data[5] << 8) | received_data[4];
      
      switch (AddrCode) {
        case EXIOINIT:
          {initialisePins();
          numReceivedPins = received_data[6];
          firstVpin = (received_data[8] << 8) + received_data[7];
          if (numReceivedPins == numPins) {
            displayEventFlag = 0;
            setupComplete = true;
          } else {
            displayEventFlag = 1;
            setupComplete = false;
          }
          int resArrayA[ARRAY_SIZE];
          resArrayA[0] = lowByte(0);
          resArrayA[1] = highByte(0);
          resArrayA[2] = lowByte(RS485_NODE);
          resArrayA[3] = highByte(RS485_NODE);
          resArrayA[4] = lowByte(EXIOPINS);
          resArrayA[5] = highByte(EXIOPINS);
          resArrayA[6] = lowByte(numDigitalPins);
          resArrayA[7] = highByte(numDigitalPins);
          resArrayA[6] = lowByte(numAnaloguePins);
          resArrayA[7] = highByte(numAnaloguePins);
          SendResponce(resArrayA);
          break;}
        case EXIOINITA:
          {
          int resArrayB[ARRAY_SIZE];
          resArrayB[0] = lowByte(0);
          resArrayB[1] = highByte(0);
          resArrayB[2] = lowByte(RS485_NODE);
          resArrayB[3] = highByte(RS485_NODE);
          resArrayB[4] = lowByte(EXIOINITA);
          resArrayB[5] = highByte(EXIOINITA);
          for (int j = 0; j < numAnaloguePins; j=j+2) {
            resArrayB[j+5] = lowByte(analoguePinMap[j]);
            resArrayB[j+6] = highByte(analoguePinMap[j+1]);
          }
          SendResponce(resArrayB);
          break;}
        // Flag to set digital pin pullups, 0 disabled, 1 enabled
        case EXIODPUP:
          {
            uint8_t pin = (received_data[7] << 8) + received_data[6];;
            bool pullup = (received_data[9] << 8) + received_data[8];;
            bool response = enableDigitalInput(pin, pullup);
            int resArrayC[ARRAY_SIZE];
            resArrayC[0] = lowByte(0);
            resArrayC[1] = highByte(0);
            resArrayC[2] = lowByte(RS485_NODE);
            resArrayC[3] = highByte(RS485_NODE);
            if (response) {
              resArrayC[4] = lowByte(EXIORDY);
              resArrayC[5] = highByte(EXIORDY);
            } else {
              resArrayC[4] = lowByte(EXIOERR);
              resArrayC[5] = highByte(EXIOERR);
            }
          SendResponce(resArrayC);
          break;}
          case EXIORDAN:
            {
            int resArrayD[ARRAY_SIZE];
            resArrayD[0] = lowByte(0);
            resArrayD[1] = highByte(0);
            resArrayD[2] = lowByte(RS485_NODE);
            resArrayD[3] = highByte(RS485_NODE);
            resArrayD[4] = lowByte(EXIORDAN);
            resArrayD[5] = highByte(EXIORDAN);
            for (int j = 0; j < analoguePinBytes; j=j+2) {
              resArrayD[j+5] = lowByte(analoguePinStates[j]);
              resArrayD[j+6] = highByte(analoguePinStates[j+1]);
            }
            SendResponce(resArrayD);
            break;}
          case EXIOWRD:
            {
            uint8_t pin = (received_data[7] << 8) + received_data[6];
            bool state = (((received_data[9] << 8) + received_data[8]) == 1)? true:false;
            bool response = writeDigitalOutput(pin, state);
            int resArrayE[ARRAY_SIZE];
            resArrayE[0] = lowByte(0);
            resArrayE[1] = highByte(0);
            resArrayE[2] = lowByte(RS485_NODE);
            resArrayE[3] = highByte(RS485_NODE);
            if (response) {
              resArrayE[4] = lowByte(EXIORDY);
              resArrayE[5] = highByte(EXIORDY);
            } else {
              resArrayE[4] = lowByte(EXIOERR);
              resArrayE[5] = highByte(EXIOERR);
            }
            SendResponce(resArrayE);
            break;}
          case EXIORDD:
            {
            int resArrayF[ARRAY_SIZE];
            resArrayF[0] = lowByte(0);
            resArrayF[1] = highByte(0);
            resArrayF[2] = lowByte(RS485_NODE);
            resArrayF[3] = highByte(RS485_NODE);
            resArrayF[4] = lowByte(EXIORDD);
            resArrayF[5] = highByte(EXIORDD);
            for (int j = 0; j < digitalPinBytes; j=j+2) {
              resArrayF[j+5] = lowByte(digitalPinStates[j]);
              resArrayF[j+6] = highByte(digitalPinStates[j+1]);
            }
            SendResponce(resArrayF);
            break;}
          case EXIOVER:
            {
            int resArrayG[ARRAY_SIZE];
            resArrayG[0] = lowByte(0);
            resArrayG[1] = highByte(0);
            resArrayG[2] = lowByte(RS485_NODE);
            resArrayG[3] = highByte(RS485_NODE);
            resArrayG[4] = lowByte(EXIOVER);
            resArrayG[5] = highByte(EXIOVER);
            resArrayG[6] = lowByte(versionBuffer[0]);
            resArrayG[7] = highByte(versionBuffer[0]);
            resArrayG[8] = lowByte(versionBuffer[1]);
            resArrayG[9] = highByte(versionBuffer[1]);
            resArrayG[10] = lowByte(versionBuffer[2]);
            resArrayG[11] = highByte(versionBuffer[2]);
            SendResponce(resArrayG);
            break;}
          case EXIOENAN:
            {
            uint8_t pin = (received_data[7] << 8) + received_data[6];
            bool response = enableAnalogue(pin);
            int resArrayH[ARRAY_SIZE];
            resArrayH[0] = lowByte(0);
            resArrayH[1] = highByte(0);
            resArrayH[2] = lowByte(RS485_NODE);
            resArrayH[3] = highByte(RS485_NODE);
            if (response) {
              resArrayH[4] = lowByte(EXIORDY);
              resArrayH[5] = highByte(EXIORDY);
            } else {
              resArrayH[4] = lowByte(EXIOERR);
              resArrayH[5] = highByte(EXIOERR);
            }
            SendResponce(resArrayH);
            break;}
          case EXIOWRAN:
            {
            uint8_t pin = (received_data[7] << 8) + received_data[6];
            uint16_t value = (received_data[7] << 8) + received_data[6];
            uint8_t profile = (received_data[7] << 8) + received_data[6];
            uint16_t duration = (received_data[7] << 8) + received_data[6];
            bool response = writeAnalogue(pin, value, profile, duration);
            int resArrayH[ARRAY_SIZE];
            resArrayH[0] = lowByte(0);
            resArrayH[1] = highByte(0);
            resArrayH[2] = lowByte(RS485_NODE);
            resArrayH[3] = highByte(RS485_NODE);
            if (response) {
              resArrayH[4] = lowByte(EXIORDY);
              resArrayH[5] = highByte(EXIORDY);
            } else {
              resArrayH[4] = lowByte(EXIOERR);
              resArrayH[5] = highByte(EXIOERR);
            }
            SendResponce(resArrayH);
            break;}
      }

    } else {
      //USB_SERIAL.println("Error: CRC mismatch!");
    }
  //}else {
    //USB_SERIAL.print(RS485_SERIAL.available());
    //USB_SERIAL.print(":");
  //}
}
