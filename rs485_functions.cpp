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
char tmpBuffer[COMMAND_BUFFER_SIZE]; 
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
int bufferLength;
byte inCommandPayload = PAYLOAD_FALSE;
/*
* Function triggered when CommandStation is sending data to this device.
*/

int getCharsRightOfPosition(char* str, char position, char* outStr, int size) {
  //if (position >= 0 && position < strlen(str)) {
    int pos;
    char retStr[size];
    for (int i = 0; str[i] != '\0'; i++) {
      if (str[i] == position) {
        pos = i;
        break; // Exit the loop once the character is found
      }
    }
    int i;
    for (i = 0; i < size; i++) {
      retStr[i] = str[i+pos+1];
    }
    memcpy(outStr, retStr, i+pos+1);
    return i+pos+2;
  //}
}

void getCharsLeft(char *str, char position, char *result) {
  int pos;
  for (int i = 0; str[i] != '\0'; i++) {
    if (str[i] == position) {
      pos = i;
      break; // Exit the loop once the character is found
    }
  }
  if (pos >= 0 && pos < strlen(str)) {
    for (int i = 0; i < strlen(str); i++) {
      if (i < pos) result[i] = str[i];
    }
  }
}

int getValues(char *buf, int lenBuf, int fieldCnt, int *outArray) {
  char curBuff[25];
  memset(curBuff, '\0', 25);
  int byteCntr = 0;
  int counter = 0;
  for (int i = 0; i< lenBuf; i++) {

    if (buf[i] == ' ' || buf[i] == '\0'){
      //outArray[counter] = 0x00;
      outArray[byteCntr] = atoi(curBuff);
      byteCntr++;
      memset(curBuff, '\0', sizeof(curBuff));
      counter = 0;
    } else {
      curBuff[counter] = buf[i];
      counter++;
    }
    if (byteCntr > fieldCnt || buf[i] == '\0') {
      break;
    }
  }
  return byteCntr;
}

void procRX(char * rxbuffer, int rxBufLen, int fieldCnt) {
  int outValues[fieldCnt];
  memset(outValues, 0, sizeof(outValues));
  int realCnt = getValues(rxbuffer, rxBufLen, fieldCnt, outValues);
  if (outValues[0] != RS485_NODE) return;
  switch(outValues[2]) {
    // Initial configuration start, must be 2 bytes
    case EXIOINIT:
        {initialisePins();
        numReceivedPins = outValues[3];
        firstVpin = (outValues[5] << 8) + outValues[4];
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
        uint8_t pin = outValues[3];
        bool pullup = outValues[4];
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
        uint8_t pin = outValues[3];
        bool state = outValues[4];
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
        uint8_t pin = outValues[3];
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
        uint8_t pin = outValues[3];
        uint16_t value = outValues[4];
        uint8_t profile = outValues[5];
        uint16_t duration = outValues[6];
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

void receiveEvent() {
  //unsigned long startMicros = micros();
  char ch;
  while (RS485_SERIAL.available()) {
    ch = RS485_SERIAL.read();
    //RS485_SERIAL.write(ch,1); // send round the ring in case not ours
    if (!inCommandPayload) {
      if (ch == '<') {
        inCommandPayload = PAYLOAD_NORMAL;
        bufferLength = 0;
        //tmpBuffer[0] = '\0';
      }
    } else { // if (inCommandPayload)
      if (inCommandPayload == PAYLOAD_NORMAL) {
        if (ch == '>') {
          //tmpBuffer[bufferLength] = '\0';
          
          char chrSize[3];
          getCharsLeft(tmpBuffer,'|', chrSize);
          char chrSend[200];
          int remainder = getCharsRightOfPosition(tmpBuffer,'|',chrSend, sizeof(tmpBuffer));
          procRX(chrSend, remainder, atoi(chrSize));
          memset(tmpBuffer, '\0', sizeof(tmpBuffer));
          bufferLength = 0;
          inCommandPayload = PAYLOAD_FALSE;
        
        }
      }
      if (bufferLength <  (COMMAND_BUFFER_SIZE-1) && inCommandPayload == PAYLOAD_NORMAL) {
        tmpBuffer[bufferLength] = ch;
        bufferLength++;
      }
    }
  }
}  

/*
* Function triggered when CommandStation polls for inputs on this device.
*/
void requestEvent() {
  
  switch(outboundFlag) {
    case EXIOINIT:
      {
      char buff[200];
      memset(buff, '\0', 200);
      sprintf(buff, "<5|%i %i %i %i %i>", 0, RS485_NODE, EXIOPINS, numDigitalPins, numAnaloguePins);
      digitalWrite(RS485_DEPIN, HIGH);
      USB_SERIAL.print(buff);
      RS485_SERIAL.println(buff);
      //RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;} 
    case EXIOINITA: {
      char buff[500];
      memset(buff, '\0', 400);
      int cntr = 0;
      char tmpStr[4];
      char tmpTotal[400];
      for (int j = 0; j < numAnaloguePins; j++) {
        sprintf(tmpStr, "%i ", analoguePinMap[j]);
        strcat(tmpTotal, tmpStr);
        memset(tmpStr, '\0', 4);
        cntr++;
      }
      
      sprintf(buff, "<%i|%i %i %i %s>", cntr+3, 0, RS485_NODE, EXIOINITA, tmpTotal);
      memset(tmpTotal, '\0', 400);
      USB_SERIAL.println(buff);
      digitalWrite(RS485_DEPIN, HIGH);
      RS485_SERIAL.println(buff);
      //RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;}
    case EXIORDAN: {
      char buff[500];
      memset(buff, '\0', 400);
      int cntr = 0;
      char tmpStr[4];
      char tmpTotalB[400];
      for (int j = 0; j < analoguePinBytes; j++) {
        sprintf(tmpStr, "%i ", analoguePinStates[j]);
        strcat(tmpTotalB, tmpStr);
        memset(tmpStr, '\0', 4);
        cntr++;
      }
      sprintf(buff, "<%i|%i %i %i %s>", cntr+3, 0, RS485_NODE, EXIORDAN, tmpTotalB);
      digitalWrite(RS485_DEPIN, HIGH);
      USB_SERIAL.print(buff);
      RS485_SERIAL.println(buff);
      //RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;}
    case EXIORDD:
      {
      char buff[500];
      memset(buff, '\0', 400);
      int cntr = 0;
      char tmpStr[4];
      char tmpTotalB[400];
      for (int j = 0; j < digitalPinBytes; j++) {
        sprintf(tmpStr, "%i ", digitalPinStates[j]);
        strcat(tmpTotalB, tmpStr);
        memset(tmpStr, '\0', 4);
        cntr++;
      }
      sprintf(buff, "<%i|%i %i %i %s>", cntr+3, 0, RS485_NODE, EXIORDD, tmpTotalB);
      digitalWrite(RS485_DEPIN, HIGH);
      USB_SERIAL.print(buff);
      RS485_SERIAL.println(buff);
      //RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;}
    case EXIOVER: {
      char buff[200];
      memset(buff, '\0', 200);
      sprintf(buff, "<6|%i %i %i %i %i %i>", 0, RS485_NODE, EXIOVER, versionBuffer[0], versionBuffer[1], versionBuffer[2]);
      digitalWrite(RS485_DEPIN, HIGH);
      USB_SERIAL.print(buff);
      RS485_SERIAL.println(buff);
      //RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;}
    case EXIODPUP:
      {char buff[200];
      memset(buff, '\0', 200);
      sprintf(buff, "<3|%i %i %i>", 0, RS485_NODE, responseBuffer);
      digitalWrite(RS485_DEPIN, HIGH);
      USB_SERIAL.print(buff);
      RS485_SERIAL.println(buff);
      //RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;}
    case EXIOENAN:
      {char buff[200];
      memset(buff, '\0', 200);
      sprintf(buff, "<3|%i %i %i>", 0, RS485_NODE, responseBuffer);
      digitalWrite(RS485_DEPIN, HIGH);
      USB_SERIAL.print(buff);
      RS485_SERIAL.println(buff);
      //RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;}
    case EXIOWRAN:
      {char buff[200];
      memset(buff, '\0', 200);
      sprintf(buff, "<3|%i %i %i>", 0, RS485_NODE, responseBuffer);
      digitalWrite(RS485_DEPIN, HIGH);
      USB_SERIAL.print(buff);
      RS485_SERIAL.println(buff);
      //RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;}
    case EXIOWRD:
      {char buff[200];
      memset(buff, '\0', 200);
      sprintf(buff, "<3|%i %i %i>", 0, RS485_NODE, responseBuffer);
      digitalWrite(RS485_DEPIN, HIGH);
      USB_SERIAL.print(buff);
      RS485_SERIAL.println(buff);
      //RS485_SERIAL.flush();
      digitalWrite(RS485_DEPIN, LOW);
      break;}
  }
  outboundFlag = 0;
}

