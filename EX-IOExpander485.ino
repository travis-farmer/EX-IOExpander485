/*
 *  © 2022, Peter Cole. All rights reserved.
 *  © 2023, Peter Cole. All rights reserved.
 *  © 2024, Travis Farmer. All rights reserved.
 * 
 *  This file is part of EX-IOExpander485.
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

/*
* EX-IOExpander is a native DCC-EX I/O expander utilising a microcontroller's digital and analogue I/O pins
* to expand the I/O capability of an EX-CommandStation.
*
* All digital I/O pins are available as digital inputs or outputs.
* Analogue I/O pins are available as digital inputs or outputs or analogue inputs (depending on architecture).
*/

/*
* Include required files and libraries.
*/
#include <Arduino.h>
#include "globals.h"
#include "version.h"
#include <Wire.h>
#include "pin_io_functions.h"
#include "display_functions.h"
#include "rs485_functions.h"
#include "serial_functions.h"
#include "test_functions.h"
#include "device_functions.h"
#include "servo_functions.h"

#ifdef CPU_TYPE_ERROR
#error Unsupported microcontroller architecture detected, you need to use a supported microcontroller. Refer to the documentation.
#endif

/*
* Include our CPU specific file
*/
#if defined(ARDUINO_AVR_NANO) || defined(ARDUINO_AVR_PRO)
#include "arduino_avr_nano.h"
#elif defined(ARDUINO_AVR_UNO)
#include "arduino_avr_uno.h"
#elif defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_AVR_MEGA)
#include "arduino_avr_mega.h"
#elif defined(ARDUINO_NUCLEO_F411RE)
#include "arduino_nucleo_f411re.h"
#elif defined(ARDUINO_NUCLEO_F412ZG)
#include "arduino_nucleo_f412zg.h"
#elif defined(ARDUINO_ARCH_SAMD)
#include "arduino_arch_samd.h"
#elif defined(ARDUINO_BLUEPILL_F103C8)
#include "arduino_bluepill_f103c8.h"
#endif

/*
* Global variables here
*/
uint8_t numPins = TOTAL_PINS;
uint8_t* analoguePinMap;  // Map which analogue pin's value is in which byte
bool outputTestState = LOW;   // Flag to set outputs high or low for testing

#ifdef DIAG
  bool diag = true;
#else
  bool diag = false;
#endif


/*
* Main setup function here.
*/
void setup() {
#if defined(ARDUINO_BLUEPILL_F103C8)
  disableJTAG();
#endif
  pinMode(RS485_DEPIN,OUTPUT);
  digitalWrite(RS485_DEPIN,LOW);
  RS485_SERIAL.begin(115200, SERIAL_8N1);

#if defined(USB_SERIAL)
  USB_SERIAL.begin(115200);
  USB_SERIAL.print(F("DCC-EX EX-IOExpander485 v"));
  USB_SERIAL.println(VERSION);
  USB_SERIAL.print(F("Detected device: "));
  USB_SERIAL.println(BOARD_TYPE);
  USB_SERIAL.print(F("Available at RS485 node "));
  USB_SERIAL.println(RS485_NODE);
#endif
  setVersion();
  setupPinDetails();
  servoDataArray = (ServoData**) calloc(numPins, sizeof(ServoData*));
  startupDisplay();
// Need to intialise every pin in INPUT mode (no pull ups) for safe start
  initialisePins();
#if defined(USB_SERIAL)
  USB_SERIAL.println(F("Initialised all pins as input only"));
#endif
#if (TEST_MODE == ANALOGUE_TEST)
  testAnalogue(true);
#elif (TEST_MODE == INPUT_TEST)
  testInput(true);
#elif (TEST_MODE == OUTPUT_TEST)
  testOutput(true);
#elif (TEST_MODE == PULLUP_TEST)
  testPullup(true);
#endif
  uint8_t analoguePin = 0;
  for (uint8_t pin = 0; pin < numPins; pin++) {
    if (bitRead(pinMap[pin].capability, ANALOGUE_INPUT)) {
      exioPins[pin].analogueLSBByte = analoguePin * 2;
      analoguePinMap[analoguePin] = pin;
      analoguePin++;
    }            
  }

}

/*
* Main loop here, just processes our inputs and updates the writeBuffer.
*/
void loop() {
  if (RS485_SERIAL.available()) {
    //USB_SERIAL.println("rx serial");
    serialLoopRx();
  }
  if (setupComplete) {
    
    processInputs();
    outputTestState = processOutputTest(outputTestState);
    processServos();
    SuperPin::loop();
  }
  if (diag) {
    displayPins();
  }
#if defined(USB_SERIAL)
  processSerialInput();
#endif
  processDisplayOutput();
}
