/*
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

#include <Arduino.h>
#include "globals.h"
#include "device_functions.h"

#if defined(ARDUINO_ARCH_AVR)
#include <avr/wdt.h>
#endif

void reset() {
#if defined(ARDUINO_ARCH_AVR)
  wdt_enable(WDTO_15MS);
  delay(50);
#elif defined(ARDUINO_ARCH_STM32)
  __disable_irq();
  NVIC_SystemReset();
  while(true) {};
#endif
}

/*
Disabling JTAG is required to avoid pin conflicts on Bluepill
*/
#if defined(ARDUINO_BLUEPILL_F103C8)
void disableJTAG() {
  // Disable JTAG and enable SWD by clearing the SWJ_CFG bits
  // Assuming the register is named AFIO_MAPR or AFIO_MAPR2
  AFIO->MAPR &= ~(AFIO_MAPR_SWJ_CFG);
  // or
  // AFIO->MAPR2 &= ~(AFIO_MAPR2_SWJ_CFG);
}
#endif
