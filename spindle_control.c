/*
  spindle_control.c - spindle control methods
  Part of Grbl

  Copyright (c) 2012-2014 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "system.h"
#include "spindle_control.h"
#include "protocol.h"
#include <avr/interrupt.h>

#ifdef VARIABLE_SPINDLE
  static uint8_t current_direction;
  static uint16_t current_rpm;
#endif

void spindle_init() {
  current_direction = 0;
  
  #ifdef VARIABLE_SPINDLE
    SPINDLE_PWM_DDR |= (1<<SPINDLE_PWM_BIT); // Set spindle pwm pin as output
    TCCRA_REGISTER = _BV(WGM11);
    TCCRB_REGISTER = _BV(WGM13)
                   | _BV(WGM12)
                   | TIMER1_PRESCALE; 
    SPINDLE_OCR_REGISTER = 0; // spindle speed 0
    ICR1 = TIMER1_TOP; 
  #else
    SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT);
    SPINDLE_DIRECTION_DDR |= (1<<SPINDLE_DIRECTION_BIT);
  #endif 
  
  spindle_stop();
}


void spindle_stop() {
#ifdef VARIABLE_SPINDLE
  SPINDLE_OCR_REGISTER = 0;
  TCCR1A &= ~(TIMER1_OUTPUT_MODE);
#else 
  #ifdef INVERT_SPINDLE
  SPINDLE_ENABLE_PORT |= (1 << SPINDLE_ENABLE_BIT);
  #else
  SPINDLE_ENABLE_PORT &= ~(1 << SPINDLE_ENABLE_BIT);
  #endif
#endif
}



void spindle_run(uint8_t direction, float rpm) 
{
  // Empty planner buffer to ensure spindle is set when programmed.
  protocol_buffer_synchronize(); 

  // Halt or set spindle direction and rpm. 
  if (direction == SPINDLE_DISABLE) {
    spindle_stop();
  } else {

    #ifndef CPU_MAP_ARDUINO_UNO_WITH_MR_BEAM_SHIELD
      if (direction == SPINDLE_ENABLE_CW) {
        SPINDLE_DIRECTION_PORT &= ~(1<<SPINDLE_DIRECTION_BIT);
      } else {
        SPINDLE_DIRECTION_PORT |= (1<<SPINDLE_DIRECTION_BIT);
      }
    #endif
    
    #ifdef VARIABLE_SPINDLE
      #define SPINDLE_RPM_RANGE (SPINDLE_MAX_RPM-SPINDLE_MIN_RPM)
      TCCRA_REGISTER |= TIMER1_OUTPUT_MODE; // restart timer1

      rpm = min(rpm, SPINDLE_MAX_RPM);
      if(rpm > 0){
        uint16_t current_pwm = floor((((float) rpm / (float) SPINDLE_MAX_RPM) * TIMER1_TOP) + 0.5); // timer1 has 16 bit
        SPINDLE_OCR_REGISTER = current_pwm;
      } else {
        // TODO use SPINDLE_MIN_RPM here
        spindle_stop();
      }
      
      #ifndef CPU_MAP_ATMEGA328P // On the Uno, spindle enable and PWM are shared.
      #ifndef CPU_MAP_ARDUINO_UNO_WITH_MR_BEAM_SHIELD // same on MrBeamShield
        SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
      #endif
      #endif
    #else   
      SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
    #endif

  }
}
