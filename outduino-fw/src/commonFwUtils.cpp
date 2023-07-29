#include "commonFwUtils.h"
#include <ArduinoOTA.h>


void handle_io_pattern(uint8_t pin, led_patterns_type_t target_pattern){
  static uint32_t pattern_counter = 0;
  static uint8_t heartbeat_pattern[] = {1,0,0,1,0,0,0,0,0,0,0,0,0};
  static uint8_t errcon_pattern[] = {1,0,1,0,1,0,1,1,1,0,0,0,0};
  
  switch (target_pattern){    
    case PATTERN_HBEAT:
      digitalWrite(pin, heartbeat_pattern[
        pattern_counter % sizeof(heartbeat_pattern)
      ]);
      break;
    
    case PATTERN_ERR:
      digitalWrite(pin, errcon_pattern[
        pattern_counter % sizeof(errcon_pattern)
      ]);
      break;
    case PATTERN_NONE:
    default:
      digitalWrite(pin,0);
      break;
  }
  pattern_counter++;
}
