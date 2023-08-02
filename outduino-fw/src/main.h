#pragma once

#include <string>
#include <array>

void serialTask( void * parameter );
void printTimestampOnPress(int input_num, long unsigned int timestamp);
void send_io_evt(int input_num);
void outduino_log(const std::string& msg);


typedef struct {
  int input_num;
  bool is_high;
  long unsigned int timestamp;
} outduino_evt_t;

typedef struct {
  int pin_pwr;
  int pin_inp;
  int pin_out;
  int pin_curr;
  void (*isr_callback)() ;
} outduino_bank_t;

void init_pins(std::array<outduino_bank_t, 4> banks);
