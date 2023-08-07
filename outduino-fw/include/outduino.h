#pragma once
#include <Arduino.h>

typedef std::function<void(const int, const int)> InIsrHandle;


#define ISR_FOR_PIN_BANK(_PIN, _BANK) [](){ \
  static long last_interrupt_time = 0; \
  long interrupt_time = millis(); \
  if (interrupt_time - last_interrupt_time > 100) { \
    last_interrupt_time = interrupt_time; \
    if (OutduinoBank::evtHandle) \
      OutduinoBank::evtHandle(_PIN, _BANK); \
  } \
  last_interrupt_time = interrupt_time; \
}

class OutduinoBank{
  public:

    OutduinoBank(int pin_pwr, int pin_inp, int pin_out, int pin_curr, void(pinIsr)()){
      this->pin_pwr = pin_pwr;
      this->pin_inp = pin_inp;
      this->pin_out = pin_out;
      this->pin_curr = pin_curr;

      this->pinIsr = pinIsr;
    } 

    static InIsrHandle evtHandle;
    
    static void setEvtHandle(InIsrHandle handle){
      evtHandle = handle;
    }

    void init(){
      pinMode(pin_pwr, OUTPUT);
      pinMode(pin_inp, INPUT);
      pinMode(pin_out, OUTPUT);
      pinMode(pin_curr, ANALOG);

      //turn off everything by default;
      digitalWrite(pin_pwr, HIGH);
      digitalWrite(pin_out, LOW);

      attachInterrupt(pin_inp, pinIsr, CHANGE);
    }

    int pin_pwr;
    int pin_inp;
    int pin_out;
    int pin_curr;

    void (*pinIsr)();
};

InIsrHandle OutduinoBank::evtHandle;
