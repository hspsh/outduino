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

#define PWR_OFF 0
#define PWR_ON 1

#define OUT_LO 0
#define OUT_HI 1

// static int counter = 0;


class OutduinoBank{
  public:
    // OutduinoBank(OutduinoBank const&) = delete;
    // OutduinoBank& operator=(OutduinoBank const&) = delete;

    OutduinoBank(int pin_pwr, int pin_inp, int pin_out, int pin_curr, void(pinIsr)()){
      this->pin_pwr = pin_pwr;
      this->pin_inp = pin_inp;
      this->pin_out = pin_out;
      this->pin_curr = pin_curr;

      this->pinIsr = pinIsr;

      // ALOGV("OutduinoBank {} created", counter++);
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
      setPower(PWR_OFF);
      setOutput(OUT_LO);

      attachInterrupt(pin_inp, pinIsr, CHANGE);
    }

    void setPower(int state){
      if (state) {
        digitalWrite(pin_pwr, LOW);
      } else {
        digitalWrite(pin_pwr, HIGH);
      }
    }

    void setOutput(int state){
      if (state) {
        digitalWrite(pin_out, HIGH);
      } else {
        digitalWrite(pin_out, LOW);
      }
    }

    std::string to_string(){
      return fmt::format(
        "bank: {} pwr: {} inp: {} out: {} curr: {}",
        name, pin_pwr, pin_inp, pin_out, pin_curr);
    }

    std::string name;

    int pin_pwr;
    int pin_inp;
    int pin_out;
    int pin_curr;

    void (*pinIsr)();
};

InIsrHandle OutduinoBank::evtHandle;
