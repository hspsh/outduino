#include <Arduino.h>
#include <pinDefs.h>

#undef B1

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <commonFwUtils.h>

#include <alfalog.h>

#include <main.h>

QueueHandle_t event_msg_queue = NULL;

TwoWire i2c = TwoWire(0);
OledLogger display = OledLogger(i2c, OLED_128x32, LOG_INFO);
SerialLogger serialLogger = SerialLogger(&Serial, LOG_DEBUG);


void bootloop_on_button_press(int pin_num){
  if (digitalRead(pin_num)==LOW){
    Serial.println("enter bootloop");
    while (1) {
      Serial.print(".");
      delay(1000);
    }
  }
}

// I hate c++
// I'll move to a better library at some point
void outduino_cb_1(){ send_io_evt(IN1);}
void outduino_cb_2(){ send_io_evt(IN2);}
void outduino_cb_3(){ send_io_evt(IN3);}
void outduino_cb_4(){ send_io_evt(IN4);}
void outduino_cb_B2(){ send_io_evt(PIN_B2);}

const std::array<outduino_bank_t, 4> banks = {{
  {PWR1, IN1, OUT1, CURR1, &outduino_cb_1},
  {PWR2, IN2, OUT2, CURR2, &outduino_cb_2},
  {PWR3, IN3, OUT3, CURR3, &outduino_cb_3},
  {PWR4, IN4, OUT4, CURR4, &outduino_cb_4}
}};

void setup() {
  init_pins(banks);

  Serial.begin(115200);
  Serial.setTxTimeoutMs(0); // prevent logger slowdown when no usb connected
  Serial.println("begin...");
  // bootloop_on_button_press(PIN_B2);

  i2c.begin(SDA, SCL, 100000);

  AlfaLogger.addBackend(&display);
  AlfaLogger.addBackend(&serialLogger);
  AlfaLogger.begin();

  ALOGI("display started");

  for (auto &b: banks){
    digitalWrite(b.pin_pwr, HIGH);
  }

  event_msg_queue = xQueueCreate( 10, sizeof( outduino_evt_t ) );  
  xTaskCreate( serialTask, "serial task",
  3000, NULL, 2, NULL );
}

void send_io_evt(int input_num){
  static long int last_evt = 0;
  outduino_evt_t evt = {
    .input_num = input_num,
    .is_high = digitalRead(input_num),
    .timestamp = millis()
  };
  if ((last_evt+100) > millis()){
    return;
  }
  last_evt = millis();
  
  if(event_msg_queue == NULL){
    return;
  }
  xQueueSendFromISR(event_msg_queue, &evt, NULL);
}


void serialTask( void * parameter ) {
  outduino_evt_t evt;
  while(1){
    if(xQueueReceive(event_msg_queue, &evt, 1000/portTICK_PERIOD_MS) == pdTRUE){
      ALOGI(
        "Input {} {} at {}ms", 
        evt.input_num,
        evt.is_high?"pressed":"released",
        evt.timestamp
      );
      Serial.println(
        fmt::format(
          "<I{}{} T{}>",
          evt.input_num, evt.is_high?"H":"L", evt.timestamp)
        .c_str());
    } else {
      ALOGD("time {}", millis());
    }
  }
}


void loop() {
    handle_io_pattern(OUT3,PATTERN_HBEAT);
    usleep(100000);
}

void init_pins(std::array<outduino_bank_t, 4> banks){
  //inilialize pins as input 

  for(int i=0; i<banks.size(); i++){
    pinMode(banks[i].pin_pwr, OUTPUT);
    pinMode(banks[i].pin_inp, INPUT);
    pinMode(banks[i].pin_out, OUTPUT);
    pinMode(banks[i].pin_curr, ANALOG);

    digitalWrite(banks[i].pin_pwr, LOW); //turn off by default
    digitalWrite(banks[i].pin_out, HIGH); //turn off by default

    attachInterrupt(
      banks[i].pin_inp,
      banks[i].isr_callback,
      CHANGE);
  }

  pinMode(BOOT_B1, INPUT);
  pinMode(PIN_B2, INPUT_PULLUP);
  pinMode(PIN_B3, INPUT_PULLUP);

  attachInterrupt(PIN_B2, outduino_cb_B2, CHANGE);
}