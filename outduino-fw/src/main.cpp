#include <Arduino.h>
#include <pinDefs.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <commonFwUtils.h>

#include <alfalog.h>

QueueHandle_t event_msg_queue = NULL;

TwoWire i2c = TwoWire(0);
OledLogger display = OledLogger(i2c);
SerialLogger serialLogger = SerialLogger(&Serial);

void serialTask( void * parameter );
void printTimestampOnPress(int input_num, long unsigned int timestamp);
void send_io_evt(int input_num);
void outduino_log(const std::string& msg);
void init_pins();

void bootloop_on_button_press(int pin_num){
  if (digitalRead(pin_num)==LOW){
    Serial.println("enter bootloop");
    while (1) {
      Serial.print(".");
      delay(1000);
    }
  }
}

typedef struct {
  int input_num;
  long unsigned int timestamp;
} outduino_evt_t;


void setup() {
  init_pins();

  Serial.begin(115200);
  Serial.setTxTimeoutMs(0); 
  Serial.println("begin...");
  bootloop_on_button_press(PIN_B2);

  i2c.begin(SDA, SCL, 100000);

  AlfaLogger.addBackend(&display);
  AlfaLogger.addBackend(&serialLogger);
  AlfaLogger.begin();

  ALOG("display started");

  event_msg_queue = xQueueCreate( 10, sizeof( outduino_evt_t ) );  
  xTaskCreate( serialTask, "serial task",
  3000, NULL, 2, NULL );
}

void send_io_evt(int input_num){
  outduino_evt_t evt = {
    .input_num = input_num,
    .timestamp = millis()
  };
  xQueueSendFromISR(event_msg_queue, &evt, NULL);
}


void serialTask( void * parameter ) {
  outduino_evt_t evt;
  while(1){
    if(xQueueReceive(event_msg_queue, &evt, 1000/portTICK_PERIOD_MS) == pdTRUE){
      ALOG("Input {} pressed at {}ms", evt.input_num, evt.timestamp);
    } else {
      ALOG("time {}", millis());
    }
  }
}


void loop() {
    handle_io_pattern(PWR1,PATTERN_HBEAT);
    usleep(100000);
}

void init_pins(){
  //inilialize pins as input 
  pinMode(IN1, INPUT);
  pinMode(IN2, INPUT);
  pinMode(IN3, INPUT);
  pinMode(IN4, INPUT);

  pinMode(BOOT_B1, INPUT);
  pinMode(PIN_B2, INPUT_PULLUP);
  pinMode(PIN_B3, INPUT_PULLUP);

  //inilialize pins as output
  pinMode(OUT1, OUTPUT);
  pinMode(OUT2, OUTPUT);
  pinMode(OUT3, OUTPUT);
  pinMode(OUT4, OUTPUT);

  pinMode(PWR1, OUTPUT);
  pinMode(PWR2, OUTPUT);
  pinMode(PWR3, OUTPUT);
  pinMode(PWR4, OUTPUT);

  digitalWrite(PWR1, HIGH);
  digitalWrite(PWR2, HIGH);
  digitalWrite(PWR3, HIGH);
  digitalWrite(PWR4, HIGH);

  digitalWrite(OUT1, HIGH);
  digitalWrite(OUT2, HIGH);
  digitalWrite(OUT3, HIGH);
  digitalWrite(OUT4, HIGH);  

  //initialize pins as analog input
  pinMode(CURR1, INPUT);
  pinMode(CURR2, INPUT);
  pinMode(CURR3, INPUT);
  pinMode(CURR4, INPUT);

  // attach interrupts to input pins
  attachInterrupt(IN1,[]{send_io_evt(IN1);}, RISING);
  attachInterrupt(IN2,[]{send_io_evt(IN2);}, RISING);
  attachInterrupt(IN3,[]{send_io_evt(IN3);}, RISING);
  attachInterrupt(IN4,[]{send_io_evt(IN4);}, RISING);
}