#include <Arduino.h>
#include <pinDefs.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


QueueHandle_t event_msg_queue = NULL;

void serialTask( void * parameter );
void printTimestampOnPress(int input_num, long unsigned int timestamp);

typedef struct {
  int input_num;
  long unsigned int timestamp;
} outduino_evt_t;


void setup() {

  Serial.begin(115200);
  Serial.println("aasi");

  //inilialize pins as input 
  pinMode(IN1, INPUT);
  pinMode(IN2, INPUT);
  pinMode(IN3, INPUT);
  pinMode(IN4, INPUT);

  pinMode(BOOT_B1, INPUT);
  pinMode(B2, INPUT);
  pinMode(B3, INPUT);

  //inilialize pins as output
  pinMode(OUT1, OUTPUT);
  pinMode(OUT2, OUTPUT);
  pinMode(OUT3, OUTPUT);
  pinMode(OUT4, OUTPUT);

  pinMode(PWR1, OUTPUT);
  pinMode(PWR2, OUTPUT);
  pinMode(PWR3, OUTPUT);
  pinMode(PWR4, OUTPUT);

  //initialize pins as analog input

  pinMode(CURR1, INPUT);
  pinMode(CURR2, INPUT);
  pinMode(CURR3, INPUT);
  pinMode(CURR4, INPUT);

  TwoWire i2c = TwoWire(0);
  Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &i2c);

  i2c.begin(SDA, SCL, 100000);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.print("hello world");
  display.display();



  event_msg_queue = xQueueCreate( 10, sizeof( outduino_evt_t ) );

  //attach interrupts to input pins
  attachInterrupt(digitalPinToInterrupt(IN1), []{
    outduino_evt_t evt = {
      .input_num = IN1,
      .timestamp = millis()
    };
    xQueueSendFromISR(event_msg_queue, &evt, NULL);
    }, RISING);

  attachInterrupt(digitalPinToInterrupt(IN2), []{
    outduino_evt_t evt = {
      .input_num = IN2,
      .timestamp = millis()
    };
    xQueueSendFromISR(event_msg_queue, &evt, NULL);
    }, RISING);

  attachInterrupt(digitalPinToInterrupt(IN3), []{
  outduino_evt_t evt = {
    .input_num = IN3,
    .timestamp = millis()
  };
  xQueueSendFromISR(event_msg_queue, &evt, NULL);
  }, RISING);

  attachInterrupt(digitalPinToInterrupt(IN4), []{
  outduino_evt_t evt = {
    .input_num = IN4,
    .timestamp = millis()
  };
  xQueueSendFromISR(event_msg_queue, &evt, NULL);
  }, RISING);

  digitalWrite(OUT1, HIGH);
  digitalWrite(OUT2, HIGH);

  xTaskCreate( serialTask, "serial task",
  3000, NULL, 2, NULL );
}

void loop() {
  // put your main code here, to run repeatedly:
    Serial.println("aasi");
    sleep(1);
}

void serialTask( void * parameter ) {
  outduino_evt_t evt;
  while(1){
    if(xQueueReceive(event_msg_queue, &evt, 1000/portTICK_PERIOD_MS) == pdTRUE){
      printTimestampOnPress(evt.input_num, evt.timestamp);
    } else {
      Serial.println("heartbeat");
    }
  }
}

void printTimestampOnPress(int input_num, long unsigned int timestamp){
  Serial.print("Input ");
  Serial.print(input_num);
  Serial.print(" pressed at ");
  Serial.println(timestamp);
}
