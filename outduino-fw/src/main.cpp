#include <Arduino.h>
#undef B1 //try to comment out this line to see something funny

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Button2.h>

#include <fmt/ranges.h>
#include <alfalog.h>
#include "advancedOledLogger.h"

#include <commonFwUtils.h>

#include <main.h>
#include <pinDefs.h>

#define FW_REV "0.9.9"

QueueHandle_t eventMsgQueue = NULL;
TimerHandle_t debounceTimer = NULL;

void SerialReceiveTask( void * parameter );
void EvtDigestTask( void * parameter );

TwoWire i2c = TwoWire(0);

std::vector<OutduinoBank> banks = {{
  OutduinoBank(PWR1, IN1, OUT1, CURR1, ISR_FOR_PIN_BANK(IN1, 1)),
  OutduinoBank(PWR2, IN2, OUT2, CURR2, ISR_FOR_PIN_BANK(IN2, 2)),
  OutduinoBank(PWR3, IN3, OUT3, CURR3, ISR_FOR_PIN_BANK(IN3, 3)),
  OutduinoBank(PWR4, IN4, OUT4, CURR4, ISR_FOR_PIN_BANK(IN4, 4))
}};

void uartPrintAlogHandle(const char* str){
  Serial.println(str);
}

void sendIoEvt(const cringeEvtContainer_t *p_evt){
  if(eventMsgQueue != NULL){
    xQueueSendFromISR(eventMsgQueue, p_evt, 0);
  }
}

void outduinoPinIsrHandle(const int pin, const int bank){
  cringeEvtContainer_t evt;
  evt.type = CRINGE_EVT_TYPE_INPUT;
  evt.inputEvt = {
    .input_num = bank,
    .is_high = digitalRead(pin) != LOW,
    .timestamp = millis()
  };
  sendIoEvt(&evt);
}

AdvancedOledLogger aOledLogger = AdvancedOledLogger(i2c, LOG_INFO, OLED_128x64);
SerialLogger serialLogger = SerialLogger(uartPrintAlogHandle, LOG_DEBUG);
Button2 userButton = Button2();

void setup() {  
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0); // prevent logger slowdown when no usb connected
  Serial.println("begin...");
  // delay(3000);
  // bootloopOnButtonPress(PIN_B2);
  i2c.begin(SDA, SCL, 100000);

  AlfaLogger.addBackend(&aOledLogger);
  AlfaLogger.addBackend(&serialLogger);
  AlfaLogger.begin();
  ALOGI("display started");
  aOledLogger.setTopBarText(BAR_CUSTOM, "NinjaTimer r. " FW_REV);

  // ALOGI(
  //   "i2c device found at 0x{:02x}", 
  //   fmt::join(scan_i2c(i2c), ", 0x"));
  // initPins(banks);

  for (auto &b: banks){
    b.setPower(PWR_ON);
  }

  userButton.begin(PIN_B2, INPUT_PULLUP, false);
  userButton.setTapHandler([](Button2 & b){
    cringeEvtContainer_t evt;
    evt.type = CRINGE_EVT_TYPE_INPUT;
    evt.inputEvt = {
      .input_num = PIN_B2,
      .is_high = digitalRead(PIN_B2) != LOW,
      .timestamp = millis()
    };
    sendIoEvt(&evt);
  });

  eventMsgQueue = xQueueCreate( 10, sizeof( cringeEvtContainer_t ) );  
  xTaskCreate( EvtDigestTask, "EvtDigestTask",
    3000, NULL, 2, NULL );

  xTaskCreate( SerialReceiveTask, "serial task",
    3000, NULL, 2, NULL );
}

#define OUTDUINO_ECHO_SERIAL
void parseSerialEvent(const std::vector<char> buf){
  if (buf.size() < 3){
    #ifdef OUTDUINO_ECHO_SERIAL
      Serial.println();
    #endif
  } else {
    if (buf.front() == '<' && buf.back() == '>'){
      std::string cmd(buf.begin()+1, buf.end()-1);

      cringeEvtContainer_t evt;
      evt.type = CRINGE_EVT_TYPE_SERIAL;
      strncpy(evt.serialEvt.cmd, cmd.c_str(), sizeof(evt.serialEvt.cmd));

      if(eventMsgQueue != NULL){
        xQueueSend(eventMsgQueue, &evt, 0);
      }
    }
  }
}

void SerialReceiveTask( void * parameter ) {
  std::vector<char> buf;
  while(1){
    while (Serial.available()) {
      char c = Serial.read();
      
      switch (c){
        case '\r': 
        #ifdef OUTDUINO_ECHO_SERIAL
          Serial.print('\r');  //echo if character accepted
        #endif
        break;
        case '\n': {
          parseSerialEvent(buf);
          buf.clear();
          break;
        }
        default: {
          if(buf.size() <= 32){
            buf.push_back(c);
            #ifdef OUTDUINO_ECHO_SERIAL
              Serial.print(c);  //echo if character accepted
            #endif
          }
        }
      }
    }
    vTaskDelay(100/portTICK_PERIOD_MS);
  }
}

void serialAction(cringeEvtContainer_t& evt){
  //frick off chatgpt wrote this
  if (strncmp(evt.serialEvt.cmd, "set", 3) == 0){
    int bank = atoi(evt.serialEvt.cmd+3);
    if (bank > 0 && bank <= banks.size()){
      banks[bank-1].setOutput(OUT_HI);
    }
  } else if (strncmp(evt.serialEvt.cmd, "clr", 3) == 0){
    int bank = atoi(evt.serialEvt.cmd+3);
    if (bank > 0 && bank <= banks.size()){
      banks[bank-1].setOutput(OUT_LO);
    }
  } else if (strncmp(evt.serialEvt.cmd, "on", 2) == 0){
    int bank = atoi(evt.serialEvt.cmd+2);
    if (bank > 0 && bank <= banks.size()){
      banks[bank-1].setPower(PWR_ON);
    }
  } else if (strncmp(evt.serialEvt.cmd, "off", 3) == 0){
    int bank = atoi(evt.serialEvt.cmd+3);
    if (bank > 0 && bank <= banks.size()){
      banks[bank-1].setPower(PWR_OFF);
    }
  } else if (strncmp(evt.serialEvt.cmd, "status", 6) == 0){
    for (auto &b: banks){
      ALOGR(b.to_string().c_str());
    }
  } else if (strncmp(evt.serialEvt.cmd, "help", 4) == 0){
    ALOGR("set1 - set bank 1");
    ALOGR("clr1 - clr bank 1");
    ALOGR("toggle1 - toggle bank 1");
    ALOGR("status - show status of all banks");
  } else if (strncmp(evt.serialEvt.cmd, "view", 4) == 0){
    aOledLogger.setTopBarText(BAR_CUSTOM, std::string(evt.serialEvt.cmd+4));
  } else if (strncmp(evt.serialEvt.cmd, "log", 3) == 0){
    ALOGI("{}", evt.serialEvt.cmd+3);
  }
}

void EvtDigestTask( void * parameter ) {
  cringeEvtContainer_t evt;
  while(1){
    if(xQueueReceive(eventMsgQueue, &evt, 1000/portTICK_PERIOD_MS) == pdTRUE){
      
      if(evt.type == CRINGE_EVT_TYPE_INPUT){
        Serial.println(evt.serialize().c_str());
      } else if (evt.type == CRINGE_EVT_TYPE_SERIAL){
        ALOGD(evt.to_string().c_str());
        serialAction(evt);
      }     

    } else {
      // ALOGD("time {}", millis());
    }
  }
}

void loop() {
    handle_io_pattern(OUT3,PATTERN_HBEAT);
    userButton.loop();
    vTaskDelay(50/portTICK_PERIOD_MS);

    static int counter = 0;
    if (counter++ % 8 == 0){
      aOledLogger.redraw();
    }
}

void initPins(std::vector<OutduinoBank> banks){
  OutduinoBank::setEvtHandle(outduinoPinIsrHandle);

  for(auto &b: banks){
    b.init();
  }

  pinMode(BOOT_B1, INPUT);
  pinMode(PIN_B3, INPUT_PULLUP);
}