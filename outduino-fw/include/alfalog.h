#pragma once

#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <vector>
#include <fmt/core.h>

// actually
// legitimate
// formatted
// arduino
// logging

#define ALOG(...) AlfaLogger.log(fmt::format(__VA_ARGS__));


class AlfaBackend {
    public:
    virtual ~AlfaBackend() {};
    virtual void log(const std::string& msg) = 0;
    virtual void begin() = 0;

    protected:
        bool _started = false;
};

class OledLogger : public AlfaBackend {
public:
OledLogger(TwoWire& i2c) {
    display = Adafruit_SSD1306(128, 32, &i2c);
}

void begin() {
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32
    display.invertDisplay(true);

    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.print("hello world");
    display.display();

    _started = true;
}

void log(const std::string& msg) {
    if (!_started) {
        return;
    }

    logs.insert(logs.begin(), msg);
    if (logs.size() > 4) {
        logs.pop_back();
    }

    display.clearDisplay();
    display.setCursor(0,0);
    for (auto& log : logs) {
        display.println(log.c_str());
    }
    display.display();
}

private:
    Adafruit_SSD1306 display;
    std::vector<std::string> logs;
};


class SerialLogger : public AlfaBackend {
public:

SerialLogger(Stream* ser_handle){
    this->_ser_handle = ser_handle;
}
void begin() {
    _started = true;
}

void log(const std::string& msg) {
    _ser_handle->println(msg.c_str());
}

private:
    Stream* _ser_handle;
};


class AlfaLogger_ {
    public:
    AlfaLogger_() = default;

    static AlfaLogger_ &getInstance(){
        static AlfaLogger_ instance;
        return instance;
    }

    public:
    void addBackend(AlfaBackend* backend) {
        _backends.push_back(backend);
    }

    void begin() {
        for (auto& backend : _backends) {
            backend->begin();
        }
    }

    void log(const std::string& msg) {
        for (auto& backend : _backends) {
            backend->log(msg);
        }
    }

    private:
        std::vector<AlfaBackend*> _backends;
};

AlfaLogger_ &AlfaLogger = AlfaLogger.getInstance();