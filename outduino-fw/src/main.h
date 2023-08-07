#pragma once

#include <string>
#include <array>

#include <cringed.h>
#include <outduino.h>

void printTimestampOnPress(int input_num, long unsigned int timestamp);
void outduino_log(const std::string& msg);
void sendIoEvt(const cringeEvtContainer_t *p_evt);
void initPins(std::vector<OutduinoBank> banks);