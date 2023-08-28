#pragma once

#include <fmt/core.h>

typedef enum {
  CRINGE_EVT_TYPE_INPUT,
  CRINGE_EVT_TYPE_SERIAL,
} cringeEvtType_t;


typedef struct {
  int input_num;
  bool is_high;
  long unsigned int timestamp;
} CringeInputEvt_t;

typedef struct {
  char cmd[32];
} CringeSerialEvt_t;

typedef struct {
  cringeEvtType_t type;
  union {
    CringeInputEvt_t inputEvt;
    CringeSerialEvt_t serialEvt;
  };

  std::string to_string() const{
    switch (this->type){
      case CRINGE_EVT_TYPE_INPUT:
        return fmt::format("input {} is {}", this->inputEvt.input_num, this->inputEvt.is_high ? "HIGH" : "LOW");
      case CRINGE_EVT_TYPE_SERIAL:
        return fmt::format("serial event: {}",this->serialEvt.cmd);
      default:
        return std::string("unknown event");
    }
  }

  std::string serialize() const{
    switch (this->type){
      case CRINGE_EVT_TYPE_INPUT:
        return fmt::format("<I{}{} T{}>",
        this->inputEvt.input_num, this->inputEvt.is_high?"H":"L", this->inputEvt.timestamp);
      case CRINGE_EVT_TYPE_SERIAL:
        return fmt::format(""); 
      default:
        return std::string("");
    }    
  }
} cringeEvtContainer_t;