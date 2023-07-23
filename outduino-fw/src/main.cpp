#include <Arduino.h>
// #include "USB.h"

// put function declarations here:
int myFunction(int, int);

void setup() {

  // productName("test");

  // put your setup code here, to run once:
  int result = myFunction(2, 3);
  Serial.begin(115200);
  Serial.println("aasi");
}

void loop() {
  // put your main code here, to run repeatedly:
    Serial.println("aasi");
    sleep(1);
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}