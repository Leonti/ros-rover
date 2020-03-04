#include <Arduino.h>
#include <Wire.h>
#include <PID_v1.h>

void receiveEvent(int howMany) {
  while(Wire.available()) {
    char c = Wire.read();
  }
}

void setup() {
  Wire.begin(4);
  Wire.onReceive(receiveEvent);
}

void loop() {
  delay(100);
}