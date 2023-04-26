#ifndef BUTTON_H
#define BUTTON_H

#include <Arduino.h>

class Button {
 public:
  Button(uint8_t pin) : _pin(pin) {
    pinMode(pin, INPUT_PULLUP);
  };
  int read();

 private:
  uint8_t _pin;
};

#endif