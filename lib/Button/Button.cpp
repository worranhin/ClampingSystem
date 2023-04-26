#include "Button.h"

/// @brief 读取按钮
/// @return 按下返回 1，否则返回 0
int Button::read() {
  if (digitalRead(_pin) == LOW) {
    delay(10);
    if (digitalRead(_pin) == LOW) {
      while (digitalRead(_pin) == LOW)
        ;
      return 1;
    }
  }
  return 0;
}