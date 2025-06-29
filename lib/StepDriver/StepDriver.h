#ifndef STEP_DRIVER_H
#define STEP_DRIVER_H

#include <Arduino.h>

#define DIR_CLAMP HIGH  // 电机运动方向
#define DIR_RELEASE LOW

enum DriverState { FREEZE, ALWAYS, STEPS };

class StepDriver {
 public:
  StepDriver();
  StepDriver(int enable, int dir, int step);

  int init(int enablePin, int dirPin, int stepPin);
  int setDirection(int dir);
  int setFrequency(unsigned int freq);
  int setPeriod(unsigned long per);
  void setSpeed(double v);
  void setPrePulse(void (*callback)());
  void setAfterSteps(void (*callback)());
  int getDirection();
  unsigned int getFrequency();
  unsigned long getPeriod();
  int getEnablePin();
  int getDirPin();
  int getStepPin();
  bool isEnable();
  int enable(bool enable);
  int start();
  int stop();
  void routine();
  int oneStep();
  int goSteps(unsigned long);

 private:
  int _holdOn = 100;                 // 脉冲保持时间，大于 1us
  int _enablePin_N;                  // 使能引脚，低电平有效
  int _dirPin;                       // 方向引脚
  int _stepPin;                      // 驱动引脚
  int _direction = 0;                // 方向
  const int _MS = 1;                 // 细分
  unsigned int _frequency = 0;       // 频率 (Hz)
  unsigned long _period = 0;         // 周期 (ms)
  bool _isEnable = false;            // 使能
  DriverState _state = FREEZE;       // 运行状态
  unsigned long _lastStep = 0;       // 上一步时间 (ms)
  unsigned long _stepLeft = 0;       // 剩余步数
  void (*_prePulse)(void) = NULL;    // 发脉冲前的回调函数
  void (*_afterSteps)(void) = NULL;  // 走完指定步数后执行的回调函数

  int _alterState(DriverState toState);
};

#endif  // STEP_DRIVER_H