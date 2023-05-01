#include "StepDriver.h"

/**
 * 构造函数
 */
StepDriver::StepDriver() {
  _direction = 0;
  _frequency = 0;
  _isEnable = false;
  _state = FREEZE;
}

/// @brief 构造函数
/// @param enable 使能引脚
/// @param dir 方向引脚
/// @param step 步进引脚
StepDriver::StepDriver(int enable, int dir, int step)
    : _enablePin_N(enable), _dirPin(dir), _stepPin(step) {
  pinMode(_enablePin_N, OUTPUT);
  pinMode(_dirPin, OUTPUT);
  pinMode(_stepPin, OUTPUT);
  digitalWrite(_enablePin_N, HIGH);
  digitalWrite(_dirPin, LOW);
  digitalWrite(_stepPin, LOW);

  _direction = 0;
  _frequency = 0;
  _isEnable = false;
  _state = FREEZE;
  _lastStep = millis();
  _stepLeft = 0;
};

/**
 * @brief 配置引脚
 * @warning 必须在使用前调用
 * @param enablePin 使能引脚
 * @param dirPin 方向引脚
 * @param stepPin 脉冲控制引脚
 */
int StepDriver::init(int enablePin, int dirPin, int stepPin) {
  _enablePin_N = enablePin;
  _dirPin = dirPin;
  _stepPin = stepPin;
  pinMode(_enablePin_N, OUTPUT);
  pinMode(_dirPin, OUTPUT);
  pinMode(_stepPin, OUTPUT);
  digitalWrite(_enablePin_N, HIGH);
  digitalWrite(_dirPin, LOW);
  digitalWrite(_stepPin, LOW);

  _direction = 0;
  _frequency = 0;
  _isEnable = false;
  _state = FREEZE;
  _lastStep = millis();
  _stepLeft = 0;

  return 0;
}

/**
 * 设置方向
 * @param dir 0 为顺时针转，1 为逆时针转
 */
int StepDriver::setDirection(int dir) {
  digitalWrite(_dirPin, dir);
  _direction = dir;
  return 0;
}

/**
 * 设置频率
 * @param freq 脉冲频率（单位 Hz）
 */
int StepDriver::setFrequency(unsigned int freq) {
  _frequency = freq;
  _period = freq == 0 ? UINT32_MAX : (1000.0 / freq);
  return 0;
}

/**
 * 设置周期
 * @param period 脉冲周期（ms）
 */
int StepDriver::setPeriod(unsigned long period) {
  _frequency = period == 0 ? UINT16_MAX : (1000.0 / period);
  _period = period;
  return 0;
}

void StepDriver::setSpeed(double v) {
  if (v >= 0) {
    setDirection(DIR_CLAMP);
    _frequency = v * 1000 / 6.15;
    _period = _frequency == 0 ? UINT32_MAX : (1000.0 / _frequency);
  } else if (v < 0) {
    setDirection(DIR_RELEASE);
    _frequency = (-v) * 1000 / 6.15;
    _period = _frequency == 0 ? UINT32_MAX : (1000.0 / _frequency);
  }
}

/**
 * 设置发出每个脉冲前的回调函数，可用于闭环控制算法
 * @param prePulse 发送脉冲前的回调函数
 * @warning 仅在 start() 时执行
 */
void StepDriver::setPrePulse(void (*callback)()) {
  _prePulse = callback;
}

/// @brief 设置在运行指定步数结束后的回调函数
/// @param callback 回调函数
void StepDriver::setAfterSteps(void (*callback)()) {
  _afterSteps = callback;
}

/**
 * 获取当前方向
 * @return 0 为顺时针，1 为逆时针
 */
int StepDriver::getDirection() {
  return _direction;
}

/**
 * 获取当前频率
 * @return 单位：Hz
 */
unsigned int StepDriver::getFrequency() {
  return _frequency;
}

/**
 * 获取当前周期
 * @return 周期，单位 ms
 */
unsigned long StepDriver::getPeriod() {
  return _period;
}

/**
 * 获取使能引脚
 */
int StepDriver::getEnablePin() {
  return _enablePin_N;
}

/**
 * 获取方向引脚
 */
int StepDriver::getDirPin() {
  return _dirPin;
}

/**
 * 获取步引脚
 */
int StepDriver::getStepPin() {
  return _stepPin;
}

/**
 * 获取使能状态
 */
bool StepDriver::isEnable() {
  return _isEnable;
}

/**
 * 使能电机驱动
 */
int StepDriver::enable(bool enable) {
  digitalWrite(_enablePin_N, !enable);
  _isEnable = enable;
  return 0;
}

/**
 * 开始连续运行
 */
int StepDriver::start() {
  _alterState(ALWAYS);
  _lastStep = millis();
  oneStep();
  return 0;
}

/**
 * 停止运行
 */
int StepDriver::stop() {
  _alterState(FREEZE);
  return 0;
}

/**
 * 主循环例程
 */
void StepDriver::routine() {
  // unsigned long delayms;
  unsigned long tempMillis;

  switch (_state) {
    case FREEZE:
      /* do nothing */
      break;

    case ALWAYS:
      // delayms = 1.0 / _frequency * 1000;  // ms
      if (_prePulse != NULL)
        _prePulse();
      tempMillis = millis();
      if (tempMillis - _lastStep > _period) {
        _lastStep = tempMillis;
        oneStep();
      }
      break;

    case STEPS:
      if (_stepLeft == 0) {
        _alterState(FREEZE);
        if (_afterSteps != NULL)
          _afterSteps();
        break;
      }

      // delayms = 1.0 / _frequency * 1000;  // ms
      tempMillis = millis();
      if (tempMillis - _lastStep > _period) {
        _lastStep = tempMillis;
        oneStep();
        _stepLeft--;
      }
      break;
  }
}

/**
 * 以当前配置单步运行，需要自行设置 enable
 */
int StepDriver::oneStep() {
  digitalWrite(_stepPin, HIGH);
  delayMicroseconds(_holdOn);
  digitalWrite(_stepPin, LOW);
  delayMicroseconds(_holdOn);
  return 0;
}

/**
 * 运行指定步数
 */
int StepDriver::goSteps(unsigned long steps) {
  _stepLeft = steps;
  _alterState(STEPS);
  return 0;
}

/**
 * 改变状态
 */
int StepDriver::_alterState(DriverState toState) {
  switch (toState) {
    case FREEZE:
      digitalWrite(_enablePin_N, HIGH);
      break;
    case ALWAYS:
    case STEPS:
      digitalWrite(_enablePin_N, LOW);
      digitalWrite(_dirPin, _direction);
      break;
  }
  _state = toState;
  return 0;
}