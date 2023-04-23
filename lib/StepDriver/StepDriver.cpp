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
  if(freq == 0) {
    _alterState(FREEZE);
    return 1;
  }
  _frequency = freq;
  _period = 1000.0 / freq;
  return 0;
}

/**
 * 设置周期
 * @param per 脉冲周期（ms）
 */
int StepDriver::setPeriod(unsigned long per) {
  if(per == 0) {
    _frequency = INT16_MAX;
    _period = 0;
    return 1;
  }
  _frequency = 1000.0 / per;
  _period = per;
  return 0;
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
      tempMillis = millis();
      if (tempMillis - _lastStep > _period) {
        _lastStep = tempMillis;
        oneStep();
      }
      break;

    case STEPS:
      if (_stepLeft == 0) {
        _alterState(FREEZE);
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