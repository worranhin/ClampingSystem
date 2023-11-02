#include "PID_control.h"

/// @brief 构造函数
/// @param kp 比例常数
/// @param ki 积分常数
/// @param kd 微分常数
/// @param target 目标力
PID_Control::PID_Control(double kp, double ki, double kd, double target) {
  _KP = kp;
  _KI = ki;
  _KD = kd;
  _targetForce = target;
}

/// @brief 初始化
void PID_Control::init() {
  _startTime = millis();
  _e0 = 0.0;
  _ei = 0.0;
  _startIntegral = false;
}

/// @brief 输入力，输出速度
/// @param force 力，单位：g
/// @return 速度，单位：mm/s
double PID_Control::forceToVelocity(double force) {
  // if (!_startIntegral && fabs(force) > fabs(_targetForce * 0.95))
  //   _startIntegral = true;
  double e = force - _targetForce;
  _ei += e;
  double de = e - _e0;
  double v = _KP * e + _KI * _ei + _KD * de;  // (mm/s)

  // if (_startIntegral)

  if (_ei > 100)
    _ei = 100;
  else if (_ei < -100)
    _ei = -100;

  _e0 = e;

  return v;
}
