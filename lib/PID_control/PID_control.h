#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include <Arduino.h>

class PID_Control {
 public:
  PID_Control(double kp, double ki, double kd, double target);
  void init();
  double forceToVelocity(double force);

 private:
  unsigned long _startTime = 0;
  double _e0 = 0.0;
  double _ei = 0.0;
  double _targetForce;  // 负号表示拉力
  double _KP;
  double _KI;
  double _KD;
  bool _startIntegral = false;  //开始积分标志
};

#endif  // PID_CONTROL_H