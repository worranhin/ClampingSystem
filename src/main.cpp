#include <Arduino.h>
#include "Button.h"
#include "PID_control.h"
#include "Sensor.h"
#include "StepDriver.h"
#include "config.h"
// #include "instruction.h"
// #include <ModbusMaster.h>
// #include <SoftwareSerial.h>

/// 函数声明 ///

void changeState(State toState);
void btnRoutine();
// void clampRoutine();
void communicationRoutine();
void preTransmission();
void postTransmission();
void prePulse();
void afterSteps();

/// 全局变量 ///

State mainState;

// StepDriver mainDriver(enablePin, dirPin, stepPin);
StepDriver mainDriver;
Sensor mainSensor(softwareRX, softwareTX, preTransmission, postTransmission);
Button freezeBtn(freezeBtnPin);
Button clampBtn(clampBtnPin);
Button releaseBtn(releaseBtnPin);
PID_Control mainPID(KP, KI, KD, TARGET_FORCE);

void setup() {
  // 串口通信初始化
  Serial.begin(9600);
  while (!Serial)
    ;
  Serial.println("Hardware serial up!");

  // 步进驱动和传感器 setup
  mainDriver.init(enablePin, dirPin, stepPin);
  mainDriver.setPrePulse(prePulse);
  mainDriver.setAfterSteps(afterSteps);
  Serial.println("step driver up!");

  mainSensor.init(57600);
  Serial.println("sensor up!");

  // 上电手动置零

  int setZeroResult = -1;
  while (setZeroResult != 0) {
    setZeroResult = mainSensor.setZero();
    if (setZeroResult != 0) {
      Serial.print("Error: ");
      Serial.println(setZeroResult, HEX);
    }
  }
  Serial.println("Set zero done!");

  // Global variable setup
  mainState = FREEZING;
  Serial.println("Global variable setup!");

  Serial.println("setup end!");

  Serial.println("Data format:");
  Serial.println("time(ms), force(g)");
}

void loop() {
  btnRoutine();
  mainDriver.routine();
  communicationRoutine();  // 通讯例程
  // if (mainState == State::CLAMPING) {
  //   clampRoutine();
  // }
}

/// @brief 改变主程序运行状态
/// @param toState 目标状态
void changeState(State toState) {
  switch (toState) {
    case State::FREEZING: {
      mainState = FREEZING;
      mainDriver.stop();
      Serial.println("state: FREEZING!");
      break;
    }

    case State::CLAMPING: {
      mainState = CLAMPING;
      double tempForce = mainSensor.getForce();
      mainPID.init();
      double tempSpeed = mainPID.forceToVelocity(tempForce);
      mainDriver.setSpeed(tempSpeed);
      mainDriver.start();
      Serial.println("state: CLAMPING!");
      Serial.println("time(ms), force(g)");
      break;
    }

    case State::RELEASING: {
      mainState = RELEASING;
      mainDriver.setFrequency(200);
      mainDriver.setDirection(DIR_RELEASE);
      mainDriver.goSteps(813);
      Serial.println("state: RELEASING!");
      break;
    }
    default:
      break;
  }
}

/// @brief 接收按键信号的例程
void btnRoutine() {
  if (freezeBtn.read())
    changeState(State::FREEZING);
  else if (clampBtn.read())
    changeState(State::CLAMPING);
  else if (releaseBtn.read())
    changeState(State::RELEASING);
}

// /// @brief 夹持状态的例程
// void clampRoutine() {
//   double result = mainSensor.getForce();
//   if (result == -1)
//     return;
//   double force = mainSensor.getForce();
//   unsigned int freq = force2Frequency_PD(force, lastForce);
//   lastForce = force;
//   mainDriver.setFrequency(freq);
// }

/// @brief 通讯例程
void communicationRoutine() {
  // static unsigned long lastCommunication = 0;
  // if (millis() - lastCommunication > SAMPLING_PERIOD) {
  //   // double tempForce = mainSensor.getForce();
  //   Serial.print(millis());
  //   Serial.print(", ");
  //   Serial.print(mainSensor.getForce());
  //   Serial.print(", ");
  //   Serial.println(lastForce);
  //   // lastForce = tempForce;
  //   lastCommunication = millis();
  // }
}

/// @brief Modbus 通信前回调函数，关闭接收，开启发送
void preTransmission() {
  // digitalWrite(MAX485_RE_NEG, 1);
}

/// @brief Modbus 通信后回调函数，开启接收，关闭发送
void postTransmission() {
  // digitalWrite(MAX485_RE_NEG, 0);
}

/// @brief 电机脉冲回调
void prePulse() {
  static unsigned long lastGetForce = 0;
  if (millis() - lastGetForce < SAMPLING_PERIOD) {
    return;
  }
  double force = mainSensor.getForce();
  Serial.print(millis());
  Serial.print(", ");
  Serial.println(force);
  mainDriver.setSpeed(mainPID.forceToVelocity(force));
  lastGetForce = millis();
}

/// @brief 按步执行后的回调函数
void afterSteps() {
  changeState(State::FREEZING);
}