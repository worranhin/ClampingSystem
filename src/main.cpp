#include <Arduino.h>
#include "Button.h"
#include "Sensor.h"
#include "StepDriver.h"
#include "config.h"
// #include "instruction.h"
// #include <ModbusMaster.h>
// #include <SoftwareSerial.h>

/// 函数声明 ///

void changeState(State toState);
void btnRoutine();
void clampRoutine();
void communicationRoutine();
unsigned int force2Frequency_PD(double force, double lastForce);
void preTransmission();
void postTransmission();
void prePulse();

/// 全局变量 ///

State mainState;
double lastForce;

// StepDriver mainDriver(enablePin, dirPin, stepPin);
StepDriver mainDriver;
Sensor mainSensor(softwareRX, softwareTX, preTransmission, postTransmission);
Button freezeBtn(freezeBtnPin);
Button clampBtn(clampBtnPin);
Button releaseBtn(releaseBtnPin);
// SoftwareSerial mySerial(softwareRX, softwareTX);  // RX, TX  软件串口
// ModbusMaster mbMaster;  // instantiate ModbusMaster object
// auto timer = timer_create_default();  // 软件定时器

void setup() {
  // 串口通信初始化
  Serial.begin(9600);
  while (!Serial)
    ;
  Serial.println("Hardware serial up!");

  // 引脚设置
  pinMode(MAX485_RE_NEG, OUTPUT);
  digitalWrite(MAX485_RE_NEG, LOW);  // Init in receive mode
  Serial.println("Pin setup!");

  // 步进驱动和传感器 setup
  mainDriver.init(enablePin, dirPin, stepPin);
  mainDriver.setPrePulse([]() {
    double force = mainSensor.getForce();
    unsigned int freq = force2Frequency_PD(force, lastForce);
    mainDriver.setFrequency(freq);
  });
  Serial.println("step driver up!");

  mainSensor.init(9600);
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
  lastForce = 0;
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
      unsigned int tempFreq = force2Frequency_PD(tempForce, tempForce);
      lastForce = tempForce;
      mainDriver.setDirection(DIR_CLAMP);
      mainDriver.setFrequency(tempFreq);
      mainDriver.start();
      Serial.println("state: CLAMPING!");
      break;
    }

    case State::RELEASING: {
      mainState = RELEASING;
      mainDriver.setFrequency(100);
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

/// @brief 夹持状态的例程
void clampRoutine() {
  double result = mainSensor.getForce();
  if (result == -1)
    return;
  double force = mainSensor.getForce();
  unsigned int freq = force2Frequency_PD(force, lastForce);
  lastForce = force;
  mainDriver.setFrequency(freq);
}

/// @brief 通讯例程
void communicationRoutine() {
  static unsigned long lastCommunication = 0;
  if (millis() - lastCommunication > 1000) {
    double tempForce = mainSensor.getForce();
    Serial.print(millis());
    Serial.print(", ");
    Serial.println(tempForce);
    lastForce = tempForce;
    lastCommunication = millis();
  }
}

/// @brief PD 控制算法，输入为力，输出速度
/// @param force 测得的力
/// @param lastForce 上一次测得的力
/// @return 返回频率
unsigned int force2Frequency_PD(double force, double lastForce) {
  // PD 控制算法
  const double targetForce = -100.0;  // 负号表示拉力
  const double KP = 0.04;
  const double KD = 0.01;
  double e = force - targetForce;  // (50 -> 0)
  double e0 = lastForce - targetForce;
  double de = e - e0;                  // 出现变化时为正数
  double velocity = KP * e + KD * de;  // (mm/s)
  velocity = velocity < 0 ? 0 : velocity;
  unsigned int frequency = velocity * 1000 / 6.15;  // (pulse/s) 即 Hz
  return frequency;
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
  if(millis() - lastGetForce < 1000) {
    return;
  }
  double result = mainSensor.getForce();
  if (result == -1)
    return;
  double force = mainSensor.getForce();
  unsigned int freq = force2Frequency_PD(force, lastForce);
  lastForce = force;
  mainDriver.setFrequency(freq);
  lastGetForce = millis();
}