#include <Arduino.h>
#include <ModbusMaster.h>
#include <SoftwareSerial.h>
// #include <arduino-timer.h>
#include "StepDriver.h"
#include "config.h"
#include "instruction.h"

// 函数声明

void handleBtn(void);
void motorFreeze(void);
void motorClamp(bool isInit);
void motorRelease(bool isInit);
double getForce(void);
void onePulse(int pulsePin, unsigned long _delayms);
void preTransmission();
void postTransmission();

// 全局变量
State mainState;
SoftwareSerial mySerial(softwareRX, softwareTX);  // RX, TX  软件串口
ModbusMaster mbMaster;  // instantiate ModbusMaster object
StepDriver mainDriver;
// auto timer = timer_create_default();  // 软件定时器

void setup() {
  // put your setup code here, to run once:

  // 串口通信初始化
  Serial.begin(9600);
  while (!Serial)
    ;
  Serial.println("Hardware serial up!");

  // 引脚设置
  pinMode(freezeBtn, INPUT_PULLUP);
  pinMode(clampBtn, INPUT_PULLUP);
  pinMode(releaseBtn, INPUT_PULLUP);
  pinMode(MAX485_RE_NEG, OUTPUT);
  digitalWrite(MAX485_RE_NEG, LOW);  // Init in receive mode
  Serial.println("Pin setup!");

  mySerial.begin(9600);
  while (!mySerial)
    ;
  Serial.println("Software serial up!");

  // Modbus setup
  mbMaster.begin(1, mySerial);  // Modbus slave ID 1
  // Callbacks allow us to configure the RS485 transceiver correctly
  mbMaster.preTransmission(preTransmission);
  mbMaster.postTransmission(postTransmission);
  Serial.println("Modbus master up!");

  // 步进驱动 setup
  mainDriver.init(enablePin, dirPin, stepPin);
  Serial.println("step driver up!");

  // 手动置零
  int result = -1;
  mbMaster.clearTransmitBuffer();
  mbMaster.setTransmitBuffer(0, 0x0001);
  while (result != mbMaster.ku8MBSuccess) {
    result = mbMaster.writeMultipleRegisters(0x005E, 1);
    Serial.println(result, HEX);
  }
  Serial.println("Set zero done!");

  // Global variable setup
  mainState = FREEZING;
  Serial.println("Global setup!");

  Serial.println("setup end!");
}

void loop() {
  // put your main code here, to run repeatedly:
  // timer.tick();
  mainDriver.routine();

  handleBtn();
  switch (mainState) {
    case FREEZING:
      motorFreeze();
      break;
    case CLAMPING:
      motorClamp(false);
      break;
    case RELEASING:
      motorRelease(false);
      break;
  }
}

// 处理按钮信号
void handleBtn(void) {
  State tempState = mainState;
  unsigned int delayms = 10;

  if (digitalRead(freezeBtn) == LOW) {
    delay(delayms);  // 消抖
    if (digitalRead(freezeBtn) == LOW) {
      tempState = FREEZING;
      motorFreeze();
      Serial.println("state: FREEZING!");
    }
  } else if (digitalRead(clampBtn) == LOW) {
    delay(delayms);
    if (digitalRead(clampBtn) == LOW) {
      tempState = CLAMPING;
      motorClamp(true);
      Serial.println("state: CLAMPING!");
    }
  } else if (digitalRead(releaseBtn) == LOW) {
    delay(delayms);
    if (digitalRead(releaseBtn) == LOW) {
      tempState = RELEASING;
      motorRelease(true);
      Serial.println("state: RELEASING!");
    }
  }
  mainState = tempState;
}

// 电机停止
void motorFreeze() {
  mainDriver.stop();
}

// 电机夹紧
void motorClamp(bool isInit) {
  if (isInit) {
    mainDriver.enable(true);
    mainDriver.setDirection(DIR_CLAMP);
  }
  // PD 控制算法
  const double targetForce = -50.0;
  const double KP = 0.04;
  const double KD = 0.01;
  double measureForce = getForce();
  Serial.print("measured force:");
  Serial.println(measureForce);
  static double e0;
  double errorForce = measureForce - targetForce;
  double e = errorForce;  // (50 -> 0)
  e0 = isInit ? e : e0;
  double de = e - e0;  // 出现变化时为正数
  e0 = e;
  double velocity = KP * e + KD * de;  // (mm/s)
  velocity = velocity < 0 ? 0 : velocity;
  Serial.print("velocity: ");
  Serial.println(velocity);

  // 电机控制程序
  double frequency = velocity * 1000 / 6.15;  // (pulse/s) 即 Hz
  double period = 1 / frequency;              // (s)
  unsigned long delayms = 1000 * period;
  // onePulse(stepPin, delayms);
  mainDriver.oneStep();
  delay(delayms);
}

/**
 * 释放电机
 * @param isInit 是否初始化，状态切换时设为 true
 */
void motorRelease(bool isInit) {
  // unsigned int pulseCount = 813;  // 5mm

  if (isInit) {
    mainDriver.enable(true);
    mainDriver.setDirection(DIR_RELEASE);
  }

  mainDriver.goSteps(900);
}

// 获取力，压力为正，拉力为负
double getForce() {
  uint8_t result;
  double force;

  Serial.println("getting force");

  mbMaster.clearResponseBuffer();
  result = mbMaster.readHoldingRegisters(0x0050, 2);
  if (result == mbMaster.ku8MBSuccess) {
    uint16_t dataHigh = mbMaster.getResponseBuffer(0);
    uint16_t dataLow = mbMaster.getResponseBuffer(1);
    uint32_t data = uint32_t(dataLow) + (uint32_t(dataHigh) << 16);
    long value = long(data);
    force = value / 10.0;
    // Serial.print("Force: ");
    // Serial.println(force);
  } else {
    Serial.print("Error: ");
    Serial.println(result, HEX);
  }

  return force;
}

// 发送单个脉冲
void onePulse(int pulsePin, unsigned long _delayms) {
  const unsigned int holdOnTime = 100;  // 大于 1us
  digitalWrite(pulsePin, HIGH);
  delayMicroseconds(holdOnTime);
  digitalWrite(pulsePin, LOW);
  delayMicroseconds(holdOnTime);
  delay(_delayms);
}

void preTransmission() {
  digitalWrite(MAX485_RE_NEG, 1);
}

void postTransmission() {
  digitalWrite(MAX485_RE_NEG, 0);
}