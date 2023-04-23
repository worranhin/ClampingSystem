#include <Arduino.h>
#include <SoftwareSerial.h>
#include <ModbusMaster.h>
// #include <arduino-timer.h>

// 状态常量
// #define FREEZING 0
// #define CLAMPING 1
// #define RELEASING 2

#define DIR_CLAMP HIGH // 电机运动方向
#define DIR_RELEASE LOW

// 引脚定义
const int freezeBtn = 9; // 控制按钮
const int clampBtn = 10;
const int releaseBtn = 8;

const int stepPin = 6; // 驱动引脚
const int dirPin = 5;
const int enablePin = 7; // 使能引脚，低电平有效

const int MAX485_RE_NEG = 12; // 低电平接收，高电平发送
const int softwareRX = 11;
const int softwareTX = 13;

void initPin()
{
  pinMode(freezeBtn, INPUT_PULLUP);
  pinMode(clampBtn, INPUT_PULLUP);
  pinMode(releaseBtn, INPUT_PULLUP);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(MAX485_RE_NEG, OUTPUT);
  digitalWrite(stepPin, LOW);
  digitalWrite(dirPin, LOW);
  digitalWrite(enablePin, HIGH);
  digitalWrite(MAX485_RE_NEG, LOW); // Init in receive mode
}

// 函数声明

void handleBtn(void);
void motorFreeze(void);
void motorClamp(bool isInit);
void motorRelease(bool isInit);
double getForce(void);
void onePulse(int pulsePin, unsigned long _delayms);
void preTransmission();
void postTransmission();
// void ISR_motor(void);


// 全局变量
// int State;
SoftwareSerial mySerial(softwareRX, softwareTX);  // RX, TX  软件串口
ModbusMaster mbMaster;  // instantiate ModbusMaster object
// auto timer = timer_create_default();  // 软件定时器

enum State {
  FREEZING,
  CLAMPING,
  RELEASING
} state;  // 运行状态

void setup()
{
  // put your setup code here, to run once:
  initPin();
  state = FREEZING;

  Serial.begin(9600);
  while (!Serial);
  Serial.println("Hardware Serial up!");

  mySerial.begin(9600);
  while (!Serial);
  Serial.println("Software serial up!");

  // 
  mbMaster.begin(1, mySerial);  // Modbus slave ID 1
  // Callbacks allow us to configure the RS485 transceiver correctly
  mbMaster.preTransmission(preTransmission);
  mbMaster.postTransmission(postTransmission);
  Serial.println("Modbus master up!");

  // 手动置零
  int result = -1;
  mbMaster.clearTransmitBuffer();
  mbMaster.setTransmitBuffer(0, 0x0001);
  while (result != mbMaster.ku8MBSuccess)
  {
    result = mbMaster.writeMultipleRegisters(0x005E, 1);
    Serial.println(result, HEX);
  }

  Serial.println("setup end!");
}

void loop()
{
  // put your main code here, to run repeatedly:
  // timer.tick();
  handleBtn();
  // const int initSpeed = 10;
  switch (state)
  {
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
void handleBtn(void)
{
  State tempState = state;
  if (digitalRead(freezeBtn) == LOW)
  {
    tempState = FREEZING;
    motorFreeze();
    Serial.println("state: FREEZING!");
  }
  else if (digitalRead(clampBtn) == LOW)
  {
    tempState = CLAMPING;
    motorClamp(true);
    Serial.println("state: CLAMPING!");
  }
  else if (digitalRead(releaseBtn) == LOW)
  {
    tempState = RELEASING;
    motorRelease(true);
    Serial.println("state: RELEASING!");
  }
  state = tempState;
}

// 电机停止
void motorFreeze()
{
  digitalWrite(enablePin, HIGH);
}

// 电机夹紧
void motorClamp(bool isInit)
{
  if (isInit)
  {
    digitalWrite(enablePin, LOW);
    digitalWrite(dirPin, DIR_CLAMP);
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
  double e = errorForce; // (50 -> 0)
  e0 = isInit ? e : e0;
  double de = e - e0;                    // 出现变化时为正数
  double velocity = KP * e + KD * de; // (mm/s)
  velocity = velocity < 0 ? 0 : velocity;
  e0 = e;

  // 电机控制程序
  double frequency = velocity * 1000 / 6.15; // (pulse/s) 即 Hz
  double period = 1 / frequency;             // (s)
  unsigned long delayms = 1000 * period;
  onePulse(stepPin, delayms);
  // timer.in(delayms, &motorClamp);
}

// 电机释放
void motorRelease(bool isInit)
{
  static unsigned int pulseCount;
  const unsigned long delayms = 5;

  if (isInit)
  {
    pulseCount = 813;  // 5mm
    digitalWrite(enablePin, LOW);
    digitalWrite(dirPin, DIR_RELEASE);
  }

  if (pulseCount-- > 0)
  {
    onePulse(stepPin, delayms);
  }
  else
  {
    state = FREEZING;
  }
}

// 获取力，压力为正，拉力为负
double getForce()
{
  uint8_t result;
  double force;

  Serial.println("getting force");

  mbMaster.clearResponseBuffer();
  result = mbMaster.readHoldingRegisters(0x0050, 2);
  if(result == mbMaster.ku8MBSuccess) {
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
void onePulse(int pulsePin, unsigned long _delayms)
{
  const unsigned int holdOnTime = 100; // 大于 1us
  digitalWrite(pulsePin, HIGH);
  delayMicroseconds(holdOnTime);
  digitalWrite(pulsePin, LOW);
  delayMicroseconds(holdOnTime);
  delay(_delayms);
}

void preTransmission()
{
  digitalWrite(MAX485_RE_NEG, 1);
}

void postTransmission()
{
  digitalWrite(MAX485_RE_NEG, 0);
}