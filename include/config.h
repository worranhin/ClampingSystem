#ifndef CONFIG_H
#define CONFIG_H

#define DIR_CLAMP HIGH  // 电机运动方向
#define DIR_RELEASE LOW

// 常量定义

const unsigned long SAMPLING_PERIOD = 10;  // 采样周期 (ms)
const double TARGET_FORCE = -200.0;        // 负号表示拉力
const double KP = 0.02;
const double KI = 0.0005;
const double KD = 0.01;

// 引脚定义
const int freezeBtnPin = 12;  // 控制按钮
const int clampBtnPin = 13;
const int releaseBtnPin = 11;

const int stepPin = 9;  // 驱动引脚
const int dirPin = 8;
const int enablePin = 10;  // 使能引脚，低电平有效

// const int MAX485_RE_NEG = 12;  // 低电平接收，高电平发送
const int softwareRX = 6;
const int softwareTX = 7;

enum State  // 运行状态
{
  FREEZING,
  CLAMPING,
  RELEASING
};

#endif  // CONFIG_H