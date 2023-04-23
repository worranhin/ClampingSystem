#ifndef CONFIG_H
#define CONFIG_H
// 状态常量
// #define FREEZING 0
// #define CLAMPING 1
// #define RELEASING 2

#define DIR_CLAMP HIGH  // 电机运动方向
#define DIR_RELEASE LOW

// 引脚定义
const int freezeBtn = 9;  // 控制按钮
const int clampBtn = 10;
const int releaseBtn = 8;

const int stepPin = 6;  // 驱动引脚
const int dirPin = 5;
const int enablePin = 7;  // 使能引脚，低电平有效

const int MAX485_RE_NEG = 12;  // 低电平接收，高电平发送
const int softwareRX = 11;
const int softwareTX = 13;

enum State  // 运行状态
{
  FREEZING,
  CLAMPING,
  RELEASING
};

#endif  // CONFIG_H