#include "Sensor.h"

/// @brief 构造函数
/// @param RX RX pin
/// @param TX TX pin
/// @param RE RX pin
Sensor::Sensor(int RX, int TX, TransCallback preTrans, TransCallback postTrans)
    : _sensorSerial(RX, TX) {
  _sensorModbus.preTransmission(preTrans);
  _sensorModbus.postTransmission(postTrans);
}

/**
 * 初始化
 * @param baud 波特率，支持以下波特率 300, 600, 1200, 2400, 4800, 9600, 14400,
 * 19200, 28800, 31250, 38400, 57600, 115200
 */
int Sensor::init(long baud) {
  _sensorSerial.begin(baud);
  _sensorModbus.begin(1, _sensorSerial);
  return 0;
}

/// @brief 获取力
/// @return 力值单位为 g
double Sensor::getForce() {
  uint8_t result;
  double force;

  _sensorModbus.clearResponseBuffer();
  result = _sensorModbus.readHoldingRegisters(0x0050, 2);
  if (result == _sensorModbus.ku8MBSuccess) {
    uint16_t dataHigh = _sensorModbus.getResponseBuffer(0);
    uint16_t dataLow = _sensorModbus.getResponseBuffer(1);
    uint32_t data = uint32_t(dataLow) + (uint32_t(dataHigh) << 16);
    long value = long(data);
    force = value / 10.0;
    return force;
  } else {
    return -1;
  }
}

/// @brief 手动置零
/// @return 若成功返回 0，否则返回 -1
int Sensor::setZero() {
  int result = -1;
  _sensorModbus.clearTransmitBuffer();
  _sensorModbus.setTransmitBuffer(0, 0x0001);
  result = _sensorModbus.writeMultipleRegisters(0x005E, 1);
  return result;
}

/// @brief TODO: 设置置零范围
/// @param range 
/// @return 
int Sensor::setZeroRange(int range) {
  return -1;
}