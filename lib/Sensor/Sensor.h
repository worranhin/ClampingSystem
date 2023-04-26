#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>
#include <ModbusMaster.h>
#include <SoftwareSerial.h>

typedef void (*TransCallback)();

class Sensor {
 private:
  SoftwareSerial _sensorSerial;
  ModbusMaster _sensorModbus;
  int _RE;

 public:
  Sensor(int RX, int TX, TransCallback preTrans, TransCallback postTrans);
  int init(long baud);
  double getForce();
  int setZero();
  int setZeroRange(int range);
};

#endif