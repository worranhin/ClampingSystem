#include "Sensor.h"
#include "StepDriver.h"
#include "config.h"
#include "unity.h"


int RX = 11;
int TX = 13;

void preTrans();
void postTrans();

StepDriver testDriver;
Sensor testSensor(RX, TX, preTrans, postTrans);

void setUp(void) {
  // set stuff up here
  int enablePin = 7;
  int dirPin = 5;
  int stepPin = 6;
  testDriver.init(enablePin, dirPin, stepPin);
  Serial.begin(9600);
}

void tearDown(void) {
  // clean stuff up here
}

void test_init(void) {
  StepDriver mainDriver;

  int enablePin = 0;
  int dirPin = 1;
  int stepPin = 2;

  mainDriver.init(enablePin, dirPin, stepPin);
  TEST_ASSERT_TRUE(mainDriver.getEnablePin() == enablePin);
  TEST_ASSERT_TRUE(mainDriver.getDirPin() == dirPin);
  TEST_ASSERT_TRUE(mainDriver.getStepPin() == stepPin);

  TEST_ASSERT(mainDriver.getDirection() == 0);
  TEST_ASSERT(mainDriver.getFrequency() == 0);
}

void test_cal(void) {
  double v = 1;  // mm/s
  double freq = v * 1000 / 6.15;
  unsigned long delaytime = 1.0 / freq * 1000;  // ms
  TEST_ASSERT_EQUAL_UINT32(6, delaytime);
}

void test_driver(void) {
  testDriver.setDirection(1);
  testDriver.setFrequency(100);
  testDriver.start();
}

void test_goSteps(void) {
  testDriver.setDirection(0);
  testDriver.setFrequency(100);
  testDriver.goSteps(800);
}

void preTrans() {
  digitalWrite(MAX485_RE_NEG, HIGH);
}

void postTrans() {
  digitalWrite(MAX485_RE_NEG, LOW);
}

void test_sensor_init(void) {
  testSensor.init(9600);
}

void test_sensor_getForce(void) {
  double force = testSensor.getForce();
  TEST_ASSERT_DOUBLE_WITHIN(10.0, 0.0, force);
}

void test_sensor_setZero(void) {
  int result = testSensor.setZero();
  // char *mes;
  // char mes[255];
  // mes[0] = result + '0';
  // char mes = "result: " + result;
  Serial.print("result: ");
  Serial.println(result);
  TEST_ASSERT(result == 0);
}

int runUnityTests(void) {
  UNITY_BEGIN();
  // RUN_TEST(test_init);
  // RUN_TEST(test_cal);
  // RUN_TEST(test_driver);
  // RUN_TEST(test_goSteps);
  RUN_TEST(test_sensor_init);
  RUN_TEST(test_sensor_setZero);
  RUN_TEST(test_sensor_getForce);
  return UNITY_END();
}

/**
 * For Arduino framework
 */
void setup() {
  // Wait ~2 seconds before the Unity test runner
  // establishes connection with a board Serial interface
  delay(2000);

  runUnityTests();
}
void loop() {
  testDriver.routine();
}