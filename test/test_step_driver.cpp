#include "unity.h"
#include "StepDriver.h"

StepDriver testDriver;

void setUp(void) {
  // set stuff up here
  int enablePin = 7;
  int dirPin = 5;
  int stepPin = 6;
  testDriver.init(enablePin, dirPin, stepPin);
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

int runUnityTests(void) {
  UNITY_BEGIN();
  // RUN_TEST(test_init);
  // RUN_TEST(test_cal);
  // RUN_TEST(test_driver);
  RUN_TEST(test_goSteps);
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