#pragma once

#include <Arduino.h>

class StepperMotor {
 public:
  void begin(int stepPin, int dirPin, int enPin, int ledcChannel, bool enActiveLow);
  void enable(bool enabled);
  void setTargetSpeed(float targetStepsPerSec, float maxAccel, float dt);

 private:
  int stepPin_{-1};
  int dirPin_{-1};
  int enPin_{-1};
  int ledcChannel_{0};
  bool enActiveLow_{true};

  float currentSpeed_{0.0f};
};
