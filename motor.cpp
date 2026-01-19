#include "motor.h"

void StepperMotor::begin(int stepPin, int dirPin, int enPin, int ledcChannel, bool enActiveLow) {
  stepPin_ = stepPin;
  dirPin_ = dirPin;
  enPin_ = enPin;
  ledcChannel_ = ledcChannel;
  enActiveLow_ = enActiveLow;

  pinMode(stepPin_, OUTPUT);
  pinMode(dirPin_, OUTPUT);
  pinMode(enPin_, OUTPUT);

  ledcSetup(ledcChannel_, 1, 10); // placeholder frequency, 10-bit resolution
  ledcAttachPin(stepPin_, ledcChannel_);
  ledcWrite(ledcChannel_, 0);
}

void StepperMotor::enable(bool enabled) {
  if (enActiveLow_) {
    digitalWrite(enPin_, enabled ? LOW : HIGH);
  } else {
    digitalWrite(enPin_, enabled ? HIGH : LOW);
  }
}

void StepperMotor::setTargetSpeed(float targetStepsPerSec, float maxAccel, float dt) {
  float delta = targetStepsPerSec - currentSpeed_;
  float maxDelta = maxAccel * dt;

  if (delta > maxDelta) {
    delta = maxDelta;
  } else if (delta < -maxDelta) {
    delta = -maxDelta;
  }
  currentSpeed_ += delta;

  if (fabsf(currentSpeed_) < 1.0f) {
    ledcWrite(ledcChannel_, 0);
    return;
  }

  bool dir = currentSpeed_ >= 0.0f;
  digitalWrite(dirPin_, dir ? HIGH : LOW);

  float freq = fabsf(currentSpeed_);
  ledcWriteTone(ledcChannel_, freq);
  ledcWrite(ledcChannel_, 512); // 50% duty
}
