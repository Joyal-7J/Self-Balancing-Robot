#pragma once

#include <Arduino.h>

struct PIDGains {
  float kp;
  float ki;
  float kd;
  float integratorLimit;
  float derivativeFilterAlpha;
};

struct PIDTerms {
  float p;
  float i;
  float d;
};

class BalancedPID {
 public:
  explicit BalancedPID(const PIDGains &gains);
  void setGains(const PIDGains &gains);
  float update(float setpoint, float measurement, float dt, PIDTerms &termsOut);
  void reset();
  PIDTerms lastTerms() const;

 private:
  PIDGains gains_{};
  float integrator_{0.0f};
  float prevError_{0.0f};
  float derivativeFiltered_{0.0f};
  PIDTerms lastTerms_{};
};
