#include "pid.h"

BalancedPID::BalancedPID(const PIDGains &gains) : gains_(gains) {}

void BalancedPID::setGains(const PIDGains &gains) {
  gains_ = gains;
}

float BalancedPID::update(float setpoint, float measurement, float dt, PIDTerms &termsOut) {
  float error = setpoint - measurement;

  integrator_ += error * dt;
  integrator_ = constrain(integrator_, -gains_.integratorLimit, gains_.integratorLimit);

  float derivative = (error - prevError_) / dt;
  derivativeFiltered_ = gains_.derivativeFilterAlpha * derivative +
                        (1.0f - gains_.derivativeFilterAlpha) * derivativeFiltered_;

  termsOut.p = gains_.kp * error;
  termsOut.i = gains_.ki * integrator_;
  termsOut.d = gains_.kd * derivativeFiltered_;

  lastTerms_ = termsOut;
  prevError_ = error;

  return termsOut.p + termsOut.i + termsOut.d;
}

void BalancedPID::reset() {
  integrator_ = 0.0f;
  prevError_ = 0.0f;
  derivativeFiltered_ = 0.0f;
  lastTerms_ = {0.0f, 0.0f, 0.0f};
}

PIDTerms BalancedPID::lastTerms() const {
  return lastTerms_;
}
