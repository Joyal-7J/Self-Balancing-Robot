#pragma once

#include <Arduino.h>
#include <Wire.h>

struct IMUReading {
  float accX;
  float accY;
  float accZ;
  float gyroX;
  float gyroY;
  float gyroZ;
};

class MPU6050IMU {
 public:
  bool begin(TwoWire &wire);
  void calibrate(uint16_t samples);
  bool read(IMUReading &out);

 private:
  TwoWire *wire_{nullptr};
  float gyroBiasX_{0.0f};
  float gyroBiasY_{0.0f};
  float gyroBiasZ_{0.0f};
  bool writeReg(uint8_t reg, uint8_t value);
  bool readRegs(uint8_t reg, uint8_t *buf, uint8_t len);
};
