#include "imu.h"

namespace {
constexpr uint8_t MPU6050_ADDR = 0x68;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_WHO_AM_I = 0x75;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr float ACC_SCALE = 16384.0f; // LSB/g for +/-2g
constexpr float GYRO_SCALE = 131.0f;   // LSB/(deg/s) for +/-250 dps
}

bool MPU6050IMU::begin(TwoWire &wire) {
  wire_ = &wire;
  uint8_t who = 0;
  if (!readRegs(REG_WHO_AM_I, &who, 1)) {
    return false;
  }
  if (who != 0x68) {
    return false;
  }
  if (!writeReg(REG_PWR_MGMT_1, 0x00)) {
    return false;
  }
  return true;
}

void MPU6050IMU::calibrate(uint16_t samples) {
  if (!wire_) {
    return;
  }
  int32_t gx = 0;
  int32_t gy = 0;
  int32_t gz = 0;

  IMUReading reading{};
  for (uint16_t i = 0; i < samples; ++i) {
    if (read(reading)) {
      gx += reading.gyroX * GYRO_SCALE;
      gy += reading.gyroY * GYRO_SCALE;
      gz += reading.gyroZ * GYRO_SCALE;
    }
    delay(5);
  }

  gyroBiasX_ = (gx / static_cast<float>(samples)) / GYRO_SCALE;
  gyroBiasY_ = (gy / static_cast<float>(samples)) / GYRO_SCALE;
  gyroBiasZ_ = (gz / static_cast<float>(samples)) / GYRO_SCALE;
}

bool MPU6050IMU::read(IMUReading &out) {
  if (!wire_) {
    return false;
  }
  uint8_t buf[14] = {0};
  if (!readRegs(REG_ACCEL_XOUT_H, buf, sizeof(buf))) {
    return false;
  }

  int16_t ax = (buf[0] << 8) | buf[1];
  int16_t ay = (buf[2] << 8) | buf[3];
  int16_t az = (buf[4] << 8) | buf[5];
  int16_t gx = (buf[8] << 8) | buf[9];
  int16_t gy = (buf[10] << 8) | buf[11];
  int16_t gz = (buf[12] << 8) | buf[13];

  out.accX = ax / ACC_SCALE;
  out.accY = ay / ACC_SCALE;
  out.accZ = az / ACC_SCALE;
  out.gyroX = gx / GYRO_SCALE - gyroBiasX_;
  out.gyroY = gy / GYRO_SCALE - gyroBiasY_;
  out.gyroZ = gz / GYRO_SCALE - gyroBiasZ_;

  return true;
}

bool MPU6050IMU::writeReg(uint8_t reg, uint8_t value) {
  if (!wire_) {
    return false;
  }
  wire_->beginTransmission(MPU6050_ADDR);
  wire_->write(reg);
  wire_->write(value);
  return wire_->endTransmission() == 0;
}

bool MPU6050IMU::readRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
  if (!wire_) {
    return false;
  }
  wire_->beginTransmission(MPU6050_ADDR);
  wire_->write(reg);
  if (wire_->endTransmission(false) != 0) {
    return false;
  }
  uint8_t readCount = wire_->requestFrom(MPU6050_ADDR, len);
  if (readCount != len) {
    return false;
  }
  for (uint8_t i = 0; i < len; ++i) {
    buf[i] = wire_->read();
  }
  return true;
}
