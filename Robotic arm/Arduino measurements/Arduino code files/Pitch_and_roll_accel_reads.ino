#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
MPU6050 mpu;
float LoopTimer;

void setup() {
  Serial.begin(57600);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Wire.setClock(400000);
  Wire.begin();
  delay(500);

  // Wake up MPU6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Initialize MPU6050 object
  mpu.initialize();

  // Print current offsets
  mpu.setXAccelOffset(-1643);
  mpu.setYAccelOffset(1724);
  mpu.setZAccelOffset(5500);

  mpu.setXGyroOffset(43);
  mpu.setYGyroOffset(-17);
  mpu.setZGyroOffset(7);
  Serial.println("MPU6050 Active Offsets:");
  Serial.print("Accel X offset: "); Serial.println(mpu.getXAccelOffset());
  Serial.print("Accel Y offset: "); Serial.println(mpu.getYAccelOffset());
  Serial.print("Accel Z offset: "); Serial.println(mpu.getZAccelOffset());
  Serial.print("Gyro X offset: "); Serial.println(mpu.getXGyroOffset());
  Serial.print("Gyro Y offset: "); Serial.println(mpu.getYGyroOffset());
  Serial.print("Gyro Z offset: "); Serial.println(mpu.getZGyroOffset());
}

void loop() {
  gyro_signals();

  Serial.print("Roll angle [°]= ");
  Serial.print(AngleRoll);
  Serial.print(" Pitch angle [°]= ");
  Serial.println(AnglePitch);

  delay(250);
}

void gyro_signals(void) {
  // Configure DLPF for gyro
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  // Configure accelerometer range
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);  // ±8g
  Wire.endTransmission();

  // Read accelerometer data
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);  // Start with ACCEL_XOUT_H
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);

  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  AccX = (float)AccXLSB / 4096.0;
  AccY = (float)AccYLSB / 4096.0;
  AccZ = (float)AccZLSB / 4096.0;

  // Gyroscope config
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);  // ±500°/s
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x43);  // Start with GYRO_XOUT_H
  Wire.endTransmission();

  // Read gyroscope data
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  RateRoll = (float)GyroY / 65.5;
  RatePitch = (float)GyroX / 65.5;
  RateYaw = (float)GyroZ / 65.5;

  AccX = (float)AccXLSB / 4096 + 0.01;
  AccY = (float)AccYLSB / 4096 + 0.01;
  AccZ = (float)AccZLSB / 4096 + 0.01;

  // Calculate angles from accelerometer
  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 180 / PI;
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 180 / PI;
}
