#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Serial.println("Initializing MPU6050...");
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  Serial.println("MPU6050 connection successful");
  Serial.println("Keep the sensor still... calibrating!");

  // Take many samples to average out offsets
  long accX = 0, accY = 0, accZ = 0;
  long gyroX = 0, gyroY = 0, gyroZ = 0;
  const int numReadings = 5000;

  for (int i = 0; i < numReadings; i++) {
    accX += mpu.getAccelerationX();
    accY += mpu.getAccelerationY();
    accZ += mpu.getAccelerationZ();
    gyroX += mpu.getRotationX();
    gyroY += mpu.getRotationY();
    gyroZ += mpu.getRotationZ();
    delay(2);
  }

  Serial.println("Finished calibration.\nOffsets:");
  Serial.print("Accel X offset: "); Serial.println(accX / numReadings);
  Serial.print("Accel Y offset: "); Serial.println(accY / numReadings);
  Serial.print("Accel Z offset: "); Serial.println(accZ / numReadings);
  Serial.print("Gyro X offset: ");  Serial.println(gyroX / numReadings);
  Serial.print("Gyro Y offset: ");  Serial.println(gyroY / numReadings);
  Serial.print("Gyro Z offset: ");  Serial.println(gyroZ / numReadings);
}

void loop() {
  // Nothing to do here
}