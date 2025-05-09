#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  Serial.println("Initializing MPU6050...");
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1); // Stop here if sensor not found
  }

  Serial.println("Connection successful!");

  // Optional: set offset manually if previously saved
    mpu.setXGyroOffset(-278.14); //220
    mpu.setYGyroOffset(-222.86); //76
    mpu.setZGyroOffset(26.14); //-85
    mpu.setXAccelOffset(-3673);
    mpu.setYAccelOffset(1768);
    mpu.setZAccelOffset(3370);

}

void loop() {
  int16_t ax, ay, az, gx, gy, gz, temp;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  temp = mpu.getTemperature();

  // Convert raw to real-world units
  float aX = ax / 16384.0;
  float aY = ay / 16384.0;
  float aZ = az / 16384.0;

  float gX = gx / 131.0;
  float gY = gy / 131.0;
  float gZ = gz / 131.0;

  float temperature = temp / 340.0 + 36.53;

  // Output
  Serial.print("aX = "); Serial.print(aX, 3); Serial.print(" g | ");
  Serial.print("aY = "); Serial.print(aY, 3); Serial.print(" g | ");
  Serial.print("aZ = "); Serial.print(aZ, 3); Serial.print(" g | ");
  Serial.print("Temp = "); Serial.print(temperature, 2); Serial.print(" °C | ");
  Serial.print("gX = "); Serial.print(gX, 2); Serial.print(" °/s | ");
  Serial.print("gY = "); Serial.print(gY, 2); Serial.print(" °/s | ");
  Serial.print("gZ = "); Serial.print(gZ, 2); Serial.println(" °/s");

  delay(1000);
}

void printOffsets() {
  Serial.println("Active Offsets:");
  Serial.print("Accel X Offset: "); Serial.println(mpu.getXAccelOffset());
  Serial.print("Accel Y Offset: "); Serial.println(mpu.getYAccelOffset());
  Serial.print("Accel Z Offset: "); Serial.println(mpu.getZAccelOffset());

  Serial.print("Gyro X Offset: "); Serial.println(mpu.getXGyroOffset());
  Serial.print("Gyro Y Offset: "); Serial.println(mpu.getYGyroOffset());
  Serial.print("Gyro Z Offset: "); Serial.println(mpu.getZGyroOffset());
}
