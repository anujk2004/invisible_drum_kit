/* ESP32 -> stream Madgwick orientation
   SDA -> 21, SCL -> 22 by default (adjust if needed)
   Outputs lines like:
     ORIENT  -23.45  12.34  -5.67
   (yaw deg, pitch deg, roll deg)
*/

#include <Wire.h>
#include "MadgwickAHRS.h"

#define SDA_PIN 21
#define SCL_PIN 22

Madgwick filter;

const uint8_t MPU_ADDR = 0x68;
const float ACCEL_SCALE = 16384.0f;
const float GYRO_SCALE  = 131.0f;

unsigned long lastMicros = 0;

void wakeMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();
  delay(10);
}

void readAndUpdateFilter(float dt_s) {
  // read 14 bytes starting at 0x3B
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14);
  if (Wire.available() < 14) return;

  int16_t ax = (Wire.read()<<8) | Wire.read();
  int16_t ay = (Wire.read()<<8) | Wire.read();
  int16_t az = (Wire.read()<<8) | Wire.read();
  int16_t gx = (Wire.read()<<8) | Wire.read();
  int16_t gy = (Wire.read()<<8) | Wire.read();
  int16_t gz = (Wire.read()<<8) | Wire.read();

  float axg = ax / ACCEL_SCALE;
  float ayg = ay / ACCEL_SCALE;
  float azg = az / ACCEL_SCALE;
  // Madgwick update expects gyros in degrees/sec
  float gxd = gx / GYRO_SCALE;
  float gyd = gy / GYRO_SCALE;
  float gzd = gz / GYRO_SCALE;

  // use updateIMU with degrees/sec and g
  filter.updateIMU(gxd, gyd, gzd, axg, ayg, azg);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Wire.begin(SDA_PIN, SCL_PIN);
  wakeMPU();
  delay(200);
  filter.begin(100); // sample rate hint (Hz) - helpful for filter internals
  lastMicros = micros();
  Serial.println("ORIENT_STREAM_READY");
}

void loop() {
  unsigned long now = micros();
  float dt = (now - lastMicros) / 1000000.0f;
  if (dt <= 0) dt = 0.01f;
  lastMicros = now;

  readAndUpdateFilter(dt);

  // get orientation from Madgwick (returns degrees in this library)
  float yaw = filter.getYaw();
  float pitch = filter.getPitch();
  float roll = filter.getRoll();

  // print as degrees with 2 decimals
  Serial.print("ORIENT ");
  Serial.print(yaw, 2);
  Serial.print(" ");
  Serial.print(pitch, 2);
  Serial.print(" ");
  Serial.println(roll, 2);

  // stream at ~80-120 Hz; adjust delay to control rate
  delay(10);
}
