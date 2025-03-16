#include <Wire.h>

#define MPU_ADDR 0x68  // I2C address of MPU-6500  SCL 22 SDA 21

float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // Initialize MPU-6500
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);  // Power management register
    Wire.write(0x00);  // Wake up MPU-6500
    Wire.endTransmission();

    // Configure accelerometer to ±2g
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1C);  // ACCEL_CONFIG register
    Wire.write(0x00);  // ±2g sensitivity
    Wire.endTransmission();

    // Configure gyroscope to ±250°/s
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1B);  // GYRO_CONFIG register
    Wire.write(0x00);  // ±250°/s sensitivity
    Wire.endTransmission();

    // Enable Digital Low Pass Filter (DLPF) to reduce noise
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1A);  // CONFIG register
    Wire.write(0x03);  // Set DLPF to 41Hz
    Wire.endTransmission();
}

void loop() {
    readMPU6500();

    // Send JSON formatted data
    Serial.print("{\"ax\":");
    Serial.print(accelX, 2);
    Serial.print(",\"ay\":");
    Serial.print(accelY, 2);
    Serial.print(",\"az\":");
    Serial.print(accelZ, 2);
    Serial.print(",\"gx\":");
    Serial.print(gyroX, 2);
    Serial.print(",\"gy\":");
    Serial.print(gyroY, 2);
    Serial.print(",\"gz\":");
    Serial.print(gyroZ, 2);
    Serial.println("}");

    delay(0.01);  // Adjust delay for stable serial transmission
}

// Function to read accelerometer and gyroscope data
void readMPU6500() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);  // Starting register for accelerometer
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true);  // Request 14 bytes

    int16_t rawAx = Wire.read() << 8 | Wire.read();
    int16_t rawAy = Wire.read() << 8 | Wire.read();
    int16_t rawAz = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read();  // Skip temperature data
    int16_t rawGx = Wire.read() << 8 | Wire.read();
    int16_t rawGy = Wire.read() << 8 | Wire.read();
    int16_t rawGz = Wire.read() << 8 | Wire.read();

    // Convert raw values to G's and °/s
    accelX = rawAx / 16384.0;  // ±2g scale (32768 / 2g)
    accelY = rawAy / 16384.0;
    accelZ = rawAz / 16384.0;
    gyroX = rawGx / 131.0;  // ±250°/s scale (32768 / 250)
    gyroY = rawGy / 131.0;
    gyroZ = rawGz / 131.0;
}
