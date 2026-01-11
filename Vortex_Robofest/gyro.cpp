#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

const float tolerance = 0.2;   // Tolerance in degrees
unsigned long previousTime = 0;
unsigned long currentTime = 0;

// Initialize the MPU6050
void initGyro() {
    Wire.begin(21, 22);  // SDA=21, SCL=22 on ESP32
    mpu.initialize();

    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        while (1);
    }
    Serial.println("MPU6050 Gyroscope Initialized");

    previousTime = millis();
}

// Read gyroscope Z-axis angular velocity in degrees per second (°/s)
float readGyroZ() {
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    return gz / 131.0;  // Sensitivity scale factor for ±250°/s
}

// Update angle based on elapsed time and gyroZ readings
float updateAngle(float elapsedTime, float currentAngle) {
    float gyroZ_dps = readGyroZ();
    currentAngle += gyroZ_dps * elapsedTime;  // Integrate angular velocity
    return currentAngle;
}


