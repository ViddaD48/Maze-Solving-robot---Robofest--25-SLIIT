#ifndef Gyro_H
#define Gyro_H


#include <Arduino.h>
#include <Wire.h>

extern const int MPU_ADDR;  // MPU-6050 I2C address

void initGyro();
float readGyroZ();
float updateAngle(float elapsedTime, float currentAngle); // Update the angle based on elapsed time since last clock

#endif // Gyro_H