#include "Sensors.h"
#include <Arduino.h>

Sensors::Sensors()
    : left_sensor(), right_sensor(), center_sensor(),
      offsetLeft(0.0f), offsetRight(0.0f), offsetCenter(0.0f) {}

void Sensors::initSensors() {
    // Configure pins as outputs
    pinMode(CE_PIN_LEFT, OUTPUT);
    pinMode(CE_PIN_RIGHT, OUTPUT);
    pinMode(XSHUT_PIN_CENTER, OUTPUT);

    // Reset all sensors (pull low)
    digitalWrite(CE_PIN_LEFT, LOW);
    digitalWrite(CE_PIN_RIGHT, LOW);
    digitalWrite(XSHUT_PIN_CENTER, LOW);
    delay(10);

    // Start I2C
    Wire.begin();

    // Bring up left sensor first
    digitalWrite(CE_PIN_LEFT, HIGH);
    delay(10);
    initLeftSensor();

    // Then right sensor
    digitalWrite(CE_PIN_RIGHT, HIGH);
    delay(10);
    initRightSensor();

    // Finally center sensor
    digitalWrite(XSHUT_PIN_CENTER, HIGH);
    delay(10);
    initCenterSensor();

    setOffsets();
}

void Sensors::setOffsets() {
    offsetLeft = OFFSET_LEFT;
    offsetRight = OFFSET_RIGHT;
    offsetCenter = OFFSET_CENTER;
}

void Sensors::initLeftSensor() {
    if (!left_sensor.init()) {
        Serial.println("Failed to init left VL53L0X");
    }
    left_sensor.setAddress(ADDR_LEFT); // unique I2C address for left sensor
    left_sensor.startContinuous();
    Serial.println("Initialized left VL53L0X");
}

void Sensors::initRightSensor() {
    if (!right_sensor.init()) {
        Serial.println("Failed to init right VL53L0X");
    }
    right_sensor.setAddress(ADDR_RIGHT); // unique I2C address for right sensor
    right_sensor.startContinuous();
    Serial.println("Initialized right VL53L0X");
}

void Sensors::initCenterSensor() {
    if (!center_sensor.init()) {
        Serial.println("Failed to init center VL53L0X");
    }
    center_sensor.setAddress(ADDR_CENTER); // unique I2C address for center sensor
    center_sensor.startContinuous();
    Serial.println("Initialized center VL53L0X");
}

float Sensors::getLeftDistance() {
    float read = (float)left_sensor.readRangeContinuousMillimeters();
    if (left_sensor.timeoutOccurred() || read >= 2000) { // ~2m max
        return -1; // error
    }
    return read + offsetLeft;
}

float Sensors::getRightDistance() {
    float read = (float)right_sensor.readRangeContinuousMillimeters();
    if (right_sensor.timeoutOccurred() || read >= 2000) {
        return -1; // error
    }
    return read + offsetRight;
}

double Sensors::getFrontDistance() {
    float read = (float)center_sensor.readRangeContinuousMillimeters();
    if (center_sensor.timeoutOccurred() || read >= 2000) {
        return -1; // error
    }
    return read + offsetCenter;
}
