#include <VL53L0X.h>

#ifndef SENSORS_H
#define SENSORS_H

#include <Wire.h>

// Control pins for powering/XSHUT
#define CE_PIN_LEFT 16
#define CE_PIN_RIGHT 5
#define XSHUT_PIN_CENTER 17

// Unique I2C addresses for each sensor
#define ADDR_LEFT   0x30
#define ADDR_RIGHT  0x31
#define ADDR_CENTER 0x32

// Calibration offsets (in mm)
#define OFFSET_LEFT   0.0f
#define OFFSET_RIGHT  2.0f
#define OFFSET_CENTER 0.0f

// Thresholds (tweak for maze logic)
#define STOP_CENTER_THRESHOLD 80 // in mm
#define SIDE_THRESHOLD 90        // in mm

class Sensors {
public:
    Sensors();

    void initSensors();

    float getLeftDistance();
    float getRightDistance();
    double getFrontDistance();

private:
    void initLeftSensor();
    void initRightSensor();
    void initCenterSensor();
    void setOffsets();

    VL53L0X left_sensor;
    VL53L0X right_sensor;
    VL53L0X center_sensor;

    float offsetLeft;
    float offsetRight;
    float offsetCenter;
};

#endif // SENSORS_H
