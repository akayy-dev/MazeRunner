#include "Gyro.h"
#include <Arduino.h>



Gyro::Gyro() {
    yaw = 0.0;
    lastMicros = 0;
}

bool Gyro::begin() {
    if (!imu.begin_I2C()) {
        return false;
    }
    lastMicros = micros();
    return true;
}

void Gyro::update() {
    sensors_event_t accel, gyro, temp;
    imu.getEvent(&accel, &gyro, &temp);

    unsigned long now = micros();
    float dt = (now - lastMicros) / 1e6; 
    lastMicros = now;

    // Z-axis gyro (rad/s → deg/s)
    float gz = gyro.gyro.z * 57.2958;

    // integrate yaw
    yaw += gz * dt;
}

float Gyro::getAngle() {
    return yaw;
}

void Gyro::reset() {
    yaw = 0.0;
}

