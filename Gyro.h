#ifndef GYRO_H
#define GYRO_H
#include <Adafruit_LSM6DSOX.h>

class Gyro {
public:
    Gyro();

    bool begin();     // initialize IMU  
    void update();    // read gyro & integrate angle  
    float getAngle(); // return yaw angle (deg)
    void reset();     // reset angle to zero

private:
    Adafruit_LSM6DSOX imu;

    float yaw;                 // angle in degrees
    unsigned long lastMicros; // timestamp
};

#endif

