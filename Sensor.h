#ifndef SENSOR_H
#endif // ! SENSOR_H

class Sensor {
public:
  int trigPin;
  int echoPin;
  long duration;
  long distance;

  Sensor(int trigPin, int echoPin);
  int getDistance();
};
