#ifndef WHEEL_H
#endif

enum WheelDirection {
  FORWARD = 1,
  BACKWARD = -1,
};

class Wheel {
public:
  int encoderPin;
  int motorPin1;
  int motorPin2;
  int motorSpeed;
  WheelDirection currentDirection;
  volatile int *wheelState;


  Wheel(int encoderPin, int motorPin1, int motorPin2, int motorSpeed, volatile int *wheelState);
  volatile int getMotorState();
  void spinWheel(WheelDirection direction, int speed);
  void coast();
  void brake();
  void onFullRotation();
};
