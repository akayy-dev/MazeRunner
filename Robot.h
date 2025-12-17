#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include "Wheel.h"
#include "PositionData.h"

#define CELL_LENGTH 580

class Robot {
public:
  Position position;
  Wheel leftWheel;
  Wheel rightWheel;
  RobotOrientation orientation;

  Cell maze[5][5];

  Robot(Wheel L, Wheel R, RobotOrientation orientation);

  void DFSSearch();

  void turnLeft();

  void turnRight();

  void moveForwardOneCell();

  void checkWheelStates();
};

#endif
