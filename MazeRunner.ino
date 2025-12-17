#include <ArduinoQueue.h>
#include <Stack.h>
#include "Gyro.h"
#include "Wheel.h"
#include "Sensor.h"
#include "PositionData.h"

// Encoder output pins
#define ENCODER_A 2
#define ENCODER_B 3

#define MOTOR1_PIN1 5
#define MOTOR1_PIN2 6
#define MOTOR1_SPEED 4

#define MOTOR2_PIN1 A3
#define MOTOR2_PIN2 A2
#define MOTOR2_SPEED A1

#define CELL_LENGTH 700
#define RIGHT_SONIC_ECHO 8
#define RIGHT_SONIC_TRIG 9

#define LEFT_SONIC_ECHO 10
#define LEFT_SONIC_TRIG 11

#define FRONT_SONIC_ECHO 12
#define FRONT_SONIC_TRIG 13

#define WHEEL_SPEED 130
#define CELL_LENGTH 700

#define TURNING_ANGLE 80

volatile int *Wheel1State = new int;
volatile int *Wheel2State = new int;

class Robot
{
public:
  Position position;
  Wheel leftWheel;
  Wheel rightWheel;
  RobotOrientation orientation;
  Gyro gyro;

  Sensor leftSensor;
  Sensor frontSensor;
  Sensor rightSensor;

  Cell maze[5][5];

  Robot(Wheel L, Wheel R, RobotOrientation orientation)
      : leftWheel(L), rightWheel(R),
        frontSensor(FRONT_SONIC_TRIG, FRONT_SONIC_ECHO),
        leftSensor(LEFT_SONIC_TRIG, LEFT_SONIC_ECHO),
        rightSensor(RIGHT_SONIC_TRIG, RIGHT_SONIC_ECHO)
  {
    this->orientation = orientation;
    position.xPos = 0;
    position.yPos = 0;
  }

  boolean isWallLeft()
  {
    return this->leftSensor.getDistance() <= 10;
  }

  boolean isWallRight()
  {
    return this->rightSensor.getDistance() <= 10;
  }

  boolean isWallFront()
  {
    return this->frontSensor.getDistance() <= 10;
  }

  void moveForwardOneCell()
  {
    *Wheel1State = 0;
    *Wheel2State = 0;

    this->leftWheel.spinWheel(FORWARD, WHEEL_SPEED);
    this->rightWheel.spinWheel(FORWARD, WHEEL_SPEED);

    Serial.println("Moving forward one cell");

    while (true)
    {
      if (*Wheel1State >= CELL_LENGTH || *Wheel2State >= CELL_LENGTH)
      {
        this->rightWheel.brake();
        this->leftWheel.brake();
        Serial.println("Moving forward complete. Reset wheel states");
        *Wheel1State = 0;
        *Wheel2State = 0;
        break;
      }
    }
  }

  void move(Direction dir)
  {
    // 1. Rotate to the correct heading
    if (dir == LEFT)
    {
      turnLeft90();
    }
    else if (dir == RIGHT)
    {
      turnRight90();
    }
    else if (dir == BACKWARD)
    {
      turnLeft90();
      turnLeft90();
    }

    // 2. Physical movement
    moveForwardOneCell();

    // 3. Update internal (X, Y) coordinates based on NEW orientation
    switch (this->orientation)
    {
    case NORTH:
      position.yPos += 1;
      break;
    case SOUTH:
      position.yPos -= 1;
      break;
    case EAST:
      position.xPos += 1;
      break;
    case WEST:
      position.xPos -= 1;
      break;
    }
  }

  void turnRight90()
  {
    this->gyro.reset();

    this->rightWheel.spinWheel(FORWARD, WHEEL_SPEED);
    this->leftWheel.spinWheel(BACKWARD, WHEEL_SPEED);

    while (true)
    {
      this->gyro.update();
      float angle = abs(this->gyro.getAngle());

      Serial.print("Angle: ");
      Serial.println(angle);

      if (angle >= TURNING_ANGLE)
      {
        break;
      }
    }

    this->leftWheel.brake();
    this->rightWheel.brake();
    switch (this->orientation)
    {
    case NORTH:
      this->orientation = EAST;
      break;
    case EAST:
      this->orientation = SOUTH;
      break;
    case SOUTH:
      this->orientation = WEST;
      break;
    case WEST:
      this->orientation = NORTH;
      break;
    }
    gyro.reset();
  }

  void turnLeft90()
  {
    this->leftWheel.spinWheel(FORWARD, WHEEL_SPEED);
    this->rightWheel.spinWheel(BACKWARD, WHEEL_SPEED);

    while (true)
    {
      this->gyro.update();
      float angle = abs(this->gyro.getAngle());

      Serial.print("Angle: ");
      Serial.println(angle);

      if (angle >= TURNING_ANGLE)
      {
        break;
      }
    }

    this->leftWheel.brake();
    this->rightWheel.brake();
    switch (this->orientation)
    {
    case NORTH:
      this->orientation = WEST;
      break;
    case WEST:
      this->orientation = SOUTH;
      break;
    case SOUTH:
      this->orientation = EAST;
      break;
    case EAST:
      this->orientation = NORTH;
      break;
    }
    gyro.reset();
  }
};

void onMotorInterrupt(volatile int *motorState)
{
  *motorState = *motorState + 1;
  if (*motorState > CELL_LENGTH)
  {
    *motorState = 0;
  }
}

Wheel wheel1 = Wheel(ENCODER_A, MOTOR1_PIN1, MOTOR1_PIN2, MOTOR1_SPEED, Wheel1State);
Wheel wheel2 = Wheel(ENCODER_B, MOTOR2_PIN1, MOTOR2_PIN2, MOTOR2_SPEED, Wheel2State);

Robot mouse = Robot(wheel1, wheel2, EAST);

void setup()
{
  Serial.begin(9600);
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);

  delay(5000);

  // Set default wheel states
  *Wheel2State = 0;
  *Wheel1State = 0;

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), []()
                  {
    *(wheel1.wheelState) = *(wheel1.wheelState) + 1;
    if (*wheel1.wheelState > CELL_LENGTH) {
      *wheel1.wheelState = 0;
    } }, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), []()
                  {
    *(wheel2.wheelState) = *(wheel2.wheelState) + 1;
    if (*wheel2.wheelState > CELL_LENGTH) {
      *wheel2.wheelState = 0;
    } }, RISING);

  Serial.println("Sent signal");
  if (mouse.gyro.begin())
  {
    Serial.println("Begun gyro");
  }
  else
  {
    Serial.println("Failed to begin gyro");
  }

  mouse.moveForwardOneCell();
  delay(1000);
  // mouse.turnRight90();
  // delay(1000);
  // mouse.moveForwardOneCell();
}

void loop()
{
  Serial.println(mouse.frontSensor.getDistance());
  if (!mouse.isWallLeft())
  {
    mouse.moveForwardOneCell();
  }
  else
  {
    mouse.turnRight90();
  }
  delay(300);
}
