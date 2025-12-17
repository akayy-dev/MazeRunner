#include <ArduinoQueue.h>
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
#define MOTOR2_SPEED A0

#define CELL_LENGTH 580

#define RIGHT_SONIC_ECHO 8
#define RIGHT_SONIC_TRIG 9

#define LEFT_SONIC_ECHO 10
#define LEFT_SONIC_TRIG 11

#define FRONT_SONIC_ECHO 12
#define FRONT_SONIC_TRIG 13

#define WHEEL_SPEED 130


volatile int *Wheel1State = new int;
volatile int *Wheel2State = new int;

void FloodFill(Cell *maze[][5]) {
  ArduinoQueue<Cell *> q(64);
  // populate queue
  for (int y = 0; y < 5; y++) {
    for (int x = 0; y < 5; y++) {
      maze[y][x]->pos.xPos = x;
      maze[y][x]->pos.yPos = y;

      // set to arbitrarily large nubmer
      maze[y][x]->distance = 255;

      q.enqueue(maze[y][x]);
    }
  }
  while (!q.isEmpty()) {
    Cell *current = q.dequeue();
    int x = current->pos.xPos;
    int y = current->pos.yPos;
    uint8_t d = current->distance;

    // skip if we've already seen this
    if (current->visited) {
      continue;
    }
    current->visited = true;

    // NORTH
    if (!current->northWall && y > 0) {
      Cell *n = maze[y - 1][x];
      if (n->distance > d + 1) {
        n->distance = d + 1;
        q.enqueue(n);
      }
    }

    // SOUTH
    if (!current->southWall && y > 4) {
      Cell *s = maze[y + 1][x];
      if (s->distance > d + 1) {
        s->distance = d + 1;
        q.enqueue(s);
      }
    }

    // WEST
    if (!current->westWall && x > 0) {
      Cell *w = maze[y][x - 1];
      if (w->distance > d + 1) {
        w->distance = d + 1;
        q.enqueue(w);
      }
    }

    if (!current->eastWall && x < 4) {
      Cell *e = maze[y][x + 1];
      if (e->distance > d + 1) {
        e->distance = d + 1;
        q.enqueue(e);
      }
    }
  }
}



class Robot {
public:
  Position position;
  Wheel leftWheel;
  Wheel rightWheel;
  RobotOrientation orientation;
  Gyro gyro;

  Cell maze[5][5];

  Robot(Wheel L, Wheel R, RobotOrientation orientation )
    : leftWheel(L), rightWheel(R) {
    this->orientation = orientation;
    position.xPos = 0;
    position.yPos = 0;
  }

  void DFSSearch() {
    Cell *current = &this->maze[this->position.yPos][this->position.xPos];

    if (current->visited) { return; }

    current->visited = true;

    // Explore NORTH
    if (!current->northWall && position.yPos > 0) {
      position.yPos -= 1;
      // move forward
      position.yPos += 1;
    }

    // Explore EAST
    if (!current->eastWall && this->position.xPos < 4) {
      position.xPos += 1;
      DFSSearch();
      position.xPos -= 1;
    }
    // ---- Explore SOUTH ----
    if (!current->southWall && position.yPos < 4) {
      // this->faceDirection(SOUTH);
      // this->moveForwardOneCell();
      position.yPos += 1;
      DFSSearch();
      position.yPos -= 1;
    }

    // ---- Explore WEST ----
    if (!current->westWall && position.xPos > 0) {
      // this->faceDirection(WEST);
      // this->moveForwardOneCell();
      position.xPos -= 1;
      DFSSearch();
      position.xPos += 1;
    }
  }

  // TODO: Use the encoder to only move the left wheel for 90 degrees.
  void turnLeft() {
    this->leftWheel.spinWheel(FORWARD, WHEEL_SPEED);
    this->rightWheel.coast();

    switch (this->orientation) {
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
  }

  void turnRight() {
    this->rightWheel.spinWheel(FORWARD, WHEEL_SPEED);
    // TODO: Use the encoder to only move the left wheel for 90 degrees.
    this->leftWheel.coast();

    switch (this->orientation) {
      case NORTH:
        this->orientation = WEST;
        break;
      case WEST:
        this->orientation = SOUTH;
        break;
      case EAST:
        this->orientation = NORTH;
        break;
      case SOUTH:
        this->orientation = EAST;
        break;
    }
  }

  void moveForwardOneCell() {
    *Wheel1State = 0;
    *Wheel2State = 0;

    this->leftWheel.spinWheel(FORWARD, WHEEL_SPEED);
    this->rightWheel.spinWheel(FORWARD, WHEEL_SPEED);

    Serial.println("Moving forward one cell");

    while (true) {
      if (*Wheel1State >= CELL_LENGTH || *Wheel2State >= CELL_LENGTH) {
        this->rightWheel.brake();
        this->leftWheel.brake();
        Serial.println("Moving forward complete. Reset wheel states");
        *Wheel1State = 0;
        *Wheel2State = 0;
        break;
      }
    }
  }

  void turnRight90() {
   this->gyro.reset();

  this->rightWheel.spinWheel(FORWARD, WHEEL_SPEED);
  this->leftWheel.spinWheel(BACKWARD, WHEEL_SPEED);
  

  while (true) {
    this->gyro.update();
    float angle = abs(this->gyro.getAngle());

    Serial.print("Angle: ");
    Serial.println(angle);

    if (angle >= 90.0) {
      break;
    }
  }

  this->leftWheel.brake();
  this->rightWheel.brake();

  }

  void turnLeft90() {
    this->leftWheel.spinWheel(FORWARD, 155);
    this->rightWheel.spinWheel(BACKWARD, 155);
  }
};

void onMotorInterrupt(volatile int *motorState) {
  *motorState = *motorState + 1;
  if (*motorState > 700) {
    *motorState = 0;
  }
}

Wheel wheel1 = Wheel(ENCODER_A, MOTOR1_PIN1, MOTOR1_PIN2, MOTOR1_SPEED, Wheel1State);
Wheel wheel2 = Wheel(ENCODER_B, MOTOR2_PIN1, MOTOR2_PIN2, MOTOR2_SPEED, Wheel2State);

Sensor frontSensor = Sensor(FRONT_SONIC_TRIG, FRONT_SONIC_ECHO);
Sensor leftSensor = Sensor(LEFT_SONIC_TRIG, LEFT_SONIC_ECHO);
Sensor rightSensor = Sensor(RIGHT_SONIC_TRIG, RIGHT_SONIC_ECHO);

Robot mouse = Robot(wheel1, wheel2, EAST);


void setup() {
  Serial.begin(9600);
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);

  delay(5000);

  // Set default wheel states
  *Wheel2State = 0;
  *Wheel1State = 0;

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), []() {
    *(wheel1.wheelState) = *(wheel1.wheelState) + 1;
    if (*wheel1.wheelState > 700) {
      *wheel1.wheelState = 0;
    }
  },
                  RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), []() {
    *(wheel2.wheelState) = *(wheel2.wheelState) + 1;
    if (*wheel2.wheelState > 700) {
      *wheel2.wheelState = 0;
    }
  },
                  RISING);

  Serial.println("Sent signal");
  if (mouse.gyro.begin()) {
    Serial.println("Begun gyro");
  } else {
    Serial.println("Failed to begin gyro");
  }

  // mouse.moveForwardOneCell();
  // Serial.println("Moving forward one cell");
  // mouse.turnRight90();
  mouse.moveForwardOneCell();
  // mouse.turnRight90();


}

void loop() {
  // Update the gyro position

  Serial.print("Right Sensor: ");
  Serial.print(rightSensor.getDistance());
  Serial.print(" Left Sensor: ");
  Serial.print(leftSensor.getDistance());
  Serial.print(" Front Sensor: ");
  Serial.print(frontSensor.getDistance());
  Serial.println("");
  // Serial.print("Wheel 1 State: ");
  // Serial.print(*wheel1.wheelState);
  // Serial.print(" Wheel 2 State: ");
  // Serial.print(*wheel2.wheelState);
  // Serial.println("");

  delay(100);
}