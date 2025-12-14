#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS1.h>
#include <ArduinoQueue.h>
// Encoder output pins
#define ENCODER_A 2
#define ENCODER_B 3

#define MOTOR1_PIN1 5
#define MOTOR1_PIN2 6
#define MOTOR1_SPEED 4

#define MOTOR2_PIN1 A3
#define MOTOR2_PIN2 A2
#define MOTOR2_SPEED A0

volatile int *Wheel1State = new int;
volatile int *Wheel2State = new int;


enum WheelDirection {
  FORWARD = 1,
  BACKWARD = -1,
};

enum RobotOrientation {
  NORTH = 1,
  EAST = 2,
  SOUTH = 3,
  WEST = 4
};




class Wheel {
public:
  int encoderPin;
  int motorPin1;
  int motorPin2;
  int motorSpeed;
  WheelDirection currentDirection;
  volatile int *wheelState;


  Wheel(int encoderPin, int motorPin1, int motorPin2, int motorSpeed, volatile int *wheelState) {
    this->encoderPin = encoderPin;
    this->motorPin1 = motorPin1;
    this->motorPin2 = motorPin2;
    this->motorSpeed = motorSpeed;
    this->wheelState = wheelState;

    pinMode(this->encoderPin, INPUT);
    pinMode(this->motorPin1, OUTPUT);
    pinMode(this->motorPin2, OUTPUT);
    pinMode(this->motorSpeed, OUTPUT);
  }

  volatile int getMotorState() {
    return *this->wheelState;
  }

  void spinWheel(WheelDirection direction, int speed) {
    if (direction == FORWARD) {
      digitalWrite(this->motorPin1, HIGH);
      digitalWrite(this->motorPin2, LOW);
      this->currentDirection = FORWARD;
    }

    if (direction == BACKWARD) {
      digitalWrite(this->motorPin1, LOW);
      digitalWrite(this->motorPin2, HIGH);
      this->currentDirection = BACKWARD;
    }
    analogWrite(this->motorSpeed, speed);
  }

  void coast() {
    digitalWrite(this->motorPin1, LOW);
    digitalWrite(this->motorPin2, LOW);
  }

  void brake() {
    digitalWrite(this->motorPin1, HIGH);
    digitalWrite(this->motorPin2, HIGH);
  }

  void onFullRotation() {
    if (this->currentDirection == FORWARD) {
      this->spinWheel(BACKWARD, 125);
    } else if (this->currentDirection == BACKWARD) {
      this->spinWheel(FORWARD, 125);
    }
  }
};

struct Position {
  int xPos;
  int yPos;
};

struct Cell {
  // Whether or not the cells are touching a wall.
  bool northWall;
  bool southWall;
  bool eastWall;
  bool westWall;
  uint8_t distance;

  // Position of the cell, only keeping here for FlodFill
  Position pos;

  // Have we visited this cell before?
  bool visited;
};

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

  Cell maze[5][5];

  Robot(Wheel L, Wheel R, RobotOrientation orientation)
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
    this->leftWheel.spinWheel(FORWARD, 255);
    this->rightWheel.coast();

    switch (this->orientation) {
      case NORTH:
        this->orientation = EAST;
      case EAST:
        this->orientation = SOUTH;
      case SOUTH:
        this->orientation = WEST;
      case WEST:
        this->orientation = NORTH;
    }
  }

  void turnRight() {
    this->rightWheel.spinWheel(FORWARD, 255);
    // TODO: Use the encoder to only move the left wheel for 90 degrees.
    this->leftWheel.coast();

    switch (this->orientation) {
      case NORTH:
        this->orientation = WEST;
      case WEST:
        this->orientation = SOUTH;
      case EAST:
        this->orientation = NORTH;
      case SOUTH:
        this->orientation = EAST;
    }
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

// Create the IMU object
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

void setup() {
  Serial.begin(9600);
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);

  if (!lsm.begin()) {
    Serial.println("Failed to detect LSM9DS1");
  }


  // Set default wheel states
  *Wheel2State = 1;
  *Wheel1State = 1;

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

  wheel1.spinWheel(FORWARD, 255);
  wheel2.spinWheel(BACKWARD, 255);
  Serial.println("Sent signal");
}

bool goingBackwards = false;
void loop() {
  Serial.print("Wheel 1 State: ");
  Serial.print(*wheel1.wheelState);
  Serial.print(" Wheel 2 State: ");
  Serial.print(*wheel2.wheelState);
  Serial.println("");

  /* sensors_event_t accel, gyro, mag, temp;

  // Read all sensors
  lsm.getEvent(&accel, &mag, &gyro, &temp);
  Serial.print("Accel: ");
  Serial.print(accel.acceleration.x);
  Serial.print(", ");
  Serial.print(accel.acceleration.y);
  Serial.print(", ");
  Serial.println(accel.acceleration.z);

  // Print gyroscope (rad/s)
  Serial.print("Gyro:  ");
  Serial.print(gyro.gyro.x);
  Serial.print(", ");
  Serial.print(gyro.gyro.y);
  Serial.print(", ");
  Serial.println(gyro.gyro.z);

  // Print magnetometer (uTesla)
  Serial.print("Mag:   ");
  Serial.print(mag.magnetic.x);
  Serial.print(", ");
  Serial.print(mag.magnetic.y);
  Serial.print(", ");
  Serial.println(mag.magnetic.z);

  Serial.println();
  delay(100); */
  delay(100);
}
