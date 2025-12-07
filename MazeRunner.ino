// Encoder output pins
#define ENCODER_A 2
#define ENCODER_B 3

#define MOTOR1_PIN1 4
#define MOTOR1_PIN2 5
#define MOTOR1_SPEED 10

#define MOTOR2_PIN1 6
#define MOTOR2_PIN2 7
#define MOTOR2_SPEED 11

volatile int *Wheel1State = new int;
volatile int *Wheel2State = new int;


enum Direction {
  FORWARD = 1,
  BACKWARD = -1
};

class Wheel {
public:
  int encoderPin;
  int motorPin1;
  int motorPin2;
  int motorSpeed;
  Direction currentDirection;
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

  void spinWheel(Direction direction, int speed) {
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

  void onFullRotation() {
    if (this->currentDirection == FORWARD) {
      this->spinWheel(BACKWARD, 125);
    }
    else if (this->currentDirection == BACKWARD) {
      this->spinWheel(FORWARD, 125);
    }
  }
};

void onMotorInterrupt(int *motorState, Wheel *wheel) {
  *motorState = *motorState + 1;
  if (*motorState > 700) {
    *motorState = 0;
    wheel->onFullRotation();
  }
}

Wheel wheel1 = Wheel(ENCODER_A, MOTOR1_PIN1, MOTOR1_PIN2, MOTOR1_SPEED, Wheel1State);
Wheel wheel2 = Wheel(ENCODER_B, MOTOR2_PIN1, MOTOR2_PIN2, MOTOR2_SPEED, Wheel2State);

void setup() {
  Serial.begin(9600);
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);



  // Set default wheel states
  *Wheel2State = 1;
  *Wheel1State = 1;

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), []() {
    onMotorInterrupt(Wheel1State, &wheel1);
  },
                  RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), []() {
    onMotorInterrupt(Wheel2State, &wheel2);
  },
                  RISING);

  wheel1.spinWheel(FORWARD, 255);
  wheel2.spinWheel(FORWARD, 255);
  Serial.println("Sent signal");
}

bool goingBackwards = false;
void loop() {
  /* Serial.print("Wheel 1 State: ");
  Serial.print(*wheel1.wheelState); */
  Serial.print(" Wheel 2 State: ");
  int state = *Wheel2State;
  Serial.print(state);
  Serial.println("");

  // full rotation
  // if (state == 0) {
  //   if (goingBackwards) {
  //     Serial.println("Wheel has gone backward a revolution, going forward");
  //     // wheel1.spinWheel(FORWARD, 255);
  //     // wheel2.spinWheel(FORWARD, 255);
  //     goingBackwards = false;
  //   } else {
  //     Serial.println("Wheel has gone forward a revolution, going backward");
  //     wheel1.spinWheel(BACKWARD, 255);
  //     wheel2.spinWheel(BACKWARD, 255);
  //     goingBackwards = true;
  //   }
  // }
}
