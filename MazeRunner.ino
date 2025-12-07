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

void onMotorInterrupt(int *motorState) {
  *motorState = *motorState + 1;
  if (*motorState > 700) {
    *motorState = 0;
  }
}

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
      analogWrite(this->motorSpeed, speed);
    }
  }
};


Wheel wheel1 = Wheel(ENCODER_A, MOTOR1_PIN1, MOTOR2_PIN2, MOTOR1_SPEED, Wheel1State);
Wheel wheel2 = Wheel(ENCODER_B, MOTOR1_PIN1, MOTOR2_PIN2, MOTOR2_SPEED, Wheel2State);

void setup() {
  Serial.begin(9600);
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);



  // Set default wheel states
  *Wheel2State = 1;
  *Wheel1State = 1;

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), []() {
    onMotorInterrupt(Wheel1State);
  },
                  RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), []() {
    onMotorInterrupt(Wheel2State);
  },
                  RISING);

  wheel1.spinWheel(FORWARD, 255);
  wheel2.spinWheel(FORWARD, 255);
  Serial.println("Sent signal");
}

volatile bool motorADir = false;
void loop() {
  /* Serial.print("Wheel 1 State: ");
  Serial.print(*wheel1.wheelState); */
  Serial.print(" Wheel 2 State: ");
  int state = *wheel2.wheelState;
  Serial.print(state);
  Serial.println("");

  // full rotation
  if (state == 0) {
    Serial.println("Wheel has made one full revolution");
  }
}
