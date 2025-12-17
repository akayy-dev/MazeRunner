#include "Wheel.h"
#include <Arduino.h>

Wheel::Wheel(int encoderPin, int motorPin1, int motorPin2, int motorSpeed, volatile int *wheelState)
{
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

volatile int Wheel::getMotorState()
{
  return *this->wheelState;
}

void Wheel::spinWheel(WheelDirection direction, int speed)
{
  if (direction == FORWARD)
  {
    digitalWrite(this->motorPin1, HIGH);
    digitalWrite(this->motorPin2, LOW);
    this->currentDirection = FORWARD;
  }

  if (direction == BACKWARD)
  {
    digitalWrite(this->motorPin1, LOW);
    digitalWrite(this->motorPin2, HIGH);
    this->currentDirection = BACKWARD;
  }
  analogWrite(this->motorSpeed, speed);
}

void Wheel::coast()
{
  digitalWrite(this->motorPin1, LOW);
  digitalWrite(this->motorPin2, LOW);
}

void Wheel::brake()
{
  digitalWrite(this->motorPin1, HIGH);
  digitalWrite(this->motorPin2, HIGH);
}

void Wheel::onFullRotation()
{
  if (this->currentDirection == FORWARD)
  {
    this->spinWheel(BACKWARD, 125);
  }
  else if (this->currentDirection == BACKWARD)
  {
    this->spinWheel(FORWARD, 125);
  }
}
