#include "Sensor.h"
#include <Arduino.h>

Sensor::Sensor(int trigPin, int echoPin) {
  this->trigPin = trigPin;
  this->echoPin = echoPin;
	pinMode(this->trigPin, OUTPUT);
	pinMode(this->echoPin, INPUT);
}

Sensor::getDistance() {
  // Clear the trig Pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
	digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
	
  // Calculating the distance
  distance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
	return distance;
}
