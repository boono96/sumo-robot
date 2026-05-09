#include "Motors.h"
#include "Config.h"

namespace {
  // Channel: drive one A4950 half-bridge in slow-decay sign-magnitude.
  // speed > 0 -> in1 = PWM, in2 = LOW   (forward / motor +)
  // speed < 0 -> in1 = LOW, in2 = PWM   (reverse / motor -)
  // speed = 0 -> brake (both HIGH) for crisp stops.
  void channel(uint8_t in1, uint8_t in2, int speed) {
    if (speed > 255)  speed = 255;
    if (speed < -255) speed = -255;

    if (speed > 0) {
      analogWrite(in1, speed);
      digitalWrite(in2, LOW);
    } else if (speed < 0) {
      digitalWrite(in1, LOW);
      analogWrite(in2, -speed);
    } else {
      // brake
      digitalWrite(in1, HIGH);
      digitalWrite(in2, HIGH);
    }
  }
}

void Motors::begin() {
  pinMode(PIN_LEFT_IN1,  OUTPUT);
  pinMode(PIN_LEFT_IN2,  OUTPUT);
  pinMode(PIN_RIGHT_IN1, OUTPUT);
  pinMode(PIN_RIGHT_IN2, OUTPUT);
  coast();
}

void Motors::drive(int left, int right) {
  channel(PIN_LEFT_IN1,  PIN_LEFT_IN2,  left);
  channel(PIN_RIGHT_IN1, PIN_RIGHT_IN2, right);
}

void Motors::brake() {
  digitalWrite(PIN_LEFT_IN1,  HIGH);
  digitalWrite(PIN_LEFT_IN2,  HIGH);
  digitalWrite(PIN_RIGHT_IN1, HIGH);
  digitalWrite(PIN_RIGHT_IN2, HIGH);
}

void Motors::coast() {
  digitalWrite(PIN_LEFT_IN1,  LOW);
  digitalWrite(PIN_LEFT_IN2,  LOW);
  digitalWrite(PIN_RIGHT_IN1, LOW);
  digitalWrite(PIN_RIGHT_IN2, LOW);
}
