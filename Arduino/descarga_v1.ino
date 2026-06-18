#include <Servo.h>

Servo myServo;
const int servoPin = 3;
const int sensorPin = 2;

void setup() {
  pinMode(sensorPin, INPUT_PULLUP);
  myServo.attach(servoPin);
  myServo.write(63); // Starts closed
}

void loop() {
  int state = digitalRead(sensorPin);

  if (state == LOW) {
    // 24V active → OPENS the lever
    myServo.write(0);
  } else {
    // 24V inactive → CLOSES the lever
    myServo.write(63);
  }

  delay(100);
}
