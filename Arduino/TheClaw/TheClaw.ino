#include <Stepper.h>

const int stepsPerRevolution = 2048;
const int speedNum = 10;
const int delayNum = 1000;

Stepper myStepper(stepsPerRevolution, 2, 3, 4, 5);

void setup() {
  // put your setup code here, to run once:
  myStepper.setSpeed(speedNum);

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  myStepper.step(1);
  delay(delayNum);
}
