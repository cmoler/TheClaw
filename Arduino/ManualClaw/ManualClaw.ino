#include <Stepper.h>

const int stepsPerRevolution = 2048;  // change this to fit the number of steps per revolution

const int PIN_L1 = A0;
const int PIN_R1 = A1;

const int PIN_L2 = A2;
const int PIN_R2 = A3;

const int PIN_L3 = A4;
const int PIN_R3 = A5;
// for your motor


// initialize the stepper library
Stepper myStepper1(stepsPerRevolution, 4, 5);
Stepper myStepper2(stepsPerRevolution, 6, 7);
Stepper myStepper3(stepsPerRevolution, 8, 9, 10, 11);

int stepSize = 128;
  
void setup() {
  /*pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  digitalWrite(8, HIGH);
  digitalWrite(9, LOW);*/
  Serial.begin(9600);
  pinMode(PIN_L1, INPUT_PULLUP);
  pinMode(PIN_R1, INPUT_PULLUP);
  pinMode(PIN_L2, INPUT_PULLUP);
  pinMode(PIN_R2, INPUT_PULLUP);
  pinMode(PIN_L3, INPUT_PULLUP);
  pinMode(PIN_R3, INPUT_PULLUP);
  myStepper1.setSpeed(10);
  myStepper2.setSpeed(15);
  myStepper3.setSpeed(15);
}

void loop() {
  int in1_left = !digitalRead(PIN_L1);
  int in1_right = ! digitalRead(PIN_R1);
  int in1_dir = in1_right - in1_left;

  int in2_left = !digitalRead(PIN_L2);
  int in2_right = ! digitalRead(PIN_R2);
  int in2_dir = in2_right - in2_left;

  int in3_left = !digitalRead(PIN_L3);
  int in3_right = ! digitalRead(PIN_R3);
  int in3_dir = in3_right - in3_left;

  if (in1_dir != 0) {
    myStepper1.step(stepSize * in1_dir);
  }

  if (in2_dir != 0) {
    myStepper2.step(stepSize * in2_dir); 
  }

  if (in3_dir != 0) {
    myStepper3.step(stepSize * in3_dir); 
  }
}
