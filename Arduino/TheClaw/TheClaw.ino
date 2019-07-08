#include <Stepper.h>
#include <Servo.h>

// Helper structs and unions
struct cmd_t {
  int motor;
  uint32_t steps;
};

struct stepper_t {
  int motor;
  float currentAngle;
  float targetAngle;
  int pins[4];
  int pinNum;
};

union ArrayToInteger {
 byte arr[4];
 uint32_t integer;
};

// Initialize buffers
const uint8_t numChars = 32;
char receivedChars[numChars];

byte recvNdx = 0;
boolean recvInProgress = false;
boolean newData = false;

const uint8_t cmdBufferCapacity = 10;
int cmdBufferLen = 0;
int cmdBufferHead = 0;
cmd_t cmdBuffer[cmdBufferCapacity];

// Initialize I/O
const int stepsPerRevolution = 2048;
const int stepSize = 128;
const int speedNum = 15;
const int delayNum = 1000;
const int MAX_STEP = 1024;

const int MOTOR_BASE = 0;
const int MOTOR_UPPER = 1;
const int MOTOR_FORE = 2;

const int PIN_MANUAL = 12;
const int PINS_LEFT[] = { A0, A2, A4 };
const int PINS_RIGHT[] = { A1, A3, A5 };
const int PIN_GRIPPER = 2;

const int PINS_BASE[2] = { 4, 5 };
const int PINS_UPPER[2] = { 6, 7 };
const int PINS_FORE[4] = { 8, 9, 10, 11 };

stepper_t baseStepper = { MOTOR_BASE, 90, 90, { 4, 5 }, 2 };
stepper_t upperStepper = { MOTOR_UPPER, 90, 90, { 6, 7 }, 2 };
stepper_t foreStepper = { MOTOR_FORE, 90, 90, { 8, 9, 10, 11 }, 4 };

Stepper base(stepsPerRevolution, baseStepper.pins[0], baseStepper.pins[1]);
Stepper upper_arm(stepsPerRevolution, upperStepper.pins[0], upperStepper.pins[1]);
Stepper fore_arm(stepsPerRevolution, foreStepper.pins[0], foreStepper.pins[1], foreStepper.pins[2], foreStepper.pins[3]);
Servo gripperServo;

// State
stepper_t steppers[3];
Stepper motors[] = { base, upper_arm, fore_arm };

int closeGripper;
int useManual;

void setup() {
    Serial.begin(9600);

    // pins
    pinMode(PIN_GRIPPER, INPUT_PULLUP);
    pinMode(PIN_MANUAL, INPUT_PULLUP);
    pinMode(13, OUTPUT);
    for (int i = 0; i < 3; i++) {
      pinMode(PINS_LEFT[i], INPUT_PULLUP);
      pinMode(PINS_RIGHT[i], INPUT_PULLUP);
    }

    // motors
    base.setSpeed(speedNum);
    upper_arm.setSpeed(speedNum);
    fore_arm.setSpeed(speedNum);
    
    steppers[0] = baseStepper;
    motors[MOTOR_BASE] = base;
    
    steppers[1] = upperStepper;
    motors[MOTOR_UPPER] = upper_arm;
    
    steppers[2] = foreStepper;
    motors[MOTOR_FORE] = fore_arm;

    gripperServo.attach(3 );
    
    // state
    useManual = false;
    closeGripper = false;
}

void loop() {
    int manualIn = !digitalRead(PIN_MANUAL);
    if (!manualIn && manualIn != useManual) {
      // reset serial parsing
      newData = false;
      recvInProgress = false;
      recvNdx = 0;
    }
    useManual = manualIn;
    digitalWrite(13, useManual);
    if (useManual) {
      recvManual();
    } else {
      recvWithStartEndMarkers();
      parseData();
    }
    moveStepper();
    moveGripper();
}

void moveGripper() {
  if (closeGripper) {
    gripperServo.write(90);
  } else {
    gripperServo.write(160);
  }
}

void moveStepper() {
    if (cmdBufferLen > 0) {
      cmd_t cmd = removeCommand();
      Serial.println((cmd.motor * 10000) + cmd.steps);
      if (cmd.motor >= 0 && cmd.motor < 3) {
        steppers[cmd.motor].targetAngle = ((float)cmd.steps / MAX_STEP) * 360;
        Serial.println(steppers[cmd.motor].targetAngle);
      }
    }

    for (int i = 0; i < 3; i++) {
      stepper_t s = steppers[i];
      Stepper m = motors[s.motor];
      float delta = (s.targetAngle - s.currentAngle);
      int dir = signum(delta);
      if (delta != 0 && abs(delta) > 1) {
        int steps = min(max(abs((delta / 360) * stepsPerRevolution), 1), stepSize) * dir;
        m.step(steps);
        Serial.println(s.targetAngle);
        Serial.println(s.currentAngle);
        Serial.println(steps);
        float change = steps * (360.0 / stepsPerRevolution);
        steppers[i].currentAngle += change;
        Serial.println(change);
      } else if (s.pinNum == 4) {
        for (int j = 0; j < 4; j++) {
          digitalWrite(s.pins[j], LOW);
        }
      }
    }
}

int signum(float n) {
  if (n > 0) {
    return 1;
  } else if (n < 0) {
    return -1;
  }
  return 0;
}

// Parse logic
void recvWithStartEndMarkers() {
    char startMarker = '<';
    char endMarker = '>';
    char rc;
    
    while (Serial.available() > 0 && newData == false && !useManual) {
        rc = Serial.read();
        
        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[recvNdx] = rc;
                recvNdx++;
                if (recvNdx >= numChars) {
                    recvNdx = numChars - 1;
                }
            }
            else {
                receivedChars[recvNdx] = '\0'; // terminate the string
                recvInProgress = false;
                recvNdx = 0;
                newData = true;
            }
        }
        else if (rc == startMarker) {
            recvInProgress = true;
            recvNdx = 0;
        }
    }
}

void parseData() {
    if (newData == true) {
        // split the data into its parts:
        // [m][,][b1][b2][b3][b4][\0]
        
        int motor = receivedChars[0];     // convert this part to an integer

        ArrayToInteger converter; //Create a converter
        for(int i = 0; i < 4; i++){
           converter.arr[i] = receivedChars[i + 2];
        }
        uint32_t numSteps = converter.integer;     // convert this part to an integer

        cmd_t newCmd = { motor, numSteps };
        addCommand(newCmd);
        
        newData = false;
    }
}

// Manual logic
void recvManual() {
  for (int i = 0; i < 3; i++) {
    int inL = !digitalRead(PINS_LEFT[i]);
    int inR = !digitalRead(PINS_RIGHT[i]);
    int dir = inR - inL;

    if (dir != 0) {
      steppers[i].targetAngle += 5 * dir;
      steppers[i].targetAngle = min(max(steppers[i].targetAngle, 0), 360);
    }
  }
  closeGripper = !digitalRead(PIN_GRIPPER);
}

// Buffer logic
void addCommand(cmd_t cmd) {
  // remove oldest input to make room for new input
  if (cmdBufferLen == cmdBufferCapacity) {
    removeCommand();
  }
  int tail = getBufferIndex(cmdBufferLen);
  cmdBuffer[tail] = cmd;
  cmdBufferLen += 1;
}

cmd_t removeCommand() {
  if (cmdBufferLen > 0) {
    cmd_t val = cmdBuffer[cmdBufferHead];
    cmdBufferHead = getBufferIndex(cmdBufferHead + 1);
    cmdBufferLen -= 1;
    return val;
  }
}

int getBufferIndex(int n) {
  int i = cmdBufferHead + n;
  while (i < 0) {
    cmdBufferHead += cmdBufferLen;
  }
  return i % cmdBufferLen;
}
  
