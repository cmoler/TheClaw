#include <Stepper.h>

// Helper structs and unions
struct cmd_t {
  int motor;
  uint32_t steps;
};

union ArrayToInteger {
 byte arr[4];
 uint32_t integer;
};

// Initialize buffers
const uint8_t numChars = 32;
char receivedChars[numChars];

boolean newData = false;

const uint8_t cmdBufferCapacity = 10;
int cmdBufferLen = 0;
int cmdBufferHead = 0;
cmd_t cmdBuffer[cmdBufferCapacity];

// Initialize I/O
const int stepsPerRevolution = 2048;
const int speedNum = 10;
const int delayNum = 1000;

Stepper base(stepsPerRevolution, 4, 5);
Stepper upper_arm(stepsPerRevolution, 6, 7);
Stepper fore_arm(stepsPerRevolution, 8, 9, 10, 11);

void setup() {
    Serial.begin(9600);
}

void loop() {
    recvWithStartEndMarkers();
    parseData();
    moveStepper();
}

void moveStepper(){
    if (cmdBufferLen > 0) {
      cmd_t cmd = removeCommand();
      Serial.println((cmd.motor * 10000) + cmd.steps);
    }
}

// Parse logic
void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
 
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();
        
        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
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
