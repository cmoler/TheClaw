#include <Stepper.h>

const int stepsPerRevolution = 2048;
const int speedNum = 10;
const int delayNum = 1000;

Stepper base(stepsPerRevolution, 4, 5);
Stepper upper_arm(stepsPerRevolution, 6, 7);
Stepper fore_arm(stepsPerRevolution, 8, 9, 10, 11);

const byte numChars = 32;
char receivedChars[numChars];

int motor;
int numSteps;

boolean newData = false;
boolean newMove = false;

void setup() {
    Serial.begin(9600);
    Serial.println("<3, 1200>");
}

void loop() {
    recvWithStartEndMarkers();
    parseData();
    moveStepper();
}

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
        char * strtokIndx; // this is used by strtok() as an index
  
        strtokIndx = strtok(receivedChars,",");      // get the first part - the motor to move
        motor = atoi(strtokIndx);     // convert this part to an integer

        strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
        numSteps = atoi(strtokIndx);     // convert this part to an integer
        
        newMove = true;
        newData = false;
    }
}

void moveStepper(){
    if(newMove == true){
      Serial.println(motor);
      Serial.println(numSteps);

      
      newMove = false;
    }
}
