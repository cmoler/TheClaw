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
uint32_t numSteps;

boolean newData = false;
boolean newMove = false;

union ArrayToInteger {
 byte array[4];
 uint32_t integer;
};

void setup() {
    Serial.begin(9600);
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
        // split the data into its parts
        
        motor = receivedChars[0];     // convert this part to an integer

        ArrayToInteger converter; //Create a converter

        for(int i=2; i < 5; i++){
           converter.array[i - 2]=receivedChars[i];
        }

        numSteps = converter.integer;     // convert this part to an integer
        
        newMove = true;
        newData = false;
    }
}

void moveStepper(){
    if(newMove == true){
      Serial.print("move stepper motor ");
      Serial.println(motor);
      Serial.print("move stepper num steps ");
      Serial.println(numSteps);

      
      newMove = false;
    }
}
