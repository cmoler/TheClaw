struct cmd_t {
  int motor;
  uint32_t steps;
};

const uint8_t cmdBufferCapacity = 20;
int cmdBufferLen = 0;
int cmdBufferHead = 5;
cmd_t cmdBuffer[cmdBufferCapacity];


void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  for (int i = 0; i < 20; i++) {
    cmd_t newCmd = { i, 0 };
    addCommand(newCmd);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  while (cmdBufferLen > 0) {
    cmd_t readCmd = removeCommand();
    Serial.println(readCmd.motor);
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
    cmdBufferHead = getBufferIndex(1);
    cmdBufferLen -= 1;
    return val;
  }
}

int getBufferIndex(int n) {
  int i = cmdBufferHead + n;
  while (i < 0) {
    i += cmdBufferCapacity;
  }
  return i % cmdBufferCapacity;
}
