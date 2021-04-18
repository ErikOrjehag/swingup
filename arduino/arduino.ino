#include <SPI.h>
#include <mcp2515.h>


unsigned pinA = 3;
unsigned pinB = 4;

int prevCounter = 0;
int counter = 0;
int omega = 0;
double prevTime = micros() / 1000000.0;

MCP2515 mcp2515(10);

void pinAChange() {
  if (digitalRead(pinA) != digitalRead(pinB)) {
    counter++;
  } else {
    counter--;
  }
}

void setup() {
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinA), pinAChange, CHANGE);

  while (!Serial);
  Serial.begin(115200);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
}

void loop() {

  noInterrupts();
  int count = counter;
  interrupts();
  
  int16_t iqControl = 40*sin(0.001f*millis());

  struct can_frame frame;
  frame.can_id = 0x140 + 1;
  frame.can_dlc = 8;
  frame.data[0] = 0xA1;
  frame.data[4] = *(uint8_t*)(&iqControl);
  frame.data[5] = *((uint8_t*)(&iqControl)+1);

  mcp2515.sendMessage(&frame);

  struct can_frame rec;

  if (mcp2515.readMessage(&rec) == MCP2515::ERROR_OK) {

    if (rec.data[0] == 0xA1) {
      omega = (rec.data[5] << 8) | rec.data[4];
    }

    /*Serial.print(rec.can_id, HEX); // print ID
    Serial.print(" "); 
    Serial.print(rec.can_dlc, HEX); // print DLC
    Serial.print(" ");
    for (int i = 0; i<rec.can_dlc; i++)  {  // print the data
      Serial.print(rec.data[i],HEX);
      Serial.print(" ");
    }
    Serial.println();*/ 
  }

  double nowTime = micros() / 1000000.0;
  double thetadot = (count - prevCounter) / (nowTime - prevTime);
  prevTime = nowTime;
  prevCounter = count;

  Serial.print(-1000);
  Serial.print(" ");
  Serial.print(1000);
  Serial.print(" ");
  Serial.print(0);
  Serial.print(" ");
  Serial.print(omega);
  Serial.print(" ");
  Serial.print(thetadot);
  Serial.print(" ");
  Serial.println(count);
  //delay(10);
}
