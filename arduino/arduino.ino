#include <SPI.h>
#include <mcp2515.h>

unsigned pinA = 3;
unsigned pinB = 4;

int32_t theta = 0;
int16_t omega = 0;

MCP2515 mcp2515(10);

struct RxData {
  int16_t u;
};

struct TxData {
  int32_t theta;
  int16_t omega;
};

struct TxPacket {
  uint16_t start_seq; // 0x0210
  uint8_t len;
  struct TxData tx_data;
  uint8_t checksum;
  uint16_t end_seq; // 0x0310
};


struct RxPacket {
  uint16_t start_seq; // 0x0210
  uint8_t len;
  struct RxData tx_data;
  uint8_t checksum;
  uint16_t end_seq; // 0x0310
};

struct TxPacket tx_packet;
struct RxData rx_data;

uint8_t calc_checksum(void* data, uint8_t len) {
  uint8_t checksum = 0;
  uint8_t *addr;
  for (addr = (uint8_t*)data; addr < ((uint8_t*)data + len); addr++) {
    checksum ^= *addr;
  }
  return checksum;
}

bool read_packet() {
  
  uint8_t payload_length, checksum, rx;
  uint8_t packet_size = sizeof(struct RxPacket);

  while (Serial.available() < packet_size);

  char tmp[packet_size];

  if (Serial.read() != 0x10) {
    return false;
  }

  if (Serial.read() != 0x02) {
    return false;
  }

  payload_length = Serial.read();

  if (payload_length == sizeof(struct RxData)) {
    if (Serial.readBytes((uint8_t*) &rx_data, payload_length) != payload_length) {
      return false;
    }
  } else {
    return false;
  }

  checksum = Serial.read();

  if (calc_checksum(&rx_data, payload_length) != checksum) {
    return false;
  }

  if (Serial.read() != 0x10) {
    return false;
  }

  if (Serial.read() != 0x03) {
    return false;
  }

  return true;
}

void send_packet(int32_t theta, int16_t omega) {
  tx_packet.len = sizeof(struct TxData);

  tx_packet.tx_data.theta = theta;
  tx_packet.tx_data.omega = omega;

  tx_packet.checksum = calc_checksum(&tx_packet.tx_data, tx_packet.len);

  Serial.write((char*)&tx_packet, sizeof(tx_packet));
}

void pinAChange() {
  if (digitalRead(pinA) != digitalRead(pinB)) {
    theta++;
  } else {
    theta--;
  }
}

void setup() {
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinA), pinAChange, CHANGE);

  Serial.begin(2000000);
  Serial.setTimeout(1);

  tx_packet.start_seq = 0x0210;
  tx_packet.end_seq = 0x0310;

  while (!Serial);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
}

void loop() {
  if (!read_packet()) return;
   
  int16_t iqControl = rx_data.u;

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

  noInterrupts();
  int32_t th = theta;
  interrupts();

  /*Serial.print(-1000);
  Serial.print(" ");
  Serial.print(1000);
  Serial.print(" ");
  Serial.print(0);
  Serial.print(" ");
  Serial.print(omega);
  Serial.print(" ");
  Serial.print(th);
  Serial.println();*/

  send_packet(th, omega);
}
