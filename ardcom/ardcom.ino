
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

void send_packet() {
  tx_packet.len = sizeof(struct TxData);

  tx_packet.tx_data.theta = 123;
  tx_packet.tx_data.omega = -42;

  tx_packet.checksum = calc_checksum(&tx_packet.tx_data, tx_packet.len);

  Serial.write((char*)&tx_packet, sizeof(tx_packet));
}

void setup() {
    Serial.begin(2000000);
    Serial.setTimeout(1);

    tx_packet.start_seq = 0x0210;
    tx_packet.end_seq = 0x0310;

    while (!Serial);
}

void loop() {
  if (read_packet()) {
    send_packet();
  }
}
