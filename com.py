
import serial
import struct
import time

def calc_checksum(data):
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum

def read_packet(arduino, fmt):
    if arduino.read() != b'\x10':
        return None
    if arduino.read() != b'\x02':
        return None
    payload_len = arduino.read()[0]
    if payload_len != struct.calcsize(fmt):
        return None
    payload = arduino.read(payload_len)
    checksum = arduino.read()[0]
    if checksum != calc_checksum(payload):
        return None
    if arduino.read() != b'\x10':
        return None
    if arduino.read() != b'\x03':
        return None
    unpacked = struct.unpack(fmt, payload)
    return unpacked

def send_packet(arduino, fmt, data):
    tx = b'\x10\x02'
    tx += struct.pack('<B', struct.calcsize(fmt))
    packed_data = struct.pack(fmt, *data)
    tx += packed_data
    tx += struct.pack('<B', calc_checksum(packed_data))
    tx += b'\x10\x03'
    arduino.write(tx)

def receiving(arduino):

    angles = [0, 30, 60]   # convert python int to uint8_t
    speeds = [-255,0,255]  # convert python int to int16_t
    timeout = 3

    while True:
        send_packet(arduino, '<h', [42])
        start_time = time.time()
        payload = None
        while payload is None and time.time() - start_time < 3:
            payload = read_packet(arduino, '<iiiHHh')
            if payload is None:
                print('none')
        if payload is None:
            print('timeout')
            continue
        print(payload, 1000*(time.time() - start_time))

def main():

    arduino = serial.Serial('/dev/ttyUSB0', 2000000, timeout=0.005)
    time.sleep(3)

    receiving(arduino)

if __name__ == '__main__':
    main()