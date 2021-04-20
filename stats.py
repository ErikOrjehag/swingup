
from com import send_packet, read_packet
import serial
import time
import numpy as np

def main():
    arduino = serial.Serial('/dev/ttyUSB0', 2000000, timeout=.01)
    time.sleep(3)

    ctrl = 30

    t0 = time.time()

    x = []

    while time.time() - t0 < 10:
        
        send_packet(arduino, '<h', [ctrl])

        payload = read_packet(arduino, '<ih')
        if payload is None:
            print('none')
            continue

        theta, omega = payload

        print(theta, omega)
        x.append([time.time() - t0, omega])
    
    send_packet(arduino, '<h', [0])

    with open('stats.csv', 'w') as f:
        for t, o in x:
            f.write(f'{t}, {o}\n')

if __name__ ==  '__main__':
    main()