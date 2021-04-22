
from com import send_packet, read_packet
import serial
import time
import numpy as np

def nm_to_ctr(nm):
    NM_PER_AMP = 0.12
    amp = nm / NM_PER_AMP
    ctrl = round(2000.0 * amp/32.0)
    return ctrl

def dps_to_radps(dps):
    radps = (dps / 360.0) * 2.0*np.pi
    return radps

def dps_to_rpm(dps):
    rpm = (dps / 360.0) * 60.0
    return rpm


def main():
    arduino = serial.Serial('/dev/ttyUSB0', 2000000, timeout=.01)
    time.sleep(3)

    #torque = 0.06/4.96 = 0.01209677419

    u = 0.06 # N/M
    ctrl = nm_to_ctr(u)

    t0 = time.time()

    x = []

    while time.time() - t0 < 10:
        
        send_packet(arduino, '<h', [ctrl])

        payload = read_packet(arduino, '<iiiHHh')
        if payload is None:
            print('none')
            continue

        theta0, theta1, theta2, dt1, dt2, omega = payload

        radps = dps_to_radps(omega)

        print(theta0, theta1, theta2, dt1, dt2, omega)

        x.append([time.time() - t0, radps])
    
    send_packet(arduino, '<h', [0])

    with open('stats.csv', 'w') as f:
        f.write('time\tradps\n')
        for t, o in x:
            f.write(f'{t:.5f}\t{o:.5f}\n'.replace('.', ','))

if __name__ ==  '__main__':
    main()