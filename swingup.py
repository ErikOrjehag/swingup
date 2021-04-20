
from com import send_packet, read_packet
import serial
import time
import mpcexport.test as mpc
import numpy as np
from matplotlib import pyplot as plt

def main():
    arduino = serial.Serial('/dev/ttyUSB0', 2000000, timeout=.01)
    time.sleep(3)

    mpc.initMPC()
    mpc.runMPC(np.pi, 0, 0, itr=200)

    fig, ax = plt.subplots(1)
    fig.canvas.draw()
    ax.set_xlim([0, mpc.T])
    ax.set_ylim([-2*np.pi, 2*np.pi])
    fig.show()

    ctrl = 0
    x = np.zeros(mpc.N+1)
    xt = np.linspace(0., mpc.T, len(x))
    line = ax.plot(xt, x, 'r', animated=True)[0]
    bg = fig.canvas.copy_from_bbox(ax.bbox)

    while True:
        t = time.time()
        
        send_packet(arduino, '<h', [ctrl])

        payload = read_packet(arduino, '<ih')
        if payload is None:
            print('none')
            continue

        theta, omega = payload

        u, x = mpc.runMPC(np.pi, 0, 0, itr=1)
        th = x[0::3]

        fig.canvas.restore_region(bg)
        line.set_ydata(th)
        ax.draw_artist(line)
        fig.canvas.blit(ax.bbox)

        dt = time.time() - t
        if dt > mpc.dt:
            print('Can not keep up')

if __name__ ==  '__main__':
    main()