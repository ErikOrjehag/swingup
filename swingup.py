
from com import send_packet, read_packet
from stats import nm_to_ctr, dps_to_radps
import serial
import time
import mpcexport.test as mpc
import numpy as np
from matplotlib import pyplot as plt

def main():
    CPR = 8192

    arduino = serial.Serial('/dev/ttyUSB0', 2000000, timeout=.01)
    time.sleep(3)

    mpc.initMPC()
    mpc.runMPC(np.pi, 0, 0, itr=200)

    fig, ax = plt.subplots(1)
    fig.canvas.draw()
    ax.set_xlim([0, mpc.T])
    ax.set_ylim([-2*np.pi, 2*np.pi])
    fig.show()

    ctrl = nm_to_ctr(0)
    x = np.zeros(mpc.N+1)
    xt = np.linspace(0., mpc.T, len(x))
    line = ax.plot(xt, x, 'r', animated=True)[0]
    bg = fig.canvas.copy_from_bbox(ax.bbox)

    while True:
        t = time.time()
        
        send_packet(arduino, '<h', [ctrl])

        payload = read_packet(arduino, '<iiiHHh')
        if payload is None:
            print('none')
            continue

        theta0, theta1, theta2, dt1, dt2, omega = payload

        # Calculate theta (rad) and thetadot (rad/s)
        theta0 = (theta0 / CPR) * 2.0 * np.pi
        theta1 = (theta1 / CPR) * 2.0 * np.pi
        theta2 = (theta2 / CPR) * 2.0 * np.pi
        dt1 /= 1e6
        dt2 /= 1e6
        p = np.polyfit([-dt1-dt2, -dt2, 0], [theta0, theta1, theta2], 2)
        theta = p[-1]    #   p[0]*0^2 +   p[1]*0^1 + p[2]
        thetadot = p[-2] # 2*p[0]*0^1 + 1*p[1]
        # Calculate omega (rad/s)
        omega = dps_to_radps(omega)

        #print(theta0, theta1, theta2, dt1, dt2, omega)
        #print(p)
        #print(theta, thetadot)

        u, x = mpc.runMPC(np.pi+theta, thetadot, omega, itr=1)
        ctrl = nm_to_ctr(u)
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
