
from com import send_packet, read_packet
from stats import nm_to_ctr, dps_to_radps
import serial
import time
import mpcexport.test as mpc
import numpy as np
from matplotlib import pyplot as plt

def main():
    #CPR = 8192
    CPR = 8192/2

    arduino = serial.Serial('/dev/ttyUSB0', 2000000, timeout=.01)
    time.sleep(3)
    send_packet(arduino, '<h', [0])
    print("Stop")
    time.sleep(2)

    mpc.initMPC()
    mpc.runMPC(np.pi, 0, 0, itr=1)

    fig, ax = plt.subplots(1)
    fig.canvas.draw()
    ax.set_xlim([0, mpc.T])
    ax.set_ylim([-2*np.pi, 2*np.pi])
    fig.show()

    ctrl = nm_to_ctr(0)
    th = [0] * (mpc.N+1)
    xt = np.linspace(0., mpc.T, len(th))

    thetat = [0] * (mpc.N+1)
    thetadott = [0] * (mpc.N+1)

    line_th = ax.plot(xt, th, 'r', animated=True)[0]
    line_theta = ax.plot(xt, thetat, 'g', animated=True)[0]
    line_thetadot = ax.plot(xt, thetadott, 'b', animated=True)[0]

    bg = fig.canvas.copy_from_bbox(ax.bbox)

    th0 = 0
    th1 = 0
    th2 = 0
    t0 = 0
    t1 = 0
    t2 = 0

    while True:
        t = time.time()
        
        send_packet(arduino, '<h', [ctrl])

        payload = read_packet(arduino, '<iiiHHh')
        if payload is None:
            print('none')
            continue

        """
        theta0, theta1, theta2, dt1, dt2, omega = payload
        if dt1 == 0 or dt2 == 0:
            continue

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
        """
        _, _, theta2, _, _, omega = payload
        omega = dps_to_radps(omega)
        theta2 = (theta2 / CPR) * 2.0 * np.pi + np.pi
        th0 = th1
        th1 = th2
        th2 = theta2
        t0 = t1
        t1 = t2
        t2 = time.time()
        if t0 == 0:
            continue
        dt1 = (t1-t0)
        dt2 = (t2-t1)
        p = np.polyfit([-dt1, 0, dt2], [th0, th1, th2], 2)
        theta = p[-1]    #   p[0]*0^2 +   p[1]*0^1 + p[2]
        thetadot = p[-2] # 2*p[0]*0^1 + 1*p[1]

        u, x = mpc.runMPC(theta, thetadot, omega, itr=1)
        ctrl = nm_to_ctr(u)

        th = x[0::3]

        thetat = [theta] + thetat[:-1]
        thetadott = [thetadot] + thetadott[:-1]

        fig.canvas.restore_region(bg)
        line_th.set_ydata(th)
        line_theta.set_ydata(thetat)
        line_thetadot.set_ydata(thetadott)
        ax.draw_artist(line_th)
        ax.draw_artist(line_theta)
        ax.draw_artist(line_thetadot)
        fig.canvas.blit(ax.bbox)

        dt = time.time() - t
        if dt > mpc.dt:
            print('Can not keep up')

if __name__ ==  '__main__':
    main()
