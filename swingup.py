
from threading import Thread
import serial
from time import time, sleep
import mpcexport.test as mpc
import numpy as np
import math
from matplotlib import pyplot as plt

last_received = ""
new_data_available = False

arm_rad = None
arm_rad_per_s = None
motor_rad_per_s = None

def receiving(ser):
    global last_received
    global new_data_available
    global arm_rad
    global arm_rad_per_s
    global motor_rad_per_s

    t = time()
    arm_rad_prev = None

    buffer = ""
    while True:

        buffer += ser.read(ser.inWaiting()).decode("UTF-8")
        if "\n" in buffer:
            last_received, buffer = buffer.split("\n")[-2:]

            if last_received[:2] == "->":
            
                data = last_received[2:].split(",")
                try:
                    motor_rad_per_s = float(data[0])
                except:
                    motor_rad_per_s = 0.0

                try:
                    arm_rad = float(data[1])
                except:
                    arm_rad = 0.0

                if arm_rad_prev is None:
                    arm_rad_prev = arm_rad

                dt = time() - t
                arm_rad_per_s = (arm_rad - arm_rad_prev) / dt
                t = time()
                arm_rad_prev = arm_rad

                new_data_available = True

if __name__ ==  '__main__':
    
    mpc.initMPC()
    mpc.runMPC(math.pi, 0, 0, itr=200)

    fig, ax = plt.subplots(1)
    ax.set_xlim([0, mpc.T])
    ax.set_ylim([-2*np.pi, 2*np.pi])
    fig.show()

    x = np.zeros(mpc.N+1)
    xt = np.linspace(0., mpc.T, len(x))
    line = ax.plot(xt, x, 'r', animated=True)[0]
    bg = fig.canvas.copy_from_bbox(ax.bbox)

    arduino = serial.Serial('/dev/ttyUSB0', 2000000, timeout=.01)

    Thread(target=receiving, args=(arduino,)).start()

    t = time()

    while True:

        if new_data_available:
            new_data_available = False
            
            """
            print(f"last_received: {last_received}")
            print(f"motor_rad_per_s: {motor_rad_per_s:.2f}")
            print(f"arm_rad: {arm_rad:.2f}")
            print(f"arm_rad_per_s: {arm_rad_per_s:.2f}")
            print("--------------------")
            """

            t_before = time()
            u, x = mpc.runMPC(arm_rad, arm_rad_per_s, motor_rad_per_s) #arm_rad_per_s, motor_rad_per_s)
            th = x[0::3]

            fig.canvas.restore_region(bg)
            line.set_ydata(th)
            ax.draw_artist(line)
            fig.canvas.blit(ax.bbox)
            
            #u = math.sin(time())*0.15
            u *= 0.15

            arduino.write(f'{u:.3f}'.encode())
            
            t_mpc = time() - t_before

            dt = time() - t
            t = time()
            #print(f"HZ: {int(1/dt)}")

            sleep_time = max(0.0, mpc.dt - dt)
            sleep(sleep_time)