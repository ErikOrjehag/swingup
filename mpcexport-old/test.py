
import ctypes
import math

import matplotlib
import matplotlib.pyplot as plt

import numpy as np
from time import sleep, time

cmpc = ctypes.CDLL("/home/erik/Code/bullet/mpcexport/libacado_exported_rti.so")

N = 100
dt = 0.025
T = N * dt

class ACADOvariables(ctypes.Structure):
   _fields_ = [
       ('dummy', ctypes.c_int),
       ('x', ctypes.c_double * ( (N+1)*3) ),
       ('u', ctypes.c_double * ( N      ) ),
       ('y', ctypes.c_double * ( N * 3  ) ),
       ('yN', ctypes.c_double *  2        ),
       ('W', ctypes.c_double *   9        ),
       ('WN', ctypes.c_double *  4        ),
       ('x0', ctypes.c_double *  3        ),
       ('lbValues', ctypes.c_double *  N),
       ('ubValues', ctypes.c_double *  N),
       ('lbAValues', ctypes.c_double * N),
       ('ubAValues', ctypes.c_double * N),
    ]

acadoVars = ACADOvariables.in_dll(cmpc, "acadoVariables")




def initMPC():
    cmpc.initMPC()

def runMPC(theta, thetadot, omega, itr=1):
    acadoVars.x0[0] = theta
    acadoVars.x0[1] = thetadot
    acadoVars.x0[2] = omega
    cmpc.runMPC(itr)
    u = acadoVars.u[0]
    x = np.array(acadoVars.x)
    return u, x

if __name__ == "__main__":

    fig, ax = plt.subplots(1)
    fig.show()

    ax.set_xlim([0, T])
    ax.set_ylim([-2*np.pi, 2*np.pi])
    fig.show()

    x = np.zeros(N+1)
    xt = np.linspace(0., T, len(x))
    line = ax.plot(xt, x, 'r', animated=True)[0]
    zline = ax.plot(xt, xt*0, 'b', animated=True)[0]
    bg = fig.canvas.copy_from_bbox(ax.bbox)

    cmpc.initMPC()

    initial_theta = math.pi

    u, x = runMPC(initial_theta, 0.0, 0.0, itr=200)

    for field_name, field_type in acadoVars._fields_:
        f = getattr(acadoVars, field_name)
        print(field_name, f)
        if type(f) is not int:
            for a in f:
                print(a)
    #exit()

    while True:

        theta = x[0::3]
        thetadot = x[1::3]
        omega = x[2::3]
        u, x = runMPC(theta[1], thetadot[1], omega[1])#arm_rad_per_s, motor_rad_per_s)

        fig.canvas.restore_region(bg)
        line.set_ydata(theta)
        ax.draw_artist(line)
        ax.draw_artist(zline)
        fig.canvas.blit(ax.bbox)

        sleep(dt)