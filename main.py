
import numpy as np
import pybullet as p
import pybullet_data
import time
import atexit
import os
import mpcexport.test as mpc
from matplotlib import pyplot as plt

def read_control_file():
    u = []
    f = open("u.txt", "r")
    for l in f:
        u.append(float(l.split()[2]))
    f.close()
    print(", ".join([f"{uu:.2f}" for uu in u]))
    return u

def get_control(u, t):
    dt = 0.05
    i = int((t-1) // dt)
    if i < 0 or i >= len(u):
        return 0.0
    else:
        return u[i]

def initMPC():
    mpc.cmpc.initMPC()
    #mpc.acadoVars.x0[0] = np.pi
    #mpc.acadoVars.x0[1] = 0.0
    #mpc.acadoVars.x0[2] = 0.0
    #mpc.cmpc.runMPC(200)

def runMPC(theta, thetadot, omega):
    mpc.acadoVars.x0[0] = theta
    mpc.acadoVars.x0[1] = thetadot
    mpc.acadoVars.x0[2] = omega
    mpc.cmpc.runMPC(1)
    u = mpc.acadoVars.u[0]
    x = np.array(mpc.acadoVars.x)[0::3]
    return u, x

def main():

    fig, ax = plt.subplots(1)
    fig.canvas.draw()
    fig.show()

    ax.set_xlim([0, 1])
    ax.set_ylim([-2*np.pi, 2*np.pi])
    fig.show()

    x = np.zeros(41)
    xt = np.linspace(0., 2., len(x))
    line = ax.plot(xt, x, 'r', animated=True)[0]
    bg = fig.canvas.copy_from_bbox(ax.bbox)

    initMPC()

    GRAVITY = -9.82
    atexit.register(p.disconnect)
    try:
        p.disconnect()
    except Exception:
        pass

    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.resetDebugVisualizerCamera(
        cameraDistance=0.6,
        cameraYaw=-170,
        cameraPitch=-20,
        cameraTargetPosition=[0, 0, 0.5])
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
    p.setRealTimeSimulation(0)
    p.setPhysicsEngineParameter(numSubSteps=4)
    p.setPhysicsEngineParameter(numSolverIterations=10)

    p.setGravity(0, 0, GRAVITY)

    prev_ts = time.time()
    target_dt = None
    min_dt = 1./240
    dt = min_dt if target_dt is None else target_dt

    plane_id = p.loadURDF("plane.urdf")
    robot_id = p.loadURDF(
        "wheel_robot.urdf", 
        [0, 0, 0.5], 
        p.getQuaternionFromEuler([0, 0, 0]),
        useFixedBase=True)
    motor_id = 1
    joint_ids = list(range(p.getNumJoints(robot_id)))

    p.setJointMotorControlArray(
        robot_id,
        [j for j in joint_ids],
        p.VELOCITY_CONTROL,
        forces=[0.0 for j in joint_ids]
    )

    setpoint_torque = [0.0 for j in joint_ids]
    feedback_position = [0.0 for j in joint_ids]
    feedback_velocity = [0.0 for j in joint_ids]

    t = 0.0

    while True:

        # Get feedback
        feedback_position, feedback_velocity, _, _ = \
            zip(*p.getJointStates(robot_id, joint_ids))

        theta = feedback_position[0] + np.pi
        thetadot = feedback_velocity[0]
        omega = feedback_velocity[1]

        #print(f"theta: {theta:.2f}, thetadot: {thetadot:.2f}, omega: {omega:.2f}")

        # Calculate control signal
        u, x = runMPC(theta, thetadot, omega)
        
        t0 = time.time()
        fig.canvas.restore_region(bg)
        line.set_ydata(x)
        ax.draw_artist(line)
        fig.canvas.blit(ax.bbox)
        print(f"mpc time: {(time.time()-t0)*1e3:.3f} ms")
        if t > 3.0:
            setpoint_torque[1] = u

        # Set control signal
        p.setJointMotorControlArray(
            robot_id, joint_ids, p.TORQUE_CONTROL, forces=setpoint_torque)

        # Step simulation
        elapsed = time.time() - prev_ts
        tdt = min_dt if target_dt is None else target_dt
        sleep_time = tdt - elapsed
        if sleep_time > 1e-9:
            time.sleep(sleep_time)
        now_ts = time.time()
        dt = now_ts - prev_ts
        prev_ts = now_ts
        #print(f"dt: {dt}")
        p.setPhysicsEngineParameter(fixedTimeStep=dt)
        p.stepSimulation()
        t += dt






if __name__ == "__main__":
    main()