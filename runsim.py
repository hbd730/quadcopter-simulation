"""
author: Peter Huang
email: hbd730@gmail.com
license: BSD
Please feel free to use and modify this, but keep the above information. Thanks!
"""

import quadPlot as plt
import controller
import trajGen
import trajGen3D
import scheduler
from model.quadcopter import Quadcopter
import numpy as np
import time as thread_time

control_frequency = 200 # Hz for attitude control loop
dt = 1.0 / control_frequency
time = [0.0]

def render(quad):
    frame = quad.world_frame()
    plt.set_frame(frame)

def attitudeControl(quad, time, waypoints, coeff_x, coeff_y, coeff_z):
    desired_state = trajGen3D.generate_trajectory(time[0], 1.2, waypoints, coeff_x, coeff_y, coeff_z)
    F, M = controller.run_LQR(quad, desired_state)
    quad.update(dt, F, M)
    time[0] += dt

def main():
    # define initial conditions
    pos = (0.5,0,0)
    attitude = (0,0,0)
    # create new quadrotor
    quadcopter = Quadcopter(pos, attitude)

    sched = scheduler.Scheduler()
    # Generate a trajectory with 9 waypoints
    waypoints = trajGen3D.get_poly_waypoints(0, 9)
    # Get coefficients of 8 degree polinomial that represents the 
    # continuos trajectory that drone will follow in space
    (coeff_x, coeff_y, coeff_z) = trajGen3D.get_MST_coefficients(waypoints)
    # 
    sched.add_task(attitudeControl, dt, (quadcopter,time,waypoints,coeff_x,coeff_y,coeff_z))
    kEvent_Render = sched.add_event(render, (quadcopter,))

    try:
        plt.plot_quad_3d(waypoints, pos, (sched, kEvent_Render))
    except KeyboardInterrupt:
        print ("attempting to close threads.")
        sched.stop()
        print ("terminated.")

if __name__ == "__main__":
    main()

