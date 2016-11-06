"""
author: Peter Huang
email: hbd730@gmail.com
license: BSD
Please feel free to use and modify this, but keep the above information. Thanks!
"""

import quadPlot as plt
import controller
import trajGen
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

def attitudeControl(quad, time):
    desired_state = trajGen.genLine(time[0])
    F, M = controller.run(quad, desired_state)
    quad.update(dt, F, M)
    time[0] += dt

def main():
    pos = (0,0,0)
    attitude = (0,0,np.pi/2)
    quadcopter = Quadcopter(pos, attitude)
    sched = scheduler.Scheduler()
    sched.add_task(attitudeControl, dt, (quadcopter,time))
    kEvent_Render = sched.add_event(render, (quadcopter,))
    plt.plot_quad_3d((sched, kEvent_Render))
    try:
        while True:
            thread_time.sleep(5)
    except KeyboardInterrupt:
        print ("attempting to close threads.")
        sched.stop()
        print ("terminated.")

if __name__ == "__main__":
    main()

