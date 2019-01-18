"""
author: Peter Huang, Antonio Cuni
email: hbd730@gmail.com, anto.cuni@gmail.com
license: BSD
Please feel free to use and modify this, but keep the above information. Thanks!
"""

from quadPlot import plot_quad_3d
import controller
import trajGen
import trajGen3D
from model.quadcopter import Quadcopter
import numpy as np

animation_frequency = 50
control_frequency = 200 # Hz for attitude control loop
control_iterations = control_frequency / animation_frequency
dt = 1.0 / control_frequency
time = [0.0]

def attitudeControl(quad, time, waypoints, coeff_x, coeff_y, coeff_z):
    desired_state = trajGen3D.generate_trajectory(time[0], 1.2, waypoints, coeff_x, coeff_y, coeff_z)
    F, M = controller.run(quad, desired_state)
    quad.update(dt, F, M)
    time[0] += dt

def main():
    pos = (0.5,0,0)
    attitude = (0,0,0)
    quadcopter = Quadcopter(pos, attitude)
    waypoints = trajGen3D.get_helix_waypoints(0, 9)
    (coeff_x, coeff_y, coeff_z) = trajGen3D.get_MST_coefficients(waypoints)

    def control_loop(i):
        for _ in range(control_iterations):
            attitudeControl(quadcopter, time, waypoints, coeff_x, coeff_y, coeff_z)
        return quadcopter.world_frame()

    plot_quad_3d(waypoints, control_loop)

if __name__ == "__main__":
    main()
