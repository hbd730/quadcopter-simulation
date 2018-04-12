"""
author: Peter Huang
email: hbd730@gmail.com
license: BSD
Please feel free to use and modify this, but keep the above information. Thanks!
"""

import numpy as np
from collections import namedtuple

# This generate a straight line along z axis for hover control
def genLine(t):
    v_max = 2.0;
    a_max = 2.0;
    yaw = 0.0;
    yawdot = 0.0;
    # 1x3 matrix x,y,z
    initial_pos = np.zeros(3)
    acc = np.zeros(3)
    vel = np.zeros(3)

    # accelarate
    if t <= v_max / a_max:
        dt = t
        acc[2] = a_max
        vel = acc * dt
        pos = 0.5 * acc * dt**2
    # constant velocity
    elif t <= 2 * v_max / a_max:
        dt = t - v_max / a_max
        vel[2] = v_max
        pos = np.array([0, 0, v_max**2 / (2 * a_max)]) + np.array([0, 0, v_max * dt])
    # decelarate
    elif t <= 3 * v_max / a_max:
        dt = t - 2 * v_max / a_max
        acc[2] = -a_max
        vel = np.array([0, 0, v_max]) + acc * dt
        pos = np.array([0, 0, 3 * v_max**2 / (2 * a_max)]) + np.array([0, 0, v_max]) * dt + 0.5 * acc * dt**2
    # hover
    else:
        pos = np.array([0, 0, 2 * v_max**2 / a_max])

    pos += initial_pos
    DesiredState = namedtuple('DesiredState', 'pos vel acc yaw yawdot')
    return DesiredState(pos, vel, acc, yaw, yawdot)
