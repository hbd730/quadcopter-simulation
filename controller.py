"""
author: Peter Huang
email: hbd730@gmail.com
license: BSD
Please feel free to use and modify this, but keep the above information. Thanks!
"""

import numpy as np
import model.params as params
from math import sin, cos

# PD Controller for quacopter
# Return [F, M] F is total force thrust, M is 3x1 moment matrix

# Constants
k_d_x = 30
k_p_x = 3
k_d_y = 30
k_p_y = 3
k_p_z = 1000
k_d_z = 200
k_p_phi = 160
k_d_phi = 3
k_p_theta = 160
k_d_theta = 3
k_p_psi = 80
k_d_psi = 5

# LQR Gains

def run(quad, des_state):
    x, y, z = quad.position()
    x_dot, y_dot, z_dot = quad.velocity()
    phi, theta, psi = quad.attitude()
    p, q, r = quad.omega()
    des_x, des_y, des_z = des_state.pos
    des_x_dot, des_y_dot, des_z_dot = des_state.vel
    des_x_ddot, des_y_ddot, des_z_ddot = des_state.acc
    des_psi = des_state.yaw
    des_psi_dot = des_state.yawdot
    # Commanded accelerations
    commanded_r_ddot_x = des_x_ddot + k_d_x * (des_x_dot - x_dot) + k_p_x * (des_x - x)
    commanded_r_ddot_y = des_y_ddot + k_d_y * (des_y_dot - y_dot) + k_p_y * (des_y - y)
    commanded_r_ddot_z = des_z_ddot + k_d_z * (des_z_dot - z_dot) + k_p_z * (des_z - z)

    # Thrust
    F = params.mass * (params.g + commanded_r_ddot_z)
    # Moment
    p_des = 0
    q_des = 0
    r_des = des_psi_dot
    des_phi = 1 / params.g * (commanded_r_ddot_x * sin(des_psi) - commanded_r_ddot_y * cos(des_psi))
    des_theta = 1 / params.g * (commanded_r_ddot_x * cos(des_psi) + commanded_r_ddot_y * sin(des_psi))

    M = np.array([[k_p_phi * (des_phi - phi) + k_d_phi * (p_des - p),
                   k_p_theta * (des_theta - theta) + k_d_theta * (q_des - q),
                   k_p_psi * (des_psi - psi) + k_d_psi * (r_des - r)]]).T

    print(F)
    print(M)
    return F, M

def run_LQR(quad, des_state):

    # get drone state
    x, y, z = quad.position()
    x_dot, y_dot, z_dot = quad.velocity()
    phi, theta, psi = quad.attitude()
    p, q, r = quad.omega()

    # get desired state
    des_x, des_y, des_z = des_state.pos
    des_x_dot, des_y_dot, des_z_dot = des_state.vel
    des_x_ddot, des_y_ddot, des_z_ddot = des_state.acc
    des_psi = des_state.yaw
    des_psi_dot = des_state.yawdot    


    # For the LQR controller, the input to the controller are the states -x-,
    # and reference values -r-, and the output -u- (inputs to the drone) 
    # ar the Thrust and desired moments, (F,M) that should be applied to 
    # the drone
    #
    # In general    u = Nu*r - K(x -Nx*r) = -K*x + (Nu + K*Nx)*r
    # where K are the LQR gains, Nu and Nx are reference input matrices.
    # See more at p.493 "Feedback Control of Dynamic Systems, Franklink G,
    # Powell, J. Emami-Naeini, Abbas"

    # This LQR controller was designed assuming a linearized model of the drone.

    # Z dynamics LQR Gains and input matrices
    K_z = np.matrix([10.0, 2.04709])
    Nu_z = np.matrix([0.0])
    Nx_z = np.matrix([[1.0],[0.0]])

    # For Z dynamics, the only coupled variables are the z and z_dot
    # so make an state vector for this dynamics
    state = np.matrix([[z],[z_dot]])
    # Calculating thrust, note it is identical to above equation for u
    F = -K_z*state +(Nu_z + K_z*Nx_z)*des_z

    # X dynamics LQR Gains and input matrices
    K_x = np.matrix([10.0, 10.6217, 14.762, 1.042])
    Nu_x = np.matrix([0.0])
    Nx_x = np.matrix([[1.0],[0.0],[0.0],[0.0]])

    # For X  dynamics, the only coupled variables are the x, x_dot, theta and theta_dot
    # so make an state vector for this dynamics
    state = np.matrix([[x],[x_dot],[theta],[q]])
    # Calculating torque, note it is identical to above equation for u
    L = -K_x*state +(Nu_x + K_x*Nx_x)*des_x


    # Y dynamics LQR Gains and input matrices
    K_y = np.matrix([-10.0, -10.6208, 14.7408, 1.039])
    Nu_y = np.matrix([0.0])
    Nx_y = np.matrix([[1.0],[0.0],[0.0],[0.0]])

    # For X  dynamics, the only coupled variables are the x, x_dot, theta and theta_dot
    # so make an state vector for this dynamics
    state = np.matrix([[y],[y_dot],[phi],[p]])
    # Calculating torque, note it is identical to above equation for u
    O = -K_y*state +(Nu_y + K_y*Nx_y)*des_y


    # Yaw dynamics LQR Gains and input matrices
    K_yaw = np.matrix([10.0, 1.14])
    Nu_yaw = np.matrix([0.0])
    Nx_yaw = np.matrix([[1.0],[0.0]])

    # For Yaw dynamics, the only coupled variables are the z and z_dot
    # so make an state vector for this dynamics
    state = np.matrix([[psi],[r]])
    # Calculating thrust, note it is identical to above equation for u
    N = -K_yaw*state +(Nu_yaw + K_yaw*Nx_yaw)*des_psi      

    # Now create the torque vector
    M = np.array([[O.item(0)], [L.item(0)], [N.item(0)]])

    print(F.item(0))
    print(M)

    return F.item(0), M