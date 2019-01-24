# Implementation of an LQR controller for a linearized model
# of a quadrotor

import numpy as np
import model.params as params
from math import sin, cos

# Z dynamics LQR Gains and input matrices
K_z = np.matrix([10.0, 2.04709])
Nu_z = np.matrix([0.0])
Nx_z = np.matrix([[1.0],[0.0]])

# X dynamics LQR Gains and input matrices
K_x = np.matrix([10., 4.5310, 9.5797, 1.02747])
Nu_x = np.matrix([0.0])
Nx_x = np.matrix([[1.0],[0.0],[0.0],[0.0]])

# Y dynamics LQR Gains and input matrices
K_y = np.matrix([-10., -4.5289, 9.5705, 1.0254])
Nu_y = np.matrix([0.0])
Nx_y = np.matrix([[1.0],[0.0],[0.0],[0.0]])

# Yaw dynamics LQR Gains and input matrices
K_yaw = np.matrix([10.0, 1.1421569])
Nu_yaw = np.matrix([0.0])
Nx_yaw = np.matrix([[1.0],[0.0]])

def run(quad, des_state):

    # get drone state
    x, y, z = quad.position()
    x_dot, y_dot, z_dot = quad.velocity()
    phi, theta, psi = quad.attitude()
    p, q, r = quad.omega()

    # get desired state \
    # This controller can control position and Yaw
    des_x, des_y, des_z = des_state.pos
    des_psi = des_state.yaw
 


    # For the LQR controller, the input to the controller are the states -x-,
    # and reference values -r-, and the output -u- (inputs to the drone) 
    # are the Thrust and desired moments, (F,M) that should be applied to 
    # the drone
    #
    # In general    u = Nu*r - K(x -Nx*r) = -K*x + (Nu + K*Nx)*r
    # where K are the LQR gains, Nu and Nx are reference input matrices.
    # See more at p.493 "Feedback Control of Dynamic Systems, Franklink G,
    # Powell, J. Emami-Naeini, Abbas"
    #
    # This LQR controller was designed assuming a linearized model of the drone.



    # For Z dynamics, the only coupled variables are the z and z_dot
    # so make an state vector for this dynamics
    state = np.matrix([[z],[z_dot]])
    # Calculating thrust, note it is identical to above equation for u
    F = -K_z*state +(Nu_z + K_z*Nx_z)*des_z

    # For X  dynamics, the only coupled variables are the x, x_dot, theta and q
    # so make an state vector for this dynamics
    state = np.matrix([[x],[x_dot],[theta],[q]])
    # Calculating torque, note it is identical to above equation for u
    L = -K_x*state +(Nu_x + K_x*Nx_x)*des_x

    # For Y  dynamics, the only coupled variables are the y, y_dot, phi and p
    # so make an state vector for this dynamics
    state = np.matrix([[y],[y_dot],[phi],[p]])
    # Calculating torque, note it is identical to above equation for u
    O = -K_y*state +(Nu_y + K_y*Nx_y)*des_y

    # For Yaw dynamics, the only coupled variables are the psi and r
    # so make an state vector for this dynamics
    state = np.matrix([[psi],[r]])
    # Calculating thrust, note it is identical to above equation for u
    N = -K_yaw*state +(Nu_yaw + K_yaw*Nx_yaw)*des_psi      

    # Now create the torque vector
    M = np.array([[O.item(0)], [L.item(0)], [N.item(0)]])

    #print(F.item(0))
    #print(M)

    return F.item(0), M