"""
author: Peter Huang
email: hbd730@gmail.com
license: BSD
Please feel free to use and modify this, but keep the above information. Thanks!
"""

import numpy as np
from numpy.linalg import inv
from collections import namedtuple
from numpy import linalg as LA

DesiredState = namedtuple('DesiredState', 'pos vel acc yaw yawdot')
yaw = 0.0
current_heading = np.array([1.0,0.0]) #heading vector should always be unitary vector. [0,0] vector has no heading!

def get_helix_waypoints(t, n):
    """ The function generate n helix waypoints from the given time t
        output waypoints shape is [n, 3]
    """
    waypoints_t = np.linspace(0, t, n)
    x = 0.5*np.cos(waypoints_t)
    y = 0.5*np.sin(waypoints_t)
    z = waypoints_t

    return np.stack((x, y, z), axis=-1)
    
def get_poly_waypoints(t,n):
    """ The function generate n waypoints from a polynomial at given t.
        The polynomial is 
            x = k1*t^2
            y = k1*t^3
            z = k2*t
        Output waypoints shape is [n, 3]
    """
    waypoints_t = np.linspace(0, t + 4*np.pi, n)
    k1 = 0.0
    k2 = 0.0
    #x = (k1*waypoints_t)**2
    #y = (k1*waypoints_t)**3
    #z = k2*waypoints_t
    x = k1*np.ones_like(waypoints_t)
    y = k2*np.ones_like(waypoints_t)
    z = waypoints_t


    return np.stack((x, y, z), axis=-1)

def get_leminiscata_waypoints(t,n):
    """ The function generate n waypoints from a leminiscata at given t.
        The leminiscata is 
            x = k1 * cos(wt/2)
            y = k1 * sin(wt)
            z = k2 * t
        Output waypoints shape is [n, 3]
    """
    waypoints_t = np.linspace(0, t, n)
    
    k1 = 0.5
    k2 = 0.5#
    w = 0.7
    
    x = k1*np.cos(w*waypoints_t/2.0)
    y = k1*np.sin(w*waypoints_t)
    z = k2*waypoints_t

    return np.stack((x, y, z), axis=-1)

def get_MST_coefficients(waypoints):
    # generate MST coefficients for each segment, coeff is now 1D array [64,]
    coeff_x = MST(waypoints[:,0]).transpose()[0]
    coeff_y = MST(waypoints[:,1]).transpose()[0]
    coeff_z = MST(waypoints[:,2]).transpose()[0]
    return (coeff_x, coeff_y, coeff_z)

def generate_trajectory(t, v, waypoints, coeff_x, coeff_y, coeff_z):
    """ The function takes known number of waypoints and time, then generates a
    minimum snap trajectory which goes through each waypoint. The output is
    the desired state associated with the next waypont for the time t.
    waypoints is [N,3] matrix, waypoints = [[x0,y0,z0]...[xn,yn,zn]].
    v is velocity in m/s

    The heading is computed such that 1) the drone is always pointing tangent to the
    computed trajectory (so it is computed from velocity vector) and 2) 
    is always magnitude 1. 
    """
    global yaw
    global current_heading
    yawdot = 0.0
    pos = np.zeros(3)
    acc = np.zeros(3)
    vel = np.zeros(3)

    # distance vector array, represents each segment's distance
    distance = waypoints[0:-1] - waypoints[1:]
    # T is now each segment's travel time
    T = (1.0 / v) * np.sqrt(distance[:,0]**2 + distance[:,1]**2 + distance[:,2]**2)
    # accumulated time
    S = np.zeros(len(T) + 1)
    S[1:] = np.cumsum(T)

    # find which segment current t belongs to
    t_index = np.where(t >= S)[0][-1]

    # prepare the next desired state
    if t == 0:
        pos = waypoints[0]
        t0 = get_poly_cc(8, 1, 0)

        # get X-Y plane project of velocity vector ( this vector is tangent to curve )
        v_proj = np.array([coeff_x[0:8].dot(t0), coeff_y[0:8].dot(t0)])
        if(LA.norm(v_proj) == 0.0):
            #  if velocity vector is of zero magnitude there should be no change in heading!
            pass
        else:
            current_heading = v_proj/LA.norm(v_proj)
            

    # stay hover at the last waypoint position
    elif t > S[-1]:
        pos = waypoints[-1]
    else:
        # scaled time
        scale = (t - S[t_index]) / T[t_index]
        start = 8 * t_index
        end = 8 * (t_index + 1)

        t0 = get_poly_cc(8, 0, scale)
        pos = np.array([coeff_x[start:end].dot(t0), coeff_y[start:end].dot(t0), coeff_z[start:end].dot(t0)])

        t1 = get_poly_cc(8, 1, scale)
        # chain rule applied
        vel = np.array([coeff_x[start:end].dot(t1), coeff_y[start:end].dot(t1), coeff_z[start:end].dot(t1)]) * (1.0 / T[t_index])

        t2 = get_poly_cc(8, 2, scale)
        # chain rule applied
        acc = np.array([coeff_x[start:end].dot(t2), coeff_y[start:end].dot(t2), coeff_z[start:end].dot(t2)]) * (1.0 / T[t_index]**2)

        # calculate desired yaw and yaw rate

        v_proj = np.array([vel[0], vel[1]])

        if( LA.norm(v_proj) == 0.0):
            # if velocity vector is zero, again there should be no change in heading
            next_heading = current_heading
        else:
            next_heading = v_proj/LA.norm(v_proj)

        """
        try :
            #current_heading = v_proj/LA.norm(v_proj) #* (1.0 / T[0])  #np.array([coeff_x[0:8].dot(t0), coeff_y[0:8].dot(t0)]) * (1.0 / T[0])
            next_heading = v_proj/LA.norm(v_proj)
        except ZeroDivisionError:
            # velocity vector magnitude was zero so there should be no change in heading!
            next_heading = current_heading
        """    

        # angle between current vector with the next heading vector
        # from   a * b  = |a|*|b|cos(angle)
        delta_psi = np.arccos(np.dot(current_heading, next_heading) / (LA.norm(current_heading)*LA.norm(next_heading)))
        # cross product allow us to determine rotating direction
        norm_v = np.cross(current_heading,next_heading)

        if norm_v > 0:
            yaw += delta_psi
        elif norm_v < 0:
            yaw -= delta_psi
        else:
            # normv = 0! if there is no change in yaw, do not modify it!
            pass

        # dirty hack, quadcopter's yaw range represented by quaternion is [-pi, pi]
        while yaw > np.pi:
            yaw = yaw - 2*np.pi

        # print next_heading, current_heading, "yaw", yaw*180/np.pi, 'pos', pos
        current_heading = next_heading
        #print(current_heading)
        yawdot = delta_psi / 0.005 # dt is control period
        max_yawdot = 5.0 #rad/s
        if(abs(yawdot) > max_yawdot):
            yawdot = (yawdot/abs(yawdot))*5.0 # make it 5rad/s with appropriate direction
        
        yaw = 0
        yawdot = 0
        print (" yaw: {}  yawdot: {}".format(yaw,yawdot))
    return DesiredState(pos, vel, acc, yaw, yawdot)

def get_poly_cc(n, k, t):
    """ This is a helper function to get the coeffitient of coefficient for n-th
        order polynomial with k-th derivative at time t.
    """
    assert (n > 0 and k >= 0), "order and derivative must be positive."

    cc = np.ones(n)
    D  = np.linspace(0, n-1, n)

    for i in range(n):
        for j in range(k):
            cc[i] = cc[i] * D[i]
            D[i] = D[i] - 1
            if D[i] == -1:
                D[i] = 0

    for i, c in enumerate(cc):
        cc[i] = c * np.power(t, D[i])

    return cc

# Minimum Snap Trajectory
def MST(waypoints):
    """ This function takes a list of desired waypoint i.e. [x0, x1, x2...xN] and
    time, returns a [8N,1] coeffitients matrix for the N+1 waypoints.

    1.The Problem
    Generate a full trajectory across N+1 waypoint made of N polynomial line segment.
    Each segment is defined as 7 order polynomial defined as follow:
    Pi = ai_0 + ai1*t + ai2*t^2 + ai3*t^3 + ai4*t^4 + ai5*t^5 + ai6*t^6 + ai7*t^7

    Each polynomial has 8 unknown coefficients, thus we will have 8*N unknown to
    solve in total, so we need to come up with 8*N constraints.

    2.The constraints
    In general, the constraints is a set of condition which define the initial
    and final state, continuity between each piecewise function. This includes
    specifying continuity in higher derivatives of the trajectory at the
    intermediate waypoints.

    3.Matrix Design
    Since we have 8*N unknown coefficients to solve, and if we are given 8*N
    equations(constraints), then the problem becomes solving a linear equation.

    A * Coeff = B

    Let's look at B matrix first, B matrix is simple because it is just some constants
    on the right hand side of the equation. There are 8xN constraints,
    so B matrix will be [8N, 1].

    Now, how do we determine the dimension of Coeff matrix? Coeff is the final
    output matrix consists of 8*N elements. Since B matrix is only one column,
    thus Coeff matrix must be [8N, 1].

    Coeff.transpose = [a10 a11..a17...aN0 aN1..aN7]

    A matrix is tricky, we then can think of A matrix as a coeffient-coeffient matrix.
    We are no longer looking at a particular polynomial Pi, but rather P1, P2...PN
    as a whole. Since now our Coeff matrix is [8N, 1], and B is [8N, 8N], thus
    A matrix must have the form [8N, 8N].

    A = [A10 A12 ... A17 ... AN0 AN1 ...AN7
         ...
        ]

    Each element in a row represents the coefficient of coeffient aij under
    a certain constraint, where aij is the jth coeffient of Pi with i = 1...N, j = 0...7.
    """

    n = len(waypoints) - 1

    # initialize A, and B matrix
    A = np.zeros((8*n, 8*n))
    B = np.zeros((8*n, 1))

    # populate B matrix.
    for i in range(n):
        B[i] = waypoints[i]
        B[i + n] = waypoints[i+1]

    # Constraint 1
    for i in range(n):
        A[i][8*i:8*(i+1)] = get_poly_cc(8, 0, 0)

    # Constraint 2
    for i in range(n):
        A[i+n][8*i:8*(i+1)] = get_poly_cc(8, 0, 1)

    # Constraint 3
    for k in range(1, 4):
        A[2*n+k-1][:8] = get_poly_cc(8, k, 0)

    # Constraint 4
    for k in range(1, 4):
        A[2*n+3+k-1][-8:] = get_poly_cc(8, k, 1)

    # Constraint 5
    for i in range(n-1):
        for k in range(1, 7):
            A[2*n+6 + i*6+k-1][i*8 : (i*8+16)] = np.concatenate((get_poly_cc(8, k, 1), -get_poly_cc(8, k, 0)))

    # solve for the coefficients
    Coeff = np.linalg.solve(A, B)
    return Coeff
