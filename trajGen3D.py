import numpy as np
from collections import namedtuple

def generate_trajectory(t, waypoints):
"   
    The function takes known number of waypoints and time, then generates a
    minimum snap trajectory which goes through each waypoint. The output is
    the desired state associated with the next waypont for the time t.  
"
    yaw = 0.0;
    yawdot = 0.0;
    pos = np.zeros(3)
    acc = np.zeros(3)
    vel = np.zeros(3)
    

    DesiredState = namedtuple('DesiredState', 'pos vel acc yaw yawdot')
    return  DesiredState(pos, vel, acc, yaw, yawdot)


def get_poly_cc(n, k, t):
"   
    This is a helper function to get the coeffitient of coefficient for n-th
    order polynomial with k-th derivative at time t.
"
    assert (n > 0 and k > 0), "order and derivative must be positive."
    
    cc = np.ones(n)
    D  = np.linspace(1, n, n)
    
    for i in n:
        for j in k:
            if D[i] < 0:
                D[i] = 0

            D[i] = D[i] - 1
            cc[i] = cc[i] * D[i]
        
    for i in len(cc):
        cc[i] = cc[i] * np.power(t, D[i])
    
    return cc

# Minimum Snap trajectory
def MST(waypoints, t):
"   
    This function takes a list of desired waypoint i.e. [x0, x1, x2...xN] and
    time, returns a [8N,1] coeffitients matrix for the N+1 waypoints.  

    1.The Problem
    Generate a full trajectory across N+1 waypoint is made of N polynomial line segment.
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

    4. Output
    Coeff = B * A.inverse()
"

    N = len(waypoints) - 1
    
    # initialize A, and B matrix
    A = np.zeros(8*N, 8*N)
    B = np.zeros(8*N, 1)

    # populate B matrix.


    # Constraint 1


    Coeff = B * A.inverse()
    return Coeff



