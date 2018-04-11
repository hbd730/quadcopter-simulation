from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from context import trajGen3D

def get_full_trajectory():
    T = np.linspace(0, 100, 1000)
    waypoints = trajGen3D.get_helix_waypoints(0, 9)
    print "T", T
    print "waypoints", waypoints

    full_traj = np.zeros((T.size,3))
    for i, t in enumerate(T):
        desired_state = trajGen3D.generate_trajectory(t, 0.1, waypoints)
        full_traj[i] = desired_state.pos

    return full_traj

def main():
    fig = plt.figure()
    ax = plt.axes(projection='3d')

    # points = trajGen3D.get_helix_waypoints(0, 50)

    points = get_full_trajectory()
    # Data for a three-dimensional line
    zline = points[:,-1]
    xline = points[:,0]
    yline = points[:,1]

    ax.plot3D(xline, yline, zline, 'blue')
    plt.show()

if __name__== "__main__":
  main()
