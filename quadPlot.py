"""
author: Peter Huang
email: hbd730@gmail.com
license: BSD
Please feel free to use and modify this, but keep the above information. Thanks!
"""

from mpl_toolkits.mplot3d import Axes3D
from matplotlib.colors import cnames
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import sys

history = np.zeros((500,3))
count = 0

def plot_quad_3d(waypoints, init_pos, args=()):
    fig = plt.figure()
    ax = fig.add_axes([0, 0, 1, 1], projection='3d')
    ax.plot([], [], [], '-', c='cyan')[0]
    ax.plot([], [], [], '-', c='red')[0]
    ax.plot([], [], [], '-', c='blue', marker='o', markevery=2)[0]
    ax.plot([], [], [], '.', c='red', markersize=4)[0]
    ax.plot([], [], [], '.', c='blue', markersize=2)[0]
    #set_limit((-0.5,0.5), (-0.5,0.5), (-0.5,8))
    set_limit2(waypoints, init_pos)

    set_ax_names("x","y","z")
    plot_waypoints(waypoints)
    an = animation.FuncAnimation(fig, _callback, fargs = args, init_func=None,
            frames=400, interval=10, blit=False)
    if len(sys.argv) > 1 and sys.argv[1] == 'save':
        print "saving"
        an.save('sim.gif', dpi=80, writer='imagemagick', fps=60)
    else:
        plt.show()

def plot_waypoints(waypoints):
    ax = plt.gca()
    lines = ax.get_lines()
    lines[-2].set_data(waypoints[:,0], waypoints[:,1])
    lines[-2].set_3d_properties(waypoints[:,2])

def set_limit(x, y, z):
    ax = plt.gca()
    ax.set_xlim(x)
    ax.set_ylim(y)
    ax.set_zlim(z)

def set_limit2(waypoints, pos):
    """
        Set the drawing limits based on the waypoints 
        the quadrotor should fly and start point.

    """
    ax = plt.gca()
    x_min = min(pos[0], min(waypoints[:,0]))
    x_max = max(pos[0], max(waypoints[:,0]))
    ax.set_xlim(x_min,x_max)

    y_min = min(pos[1], min(waypoints[:,1]))
    y_max = max(pos[1], max(waypoints[:,1]))
    ax.set_ylim(y_min,y_max)

    z_min = min(pos[2], min(waypoints[:,2]))
    z_max = max(pos[2], max(waypoints[:,2]))
    ax.set_zlim(z_min,z_max)

def set_ax_names(x, y, z):
    ax = plt.gca()
    ax.set_xlabel(x)
    ax.set_ylabel(y)
    ax.set_zlabel(z)


def set_frame(frame):
    # convert 3x6 world_frame matrix into three line_data objects which is 3x2 (row:point index, column:x,y,z)
    lines_data = [frame[:,[0,2]], frame[:,[1,3]], frame[:,[4,5]]]
    ax = plt.gca()
    lines = ax.get_lines()
    for line, line_data in zip(lines[:3], lines_data):
        x, y, z = line_data
        line.set_data(x, y)
        line.set_3d_properties(z)

    global history, count
    # plot history trajectory
    history[count] = frame[:,4]
    if count < np.size(history, 0) - 1:
        count += 1
    zline = history[:count,-1]
    xline = history[:count,0]
    yline = history[:count,1]
    lines[-1].set_data(xline, yline)
    lines[-1].set_3d_properties(zline)
    # ax.plot3D(xline, yline, zline, 'blue')

def _callback(i, sched, id):
    # forward the event from GUI thread to scheduler threadA
    # do the actual rendering in _render method
    # start scheduler after we get the first frame so that we can see the initial state
    sched.start()
    sched.postEvent(id)
