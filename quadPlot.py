from mpl_toolkits.mplot3d import Axes3D
from matplotlib.colors import cnames
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import sys

# TODO add functionality for plotting state and desired_state

def plot_quad_3d(args=()):
    fig = plt.figure()
    ax = fig.add_axes([0, 0, 1, 1], projection='3d')
    ax.plot([], [], [], '-', c='cyan')[0]
    ax.plot([], [], [], '-', c='red')[0]
    ax.plot([], [], [], '-', c='blue', marker='o', markevery=2)[0]
    set_limit((-0.5,0.5), (-0.5,0.5), (-0.5,5))
    an = animation.FuncAnimation(fig, _callback, fargs = args, init_func=None,
            frames=400, interval=10, blit=False)
    if len(sys.argv) > 1 and sys.argv[1] == 'save':
        an.save('sim.gif', dpi=80, writer='imagemagick', fps=60)
    else:
        plt.show()

def set_limit(x, y, z):
    ax = plt.gca()
    ax.set_xlim(x)
    ax.set_ylim(y)
    ax.set_zlim(z)

def set_frame(frame):
    # convert 3x6 world_frame matrix into three line_data objects which is 3x2 (row:point index, column:x,y,z)
    lines_data = [frame[:,[0,2]], frame[:,[1,3]], frame[:,[4,5]]]
    ax = plt.gca()
    lines = ax.get_lines()
    for line, line_data in zip(lines, lines_data):
        x, y, z = line_data
        line.set_data(x, y)
        line.set_3d_properties(z)

def _callback(i, sched, id):
    # forward the event from GUI thread to scheduler threadA
    # do the actual rendering in _render method
    # start scheduler after we get the first frame so that we can see the initial state
    sched.start()
    sched.postEvent(id)


