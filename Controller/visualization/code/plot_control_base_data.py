# Author(s): Djordje Vukcevic
# Year: 2019
import sys, os
import numpy as np
import sympy as sp
from sympy import *
import matplotlib.pyplot as plt
import pyinotify
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d

desired_dim = np.int(sys.argv[1])
# show_tube   = np.int(sys.argv[2])

print("Selected dimension: ", desired_dim)
variable_num = 2

filename = "../archive/control_base_error.txt"

def restart_program(): #restart application
    python = sys.executable
    os.execl(python, python, * sys.argv)

class ModHandler(pyinotify.ProcessEvent):
    # evt has useful properties, including pathname
    def process_IN_CLOSE_WRITE(self, evt):
        print("Data file has changed")
        restart_program()

class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))
        FancyArrowPatch.draw(self, renderer)

def set_axes_radius(ax, origin, radius):
    ax.set_xlim3d([origin[0] - radius, origin[0] + radius])
    ax.set_ylim3d([origin[1] - radius, origin[1] + radius])
    ax.set_zlim3d([origin[2] - radius, origin[2] + radius])

def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''

    limits = np.array([
        ax.get_xlim3d(),
        ax.get_ylim3d(),
        ax.get_zlim3d(),
    ])

    origin = np.mean(limits, axis=1)
    radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))
    set_axes_radius(ax, origin, radius)

handler = ModHandler()
wm = pyinotify.WatchManager()
notifier = pyinotify.Notifier(wm, handler)
wdd = wm.add_watch(filename, pyinotify.IN_CLOSE_WRITE)

with open(filename, "r") as f:
    all_data = [x.split() for x in f.readlines()]
    input_data = np.array(all_data)[:-2]

rows = np.shape(input_data)[0] - 1
cols = np.shape(input_data[0])[0]
rows = rows - (rows % variable_num)
num_samples = np.int(rows / variable_num) 
print("Data size: ", num_samples, ",", cols)

measured_pos_x    = []
measured_pos_y    = []
measured_pos_z    = []
desired_pos_x     = []
desired_pos_y     = []
desired_pos_z     = []

measured_wrench_x = []
measured_wrench_y = []
measured_wrench_z = []
desired_wrench_x  = []
desired_wrench_y  = []
desired_wrench_z  = []

measured_wrench_xa = []
measured_wrench_ya = []
measured_wrench_za = []
desired_wrench_xa  = []
desired_wrench_ya  = []
desired_wrench_za  = []

measured_weight_x    = []
measured_weight_y    = []
measured_weight_z    = []
desired_weight_x     = []
desired_weight_y     = []
desired_weight_z     = []
contact_time_tick = []

for sample_ in range (1, rows + 1, variable_num):
    tick = np.float32( input_data[sample_][9] )
    if (tick > 0): contact_time_tick.append(tick)

    if (desired_dim == 0):
        measured_pos_x.append( np.float32( input_data[    sample_][0]) )
        measured_pos_y.append( np.float32( input_data[    sample_][1]) )
        measured_pos_z.append( np.float32( input_data[    sample_][2]) )
        desired_pos_x.append(  np.float32( input_data[1 + sample_][0]) )
        desired_pos_y.append(  np.float32( input_data[1 + sample_][1]) )
        desired_pos_z.append(  np.float32( input_data[1 + sample_][2]) )

    elif (desired_dim == 1):
        measured_wrench_x.append( np.float32( input_data[    sample_][3]) )
        measured_wrench_y.append( np.float32( input_data[    sample_][4]) )
        measured_wrench_z.append( np.float32( input_data[    sample_][5]) )
        desired_wrench_x.append(  np.float32( input_data[1 + sample_][3]) )
        desired_wrench_y.append(  np.float32( input_data[1 + sample_][4]) )
        desired_wrench_z.append(  np.float32( input_data[1 + sample_][5]) )
        
        measured_wrench_xa.append( np.float32( input_data[    sample_][6]) )
        measured_wrench_ya.append( np.float32( input_data[    sample_][7]) )
        measured_wrench_za.append( np.float32( input_data[    sample_][8]) )
        desired_wrench_xa.append(  np.float32( input_data[1 + sample_][6]) )
        desired_wrench_ya.append(  np.float32( input_data[1 + sample_][7]) )
        desired_wrench_za.append(  np.float32( input_data[1 + sample_][8]) )

    elif (desired_dim == 2):
        measured_weight_x.append( np.float32( input_data[    sample_][10]) )
        measured_weight_y.append( np.float32( input_data[    sample_][11]) )
        measured_weight_z.append( np.float32( input_data[    sample_][12]) )
        desired_weight_x.append(  np.float32( input_data[1 + sample_][10]) )
        desired_weight_y.append(  np.float32( input_data[1 + sample_][11]) )
        desired_weight_z.append(  np.float32( input_data[1 + sample_][12]) )

samples   = np.arange(0, num_samples, 1)
measured_pos_x    = np.array(measured_pos_x)
measured_pos_y    = np.array(measured_pos_y)
measured_pos_z    = np.array(measured_pos_z)
desired_pos_x     = np.array(desired_pos_x)
desired_pos_y     = np.array(desired_pos_y)
desired_pos_z     = np.array(desired_pos_z)

measured_wrench_x = np.array(measured_wrench_x)
measured_wrench_y = np.array(measured_wrench_y)
measured_wrench_z = np.array(measured_wrench_z)
desired_wrench_x  = np.array(desired_wrench_x)
desired_wrench_y  = np.array(desired_wrench_y)
desired_wrench_z  = np.array(desired_wrench_z)

measured_wrench_xa = np.array(measured_wrench_xa)
measured_wrench_ya = np.array(measured_wrench_ya)
measured_wrench_za = np.array(measured_wrench_za)
desired_wrench_xa  = np.array(desired_wrench_xa)
desired_wrench_ya  = np.array(desired_wrench_ya)
desired_wrench_za  = np.array(desired_wrench_za)

measured_weight_x    = np.array(measured_weight_x)
measured_weight_y    = np.array(measured_weight_y)
measured_weight_z    = np.array(measured_weight_z)
desired_weight_x     = np.array(desired_weight_x)
desired_weight_y     = np.array(desired_weight_y)
desired_weight_z     = np.array(desired_weight_z)

# plt.ion()
if (desired_dim != 0): figure = plt.figure(figsize = (18, 10))
else: figure = plt.figure(figsize = (10, 10))
if  (desired_dim is 0): plt.suptitle('Position in Base Frame', fontsize=20)
elif(desired_dim is 1): plt.suptitle('End-Effector Wrench in Base Frame', fontsize=20)
elif(desired_dim is 2): plt.suptitle('Estimated Weight in Base Frame', fontsize=20)

tick_freq = 1000
if (desired_dim != 0):
    if   (num_samples > 4000 and num_samples < 7000): tick_freq = 500
    elif (num_samples < 4000 and num_samples > 1000): tick_freq = 200
    elif (num_samples < 1000 and num_samples > 500): tick_freq = 100
    elif (num_samples < 500): tick_freq = 50

if (desired_dim != 0):
    plt.gca().set_axis_off()
    plt.subplots_adjust(hspace = 0.02, wspace = 15)
    plt.subplots_adjust(left=0.05, right=0.99, top=0.95, bottom=0.03)
    plt.margins(0,0)

num_of_plots = 0
if   (desired_dim == 0): num_of_plots = 1
elif (desired_dim == 1): num_of_plots = 6
elif (desired_dim == 2): num_of_plots = 3

if (desired_dim == 0):
    # ax = plt.axes(projection='3d')
    ax = figure.add_subplot(111, projection='3d')
    # ax.view_init(25, 105)
    ax.plot3D(measured_pos_x, measured_pos_y, measured_pos_z, c = 'limegreen', label='Measured', linewidth = 2, zorder = 2)
    ax.plot3D(desired_pos_x,  desired_pos_y,  desired_pos_z, label='Desired', linewidth = 2, color = 'black', zorder = 3)
    ax.legend(loc=3, fontsize = 'x-large')
    ax.set_xlim3d(min(measured_pos_x), max(measured_pos_x))
    ax.set_ylim3d(min(measured_pos_y), max(measured_pos_y))
    ax.set_zlim3d(min(measured_pos_z), max(measured_pos_z))
    set_axes_equal(ax) # important!
else: 
    plt.subplot(num_of_plots, 1, 1)
    plt.grid(True)
    for tick in contact_time_tick:
        plt.axvline(x = tick, linewidth = 1.3,  color='blue', zorder = 1)

if (desired_dim == 1):
    plt.plot(measured_wrench_x, c = 'limegreen', label='Measured', linewidth = 2, zorder = 2)
    plt.plot(desired_wrench_x, label='Desired', linewidth = 2, color = 'black', zorder = 3)
    plt.legend(loc=4, fontsize = 'x-large')
    plt.xticks(np.arange(0, num_samples, tick_freq))
if (desired_dim == 2):
    plt.plot(measured_weight_x, c = 'limegreen', label='Measured', linewidth = 2, zorder = 2)
    plt.plot(desired_weight_x, label='Desired', linewidth = 2, color = 'black', zorder = 3)
    plt.xticks(np.arange(0, num_samples, tick_freq))

if (desired_dim != 0):
    plt.subplot(num_of_plots, 1, 2)
    if (desired_dim == 1):
        plt.plot(measured_wrench_y, c = 'limegreen', label='Measured', linewidth = 2, zorder = 2)
        plt.plot(desired_wrench_y, label='Desired', linewidth = 2, color = 'black', zorder = 3)
        plt.legend(loc=4, fontsize = 'x-large')
    if (desired_dim == 2):
        plt.plot(measured_weight_y, c = 'limegreen', label='Measured', linewidth = 2, zorder = 2)
        plt.plot(desired_weight_y, label='Desired', linewidth = 2, color = 'black', zorder = 3)
    for tick in contact_time_tick:
        plt.axvline(x = tick, linewidth = 1.3,  color='blue', zorder = 1)
    plt.xticks(np.arange(0, num_samples, tick_freq))
    plt.grid(True)

    plt.subplot(num_of_plots, 1, 3)
    if (desired_dim == 1):
        plt.plot(measured_wrench_z, c = 'limegreen', label='Measured', linewidth = 2, zorder = 2)
        plt.plot(desired_wrench_z, label='Desired', linewidth = 2, color = 'black', zorder = 3)
        plt.legend(loc=4, fontsize = 'x-large')
    if (desired_dim == 2):
        plt.plot(measured_weight_z, c = 'limegreen', label='Measured', linewidth = 2, zorder = 2)
        plt.plot(desired_weight_z, label='Desired', linewidth = 2, color = 'black', zorder = 3)
    for tick in contact_time_tick:
        plt.axvline(x = tick, linewidth = 1.3,  color='blue', zorder = 1)
    plt.xticks(np.arange(0, num_samples, tick_freq))
    plt.grid(True)


if (desired_dim == 1):
    plt.subplot(num_of_plots, 1, 4)
    plt.plot(measured_wrench_xa, c = 'limegreen', label='Measured', linewidth = 2, zorder = 2)
    plt.plot(desired_wrench_xa, label='Desired', linewidth = 2, color = 'black', zorder = 3)
    for tick in contact_time_tick:
        plt.axvline(x = tick, linewidth = 1.3,  color='blue', zorder = 1)
    plt.legend(loc=4, fontsize = 'x-large')
    plt.xticks(np.arange(0, num_samples, tick_freq))
    plt.grid(True)

    plt.subplot(num_of_plots, 1, 5)
    plt.plot(measured_wrench_ya, c = 'limegreen', label='Measured', linewidth = 2, zorder = 2)
    plt.plot(desired_wrench_ya, label='Desired', linewidth = 2, color = 'black', zorder = 3)
    for tick in contact_time_tick:
        plt.axvline(x = tick, linewidth = 1.3,  color='blue', zorder = 1)
    plt.legend(loc=4, fontsize = 'x-large')
    plt.xticks(np.arange(0, num_samples, tick_freq))
    plt.grid(True)

    plt.subplot(num_of_plots, 1, 6)
    plt.plot(measured_wrench_za, c = 'limegreen', label='Measured', linewidth = 2, zorder = 2)
    plt.plot(desired_wrench_za, label='Desired', linewidth = 2, color = 'black', zorder = 3)
    for tick in contact_time_tick:
        plt.axvline(x = tick, linewidth = 1.3,  color='blue', zorder = 1)
    plt.legend(loc=4, fontsize = 'x-large')
    plt.xticks(np.arange(0, num_samples, tick_freq))
    plt.grid(True)

plt.pause(0.001)
if (desired_dim != 0): 
    plt.draw()
elif(desired_dim is 1): plt.savefig('../archive/ee_wrench_base.pdf')
elif(desired_dim is 2): plt.savefig('../archive/weight_base.pdf')
else:
    plt.grid(True)
    plt.show()
    plt.savefig('../archive/position_base.pdf')
notifier.loop()