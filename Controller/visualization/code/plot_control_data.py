# Author(s): Djordje Vukcevic
# Year: 2019
import sys, os
import numpy as np
import sympy as sp
from sympy import *
import matplotlib.pyplot as plt
import pyinotify

desired_dim = np.int(sys.argv[1])
show_tube = np.int(sys.argv[2])

print("Selected dimension: ", desired_dim)
variable_num = 7

filename = "../control_error.txt"

def restart_program(): #restart application
    python = sys.executable
    os.execl(python, python, * sys.argv)

class ModHandler(pyinotify.ProcessEvent):
    # evt has useful properties, including pathname
    def process_IN_CLOSE_WRITE(self, evt):
        print("Data file has changed")
        restart_program()


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

measured  = []
desired   = []
raw_error = []
error     = []
bias      = []
gain      = []
command   = []

for sample_ in range(1, rows - 1, variable_num):
    measured.append(    np.float32( input_data[    sample_][desired_dim]) )
    desired.append(     np.float32( input_data[1 + sample_][desired_dim]) )
    raw_error.append(   np.float32( input_data[2 + sample_][desired_dim]) )
    if(desired_dim == 6):
        error.append(       np.float32( input_data[3 + sample_][0] ) )
        bias.append(        np.float32( input_data[4 + sample_][0] ) )
        gain.append(        np.float32( input_data[5 + sample_][0] ) )
        command.append(     np.float32( input_data[6 + sample_][0] ) )
    else:
        error.append(       np.float32( input_data[3 + sample_][desired_dim] ) )
        bias.append(        np.float32( input_data[4 + sample_][desired_dim] ) )
        gain.append(        np.float32( input_data[5 + sample_][desired_dim] ) )
        command.append(     np.float32( input_data[6 + sample_][desired_dim] ) )

samples   = np.arange(0, num_samples, 1)
measured  = np.array(measured)
desired   = np.array(desired)
raw_error = np.array(raw_error)
bias      = np.array(bias)
gain      = np.array(gain)
command   = np.array(command)

plt.ion()
plt.figure(figsize = (18, 10))
if(desired_dim is 0):   plt.suptitle('Linear X', fontsize=20)
elif(desired_dim is 1): plt.suptitle('Linear Y', fontsize=20)
elif(desired_dim is 2): plt.suptitle('Linear Z', fontsize=20)
elif(desired_dim is 3): plt.suptitle('Angular X', fontsize=20)
elif(desired_dim is 4): plt.suptitle('Angular Y', fontsize=20)
elif(desired_dim is 5): plt.suptitle('Angular Z', fontsize=20)
elif(desired_dim is 6): plt.suptitle('Linear Velocity X', fontsize=20)

plt.gca().set_axis_off()
plt.subplots_adjust(hspace = 0.02, wspace = 15)
plt.subplots_adjust(left=0.05, right=0.99, top=0.95, bottom=0.03)
plt.margins(0,0)

plt.subplot(4, 1, 1)
if(desired_dim < 3 or desired_dim == 6):
    plt.plot(measured, c = 'limegreen', label='Measured', linewidth = 2, zorder = 2)
    plt.plot(desired, label='Desired', linewidth = 2, color = 'black', zorder = 3)

if(show_tube):
    tube_tolerance = np.array(np.full((num_samples, ), np.float32( input_data[0][desired_dim] )))
    plt.plot(     tube_tolerance, c = 'red', label='tube_upper_limit', linewidth = 1.3, zorder = 2)
    plt.plot(-1 * tube_tolerance, c = 'blue', label='tube_lower_limit', linewidth = 1.3, zorder = 2)

plt.legend(loc=4, fontsize = 'x-large')
plt.grid(True)

plt.subplot(4, 1, 2)
plt.plot(raw_error, c = 'orange', label=r'raw error: $e = y_d - y_k$', linewidth=1, zorder=2)
plt.legend(fontsize = 'x-large')
plt.grid(True)

plt.subplot(4, 1, 3)
plt.plot(error, c = 'orange', label=r'low-pass filtered error sign: $\bar{e}$', linewidth=1, zorder=2)
plt.legend(fontsize = 'x-large')
plt.ylim(-1.2, 1.2)
plt.grid(True)

plt.subplot(4, 1, 4)
plt.plot(bias, c = 'green', label='bias', linewidth = 2, zorder = 4)
plt.plot(gain, c = 'red', label=r'gain * sign(e)', linewidth = 2, zorder = 2)
plt.plot(command, c = 'blue', label='u', linewidth = 1.0, zorder = 3)
plt.ylim(-1.05, 1.05)
plt.yticks(np.arange(-1.0, 1.05, 0.25))


plt.legend(fontsize = 'x-large')
plt.grid(True)

plt.draw()
plt.pause(0.001)
if(desired_dim is 0):   plt.savefig('../x_linear_control.pdf')
elif(desired_dim is 1): plt.savefig('../y_linear_control.pdf')
elif(desired_dim is 2): plt.savefig('../z_linear_control.pdf')
elif(desired_dim is 3): plt.savefig('../x_angular_control.pdf')
elif(desired_dim is 4): plt.savefig('../y_angular_control.pdf')
elif(desired_dim is 5): plt.savefig('../z_angular_control.pdf')
elif(desired_dim is 6): plt.savefig('../x_linear_velocity_control.pdf')
notifier.loop()