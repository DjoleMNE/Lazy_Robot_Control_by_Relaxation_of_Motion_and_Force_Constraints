# Author(s): Djordje Vukcevic
# Year: 2019
import sys, os
import numpy as np
import sympy as sp
from sympy import *
import matplotlib.pyplot as plt
import pyinotify

desired_dim = np.int(sys.argv[1])
print("Selected dimension: ", desired_dim)
variable_num = 6

filename = "control_error.txt"

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

rows = np.shape(input_data)[0]
cols = np.shape(input_data[0])[0]
rows = rows - (rows % variable_num)
num_samples = np.int(rows / variable_num) 
print("Data size: ", num_samples, ",", cols)

measured = []
command  = []
desired  = []
error    = []
bias     = []
gain     = []

for sample_ in range(0, rows, variable_num):
    measured.append(np.float32( input_data[    sample_][desired_dim]) )
    desired.append( np.float32( input_data[1 + sample_][desired_dim]) )
    error.append(   np.float32( input_data[2 + sample_][desired_dim]) )
    bias.append(    np.float32( input_data[3 + sample_][desired_dim]) )
    gain.append(    np.float32( input_data[4 + sample_][desired_dim]) )
    command.append( np.float32( input_data[5 + sample_][desired_dim]) )

samples = np.arange(0, num_samples, 1)
measured = np.array(measured)
desired = np.array(desired)
bias = np.array(bias)
gain = np.array(gain)
command = np.array(command)

plt.ion()
plt.figure(figsize = (8,10))
if(desired_dim is 0):   plt.suptitle('Linear X')
elif(desired_dim is 1): plt.suptitle('Linear Y')
elif(desired_dim is 2): plt.suptitle('Linear Z')
elif(desired_dim is 3): plt.suptitle('Angular X')
elif(desired_dim is 4): plt.suptitle('Angular Y')
elif(desired_dim is 5): plt.suptitle('Angular Z')

plt.gca().set_axis_off()
plt.subplots_adjust(hspace = 0.02, wspace = 0)
plt.margins(0,0)

plt.subplot(3, 1, 1)
if(desired_dim < 3):
    plt.plot(measured, c = 'purple', label='X_measured', linewidth = 2, zorder = 2)
    if not num_samples == 1:
        plt.step(samples, desired, label='X_desired', linewidth = 2, where='post', color = 'black', zorder = 3)
        # l = plt.hlines(xmin = 800, xmax = 1000, y = desired[-1], color = 'black', linewidth=2, zorder=1)
    else:
        l = plt.axhline(y = desired[0], label='X_d', c = 'black', linewidth=2)
else:
    plt.plot(measured, c = 'orange', label='Raw Error', linewidth = 2, zorder = 2)

plt.legend(loc=4, fontsize = 'x-large')
plt.grid(True)

plt.subplot(3, 1, 2)
plt.plot(bias, c = 'green', label='bias', linewidth=2, zorder=3)
plt.plot(gain, c = 'red', label='gain', linewidth=2, zorder=4)
plt.plot(command, c = 'blue', label='u', linewidth=2, zorder=2)
plt.legend(fontsize = 'x-large')
plt.grid(True)

plt.subplot(3, 1, 3)
plt.plot(error, c = 'orange', label='low_pass-error', linewidth=2, zorder=2)
plt.legend(fontsize = 'x-large')
plt.grid(True)

plt.draw()
plt.pause(0.001)
if(desired_dim is 0):   plt.savefig('x_linear_control.pdf')
elif(desired_dim is 1): plt.savefig('y_linear_control.pdf')
elif(desired_dim is 2): plt.savefig('z_linear_control.pdf')
elif(desired_dim is 3): plt.savefig('x_angular_control.pdf')
elif(desired_dim is 4): plt.savefig('y_angular_control.pdf')
elif(desired_dim is 5): plt.savefig('z_angular_control.pdf')
notifier.loop()