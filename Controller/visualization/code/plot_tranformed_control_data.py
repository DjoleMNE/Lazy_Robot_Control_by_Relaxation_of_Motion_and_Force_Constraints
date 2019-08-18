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

filename = "../archive/transformed_error.txt"

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

raw_error      = []
error          = []
bias           = []
gain           = []
command        = []
x_axis         = []
y_axis         = []
z_axis         = []

for sample_ in range(0, rows, variable_num):
    raw_error.append(   np.float32( input_data[    sample_][desired_dim]) )
    error.append(       np.float32( input_data[1 + sample_][desired_dim]) )
    bias.append(        np.float32( input_data[2 + sample_][desired_dim]) )
    gain.append(        np.float32( input_data[3 + sample_][desired_dim]) )
    command.append(     np.float32( input_data[4 + sample_][desired_dim]) )
    if(desired_dim is 0):   
        x_axis.append(  np.float32( input_data[5 + sample_][0]))
        y_axis.append(  np.float32( input_data[5 + sample_][1]))
        z_axis.append(  np.float32( input_data[5 + sample_][2]))

    elif(desired_dim is 3): 
        x_axis.append(  np.float32( input_data[5 + sample_][3]))
        y_axis.append(  np.float32( input_data[5 + sample_][4]))
        z_axis.append(  np.float32( input_data[5 + sample_][5]))

samples        = np.arange(0, num_samples, 1)
raw_error      = np.array(raw_error)
bias           = np.array(bias)
gain           = np.array(gain)
command        = np.array(command)
x_axis         = np.array(x_axis)
y_axis         = np.array(y_axis)
z_axis         = np.array(z_axis)

plt.ion()
plt.figure(figsize = (18,10))
if(desired_dim is 0):   plt.suptitle('Linear', fontsize=20)
elif(desired_dim is 3): plt.suptitle('Angular', fontsize=20)


plt.gca().set_axis_off()
plt.subplots_adjust(hspace = 0.02, wspace = 15)
plt.subplots_adjust(left=0.05, right=0.99, top=0.95, bottom=0.03)
plt.margins(0,0)

plt.subplot(6, 1, 1)
plt.plot(raw_error, c = 'orange', label=r'raw error: $e = ||V \in{R^3}||$', linewidth=1, zorder=2)
plt.legend(fontsize = 'x-large')
plt.grid(True)

plt.subplot(6, 1, 2)
plt.plot(error, c = 'orange', label=r'low-pass filtered error magnitude: $\bar{e}$', linewidth=1, zorder=2)
plt.legend(fontsize = 'x-large')
plt.ylim(-1.2, 1.2)
plt.grid(True)

plt.subplot(6, 1, 3)
plt.plot(bias, c = 'green', label='bias', linewidth = 2, zorder = 4)
plt.plot(gain, c = 'red', label=r'gain * sign(e)', linewidth = 2, zorder = 2)
plt.plot(command, c = 'blue', label='u', linewidth = 1.0, zorder = 3)
plt.ylim(-1.05, 1.05)
plt.yticks(np.arange(-1.0, 1.05, 0.25))
plt.legend(fontsize = 'x-large')
plt.grid(True)

plt.subplot(6, 1, 4)
plt.plot(x_axis, c = 'blue', label='x axis distribution', linewidth = 1)
plt.ylim(-1.05, 1.05)
plt.yticks(np.arange(-1.0, 1.05, 0.25))
plt.legend(fontsize = 'x-large')
plt.grid(True)

plt.subplot(6, 1, 5)
plt.plot(y_axis, c = 'blue', label=  'y axis distribution', linewidth =1)
plt.ylim(-1.05, 1.05)
plt.yticks(np.arange(-1.0, 1.05, 0.25))
plt.legend(fontsize = 'x-large')
plt.grid(True)

plt.subplot(6, 1, 6)
plt.plot(z_axis, c = 'blue', label= 'z axis distribution', linewidth =1)
plt.ylim(-1.05, 1.05)
plt.yticks(np.arange(-1.0, 1.05, 0.25))
plt.legend(fontsize = 'x-large')
plt.grid(True)

plt.draw()
plt.pause(0.001)
if(desired_dim is 0):   plt.savefig('../archive/linear_tranformed_control.pdf')
elif(desired_dim is 3): plt.savefig('../archive/angular_tranformed_control.pdf')
notifier.loop()