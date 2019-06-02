# Author(s): Djordje Vukcevic
# Year: 2019
import sys, os
import numpy as np
import sympy as sp
from sympy import *
import matplotlib.pyplot as plt
import pyinotify

filename = "../ext_force_data.txt"

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
num_samples = rows
print("Data size: ", num_samples, ",", cols)

linear_x_force  = []
linear_y_force  = []
linear_z_force  = []
angular_x_force = []
angular_y_force = []
angular_z_force = []

for sample_ in range(0, rows):
    linear_x_force.append( np.float32( input_data[sample_][0] ) )
    linear_y_force.append( np.float32( input_data[sample_][1] ) )
    linear_z_force.append( np.float32( input_data[sample_][2] ) )
    angular_x_force.append(np.float32( input_data[sample_][3] ) )
    angular_y_force.append(np.float32( input_data[sample_][4] ) )
    angular_z_force.append(np.float32( input_data[sample_][5] ) )

samples = np.arange(0, num_samples, 1)
linear_x_force  = np.array(linear_x_force)
linear_y_force  = np.array(linear_y_force)
linear_z_force  = np.array(linear_z_force)

angular_x_force = np.array(angular_x_force)
angular_y_force = np.array(angular_y_force)
angular_z_force = np.array(angular_z_force)

plt.ion()
plt.figure(figsize = (18, 10))
plt.suptitle('External Force-Torque', fontsize=20)

plt.gca().set_axis_off()
plt.subplots_adjust(hspace = 0.02, wspace = 15)
plt.subplots_adjust(left=0.05, right=0.99, top=0.95, bottom=0.03)
plt.margins(0,0)

plt.subplot(6, 1, 1)
plt.plot(linear_x_force, label='linear_x_force', c = 'red', linewidth = 2, zorder = 2)
plt.legend(loc=4, fontsize = 'x-large')
plt.grid(True)

plt.subplot(6, 1, 2)
plt.plot(linear_y_force, label='linear_y_force', linewidth = 2, color = 'limegreen', zorder = 2)
plt.legend(loc=4, fontsize = 'x-large')
plt.grid(True)

plt.subplot(6, 1, 3)
plt.plot(linear_z_force, label='linear_z_force', linewidth = 2, color = 'blue', zorder = 2)
plt.legend(loc=4, fontsize = 'x-large')
plt.grid(True)

plt.subplot(6, 1, 4)
plt.plot(angular_x_force, label='angular_x_force', c = 'red', linewidth = 2, zorder = 2)
plt.legend(fontsize = 'x-large')
plt.grid(True)


plt.subplot(6, 1, 5)
plt.plot(angular_y_force, label='angular_y_force', linewidth = 2, color = 'limegreen', zorder = 2)
plt.legend(fontsize = 'x-large')
plt.grid(True)


plt.subplot(6, 1, 6)
plt.plot(angular_z_force, label='angular_z_force', linewidth = 2, color = 'blue', zorder = 2)
plt.legend(fontsize = 'x-large')
plt.grid(True)


plt.draw()
plt.pause(0.001)
plt.savefig('../ext_force_data.pdf')

notifier.loop()