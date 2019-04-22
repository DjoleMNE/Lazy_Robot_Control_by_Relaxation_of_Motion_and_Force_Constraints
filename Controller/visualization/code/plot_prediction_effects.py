# Author(s): Djordje Vukcevic
# Year: 2019
import sys, os
import numpy as np
import sympy as sp
from sympy import *
import matplotlib.pyplot as plt
import pyinotify

variable_num = 7
filename = "../prediction_effects.txt"

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
    input_data = np.array(all_data)

rows = np.shape(input_data)[0] - 1
print("Data size: ", rows)

X_vel = []
Y_vel = []
Z_vel = []
energy = []
horizon = []


for sample_ in range(0, rows):
    X_vel.append(   np.float32( input_data[sample_][0] ) )
    Y_vel.append(   np.float32( input_data[sample_][1] ) )
    Z_vel.append(   np.float32( input_data[sample_][2] ) )
    energy.append(  np.float32( input_data[sample_][3] ) )
    horizon.append( np.float32( input_data[sample_][4] ) )


samples = np.arange(0, rows, 1)
energy  = np.array(energy)
horizon = np.array(horizon)
X_vel   = np.array(X_vel)
Y_vel   = np.array(Y_vel)
Z_vel   = np.array(Z_vel)


plt.ion()
plt.figure(figsize = (18,10))
plt.gca().set_axis_off()
plt.subplots_adjust(hspace = 0.02, wspace = 15)
plt.subplots_adjust(left=0.05, right=0.99, top=0.99, bottom=0.03)
plt.margins(0,0)

plt.subplot(5, 1, 1)
plt.plot(energy, c = 'orange', label='energy', linewidth = 0.9, zorder = 3)
plt.legend(loc = 1, fontsize = 'x-large')
# plt.ylim(-1 *joint_0_limit[0] - 10, joint_0_limit[0] + 10)
plt.grid(True)

 
plt.subplot(5, 1, 2)
plt.plot(horizon, c = 'black', label='horizon', linewidth = 0.5, zorder = 3)
plt.legend(loc = 1, fontsize = 'x-large')
# plt.ylim(-1 *joint_0_limit[0] - 10, joint_0_limit[0] + 10)
plt.grid(True)

plt.subplot(5, 1, 3)
plt.plot(X_vel, c = 'red', label='X_vel', linewidth = 0.5, zorder = 3)
plt.legend(loc = 1, fontsize = 'x-large')
# plt.ylim(-1 *joint_0_limit[0] - 10, joint_0_limit[0] + 10)
plt.grid(True)

plt.subplot(5, 1, 4)
plt.plot(Y_vel, c = 'green', label='Y_vel', linewidth = 0.5, zorder = 3)
plt.legend(loc = 1, fontsize = 'x-large')
# plt.ylim(-1 *joint_0_limit[0] - 10, joint_0_limit[0] + 10)
plt.grid(True)

plt.subplot(5, 1, 5)
plt.plot(Z_vel, c = 'blue', label='Z_vel', linewidth = 0.5, zorder = 3)
plt.legend(loc = 1, fontsize = 'x-large')
# plt.ylim(-1 *joint_0_limit[0] - 10, joint_0_limit[0] + 10)
plt.grid(True)

plt.draw()
plt.pause(0.001)
plt.savefig('../prediction_effects.pdf')

notifier.loop()