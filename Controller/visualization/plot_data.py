# Author(s): Djordje Vukcevic
# Year: 2019

import numpy as np
import sympy as sp
from sympy import *
import matplotlib.pyplot as plt

input_data = np.loadtxt("control_error.txt", dtype='double', delimiter=' ')
rows = input_data.shape[0]
cols = input_data.shape[1]
num_samples = rows / 5
final_data = []
measured = []
desired = []
bias = []
gain = []
command = []

desired_dim = 2

for sample_ in range(0, rows, 5):
    measured.append(input_data[sample_, desired_dim])
    desired.append(input_data[1 + sample_, desired_dim])
    bias.append(input_data[2 + sample_, desired_dim])
    gain.append(input_data[3 + sample_, desired_dim])
    command.append(input_data[4 + sample_, desired_dim])

samples = np.arange(0, num_samples, 1)
measured = np.array(measured)
desired = np.array(desired)
bias = np.array(bias)
gain = np.array(gain)
command = np.array(command)

plt.figure(figsize = (8,8))
plt.subplot(2, 1, 1)
plt.plot(measured, c = 'purple', label='X_m', linewidth = 2, zorder=1)
if not num_samples == 1:
    plt.step(samples, desired, label='X_d', linewidth = 2, where='post', color = 'black')
    # l = plt.hlines(xmin = 800, xmax = 1000, y = desired[-1], color = 'black', linewidth=2, zorder=2)
else:
    l = plt.axhline(y = desired[0], label='X_d', c = 'black', linewidth=2)    
plt.legend(loc=4, fontsize = 'x-large')

plt.subplot(2, 1, 2)
plt.plot(bias, c = 'green', label='b', linewidth=2, zorder=2)
plt.plot(gain, c = 'red', label='g', linewidth=2, zorder=3)
plt.plot(command, c = 'blue', label='u', linewidth=2, zorder=1)
plt.legend(loc=1, fontsize = 'x-large')
plt.show()
