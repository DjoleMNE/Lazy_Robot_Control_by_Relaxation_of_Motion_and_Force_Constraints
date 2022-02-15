# Author(s): Djordje Vukcevic
# Year: 2021
import sys, os
import numpy as np
import sympy as sp
from sympy import *
import matplotlib.pyplot as plt
import pyinotify
import matplotlib.ticker as mtick
import matplotlib.gridspec as gridspec

desired_dim = np.int(sys.argv[1])
print(desired_dim)
control_freq = 999
filename = "../archive/cart_control.txt"

with open(filename, "r") as f:
    all_data = [x.split() for x in f.readlines()]
    input_data = np.array(all_data)[:-1]

rows = np.shape(input_data)[0]
cols = np.shape(input_data[0])[0]
num_samples = rows
print("Data size: ", num_samples, ",", cols)

desired  = []
measured = []
filtered_error = []
bias = []
gain = []
force    = []

#desided_dim can go from 0 to 5... x,y,z linear position, x,y,z linear vel
for sample_ in range (0, rows):
    desired.append(np.float32(input_data[sample_][desired_dim]) )
    measured.append(np.float32(input_data[sample_][desired_dim + 6]) )
    if   (desired_dim is 0 or desired_dim is 3):
        filtered_error.append(np.float32(input_data[sample_][12]) )
        bias.append(np.float32(input_data[sample_][15]) )
        gain.append(np.float32(input_data[sample_][18]) )
        force.append(np.float32(input_data[sample_][21]) )
    elif (desired_dim is 1 or desired_dim is 4):
        filtered_error.append(np.float32(input_data[sample_][13]) )
        bias.append(np.float32(input_data[sample_][16]) )
        gain.append(np.float32(input_data[sample_][19]) )
        force.append(np.float32(input_data[sample_][22]) )
    elif (desired_dim is 2 or desired_dim is 5):
        filtered_error.append(np.float32(input_data[sample_][14]) )
        bias.append(np.float32(input_data[sample_][17]) )
        gain.append(np.float32(input_data[sample_][20]) )
        force.append(np.float32(input_data[sample_][23]) )
samples = np.arange(0, num_samples, 1)
desired = np.array(desired)
measured = np.array(measured)
filtered_error = np.array(filtered_error)
bias = np.array(bias)
gain = np.array(gain)
force = np.array(force)

# creating grid for subplots 
# fig = plt.figure(figsize = (19,15)) 
fig3 = plt.figure(figsize = (19,15))

plt.gca().set_axis_off()
plt.subplots_adjust(hspace = 0.09, wspace = 15)
plt.subplots_adjust(left=0.09, right=0.96, top=0.98, bottom=0.08)
plt.margins(0,0)
subplot_num = 3

# gs = fig3.add_gridspec(7, 3)

# #################################################################
plt.subplot(subplot_num, 1, 1)
plt.plot(desired, label='$x_{desired}$',linestyle='--', linewidth = 1.0)
plt.plot(measured, label='$x_{measured}$', linewidth = 1.0)
plt.legend(fontsize = 12)
plt.yticks(fontsize=20)
# plt.xlim(0, time_ticks[-1])
# plt.ylim( np.min(command_torque)-1,  np.max(command_torque)+1)
# plt.ylim(6.275,6.285)
plt.xticks(np.arange(0, num_samples, control_freq))
plt.grid(True, linestyle='--')
#plt.ylabel('[rad]', fontsize=20)
# plt.gca().yaxis.set_major_formatter(mtick.FormatStrFormatter('%.1e'))
plt.setp( plt.gca().get_xticklabels(), visible=False)

# #################################################################
plt.subplot(subplot_num, 1, 2)
plt.plot(filtered_error, label='$x_{filtered-error-sign}$', linewidth = 1.0, c='red')
plt.legend(fontsize = 16, ncol=5)
plt.yticks(fontsize=20)
# plt.xlim(0, time_ticks[-1])
plt.xticks(np.arange(0, num_samples, control_freq))
plt.grid(True, linestyle='--')
# plt.ylim(1.3, 1.9)
#plt.ylabel('[rad/s]', fontsize=20)
# plt.gca().yaxis.set_major_formatter(mtick.FormatStrFormatter('%.1e'))
plt.setp( plt.gca().get_xticklabels(), visible=False)


# #################################################################
plt.subplot(subplot_num, 1, 3)
plt.plot(bias, label='$x_{cart-bias}$', linewidth = 1.0, c='green', zorder=3)
plt.plot(gain, label='$x_{cart-gain}$', linewidth = 1.00, c='red')
plt.plot(force, label='$x_{cart-force}$', linewidth = 1.00, c='blue')
plt.legend(fontsize = 16, loc=1, ncol=6)
time_ticks = np.arange(0, num_samples / control_freq, 1)
plt.xticks(np.arange(0, num_samples, control_freq), time_ticks, fontsize=14)
plt.grid(True, linestyle='--')
plt.ylabel('[N]',  fontsize=20)
# plt.gca().yaxis.set_major_formatter(mtick.FormatStrFormatter('%.1e'))
# plt.setp( plt.gca().get_xticklabels(), visible=False)
# plt.ylim(np.min(measured_current)-0.5, np.max(measured_current)+0.5)
#plt.ylim(-25,25)

# ###########################
plt.savefig('../experiments/cart_control.pdf')
plt.show()

