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

# desired_joint = np.int(sys.argv[1]) #### 1 for large, 2 for small joint

control_freq = 999
filename = "../archive/single_joint_control.txt"

with open(filename, "r") as f:
    all_data = [x.split() for x in f.readlines()]
    input_data = np.array(all_data)[:-2]

rows = np.shape(input_data)[0]
cols = np.shape(input_data[0])[0]
num_samples = rows
print("Data size: ", num_samples, ",", cols)

desired_pos     = []
measured_pos    = []
nominal_pos     = []
desired_vel     = []
measured_vel    = []
nominal_vel     = []
measured_torque = []
command_torque  = []
friction_torque = []
command_current = []
abag_error      = []
abag_bias       = []
abag_gain       = []
abag_command    = []
time_ticks      = []

for sample_ in range (0, rows):
    desired_pos.append(     np.float32(input_data[sample_][0]) )
    measured_pos.append(    np.float32(input_data[sample_][1]) )
    nominal_pos.append(     np.float32(input_data[sample_][2]) )
    measured_vel.append(    np.float32(input_data[sample_][3]) )
    nominal_vel.append(     np.float32(input_data[sample_][4]) )
    measured_torque.append( np.float32(input_data[sample_][5]) )
    command_torque.append(  np.float32(input_data[sample_][6]) )
    friction_torque.append( np.float32(input_data[sample_][7]) )
    command_current.append( np.float32(input_data[sample_][8]) )
    abag_error.append(      np.float32(input_data[sample_][9]) )
    abag_bias.append(       np.float32(input_data[sample_][10]) )
    abag_gain.append(       np.float32(input_data[sample_][11]) )
    abag_command.append(    np.float32(input_data[sample_][12]) )

samples         = np.arange(0, num_samples, 1)
desired_pos     = np.array(desired_pos)
measured_pos    = np.array(measured_pos)
nominal_pos     = np.array(nominal_pos)
measured_vel    = np.array(measured_vel)
nominal_vel     = np.array(nominal_vel) 
measured_torque = np.array(measured_torque)
command_torque  = np.array(command_torque)
friction_torque = np.array(friction_torque)
command_current = np.array(command_current)
abag_error      = np.array(abag_error)
abag_bias       = np.array(abag_bias)
abag_gain       = np.array(abag_gain)
abag_command    = np.array(abag_command)

# creating grid for subplots 
# fig = plt.figure(figsize = (19,15)) 
fig3 = plt.figure(figsize = (19,15))

plt.gca().set_axis_off()
plt.subplots_adjust(hspace = 0.09, wspace = 15)
plt.subplots_adjust(left=0.09, right=0.96, top=0.98, bottom=0.08)
plt.margins(0,0)
subplot_num = 5

# gs = fig3.add_gridspec(7, 3)

# #################################################################
plt.subplot(subplot_num, 1, 1)

# f3_ax1 = fig3.add_subplot(gs[0:3, :])
# plt.plot(desired_pos, label='$\\theta_{desired}$', linewidth = 1.5, c='blue')
plt.plot(measured_pos, label='$\\theta_{measured}$', linewidth = 1.5, c='orange')
plt.plot(nominal_pos, label='$\\theta_{nominal}$', linewidth = 1.0, linestyle='--', c='green')
# plt.plot(desired_pos - measured_pos, label='$\\theta_{error}$', linewidth = 1.5)
# plt.plot(desired_pos - nominal_pos, label='$\\theta_{error-nominal}$', linewidth = 1.5)
plt.legend(fontsize = 12)
plt.yticks(fontsize=20)
time_ticks = np.arange(0, num_samples / control_freq, 1)
# plt.xlim(0, time_ticks[-1])
# plt.ylim( np.min(command_torque)-1,  np.max(command_torque)+1)
# plt.ylim(6.275,6.285)
plt.xticks(np.arange(0, num_samples, control_freq))
plt.grid(True, linestyle='--')
plt.ylabel('[rad]', fontsize=20)
# plt.gca().yaxis.set_major_formatter(mtick.FormatStrFormatter('%.1e'))
plt.setp( plt.gca().get_xticklabels(), visible=False)

# #################################################################
plt.subplot(subplot_num, 1, 2)
# f3_ax2 = fig3.add_subplot(gs[3:4, :])
desired_vel = desired_pos
plt.plot(desired_vel, label='$\\dot{\\theta}_{desired}$', linewidth = 1.5, c='blue')
plt.plot(measured_vel, label='$\\dot{\\theta}_{measured}$', linewidth = 1.5, c='orange')
plt.plot(nominal_vel, label='$\\dot{\\theta}_{nominal}$', linewidth = 1.0,linestyle='--', c='green')
plt.legend(fontsize = 16, ncol=5)
plt.yticks(fontsize=20)
# plt.xlim(0, time_ticks[-1])
plt.xticks(np.arange(0, num_samples, control_freq))
plt.grid(True, linestyle='--')
# plt.ylim(1.3, 1.9)
plt.ylabel('[rad/s]', fontsize=20)
# plt.gca().yaxis.set_major_formatter(mtick.FormatStrFormatter('%.1e'))
plt.setp( plt.gca().get_xticklabels(), visible=False)
# plt.yticks(np.arange(np.min(error_torque) -5, np.max(error_torque) +5, 8),  fontsize=20)

# #################################################################
plt.subplot(subplot_num, 1, 3)
# f3_ax2 = fig3.add_subplot(gs[3:4, :])
plt.plot(abag_bias, label='$abag_{bias}$', linewidth = 1.5, c='green',zorder = 4)
plt.plot(abag_gain, label='$abag_{gain}$', linewidth = 1.5, c='red',zorder = 2)
plt.plot(abag_command, label='$abag_{cmd}$', linewidth = 1.5, c='blue',zorder = 1)
plt.plot(abag_error, label='$abag_{low-passed-error-sign}$', linewidth = 1.0,linestyle='--', c='orange', zorder =0)
plt.legend(fontsize = 16, ncol=5)
plt.yticks(fontsize=20)
# plt.xlim(0, time_ticks[-1])
plt.xticks(np.arange(0, num_samples, control_freq))
plt.grid(True, linestyle='--')
plt.ylim(-0.22,0.22)
plt.ylabel('[%]', fontsize=20)
# plt.gca().yaxis.set_major_formatter(mtick.FormatStrFormatter('%.1e'))
plt.setp( plt.gca().get_xticklabels(), visible=False)
# plt.yticks(np.arange(np.min(error_torque) -5, np.max(error_torque) +5, 8),  fontsize=20)
# average_bias = abag_bias[2000:4000]
# print("Average bias:  ", np.average(average_bias))
# average_friction = friction_torque[2000:4000]
# print("Average friction:  ", np.average(average_friction))
# print("MAx friction:  ", np.max(abs(average_friction)))

# #################################################################
plt.subplot(subplot_num, 1, 4)
# f3_ax3 = fig3.add_subplot(gs[3:5, :])
plt.plot(measured_torque, label='$\\tau_{measured}$', linewidth = 1.0, c='orange')
plt.plot(command_torque, label='$\\tau_{control}$', linewidth = 1.0)
plt.plot(command_torque - measured_torque, label='$\\tau_{control-measured}$', linewidth = 1.0, c='purple')
plt.plot(friction_torque, label='$\\tau_{friction}$', linewidth = 1.0,linestyle='--', c='green', zorder=4)
plt.plot(command_torque - friction_torque, label='$\\tau_{control} - \\tau_{friction}$', linewidth = 1.0, linestyle='--', c='red')
plt.legend(fontsize = 16, ncol=5, loc=1)
# plt.xlim(0, time_ticks[-1])
plt.xticks(np.arange(0, num_samples, control_freq))
plt.grid(True, linestyle='--')
plt.ylabel('[Nm]',  fontsize=20)
# plt.gca().yaxis.set_major_formatter(mtick.FormatStrFormatter('%.1e'))
plt.setp( plt.gca().get_xticklabels(), visible=False)
# plt.ylim(np.min(measured_current)-0.5, np.max(measured_current)+0.5)
plt.ylim(-5,5)
# plt.yticks(np.arange(np.min(measured_current), np.max(measured_current), 0.5),  fontsize=20)


# #################################################################
plt.subplot(subplot_num, 1, 5)
# f3_ax3 = fig3.add_subplot(gs[3:5, :])
plt.plot(command_current, label='$I_{command} = (\\tau_{control} - \\tau_{friction}) / K_t$', linewidth = 1.5)
# command_current = command_current * 0.0 +0.3
# plt.plot(command_current, label='$I_{command} = 0.3$', linewidth = 1.5)
plt.legend(fontsize = 16)
plt.xticks(np.arange(0, rows, control_freq), time_ticks, fontsize=18)
plt.xlabel('time [s]', fontsize=18)
plt.grid(True, linestyle='--')
plt.ylabel('[A]',  fontsize=20)
# plt.ylim(-0.0,1)
# plt.gca().yaxis.set_major_formatter(mtick.FormatStrFormatter('%.1e'))
# plt.setp( plt.gca().get_xticklabels(), visible=False)
# plt.ylim(np.min(9.5)-0.5, np.max(9.5)+0.5)
# plt.yticks(np.arange(np.min(measured_current), np.max(measured_current), 0.5),  fontsize=20)

# ###########################
plt.savefig('../experiments/single_joint_control.pdf')
plt.show()

