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
filename = "../archive/full_body_friction.txt"
variable_num = 3

with open(filename, "r") as f:
    all_data = [x.split() for x in f.readlines()]
    input_data = np.array(all_data)[:-1]

rows = np.shape(input_data)[0]
cols = np.shape(input_data[0])[0]
rows = rows - (rows % variable_num)
num_samples = np.int(rows / variable_num)
print("Data size: ", num_samples, ",", cols)

measured_pos    = []
nominal_pos     = []
measured_vel    = []
nominal_vel     = []
measured_torque           = []
command_torque            = []
friction_torque           = [] ## estimated torque
simulated_friction_torque = []
RNE_torque      = [] #gravity + centripetal torque
current_command = [] 

for sample_ in range (0, rows, variable_num):
    measured_pos.append(    np.float32(input_data[desired_dim + sample_][0]) )
    nominal_pos.append(     np.float32(input_data[desired_dim + sample_][1]) )
    measured_vel.append(    np.float32(input_data[desired_dim + sample_][2]) )
    nominal_vel.append(     np.float32(input_data[desired_dim + sample_][3]) )
    measured_torque.append( np.float32(input_data[desired_dim + sample_][4]) )
    command_torque.append(  np.float32(input_data[desired_dim + sample_][5]) )
    friction_torque.append( np.float32(input_data[desired_dim + sample_][6]) )
    simulated_friction_torque.append( np.float32(input_data[desired_dim + sample_][7]) )
    RNE_torque.append( np.float32(input_data[desired_dim + sample_][8]) )
    current_command.append( np.float32(input_data[desired_dim + sample_][9]) )

samples         = np.arange(0, num_samples, 1)
measured_pos    = np.array(measured_pos)
nominal_pos     = np.array(nominal_pos)
measured_vel    = np.array(measured_vel)
nominal_vel     = np.array(nominal_vel) 
measured_torque = np.array(measured_torque)
command_torque  = np.array(command_torque)
friction_torque = np.array(friction_torque)
simulated_friction_torque = np.array(simulated_friction_torque)
RNE_torque = np.array(RNE_torque)
current_command = np.array(current_command)

# creating grid for subplots 
# fig = plt.figure(figsize = (19,15)) 
fig3 = plt.figure(figsize = (19,15))

plt.gca().set_axis_off()
plt.subplots_adjust(hspace = 0.09, wspace = 15)
plt.subplots_adjust(left=0.09, right=0.96, top=0.98, bottom=0.08)
plt.margins(0,0)
subplot_num = 5

# #################################################################
plt.subplot(subplot_num, 1, 1)
plt.plot(measured_pos, label='$\\theta_{measured}$',linestyle='--', linewidth = 1.0, c='orange')
plt.plot(nominal_pos, label='$\\theta_{nominal}$', linewidth = 1.0,  c='green')
plt.legend(fontsize = 12)
plt.yticks(fontsize=20)
plt.xticks(np.arange(0, num_samples, control_freq))
plt.grid(True, linestyle='--')
plt.ylabel('[rad]', fontsize=20)
# plt.gca().yaxis.set_major_formatter(mtick.FormatStrFormatter('%.1e'))
plt.setp( plt.gca().get_xticklabels(), visible=False)

# #################################################################
plt.subplot(subplot_num, 1, 2)
plt.plot(measured_vel, label='$\\dot{\\theta}_{measured}$', linewidth = 1.0, c='orange')
plt.plot(nominal_vel, label='$\\dot{\\theta}_{nominal}$', linewidth = 1.0, c='green')
plt.legend(fontsize = 16, ncol=5)
plt.yticks(fontsize=20)
plt.xticks(np.arange(0, num_samples, control_freq))
plt.grid(True, linestyle='--')
# plt.ylim(1.3, 1.9)
plt.ylabel('[rad/s]', fontsize=20)
# plt.gca().yaxis.set_major_formatter(mtick.FormatStrFormatter('%.1e'))
plt.setp( plt.gca().get_xticklabels(), visible=False)


# #################################################################
plt.subplot(subplot_num, 1, 3)
plt.plot(command_torque - RNE_torque, label='$\\tau_{impedance}$', linewidth = 1.0,linestyle='--',c='purple')
# plt.plot(measured_torque, label='$\\tau_{measured}$', linewidth = 1.0, c='orange')
plt.plot(RNE_torque, label='$\\tau_{RNE}$', linewidth = 1.0, c='blue')
plt.plot(command_torque, label='$\\tau_{control..(RNE+impedance)}$', linewidth = 1.0,linestyle='--', c='red')
# plt.plot(command_torque - friction_torque, label='$\\tau_{control} - \\tau_{friction}$', linewidth = 1.0, c='black')
# plt.plot(friction_torque, label='$\\tau_{friction}$', linewidth = 1.0, c='green')
plt.legend(fontsize = 16, loc=1, ncol=6)
# plt.xlim(0, time_ticks[-1])
plt.xticks(np.arange(0, num_samples, control_freq))
plt.grid(True, linestyle='--')
plt.ylabel('[Nm]',  fontsize=20)
#plt.ylim(-120,120)
# plt.gca().yaxis.set_major_formatter(mtick.FormatStrFormatter('%.1e'))
plt.setp( plt.gca().get_xticklabels(), visible=False)


# #################################################################
plt.subplot(subplot_num, 1, 4)
plt.plot(friction_torque, label='$\\tau_{friction}$', linewidth = 1.0, c='green')
plt.plot(simulated_friction_torque, label='$\\tau_{sim-friction}$', linewidth = 1.0, c='black',linestyle='--',zorder=0)
plt.legend(fontsize = 16, ncol=5)
# plt.xlim(0, time_ticks[-1])
plt.xticks(np.arange(0, num_samples, control_freq))
plt.grid(True, linestyle='--')
plt.ylabel('[Nm]',  fontsize=20)
# plt.gca().yaxis.set_major_formatter(mtick.FormatStrFormatter('%.1e'))
plt.setp( plt.gca().get_xticklabels(), visible=False)
# plt.ylim(np.min(measured_current)-0.5, np.max(measured_current)+0.5)
#plt.ylim(-25,25)
# plt.yticks(np.arange(np.min(measured_current), np.max(measured_current), 0.5),  fontsize=20)


# #################################################################
plt.subplot(subplot_num, 1,5)
plt.plot(current_command, label='$I_{cmd}$', linewidth = 1.0)
plt.legend(fontsize = 16, ncol=5)
# plt.xlim(0, time_ticks[-1])
time_ticks = np.arange(0, num_samples / control_freq, 1)
plt.xticks(np.arange(0, num_samples, control_freq), time_ticks, fontsize=14)
plt.grid(True, linestyle='--')
plt.ylabel('[Amp]',  fontsize=20)
# plt.gca().yaxis.set_major_formatter(mtick.FormatStrFormatter('%.1e'))
# plt.setp( plt.gca().get_xticklabels(), visible=False)
# plt.ylim(np.min(measured_current)-0.5, np.max(measured_current)+0.5)
#plt.ylim(-25,25)

# ###########################
plt.savefig('../experiments/full_body_friction.pdf')
plt.show()

