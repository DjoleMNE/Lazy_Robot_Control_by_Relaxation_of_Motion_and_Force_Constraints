# Author(s): Djordje Vukcevic
# Year: 2019
import sys, os
import numpy as np
import sympy as sp
from sympy import *
import matplotlib.pyplot as plt
import pyinotify

desired_dim = np.int(sys.argv[1])
control_freq = 660/2
# show_tube = np.int(sys.argv[2])

print("Selected dimension: ", desired_dim)
variable_num = 7

filename = "../archive/compensation_data.txt"

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

bias          = []
gain          = []
filtered_bias = []
variance_bias = []
variance_gain = []
slope_bias    = []
contact_time_tick = []

for sample_ in range (1, rows + 1, variable_num):
    bias.append(          np.float32( input_data[    sample_][desired_dim] ) )
    gain.append(          np.float32( input_data[1 + sample_][desired_dim] ) )
    filtered_bias.append( np.float32( input_data[2 + sample_][desired_dim] ) )
    variance_bias.append( np.float32( input_data[3 + sample_][desired_dim] ) )
    variance_gain.append( np.float32( input_data[4 + sample_][desired_dim] ) )
    slope_bias.append(    np.float32( input_data[5 + sample_][desired_dim] ) )
    tick                = np.float32( input_data[6 + sample_][          0] )
    if (tick > 0): contact_time_tick.append(tick)

samples       = np.arange(0, num_samples, 1)
bias          = np.array(bias)
gain          = np.array(gain)
filtered_bias = np.array(filtered_bias)
variance_bias = np.array(variance_bias) 
variance_gain = np.array(variance_gain)
slope_bias    = np.array(slope_bias)

plt.ion()
plt.figure(figsize = (18, 10))
if(desired_dim is 0):   plt.suptitle('Compensation in Linear X direction', fontsize=20)
elif(desired_dim is 1): plt.suptitle('Compensation in Linear Y direction', fontsize=20)
elif(desired_dim is 2): plt.suptitle('Compensation in Linear Z direction', fontsize=20)
elif(desired_dim is 3): plt.suptitle('Compensation in Angular X direction', fontsize=20)
elif(desired_dim is 4): plt.suptitle('Compensation in Angular Y direction', fontsize=20)
elif(desired_dim is 5): plt.suptitle('Compensation in Angular Z direction', fontsize=20)


tick_freq = 1000
if   (num_samples > 4000 and num_samples < 7000): tick_freq = 500
elif (num_samples < 4000 and num_samples > 1000): tick_freq = 200
elif (num_samples < 1000 and num_samples > 500): tick_freq = 100
elif (num_samples < 500): tick_freq = 50

plt.gca().set_axis_off()
plt.subplots_adjust(hspace = 0.02, wspace = 15)
plt.subplots_adjust(left=0.05, right=0.99, top=0.95, bottom=0.03)
plt.margins(0,0)

plt.subplot(4, 1, 1)
plt.plot(bias, c = 'green', label='bias', linewidth = 2, zorder = 4)
plt.plot(gain, c = 'red', label=r'gain * sign(e)', linewidth = 2, zorder = 2)
plt.plot(filtered_bias, c = 'blue', label='filered_bias', linewidth = 1.0, zorder = 3)
for tick in contact_time_tick:
    plt.axvline(x = tick, linewidth = 1.2,  color='blue', zorder = 4)
# plt.ylim(-1.05, 1.05)
# plt.yticks(np.arange(-1.0, 1.05, 0.25))
plt.legend(fontsize = 'x-large')
time_ticks = np.arange(0, num_samples / control_freq, 0.5)
plt.xlim(0, time_ticks[-1])
plt.xticks(np.arange(0, num_samples, control_freq), " ")
# plt.xticks(np.arange(0, num_samples, tick_freq))
plt.grid(True)

plt.subplot(4, 1, 2)
plt.plot(variance_bias, c = 'limegreen', label='bias variance', linewidth = 2, zorder = 2)
tube_tolerance = np.array(np.full((num_samples, ), np.float32( input_data[0][0] )))
plt.plot(tube_tolerance, c = 'blue', label='tube_upper_limit', linewidth = 1.3, linestyle = '--', zorder = 2)
tube_tolerance_2 = np.array(np.full((num_samples, ), 0.0))
plt.plot(tube_tolerance_2, c = 'blue', label='tube_upper_limit', linewidth = 1.3, linestyle = '--', zorder = 2)
for tick in contact_time_tick:
    plt.axvline(x = tick, linewidth = 1.2,  color='blue', zorder = 4)
plt.legend(loc=4, fontsize = 'x-large')
plt.xlim(0, time_ticks[-1])
plt.xticks(np.arange(0, num_samples, control_freq), " ")
plt.ylim(-0.0001, 0.0003)
plt.grid(True)

plt.subplot(4, 1, 3)
plt.plot(variance_gain, label='gain variance', linewidth = 2, color = 'black', zorder = 3)
tube_tolerance = np.array( np.full((num_samples, ), np.float32( input_data[0][1] )))
plt.plot(tube_tolerance, c = 'blue', label='tube_upper_limit', linewidth = 1.3, linestyle = '--', zorder = 2)
tube_tolerance_2 = np.array(np.full((num_samples, ), 0.0))
plt.plot(tube_tolerance_2, c = 'blue', label='tube_upper_limit', linewidth = 1.3, linestyle = '--', zorder = 2)
for tick in contact_time_tick:
    plt.axvline(x = tick, linewidth = 1.2,  color='blue', zorder = 4)
plt.legend(loc=4, fontsize = 'x-large')
plt.xlim(0, time_ticks[-1])
plt.xticks(np.arange(0, num_samples, control_freq), " ")
plt.ylim(-0.001, 0.003)
plt.grid(True)

plt.subplot(4, 1, 4)
plt.plot(slope_bias, c = 'red', label='bias slope', linewidth=1.7, zorder=2)
tube_tolerance = np.array(np.full((num_samples, ), np.float32( input_data[0][2] )))
plt.plot(tube_tolerance, c = 'blue', label='tube_upper_limit', linewidth = 1.3, linestyle = '--', zorder = 2)
plt.plot(-tube_tolerance, c = 'blue', label='tube_upper_limit', linewidth = 1.3, linestyle = '--', zorder = 2)
for tick in contact_time_tick:
    plt.axvline(x = tick, linewidth = 1.2,  color='blue', zorder = 4)
plt.legend(fontsize = 'x-large')
plt.xlim(0, time_ticks[-1])
plt.xticks(np.arange(0, num_samples, control_freq), time_ticks, fontsize=14)
plt.grid(True)
plt.ylim(-0.00005, 0.00005)


plt.draw()
plt.pause(0.001)
# if(desired_dim is 0):   plt.savefig('../archive/x_linear_control.pdf')
# elif(desired_dim is 1): plt.savefig('../archive/y_linear_control.pdf')
# elif(desired_dim is 2): plt.savefig('../archive/z_linear_control.pdf')
# elif(desired_dim is 3): plt.savefig('../archive/x_angular_control.pdf')
# elif(desired_dim is 4): plt.savefig('../archive/y_angular_control.pdf')
# elif(desired_dim is 5): plt.savefig('../archive/z_angular_control.pdf')
# elif(desired_dim is 6): plt.savefig('../archive/x_linear_velocity_control.pdf')
# elif(desired_dim is 7): plt.savefig('../archive/z_angular_velocity_control.pdf')
# elif(desired_dim is 8): plt.savefig('../archive/z_linear_force_control.pdf')
# elif(desired_dim is 9): plt.savefig('../archive/x_angular_force_control.pdf')
# elif(desired_dim is 10): plt.savefig('../archive/y_angular_force_control.pdf')

notifier.loop()