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
variable_num = 7
time_streach = 7

filename = "../archive/stop_motion_error.txt"

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

rows = np.shape(input_data)[0] - 2
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
time_ticks = []
control_freq = np.float32(input_data[0][0])

total_time = num_samples / control_freq
print("Total time: ", total_time)
control_freq = np.float32(input_data[0][0])/time_streach

for sample_ in range (1, rows + 1, variable_num):
    measured.append(    np.float32( input_data[    sample_][desired_dim]) )
    desired.append(     np.float32( input_data[1 + sample_][desired_dim]) )
    raw_error.append(   np.float32( input_data[2 + sample_][desired_dim] ) )
    error.append(       np.float32( input_data[3 + sample_][desired_dim] ) )
    bias.append(        np.float32( input_data[4 + sample_][desired_dim] ) )
    gain.append(        np.float32( input_data[5 + sample_][desired_dim] ) )
    command.append(     np.float32( input_data[6 + sample_][desired_dim] ) )

samples   = np.arange(0, num_samples, 1)
measured  = np.array(measured)
desired   = np.array(desired)
raw_error = np.array(raw_error)
error     = np.array(error) 
bias      = np.array(bias)
gain      = np.array(gain)
command   = np.array(command)

plt.ion()
plt.figure(figsize = (25, 12))
if  (desired_dim is 0): plt.suptitle('Stop Motion Control - Joint 1', fontsize=20)
elif(desired_dim is 1): plt.suptitle('Stop Motion Control - Joint 2', fontsize=20)
elif(desired_dim is 2): plt.suptitle('Stop Motion Control - Joint 3', fontsize=20)
elif(desired_dim is 3): plt.suptitle('Stop Motion Control - Joint 4', fontsize=20)
elif(desired_dim is 4): plt.suptitle('Stop Motion Control - Joint 5', fontsize=20)
elif(desired_dim is 5): plt.suptitle('Stop Motion Control - Joint 6', fontsize=20)
elif(desired_dim is 6): plt.suptitle('Stop Motion Control - Joint 7', fontsize=20)

# tick_freq = 0.1
# if   (num_samples > 4000 and num_samples < 7000): tick_freq = 500
# elif (num_samples < 4000 and num_samples > 1000): tick_freq = 200
# elif (num_samples < 1000 and num_samples > 500):  tick_freq = 100
# elif (num_samples < 500):                         tick_freq = 50


plt.gca().set_axis_off()
plt.subplots_adjust(hspace = 0.05, wspace = 15)
plt.subplots_adjust(left=0.07, right=0.98, top=0.95, bottom=0.07)
plt.margins(0,0)

plt.subplot(4, 1, 1)
plt.plot(measured, c = 'limegreen', label='measured velocity', linewidth = 2, zorder = 2)
plt.plot(desired, label='desired velocity', linewidth = 2, color = 'black', zorder = 3)
plt.legend(fontsize = 14, loc=0)
plt.yticks(fontsize=14)
time_ticks = np.round(np.arange(0, num_samples / control_freq, 1/time_streach), 2)
plt.xlim(0, time_ticks[-1])
plt.xticks(np.arange(0, num_samples, control_freq), " ")
plt.grid(True)
plt.ylabel(r'$[\frac{rad}{s}]$', fontsize=20)

plt.subplot(4, 1, 2)
plt.plot(raw_error, c = 'purple', label=r'input error: e', linewidth=1, zorder=2)
plt.legend(fontsize = 14)
plt.yticks(fontsize=14)
plt.xlim(0, time_ticks[-1])
plt.xticks(np.arange(0, num_samples, control_freq), " ")
plt.grid(True)
plt.ylabel(r'$[\frac{rad}{s}]$', fontsize=20)

plt.subplot(4, 1, 3)
plt.plot(error, c = 'darkorange', label=r'ABAG: low-pass filtered error sign', linewidth=1, zorder=2)
plt.legend(fontsize = 14)
plt.ylim(-1.2, 1.2)
plt.yticks(fontsize=14)
plt.xlim(0, time_ticks[-1])
plt.xticks(np.arange(0, num_samples, control_freq), " ")
plt.grid(True)
plt.ylabel('[%]', fontsize=20)


plt.subplot(4, 1, 4)
plt.plot(bias, c = 'green', label='ABAG: bias', linewidth = 2, zorder = 4)
plt.plot(gain, c = 'red', label=r'ABAG: gain * sign(e)', linewidth = 2, zorder = 2)
plt.plot(command, c = 'blue', label='ABAG: u', linewidth = 1.0, zorder = 3)
plt.yticks(fontsize=14)
plt.legend(fontsize = 14, loc=8)
plt.xlim(0, time_ticks[-1])
plt.xticks(np.arange(0, num_samples, control_freq), time_ticks, fontsize=14)
plt.grid(True)
plt.ylabel('[%]', fontsize=20)
plt.xlabel('time [s]', fontsize=20)

plt.draw()
plt.pause(0.001)
if  (desired_dim is 0):   plt.savefig('../experiments/stop_motion_1.pdf')
elif(desired_dim is 1): plt.savefig('../experiments/stop_motion_2.pdf')
elif(desired_dim is 2): plt.savefig('../experiments/stop_motion_3.pdf')
elif(desired_dim is 3): plt.savefig('../experiments/stop_motion_4.pdf')
elif(desired_dim is 4): plt.savefig('../experiments/stop_motion_5.pdf')
elif(desired_dim is 5): plt.savefig('../experiments/stop_motion_6.pdf')
elif(desired_dim is 6): plt.savefig('../experiments/stop_motion_7.pdf')

notifier.loop()