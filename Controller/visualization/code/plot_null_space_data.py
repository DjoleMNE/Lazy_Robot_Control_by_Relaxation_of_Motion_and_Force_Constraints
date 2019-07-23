# Author(s): Djordje Vukcevic
# Year: 2019
import sys, os
import numpy as np
import sympy as sp
from sympy import *
import matplotlib.pyplot as plt
import pyinotify

# show_tube = np.int(sys.argv[1])
variable_num = 7

filename = "../null_space_error.txt"

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
cols = variable_num
print("Data size: ", rows, ",", cols)

measured  = []
desired   = []
raw_error = []
error     = []
bias      = []
gain      = []
command   = []
contact_time_tick = []

for sample_ in range (0, rows):
    tick = np.float32( input_data[sample_][7] )
    if (tick > 0): contact_time_tick.append(tick)
    measured.append(  np.float32( input_data[sample_][0] ) )
    desired.append(   np.float32( input_data[sample_][1] ) )
    raw_error.append( np.float32( input_data[sample_][2] ) )
    error.append(     np.float32( input_data[sample_][3] ) )
    bias.append(      np.float32( input_data[sample_][4] ) )
    gain.append(      np.float32( input_data[sample_][5] ) )
    command.append(   np.float32( input_data[sample_][6] ) )

measured  = np.array(measured)
desired   = np.array(desired)
raw_error = np.array(raw_error)
error     = np.array(error) 
bias      = np.array(bias)
gain      = np.array(gain)
command   = np.array(command)

plt.ion()
plt.figure(figsize = (18, 10))
plt.suptitle('Null Space Control', fontsize=20)

tick_freq = 1000
if (rows > 4000 and rows < 7000): tick_freq = 500
elif (rows < 4000 and rows > 1000): tick_freq = 200
elif (rows < 1000 and rows > 500): tick_freq = 100
elif (rows < 500): tick_freq = 50

plt.gca().set_axis_off()
plt.subplots_adjust(hspace = 0.02, wspace = 15)
plt.subplots_adjust(left=0.05, right=0.99, top=0.95, bottom=0.03)
plt.margins(0,0)

plt.subplot(4, 1, 1)
plt.plot(measured, c = 'limegreen', label='Measured', linewidth = 2, zorder = 2)
plt.plot(desired, label='Desired', linewidth = 2, color = 'black', zorder = 3)
for tick in contact_time_tick:
    plt.axvline(x = tick, linewidth = 1.3,  color='blue')

# tube_tolerance = np.array(np.full((rows, ), np.float32(0.5235)))
# plt.plot(desired + tube_tolerance, c = 'red', label='tube_upper_limit', linewidth = 1.3, linestyle = '--', zorder = 2)
# plt.plot(desired - tube_tolerance, c = 'blue', label='tube_lower_limit', linewidth = 1.3, linestyle = '--', zorder = 2)
# plt.ylim(desired[0] - tube_tolerance[0] - 0.1, desired[0] - tube_tolerance[0] + 0.1)

plt.legend(loc=4, fontsize = 'x-large')
plt.xticks(np.arange(0, rows, tick_freq))
plt.grid(True)

plt.subplot(4, 1, 2)
plt.plot(raw_error, c = 'orange', label=r'raw error: $e = y_d - y_k$', linewidth=1, zorder=2)
for tick in contact_time_tick:
    plt.axvline(x = tick, linewidth = 1.3,  color='blue')
plt.legend(fontsize = 'x-large')
plt.xticks(np.arange(0, rows, tick_freq))
plt.grid(True)

plt.subplot(4, 1, 3)
plt.plot(error, c = 'orange', label=r'low-pass filtered error sign: $\bar{e}$', linewidth=1, zorder=2)
for tick in contact_time_tick:
    plt.axvline(x = tick, linewidth = 1.3,  color='blue')
plt.legend(fontsize = 'x-large')
plt.ylim(-1.2, 1.2)
plt.xticks(np.arange(0, rows, tick_freq))
plt.grid(True)

plt.subplot(4, 1, 4)
plt.plot(bias, c = 'green', label='bias', linewidth = 2, zorder = 3)
plt.plot(gain, c = 'red', label=r'gain * sign(e)', linewidth = 2, zorder = 2)
plt.plot(command, c = 'blue', label='u', linewidth = 1.0, zorder = 4)
for tick in contact_time_tick:
    plt.axvline(x = tick, linewidth = 1.3,  color='blue')
plt.ylim(-1.05, 1.05)
plt.yticks(np.arange(-1.0, 1.05, 0.25))

plt.legend(fontsize = 'x-large')
plt.xticks(np.arange(0, rows, tick_freq))
plt.grid(True)

plt.draw()
plt.pause(0.001)
plt.savefig('../null_space_control.pdf')

notifier.loop()