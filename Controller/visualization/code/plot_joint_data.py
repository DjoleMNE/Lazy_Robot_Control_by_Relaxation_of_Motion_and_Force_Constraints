# Author(s): Djordje Vukcevic
# Year: 2019
import sys, os
import numpy as np
import sympy as sp
from sympy import *
import matplotlib.pyplot as plt
import pyinotify

joint_num = np.int(sys.argv[1])
control_freq = 700

variable_num = joint_num
filename = "../archive/joint_torques.txt"

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

joint_0 = []
joint_1 = []
joint_2 = []
joint_3 = []
joint_4 = []
joint_5 = []
joint_6 = []

joint_0_ext = []
joint_1_ext = []
joint_2_ext = []
joint_3_ext = []
joint_4_ext = []
joint_5_ext = []
joint_6_ext = []

# commanded control torques
for sample_ in range(1, rows - 1, 2):
    joint_0.append( np.float32( input_data[sample_][0] ) )
    joint_1.append( np.float32( input_data[sample_][1] ) )
    joint_2.append( np.float32( input_data[sample_][2] ) )
    joint_3.append( np.float32( input_data[sample_][3] ) )
    joint_4.append( np.float32( input_data[sample_][4] ) )
    if (joint_num > 5):
        joint_5.append( np.float32( input_data[sample_][5] ) )
        joint_6.append( np.float32( input_data[sample_][6] ) )

# estimated external torques
for sample_ in range(2, rows - 1, 2):
    joint_0_ext.append( np.float32( input_data[sample_][0] ) )
    joint_1_ext.append( np.float32( input_data[sample_][1] ) )
    joint_2_ext.append( np.float32( input_data[sample_][2] ) )
    joint_3_ext.append( np.float32( input_data[sample_][3] ) )
    joint_4_ext.append( np.float32( input_data[sample_][4] ) )
    if (joint_num > 5):
        joint_5_ext.append( np.float32( input_data[sample_][5] ) )
        joint_6_ext.append( np.float32( input_data[sample_][6] ) )

samples = np.arange(0, rows / 2, 1)
joint_0 = np.array(joint_0)
joint_1 = np.array(joint_1)
joint_2 = np.array(joint_2)
joint_3 = np.array(joint_3)
joint_4 = np.array(joint_4)
joint_5 = np.array(joint_5)
joint_6 = np.array(joint_6)

joint_0_ext = np.array(joint_0_ext)
joint_1_ext = np.array(joint_1_ext)
joint_2_ext = np.array(joint_2_ext)
joint_3_ext = np.array(joint_3_ext)
joint_4_ext = np.array(joint_4_ext)
joint_5_ext = np.array(joint_5_ext)
joint_6_ext = np.array(joint_6_ext)


rows = int(rows / 2)
joint_0_limit = np.array(np.full((rows, ), np.float32( input_data[0][0])))
joint_1_limit = np.array(np.full((rows, ), np.float32( input_data[0][1])))
joint_2_limit = np.array(np.full((rows, ), np.float32( input_data[0][2])))
joint_3_limit = np.array(np.full((rows, ), np.float32( input_data[0][3])))
joint_4_limit = np.array(np.full((rows, ), np.float32( input_data[0][4])))
if (joint_num > 5):
    joint_5_limit = np.array(np.full((rows, ), np.float32( input_data[0][5])))
    joint_6_limit = np.array(np.full((rows, ), np.float32( input_data[0][6])))

tick_freq = 1000
if   (rows > 4000 and rows < 7000): tick_freq = 500
elif (rows < 4000 and rows > 1000): tick_freq = 200
elif (rows < 1000 and rows > 500): tick_freq = 100
elif (rows < 500): tick_freq = 50

plt.ion()
fig = plt.figure(figsize = (18,10))
plt.gca().set_axis_off()
plt.subplots_adjust(hspace = 0.05, wspace = 0.2)
plt.subplots_adjust(left=0.06, right=0.98, top=0.94, bottom=0.07)
plt.margins(0,0)
plt.suptitle('Commanded and Estimated External Joint Torques', fontsize=20)

plt.subplot(joint_num, 1, 1)
t1 = plt.plot(joint_0, c = 'green', linewidth = 1.5, zorder = 3)
t2 = plt.plot(joint_0_ext, c = 'purple', linewidth = 1.5, zorder = 2)
l1 = plt.plot(joint_0_limit, c = 'red', linewidth = 2.3, zorder = 1)
l2 = plt.plot(-1 * joint_0_limit, c = 'blue', linewidth = 2.3, zorder = 1)
# plt.axvline(x = 1975.0, linewidth = 1.3,  color='blue', zorder = 1)
# plt.ylim(-1 * joint_0_limit[0] - 2, joint_0_limit[0] + 2)
plt.yticks(fontsize=14)
time_ticks = np.arange(0, rows / control_freq, 1)
plt.xlim(0, time_ticks[-1])
plt.xticks(np.arange(0, rows, control_freq), time_ticks, fontsize=22)
plt.grid(True)
plt.ylabel('[Nm]', fontsize=20)

plt.subplot(joint_num, 1, 2)
plt.plot(joint_1, c = 'green', linewidth = 1.5, zorder = 3)
plt.plot(joint_1_ext, c = 'purple', linewidth = 1.5, zorder = 2)
plt.plot(joint_1_limit, c = 'red',  linewidth = 2.3, zorder = 1)
plt.plot(-1 *joint_1_limit, c = 'blue', linewidth = 2.3, zorder = 1)
# plt.axvline(x = 1975.0, linewidth = 1.3,  color='blue', zorder = 1)
plt.ylim(-1 *joint_1_limit[0] - 2, joint_1_limit[0]+ 2)
plt.yticks(fontsize=14)
plt.xlim(0, time_ticks[-1])
plt.xticks(np.arange(0, rows, control_freq), time_ticks, fontsize=22)
plt.grid(True)
plt.ylabel('[Nm]', fontsize=20)

plt.subplot(joint_num, 1, 3)
plt.plot(joint_2, c = 'green', linewidth = 1.5, zorder = 3)
plt.plot(joint_2_ext, c = 'purple', linewidth = 1.5, zorder = 2)
plt.plot(joint_2_limit, c = 'red', linewidth = 2.5, zorder = 1)
plt.plot(-1 *joint_2_limit, c = 'blue', linewidth = 2.5, zorder = 1)
plt.ylim(-1 * joint_2_limit[0] - 2, joint_2_limit[0] + 2)
plt.yticks(fontsize=14)
# plt.axvline(x = 1975.0, linewidth = 1.3,  color='blue', zorder = 1)
plt.xlim(0, time_ticks[-1])
plt.xticks(np.arange(0, rows, control_freq), time_ticks, fontsize=22)
plt.grid(True)
plt.ylabel('[Nm]', fontsize=20)

plt.subplot(joint_num, 1, 4)
plt.plot(joint_3, c = 'green', linewidth = 1.5, zorder = 3)
plt.plot(joint_3_ext, c = 'purple',  linewidth = 1.5, zorder = 2)
plt.plot(joint_3_limit, c = 'red', linewidth = 2.5, zorder = 1)
plt.plot(-1 *joint_3_limit, c = 'blue', linewidth = 2.5, zorder = 1)
# plt.axvline(x = 1975.0, linewidth = 1.3,  color='blue', zorder = 1)
plt.ylim(-1 * joint_3_limit[0] - 1, joint_3_limit[0] + 1)
plt.yticks(fontsize=14)
plt.xlim(0, time_ticks[-1])
plt.xticks(np.arange(0, rows, control_freq), time_ticks, fontsize=22)
plt.grid(True)
plt.ylabel('[Nm]', fontsize=20)

plt.subplot(joint_num, 1, 5)
plt.plot(joint_4, c = 'green', linewidth =1.5, zorder = 3)
plt.plot(joint_4_ext, c = 'purple', linewidth = 1.5, zorder = 2)
plt.plot(joint_4_limit, c = 'red', linewidth = 2.5, zorder = 1)
plt.plot(-1 *joint_4_limit, c = 'blue', linewidth = 2.5, zorder = 1)
# plt.axvline(x = 1975.0, linewidth = 1.3,  color='blue', zorder = 1)
plt.ylim(-1 *joint_4_limit[0] - 6, joint_4_limit[0] + 6)
plt.yticks(fontsize=14)
plt.xlim(0, time_ticks[-1])
if (joint_num > 5):
    plt.xticks(np.arange(0, rows, control_freq), time_ticks, fontsize=22)
else:
    plt.xticks(np.arange(0, rows, control_freq), time_ticks, fontsize=22)
    plt.xlabel('time [s]', fontsize=20)
plt.grid(True)
plt.ylabel('[Nm]', fontsize=20)


if (joint_num > 5):
    plt.subplot(joint_num, 1, 6)
    plt.plot(joint_5, c = 'green', linewidth = 1.5, zorder = 3)
    plt.plot(joint_5_ext, c = 'purple', linewidth = 1.5, zorder = 2)
    plt.plot(joint_5_limit, c = 'red', linewidth = 2.3, zorder = 1)
    plt.plot(-1 *joint_5_limit, c = 'blue', linewidth = 2.3, zorder = 1)
    # plt.axvline(x = 1975.0, linewidth = 1.3,  color='blue', zorder = 1)
    plt.ylim(-1 * joint_5_limit[0] - 6, joint_5_limit[0] + 6)
    plt.yticks(fontsize=14)
    plt.xlim(0, time_ticks[-1])
    plt.xticks(np.arange(0, rows, control_freq), time_ticks, fontsize=22)
    plt.grid(True)
    plt.ylabel('[Nm]', fontsize=20)
    plt.xlabel('time [s]', fontsize=20)

    plt.subplot(joint_num, 1, 7)
    plt.plot(joint_6, c = 'green', linewidth = 1.5, zorder = 3)
    plt.plot(joint_6_ext, c = 'purple', linewidth = 1.5, zorder = 2)
    plt.plot(joint_6_limit, c = 'red', linewidth = 2.3, zorder = 1)
    plt.plot(-1 *joint_6_limit, c = 'blue', linewidth = 2.3, zorder = 1)
    # plt.axvline(x = 1975.0, linewidth = 1.3,  color='blue', zorder = 1)
    plt.ylim(-1 * joint_6_limit[0] - 6, joint_6_limit[0] + 6)
    plt.yticks(fontsize=14)
    plt.xlim(0, time_ticks[-1])
    plt.xticks(np.arange(0, rows, control_freq), time_ticks, fontsize=22)
    plt.grid(True)
    plt.ylabel('[Nm]', fontsize=20)
    plt.xlabel('time [s]', fontsize=20)


# fig.legend((l1[0],l2[0]), ("Upper joint torque limit", "Lower joint torque limit"), fontsize = 16)
plt.figlegend((l1[0],l2[0]), ("Upper limit for commanded torque", "Lower limit for commanded torque"), loc=2, fontsize = 12)
plt.figlegend((t1[0],t2[0]), ("Commanded torque", "External torque"), loc=1, fontsize = 12)
plt.draw()
plt.pause(0.001)
plt.savefig('../experiments/joint_torques.pdf')

notifier.loop()