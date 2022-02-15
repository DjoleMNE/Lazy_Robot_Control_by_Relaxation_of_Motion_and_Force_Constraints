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
# control_freq = 660

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
    joint_5.append( np.float32( input_data[sample_][5] ) )
    joint_6.append( np.float32( input_data[sample_][6] ) )

# estimated external torques
for sample_ in range(2, rows - 1, 2):
    joint_0_ext.append( np.float32( input_data[sample_][0] ) )
    joint_1_ext.append( np.float32( input_data[sample_][1] ) )
    joint_2_ext.append( np.float32( input_data[sample_][2] ) )
    joint_3_ext.append( np.float32( input_data[sample_][3] ) )
    joint_4_ext.append( np.float32( input_data[sample_][4] ) )
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
joint_5_limit = np.array(np.full((rows, ), np.float32( input_data[0][5])))
joint_6_limit = np.array(np.full((rows, ), np.float32( input_data[0][6])))

tick_freq = 1000
if   (rows > 4000 and rows < 7000): tick_freq = 500
elif (rows < 4000 and rows > 1000): tick_freq = 200
elif (rows < 1000 and rows > 500): tick_freq = 100
elif (rows < 500): tick_freq = 50

plt.ion()
plt.figure(figsize = (15,15))

if (joint_num == 1):
    t1 = plt.plot(joint_0, c = 'black', label='Joint 1 torque', linewidth = 1.5, zorder = 3)
    t2 = plt.plot(joint_1, c = 'orange', label='Joint 2 torque', linewidth = 1.5, zorder = 3)
    t3 = plt.plot(joint_2, c = 'green', label='Joint 3 torque', linewidth = 1.5, zorder = 3)
    t4 = plt.plot(joint_3, c = 'purple', label='Joint 4 torque', linewidth = 1.5, zorder = 3)
    l1 = plt.plot(joint_0_limit, c = 'red',label="Upper torque limit for joints 1, 2, 3, 4", linewidth = 3.3, zorder = 1, linestyle = '--')
    l2 = plt.plot(-1 * joint_0_limit, c = 'blue',label="Lower torque limit for joints 1, 2, 3, 4", linewidth = 3.3, zorder = 1, linestyle = '--')
    plt.yticks(fontsize=20)
    plt.ylim(-1 *joint_0_limit[0] - 5, joint_0_limit[0] + 5)
    time_ticks = np.arange(0, rows / control_freq, 1)
    plt.xlim(0, time_ticks[-1])
    plt.xticks(np.arange(0, rows, control_freq), time_ticks, fontsize=20)
    # plt.setp( plt.gca().get_xticklabels(), visible=False)
    plt.grid(True, linestyle='--')
    plt.ylabel('[Nm]', fontsize=20)
    plt.xlabel('time [s]', fontsize=20)
    plt.legend(loc=0, fontsize = 16, ncol=1)

elif (joint_num == 2):
    t5 = plt.plot(joint_4, c = 'black', label='Joint 5 torque', linewidth =1.5, zorder = 3)
    t6 = plt.plot(joint_5, c = 'orange', label='Joint 6 torque', linewidth =1.5, zorder = 3)
    t7 = plt.plot(joint_6, c = 'green', label='Joint 7 torque', linewidth =1.5, zorder = 3)
    l3 = plt.plot(joint_4_limit, c = 'red', label="Upper torque limit for joints 5, 6, 7",linewidth = 3.5, zorder = 1, linestyle = '--')
    l4 = plt.plot(-1 *joint_4_limit, c = 'blue', label="Upper torque limit for joints 5, 6, 7", linewidth = 3.5, zorder = 1, linestyle = '--')
    plt.ylim(-1 *joint_4_limit[0] - 5, joint_4_limit[0] + 5)
    plt.yticks(fontsize=20)
    time_ticks = np.arange(0, rows / control_freq, 1)
    plt.xlim(0, time_ticks[-1])
    plt.xticks(np.arange(0, rows, control_freq), time_ticks, fontsize=20)
    plt.xlabel('time [s]', fontsize=20)
    plt.grid(True, linestyle='--')
    plt.ylabel('[Nm]', fontsize=20)
    plt.legend(loc=0, fontsize = 16, ncol=1)

elif (joint_num == 3):
    t1 = plt.plot(joint_0/joint_0_limit[0], label='Joint 1 torque', linewidth = 1.5, zorder = 3)
    t2 = plt.plot(joint_1/joint_0_limit[0], label='Joint 2 torque', linewidth = 1.5, zorder = 3)
    t3 = plt.plot(joint_2/joint_0_limit[0], label='Joint 3 torque', linewidth = 1.5, zorder = 3)
    t4 = plt.plot(joint_3/joint_0_limit[0], label='Joint 4 torque', linewidth = 1.5, zorder = 3)
    t5 = plt.plot(joint_4/joint_4_limit[0], label='Joint 5 torque', linewidth =1.5,  zorder = 3)
    t6 = plt.plot(joint_5/joint_4_limit[0], label='Joint 6 torque', linewidth =1.5,  zorder = 3)
    t7 = plt.plot(joint_6/joint_4_limit[0], label='Joint 7 torque', linewidth =1.5,  zorder = 3)

    l1 = plt.plot(joint_0_limit/joint_0_limit[0], c = 'red',label="Upper torque limit", linewidth =2, zorder = 1, linestyle = '--')
    l2 = plt.plot(-1 * joint_0_limit/joint_0_limit[0], c = 'blue',label="Lower torque limit", linewidth = 2, zorder = 1, linestyle = '--')

    # l1 = plt.plot(joint_0_limit/joint_0_limit[0], c = 'red',label="Upper torque limit for joints 1, 2, 3, 4", linewidth =2, zorder = 1, linestyle = '--')
    # l2 = plt.plot(-1 * joint_0_limit/joint_0_limit[0], c = 'blue',label="Lower torque limit for joints 1, 2, 3, 4", linewidth = 2, zorder = 1, linestyle = '--')

    # l3 = plt.plot(joint_4_limit/joint_4_limit[0],  label="Upper torque limit for joints 5, 6, 7",linewidth = 2, zorder = 1, linestyle = '--')
    # l4 = plt.plot(-1 *joint_4_limit/joint_4_limit[0], label="Upper torque limit for joints 5, 6, 7", linewidth = 2, zorder = 1, linestyle = '--')

    # plt.ylim(-1 *joint_0_limit[0] - 1, joint_0_limit[0] +1)
    # plt.yticks(np.arange(-joint_0_limit[0] -1, joint_0_limit[0]+2, 5),  fontsize=20)

    plt.yticks(fontsize=18)
    time_ticks = np.arange(0, rows / control_freq, 1)
    plt.xlim(0, time_ticks[-1])
    plt.xticks(np.arange(0, rows, control_freq), time_ticks, fontsize=18)
    plt.xlabel('time [s]', fontsize=18)
    plt.grid(True, linestyle='--')
    plt.ylabel('[Nm]', fontsize=18)
    plt.legend(loc=0, fontsize = 16, ncol=2)

else: 
    print("Wrong input number. Must be between 1-3.")
    sys.exit()

plt.draw()
plt.pause(0.001)
if (joint_num == 1): plt.savefig('../experiments/joint_torques_1.pdf')
elif (joint_num == 2): plt.savefig('../experiments/joint_torques_2.pdf')
elif (joint_num == 3): plt.savefig('../experiments/joint_torques_3.pdf')

notifier.loop()