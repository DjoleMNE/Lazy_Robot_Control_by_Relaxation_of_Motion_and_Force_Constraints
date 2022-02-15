# Author(s): Djordje Vukcevic
# Year: 2019
import sys, os
import numpy as np
import sympy as sp
from sympy import *
import matplotlib.pyplot as plt
import pyinotify

desired_dim = np.int(sys.argv[1])
show_tube = np.int(sys.argv[2])

print("Selected dimension: ", desired_dim)
variable_num = 7

control_freq = 999

filename = "../archive/control_error.txt"
prediction_filename = "../archive/prediction_effects.txt"

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
wdd_2 = wm.add_watch(prediction_filename, pyinotify.IN_CLOSE_WRITE)

with open(filename, "r") as f:
    all_data = [x.split() for x in f.readlines()]
    input_data = np.array(all_data)[:-2]

with open(prediction_filename, "r") as f:
    all_data_predictions = [x.split() for x in f.readlines()]
    input_data_predictions = np.array(all_data_predictions)[:-1]

rows = np.shape(input_data)[0] - 2
cols = np.shape(input_data[0])[0]
rows = rows - (rows % variable_num)
num_samples = np.int(rows / variable_num) 
print("Data size: ", num_samples, ",", cols)

measured  = []
desired   = []
predicted = []
raw_error = []
error     = []
bias      = []
gain      = []
command   = []
contact_time_tick = []
time_ticks = []
for sample_ in range (2, rows + 2, variable_num):
    tick = np.float32( input_data[sample_][11] )
    if (tick > 0): contact_time_tick.append(tick)

    measured.append(    np.float32( input_data[    sample_][desired_dim]) )
    desired.append(     np.float32( input_data[1 + sample_][desired_dim]) )
    if (desired_dim == 6):
        raw_error.append(   np.float32( input_data[2 + sample_][6] ) )
        error.append(       np.float32( input_data[3 + sample_][0] ) )
        bias.append(        np.float32( input_data[4 + sample_][0] ) )
        gain.append(        np.float32( input_data[5 + sample_][0] ) )
        command.append(     np.float32( input_data[6 + sample_][0] ) )

    elif (desired_dim == 7):
        raw_error.append(   np.float32( input_data[2 + sample_][5] ) )
        error.append(       np.float32( input_data[3 + sample_][5] ) )
        bias.append(        np.float32( input_data[4 + sample_][5] ) )
        gain.append(        np.float32( input_data[5 + sample_][5] ) )
        command.append(     np.float32( input_data[6 + sample_][5] ) )
    
    elif (desired_dim == 8):
        raw_error.append(   np.float32( input_data[2 + sample_][2] ) )
        error.append(       np.float32( input_data[3 + sample_][2] ) )
        bias.append(        np.float32( input_data[4 + sample_][2] ) )
        gain.append(        np.float32( input_data[5 + sample_][2] ) )
        command.append(     np.float32( input_data[6 + sample_][2] ) )

    elif (desired_dim == 9):
        raw_error.append(   np.float32( input_data[2 + sample_][3] ) )
        error.append(       np.float32( input_data[3 + sample_][3] ) )
        bias.append(        np.float32( input_data[4 + sample_][3] ) )
        gain.append(        np.float32( input_data[5 + sample_][3] ) )
        command.append(     np.float32( input_data[6 + sample_][3] ) )

    elif (desired_dim == 10):
        raw_error.append(   np.float32( input_data[2 + sample_][4] ) )
        error.append(       np.float32( input_data[3 + sample_][4] ) )
        bias.append(        np.float32( input_data[4 + sample_][4] ) )
        gain.append(        np.float32( input_data[5 + sample_][4] ) )
        command.append(     np.float32( input_data[6 + sample_][4] ) )
 
    else:
        raw_error.append(   np.float32( input_data[2 + sample_][desired_dim] ) )
        error.append(       np.float32( input_data[3 + sample_][desired_dim] ) )
        bias.append(        np.float32( input_data[4 + sample_][desired_dim] ) )
        gain.append(        np.float32( input_data[5 + sample_][desired_dim] ) )
        command.append(     np.float32( input_data[6 + sample_][desired_dim] ) )

if (desired_dim < 4):
    for sample_ in range (0, num_samples):
        predicted.append(np.float32(input_data_predictions[sample_][desired_dim]))

samples   = np.arange(0, num_samples, 1)
measured  = np.array(measured)
desired   = np.array(desired)
predicted = np.array(predicted)
raw_error = np.array(raw_error)
error     = np.array(error) 
bias      = np.array(bias)
gain      = np.array(gain)
command   = np.array(command)

plt.ion()
plt.figure(figsize = (18, 10))
if(desired_dim is 0):   plt.suptitle('Robot Position in Linear X Direction', fontsize=20)
elif(desired_dim is 1): plt.suptitle('Control of Robot Position in Linear Y Direction', fontsize=20)
elif(desired_dim is 2): plt.suptitle('Control of Robot Position in Linear Z Direction', fontsize=20)
elif(desired_dim is 3): plt.suptitle('Control of Robot Angular X Direction', fontsize=20)
elif(desired_dim is 4): plt.suptitle('Control of Robot Angular Y Direction', fontsize=20)
elif(desired_dim is 5): plt.suptitle('Control of Robot Angular Z Direction', fontsize=20)
elif(desired_dim is 6): plt.suptitle('Control of Robot Velocity in Linear X Direction', fontsize=20)
elif(desired_dim is 7): plt.suptitle('Control of Robot Velocity in Angular Z Direction', fontsize=20)
elif(desired_dim is 8): plt.suptitle('Control of Robot Force in Linear Z Direction', fontsize=20)
elif(desired_dim is 9): plt.suptitle('Control of Robot Moment in Angular X Direction', fontsize=20)
elif(desired_dim is 10): plt.suptitle('Control of Robot Moment in Angular Y Direction', fontsize=20)

# tick_freq = 0.1
# if   (num_samples > 4000 and num_samples < 7000): tick_freq = 500
# elif (num_samples < 4000 and num_samples > 1000): tick_freq = 200
# elif (num_samples < 1000 and num_samples > 500):  tick_freq = 100
# elif (num_samples < 500):                         tick_freq = 50

tube_tolerance = np.array(np.full((num_samples, ), np.float32( 0.0 )))
if (desired_dim is 0):
    plt.figure(figsize=(25,5))
    plt.title('Monitored Robot Position in Linear X Direction', fontsize=20)
    plt.plot(measured, c = 'limegreen', label='measured position', linewidth = 2, zorder = 2)
    plt.plot(desired, label='desired position', linewidth = 2, color = 'black', zorder = 3)
    if(show_tube):
        tube_tolerance = np.array(np.full((num_samples, ), np.float32( input_data[0][desired_dim] )))
    plt.plot(desired + tube_tolerance, c = 'red', label='upper tolerance', linewidth = 1.3, linestyle = '--', zorder = 2)
    plt.plot(desired - tube_tolerance, c = 'blue', label='lower tolerance', linewidth = 1.3, linestyle = '--', zorder = 2)       
    plt.legend(fontsize = 14)
    for tick in contact_time_tick:
        plt.axvline(x = contact_time_tick[0], linewidth = 1.3,  color='blue', zorder = 1)
    print(tick)
    plt.yticks(fontsize=14)
    if (control_freq > num_samples):
        time_ticks = np.arange(0, num_samples, 0.5)
    else:
        time_ticks = np.arange(0, num_samples / control_freq, 0.5)

    plt.xlim(0, time_ticks[-1])
    # plt.xticks(np.arange(0, num_samples, control_freq), time_ticks, fontsize=14)
    plt.grid(True)
    plt.ylabel('[m]', fontsize=20)
    plt.xlabel('time [s]', fontsize=20)

else:
    plt.gca().set_axis_off()
    plt.subplots_adjust(hspace = 0.05, wspace = 15)
    plt.subplots_adjust(left=0.07, right=0.98, top=0.95, bottom=0.07)
    plt.margins(0,0)

    plt.subplot(4, 1, 1)
    if(desired_dim < 3):
        plt.plot(measured, c = 'limegreen', label='measured position', linewidth = 2, zorder = 2)
        plt.plot(predicted, c = 'orange', label='predicted position', linewidth = 2, zorder = 2)
        plt.plot(desired, label='desired  position', linewidth = 2, color = 'black', zorder = 3)

    elif(desired_dim == 3):
        plt.plot(measured, c = 'limegreen', label='measured angle', linewidth = 2, zorder = 2)
        plt.plot(predicted, c = 'orange', label='predicted angle', linewidth = 2, zorder = 2)
        plt.plot(desired, label='desired angle', linewidth = 2, color = 'black', zorder = 3)

    elif(desired_dim == 6):
        plt.plot(measured, c = 'limegreen', label='measured velocity', linewidth = 2, zorder = 2)
        plt.plot(desired, label='desired velocity', linewidth = 2, color = 'black', zorder = 3)

    elif(desired_dim == 7):
        plt.plot(measured, c = 'limegreen', label='measured angular vel', linewidth = 2, zorder = 2)
        plt.plot(desired, label='desired angle', linewidth = 2, color = 'black', zorder = 3)

    elif(desired_dim == 8):
        plt.plot(measured, c = 'limegreen', label='measured force', linewidth = 2, zorder = 2)
        plt.plot(desired, label='desired force', linewidth = 2, color = 'black', zorder = 3)

    else:
        plt.plot(measured, c = 'limegreen', label='measured moment', linewidth = 2, zorder = 2)
        plt.plot(desired, label='desired moment', linewidth = 2, color = 'black', zorder = 3) 

    if (show_tube):
        if (desired_dim > 7):
            tube_tolerance = np.array(np.full((num_samples, ), np.float32( input_data[0][desired_dim-6] )))
        else:
            tube_tolerance = np.array(np.full((num_samples, ), np.float32( input_data[0][desired_dim] )))
        if (desired_dim==6):
            plt.plot(desired + tube_tolerance, c = 'red', label='upper tolerance', linewidth = 1.0, linestyle = '--', zorder = 2)
            plt.plot(desired - tube_tolerance, c = 'blue', label='lower tolerance', linewidth = 1.0, linestyle = '--', zorder = 2)
        else:
            plt.plot(desired + tube_tolerance, c = 'red', label='upper tolerance', linewidth = 2.5, linestyle = '--', zorder = 2)
            plt.plot(desired - tube_tolerance, c = 'blue', label='lower tolerance', linewidth = 2.5, linestyle = '--', zorder = 2)
        for tick in contact_time_tick:
            plt.axvline(x = tick, linewidth = 1.3,  color='blue', zorder = 1)

    # if (desired_dim == 8):
        # plt.ylim(desired[0] - tube_tolerance[0] - 0.1, desired[0] - tube_tolerance[0] + 0.1)
    plt.legend(fontsize = 14, loc=0)
    plt.yticks(fontsize=14)
    time_ticks = np.arange(0, num_samples / control_freq, 1)
    plt.xlim(0, time_ticks[-1])
    plt.xticks(np.arange(0, num_samples, control_freq))
    plt.grid(True)

    if (desired_dim == 6):
        plt.ylabel(r'$[\frac{m}{s}]$', fontsize=20)
    elif (desired_dim == 8):
        plt.ylabel('[N]', fontsize=20)
    elif (desired_dim > 8):
        plt.ylabel('[Nm]',  fontsize=22)
    elif (desired_dim == 3):
        plt.ylabel('[rad]', fontsize=20)
    else:
        plt.ylabel('[m]', fontsize=20)

    plt.subplot(4, 1, 2)
    plt.plot(raw_error, c = 'purple', label=r'input error: e', linewidth=1, zorder=2)
    for tick in contact_time_tick:
        plt.axvline(x = tick, linewidth = 1.3,  color='blue', zorder = 1)
    plt.legend(fontsize = 14)
    # if (desired_dim==8): plt.ylim(-0.20, 0.10)
    plt.yticks(fontsize=14)
    plt.xlim(0, time_ticks[-1])
    plt.xticks(np.arange(0, num_samples, control_freq))
    plt.grid(True)
    plt.ylabel('[m]', fontsize=20)

    if (desired_dim == 6):
        plt.ylabel(r'$[\frac{m}{s}]$', fontsize=20)
    elif (desired_dim == 8):
        plt.ylabel('[N]', fontsize=20)
    elif (desired_dim > 8):
        plt.ylabel('[rad]', fontsize=20)
    else:
        plt.ylabel('[m]', fontsize=20)

    plt.subplot(4, 1, 3)
    plt.plot(error, c = 'darkorange', label=r'ABAG: low-pass filtered error sign', linewidth=1, zorder=2)
    for tick in contact_time_tick:
        plt.axvline(x = tick, linewidth = 1.3,  color='blue', zorder = 1)
    plt.legend(fontsize = 14)
    plt.ylim(-1.2, 1.2)
    plt.yticks(fontsize=14)
    plt.xlim(0, time_ticks[-1])
    plt.xticks(np.arange(0, num_samples, control_freq), time_ticks,  fontsize=22)
    plt.grid(True)
    plt.ylabel('[%]', fontsize=20)


    plt.subplot(4, 1, 4)
    plt.plot(bias, c = 'green', label='ABAG: bias', linewidth = 2, zorder = 4)
    plt.plot(gain, c = 'red', label=r'ABAG: gain * sign(e)', linewidth = 2, zorder = 2)
    plt.plot(command, c = 'blue', label='ABAG: u', linewidth = 1.0, zorder = 3)
    for tick in contact_time_tick:
        plt.axvline(x = tick, linewidth = 1.3,  color='blue', zorder = 1)
    
    # if(desired_dim is not 8):
    #     plt.ylim(-0.70, 0.70)
    #     plt.yticks(np.arange(-0.70, 0.70, 0.25), fontsize=14)
    plt.yticks(fontsize=14)
    plt.legend(fontsize = 14, loc=8)
    plt.xlim(0, time_ticks[-1])
    plt.xticks(np.arange(0, num_samples, control_freq), time_ticks, fontsize=14)
    plt.grid(True)
    plt.ylabel('[%]', fontsize=20)
    plt.xlabel('time [s]', fontsize=20)


# for i in range (len(bias)):
#     print(bias[i], end='   ')
# print("\n \n \n \n \n \n \n \n \n \n \n \n \n")

plt.draw()
plt.pause(0.001)
if(desired_dim is 0):   plt.savefig('../experiments/x_linear_control.pdf')
elif(desired_dim is 1): plt.savefig('../experiments/y_linear_control.pdf')
elif(desired_dim is 2): plt.savefig('../experiments/z_linear_control.pdf')
elif(desired_dim is 3): plt.savefig('../experiments/x_angular_control.pdf')
elif(desired_dim is 4): plt.savefig('../experiments/y_angular_control.pdf')
elif(desired_dim is 5): plt.savefig('../experiments/z_angular_control.pdf')
elif(desired_dim is 6): plt.savefig('../experiments/x_linear_velocity_control.pdf')
elif(desired_dim is 7): plt.savefig('../experiments/z_angular_velocity_control.pdf')
elif(desired_dim is 8): plt.savefig('../experiments/z_linear_force_control.pdf')
elif(desired_dim is 9): plt.savefig('../experiments/x_angular_force_control.pdf')
elif(desired_dim is 10): plt.savefig('../experiments/y_angular_force_control.pdf')

notifier.loop()