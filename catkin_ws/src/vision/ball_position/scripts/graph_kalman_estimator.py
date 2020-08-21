#!/usr/bin/env python
import json
from matplotlib import pyplot as plt   

with open('kalman_data.txt') as positions_data:
    all_positions = positions_data.readlines()
    data_number = len(all_positions)

time  = 0
index = 0

time_axis  = []
exact_position_x = []
exact_position_y = []
measurent_position_x = []
measurent_position_y = []
prediction_position_x = []
prediction_position_y = []
estimation_position_x = []
estimation_position_y = []

print "Number of positions loaded->", data_number 
for position in all_positions:
    axis = json.loads(all_positions[index])

    exact_position_x.append(axis[0])
    exact_position_y.append(axis[1])
    measurent_position_x.append(axis[2])
    measurent_position_y.append(axis[3])
    prediction_position_x.append(axis[4])
    prediction_position_y.append(axis[5])
    estimation_position_x.append(axis[6])
    estimation_position_y.append(axis[7])

    index += 1
    time  += 0.03
    time_axis.append(time)   

plt.figure(1)
plt.plot(time_axis, exact_position_x, 'k.', time_axis, measurent_position_x, 'b--', time_axis, prediction_position_x, 'g*',
         time_axis, estimation_position_x, 'r+')
plt.xlabel('Time[s]')
plt.ylabel('X position [m]')
plt.title('Time vs X position')
plt.legend(('Real', 'Measured', 'Predicted', 'Estimated'), loc = 'upper right')

plt.figure(2)
plt.plot(time_axis, exact_position_y, 'k.', time_axis, measurent_position_y, 'b--', time_axis, prediction_position_y, 'g*',
         time_axis, estimation_position_y, 'r+')
plt.xlabel('Time[s]')
plt.ylabel('Y position [m]')
plt.title('Time vs Y position')
plt.legend(('Real', 'Measured', 'Predicted', 'Estimated'), loc = 'upper right')

plt.show()