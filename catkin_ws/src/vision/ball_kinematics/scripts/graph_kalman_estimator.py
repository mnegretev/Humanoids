#!/usr/bin/env python

import json
from matplotlib import pyplot as plt   

with open('kalman_data.txt') as positions_data:
    number_positions = positions_data.readlines()

index = 0
t = 0
time = []
position_x = []
position_y = []
estimator_x = []
estimator_y = []
predictor_x = []
predictor_y = []

print "Number of positions loaded->", len(number_positions)
for i in number_positions:
    axis = json.loads(number_positions[index])
    #print "axis", axis
    t = t + axis[4]
    time.append(t)	 
    position_x.append(axis[0])
    position_y.append(axis[1])
    estimator_x.append(axis[5])
    estimator_y.append(axis[6])
    predictor_x.append(axis[7])
    predictor_y.append(axis[8])
    index+=1

print "data size: ", len(axis)

plt.figure(1)
plt.plot(time, position_x, 'k.', time, estimator_x, 'b^', time, predictor_x, 'g*')
plt.xlabel('Time [s]')
plt.ylabel(' X position [m]')
plt.title('X Position vs Time')
plt.legend(('Measurement', 'Estimation', 'Prediction'), loc='upper right')

plt.figure(2)
plt.plot(time, position_y, 'k.', time, estimator_y, 'b^', time, predictor_y, 'g*')
plt.xlabel('Time [s]')
plt.ylabel(' Y position [m]')
plt.title('Y Position vs Time')
plt.legend(('Measurement', 'Estimation', 'Prediction'), loc='upper right')
plt.show()