#!/usr/bin/env python

import json
from matplotlib import pyplot as plt   

with open('positions_data.txt') as positions_data:
    number_positions = positions_data.readlines()

index = 0
t = 0
time = []
position_x = []
position_y = []

print "Number of positions saved->", len(number_positions)
for i in number_positions:
    axis = json.loads(number_positions[index])
    
    t = t + axis[4]
    time.append(t)
    position_x.append(axis[0])
    position_y.append(axis[1])
    index+=1

print "data size: ", len(axis)

plt.figure(1)
plt.plot(time, position_x, color='k', linestyle='', marker='.')
plt.xlabel('Time [s]')
plt.ylabel(' X position [m]')
plt.title('X Position vs Time')

plt.figure(2)
plt.plot(time, position_y, color='k', linestyle='', marker='.')
plt.xlabel('Time [s]')
plt.ylabel(' Y position [m]')
plt.title('Y Position vs Time')
plt.show()

plt.show()
