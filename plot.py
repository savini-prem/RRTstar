import matplotlib.pyplot as plt
import numpy as np

plt.axis('equal')

#PLOT ENVIRONMENT
plt.plot([0, 0, 1, 1, 0], [0, 1, 1, 0, 0], marker='', color='black', linewidth=2)  

#PLOT OBSTACLES 
plt.plot([0.3, 0.3, 0.7, 0.7, 0.3], [0.2, 0.0, 0.0, 0.2, 0.2], marker='', color='olive', linewidth=2) 
plt.plot([0.4, 0.4, 0.6, 0.6, 0.4], [1.0, 0.7, 0.7, 1.0, 1.0], marker='', color='olive', linewidth=2)  

#PLOT GOAL REGIONS 
plt.plot([0.1, 0.1, 0.3, 0.1], [0.9, 0.7, 0.7, 0.9], marker='', color='blue', linewidth=2)  
#plt.plot([0.7, 0.7, 0.9, 0.7], [0.9, 0.7, 0.7, 0.9], marker='', color='blue', linewidth=2)  
#plt.plot([0.7, 0.7, 0.9, 0.7], [0.5, 0.3, 0.3, 0.5], marker='', color='blue', linewidth=2)  
#plt.plot([0.3, 0.3, 0.5, 0.3], [0.5, 0.3, 0.3, 0.5], marker='', color='blue', linewidth=2)  

#OPEN PATH FILE & PLOT PATHS
file = open('paths.txt', 'r') 

while True: 
	line = file.readline()
	if not line: break;

	line = line.rstrip(' \n')
	x = line.split(' ')
	x = map(float,x)

	line = file.readline()
	line = line.rstrip(' \n')
	y = line.split(' ')
	y = map(float,y)

	plt.plot(x, y, marker='', color='red', linewidth=0.5)  

#PLOT INIT
plt.plot( x[-1], y[-1], marker='o', markerfacecolor='red', markersize=2)

#SHOW PLOT & SAVE
fig = plt.gcf()
fig.savefig('plot.png')
plt.show()