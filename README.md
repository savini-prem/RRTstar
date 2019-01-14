## RRTstar in C 
Sampling-based path planning algorithm known as RRT*, which is an asymptotically optimal version of its probabilistically
complete counterpart RRT (rapidly exploring random trees). C implementation of algorithm 6 in arXiv:1105.1186. 

See plots below for sample output with different goal regions, where the initial location is (0.1, 0.9). 
![plot1](https://user-images.githubusercontent.com/44141658/51094535-ea402780-177b-11e9-8635-810cf65c85da.png)

![plot2](https://user-images.githubusercontent.com/44141658/51094541-f62be980-177b-11e9-9d21-25229f930c0e.png)

![plot3](https://user-images.githubusercontent.com/44141658/51094547-05129c00-177c-11e9-884f-e8112d410db9.png)

![plot4](https://user-images.githubusercontent.com/44141658/51094555-1491e500-177c-11e9-9beb-8287ff660a64.png)

## Instructions: 
1. Clone this repository and copy all code. 
2. Compile using g++ compiler
3. Run main.c (output: coordinates of all paths, including one feasible path)
4. Run plot.py (output: plot of obstacles, goal region, and all paths)

## References: 
Karaman, S., & Frazzoli, E. (2011). Sampling-Based Algorithms for Optimal Motion 
    Planning. Robotics. 
