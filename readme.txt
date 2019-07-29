A star algorithm with simulation on V-REP

- Libraries to be imported:

	import math
	import numpy as np
	import matplotlib.pyplot as plt
	from sys import exit
	import vrep
	import time

- The zipped folder has to be extracted and the code has to be run from the terminal of the same folder.
- To run the file, write "python3 Project3.py"

- Depending upon the operating software, the .so file or .dll file has to be updated in the folder. This upload has .so file.

INPUT (USER)
- The input for the code, is in metres.
- The code is developed with the (0,0) coordinate at the center of the map.
- The inputs for x-axis ranges between (-5.55 to 5.55) and y-axis ranges between (-5.05 to 5.05).
- There are five inputs (initial x, initial y, theta, final x, final y)

INPUT (CODE)
- The clearance from the obstacles is 0.5 m. The threshold to the final goal point circle is 0.1 m.
- Currently the input for rpm is v1 = 5 rad/sec and v2 = 10 rad/sec. The dtime in the formula is 1.5 seconds 

OBSTACLE SPACE
- The map generated has red dots for explored nodes.
- The blue dots is the path the robot will follow.

VREP
- The Turtlebot has to be manually placed according to the input given.
- Play the VREP environment and run the code in the terminal. Turtlebot will start moving after closing the obstacle space.

VIDEO UPLOAD
- The start and end point is as specified below.
- The number of iterations calculated is specified in front of it.
- The velocities generated is written below it.
- The Turtlebot roughly reaches the mentioned points mentioned below.
- The input for the videos is Zero Radians.

(-4,-4) to (1,1) --- 6040 iterations 
[[5, 10], [5, 5], [10, 5], [0, 5], [10, 10], [10, 10], [10, 10], [10, 10], [10, 10], [10, 10], [10, 10], [10, 5], [10, 10], [10, 10], [10, 10], [5, 10], [10, 0], [0, 10], [0, 10]]
Vrep final Coordinates (0.9, 0.7), Video 1


(-4,-4) to (1,4) --- 4824 iterations
[[0, 10], [10, 0], [0, 5], [10, 10], [10, 10], [10, 5], [5, 10], [10, 10], [10, 10], [10, 10], [10, 10], [10, 10], [10, 5], [5, 10], [5, 0], [5, 5], [5, 10], [5, 0], [5, 10], [10, 10], [5, 5], [5, 5], [5, 5], [5, 5], [5, 5], [10, 5], [0, 5], [0, 5]]
Vrep Final Coordinates (1.5, 3.2), Video 2


(-2, 0) to (0, 4) -- 992 iterations
[[0, 10], [5, 0], [5, 5], [10, 5], [5, 5], [5, 10], [10, 5], [5, 10], [5, 5], [5, 0], [0, 5], [10, 10], [5, 10], [10, 5], [5, 5], [5, 5], [5, 5]]
Vrep Final Coordinates (0.7, 3.5), Video 3




