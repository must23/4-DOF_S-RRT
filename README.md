# Velocity and Wheel Slip Aware Off-Road Path Planning for Autonomous Vehicles

A thesis project in off-road path planning for self-driving cars. The project utilizes simulated sensors reading data from RGB camera, IMU sensors, and GPS data to be fed into machine learning and deep learning model to predict continuous wheel slip values and classify terrains. The estimated value is used to determine whether an area in the map is feasible (within the safety slip threshold) to traverse using a modified
RRT path planning algorithm. The modified RRT introduces a higher sampling dimension, a velocity as the fourth state spaces of the vehicle compared to the original RRT which only takes coordinate and heading angles. The application is crucial for off-road autonomous navigation for safe navigation and optimizing trajectory planning. Part of the result of this project is published at ROSE 2022 conference[(Link)](https://ieeexplore.ieee.org/document/9977432/).


<figure>
  <img
  src="https://github.com/must23/4-DOF_S-RRT/blob/main/documents/data_collection.gif"
  alt="Figure 1 data collection.">
</figure>

## Required packages
- [Pygame](https://pypi.org/project/pygame/)
- [Scipy interpolate](https://pypi.org/project/scipy/)
- [Dubins python wrapper](https://github.com/AndrewWalker/Dubins-Curves)


## Important files
- `src/RRT.py`: To run the 4DOF-S-RRT  
- `src/RRTbasePy.py`: Code to write RRT Graph and map class 
- `src/diffRobot.py` : Containts differential robot methods
- `src/tester.py`: This file containts methods to be written for getting simulated environment info such asa slope, and it is still under development. 
- `src/utils.py`: Matplotlib based plotting utilities.

## Procedure to replicate experiment
1.	Run RRT.py
2.	To change the map, and map size, edit the mapDimension and change the directory of the map image in RRTbasePy.py, 



<figure>
  <img
  src="https://github.com/must23/4-DOF_S-RRT/blob/main/documents/proposed_4DOF_SRRT.png"
  alt="Figure 2 data collection.">
</figure>
