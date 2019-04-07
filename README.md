# Highway Driving using Path Planning Algorithms

## Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Execution

The current driving lane of the host vehicle is determined first using the d value of the host vehicle taken from localization data. Each lane is assgined an ID (shown in the diagram below).

<img src="/Supporting_Files/Lane_ID.png"/>
*fig1: Diagram showing the lane IDs* 

To decide the behavior of the host vehicle, five surrounding cars are tracked. The cars identified by the following positions (also shows in figure 2):
Pos0: Car in the host lane and closes to the host vehicle.
Pos1: Car closest to the host vehicle in the lane left to the host vehicle and behind the host vehicle. 
Pos2: Car closest to the host vehicle in the lane left to the host vehicle and ahead of the host vehicle. 
Pos3: Car closest to the host vehicle in the lane right to the host vehicle and behind the host vehicle. 
Pos4: Car closest to the host vehicle in the lane right to the host vehicle and behind the host vehicle. 

<img src="/Supporting_Files/Obj_Pos.png"/>
*fig1: Diagram showing the Position IDs of surrounding vehicles* 