# Highway Driving using Path Planning Algorithms

## Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Execution

The current driving lane of the host vehicle is determined first using the d value of the host vehicle taken from localization data. Each lane is assgined an ID (shown in the diagram below).

||<space><space>Lane 0<space><space>|<space><space>Lane 1<space><space>|<space><space>Lane 2<space><space>|<br>
||<space><space><space><space><space><space><space><space><space><space>|<space><space><space><space><space><space><space><space><space><space>|<space><space><space><space><space><space><space><space><space><space>|<br>
||<space><space><space><space><space><space><space><space><space><space>|<space><space><space><space><space><space><space><space><space><space>|<space><space><space><space><space><space><space><space><space><space>|<br>
||<space><space><space><space><space><space><space><space><space><space>|<space><space><space><space><space><space><space><space><space><space>|<space><space><space><space><space><space><space><space><space><space>|<br>
||<space><space><space><space><space><space><space><space><space><space>|<space><space><space><space><space><space><space><space><space><space>|<space><space><space><space><space><space><space><space><space><space>|<br>
d=0        d=4        d=8        d=12