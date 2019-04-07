# Highway Driving using Path Planning Algorithms

## Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Execution

The current driving lane of the host vehicle is determined first using the d value of the host vehicle taken from localization data. Each lane is assgined an ID (shown in the diagram below).

<img src="/Supporting_Files/Lane_ID.png"/>
fig1: Diagram showing the lane IDs

To decide the behavior of the host vehicle, five surrounding cars are tracked. The cars identified by the following positions (also shows in figure 2): <br>
Pos0: Car in the host lane and closes to the host vehicle.<br>
Pos1: Car closest to the host vehicle in the lane left to the host vehicle and behind the host vehicle. <br>
Pos2: Car closest to the host vehicle in the lane left to the host vehicle and ahead of the host vehicle. <br>
Pos3: Car closest to the host vehicle in the lane right to the host vehicle and behind the host vehicle. <br>
Pos4: Car closest to the host vehicle in the lane right to the host vehicle and behind the host vehicle. <br>

<img src="/Supporting_Files/Obj_Pos.png"/>
fig1: Diagram showing the Position IDs of surrounding vehicles

Next, the algorithm loops overt the sensor fusion data to find the surrounding objects (pos0 - pos4). This can be found in "Section 3" of the code. <br>

Once the objects are identified, the algorithm decides if the a lane change is required. Lane change is required if an object is stuck within a slower car. To decide what kind of maneuver is needed the following decidion treee is used (find in "Section 4" of the code:<br>

- Overtaking from the left side is given preference given equal conditions on the left and right lanes.<br>
- Check if there are no cars on the left lane. Move to left lane with current speed.<br>
- Check if there are no cars on the right lane. Move to right lane with current speed.<br>
- Check if there is no car in front of the host vehicle but there is a car behind in the left lane then check if there is sufficient distance between the car and host vehicle. The distance threshold is decided based on the velocity difference between that car and the host. If satissfied move to that lane. <br>
- Check if there is no car in front of the host vehicle but there is a car behind in the right lane then check if there is sufficient distance between the car and host vehicle. The distance threshold is decided based on the velocity difference between that car and the host. If satissfied move to that lane. <br>
- Check if there is a car in front of the host vehicle but there is no car behind in the left lane then check if there is sufficient distance between the car and host vehicle. The distance threshold is decided based on the velocity difference between that car and the host. If satissfied move to that lane. <br>
- Check if there is a car in front of the host vehicle but there is no car behind in the right lane then check if there is sufficient distance between the car and host vehicle. The distance threshold is decided based on the velocity difference between that car and the host. If satissfied move to that lane. <br>
- Check if there is a car in front and behind the host vehicle in the left lane then check if there is sufficient distance between the cars for host vehicle to move in that lane. The distance threshold is decided based on the velocity difference between that car and the host. If satissfied move to that lane. <br>
- Check if there is a car in front and behind the host vehicle in the right lane then check if there is sufficient distance between the cars for host vehicle to move in that lane. The distance threshold is decided based on the velocity difference between that car and the host. If satissfied move to that lane. <br>

Once the next car maneuver is identified by the algorithm, then the actual path to be followed by the host vehicle if found out. For this the technique described by Aron Brown (from Udacity) is considered. First, three waypoints are considered. For a simple task of keeping within lane, waypoints are considered closer to each other since complex motion of the car is not expected. Whereas, if a lane change is required complex motion of the host vehicle is expected. Hence longer waypoints were considered so that the final path generated is smoother and accelarations values are within acceptable level. The waypoint values were achieved through trial and error process. <br>

After the waypoints are known, a spline curve is generated which smoothly connects these five points (three points from the waypoint discussed in the above paragraph) plus two points from the previous path. After generating the spline, points lying on that curve need to be found out. The separation of points is determined by the target velocity for that cycle. The pints are separated such that the velocity and accelarations needed to reach those points are within acceptable levels. 

## Results
The algorithm was able to navigate around the track safetly meeting the required criteria of being able to drive 4.32 miles without incedent. 

## Improvements
There are several improvement that can be made in the algorithm: <br>
- A cost function can be devised to bake in all the expected scenarios rather than follow a decidion tree. <br>
- Speed of the host vehicle can be altered to allow for quicker maneuver to different lanes. <br>