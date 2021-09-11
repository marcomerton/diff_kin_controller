# Differential Kinematic Controller

This ROS package implements a simple differential kinematic controller as the one described in [[1]](#1).
The controller uses the damped SVD to invert the Jacobian and compute the joints velocities given the position and orientation errors of the end effector. The velocities are then integrated to compute the new joints positions. The controller also supports velocity commands for the end effector. Eventual redundancy in the arm is not exploited.

It also contains an example of pick and place application. Given the pick pose and the place pose, and possibly some waypoints to pass through, a ROS node generates a sequence of commands for the controller, interpolating between successive waypoints.


## Execution


### Controller node
To start the node executing the differential control loop, use:
```
$ rosrun diff_kin_controller ArmController.py <urdf>
```
where ``<urdf>`` is the path to the robot's description file.

*Note*: from the file, a chain is extracted starting from the link 'iiwa_link_0' to the link 'iiwa_link_7'


### Master node
To start the master node sending the commands to the controller, use:
```
$ rosrun diff_kin_controller pick_and_place.py
```
The trajectory is specified through two parameters in the ROS parameter server: ``pick_trajectory`` and ``place_trajectory``. The file ``config/trajectory.yaml`` already contain a test trajectory.

Each waypoint in the trajectory must be specified through 4 fields:
1. ``position``: 3D vector (x, y, z) specifying the position in space
2. ``orientation``: quaternion (x, y, z, w) specifying the orientation in space
3. ``time``: duration, in seconds, of the movement from the previous waypoint to the new one
4. ``nsteps``: number of interpolation steps between the previous and current waypoint


### All-in-one launch file
A launch file starting both nodes with the right parameters setup is already provided. Use the command
```
$ roslaunch diff_kin_controller pick_and_place.launch
```


## References
<a id="1">[1]</a>
Bruno Siciliano et al,
Robotics: Modelling, Planning and Control, pp 132-134.
