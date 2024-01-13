# RT_Assignment2_Amani_Ghomrani-
----------------------
In this assignment, a ROS package was created to manage a robot's movement and acquire data regarding its position and speed. The package comprises three nodes:

(a) An action client node which enables user to set a target (x, y) or cancel it. Utilizing the feedback/status of the action server, I ensured awareness of when the target has been reached. Additionally, this node publishes the robot's position and velocity as a vel_pos message (pos_x, pos_y, vel_x, vel_y) by extracting values from the /odom topic.

(b) A service node has been implemented to return the coordinates of the last target sent by the user upon invocation.

(c) Another service node has been created, subscribing to the robot's position and velocity using the vel_pos message. This node acts as a server, providing the distance of the robot from the target and the robot's average speed.

A launch file has been constructed to initiate the entire simulation


## Action Node structure
----------------------

![RT2_Flowchart](https://github.com/AmaniGhm/RT_Assignment2_Amani_Ghomrani-/assets/125284569/87977232-6518-4cdd-a268-e5152991f9dc)


## How to RUN the code
-----------------------------
First you need to clone this repository inside the src file of your ROS workspace
Second run the command : 
```bash
$ catkin_make
```
Then you need to make the python scripts executable using the command :

```bash
$ chmod +x action_client.py bug_as.py go_to_point_service.py last_trgt_coord_service.py position_node.py wall_follow_service.py 
```
Then to launch the package you need to execute assignment1.launch file use the following :

```bash
$  roslaunch assignment_2_2023 assignment1.launch  
```
## How it works
The after executing the launch file the user will see 2 terminals and one Gazibo window and a Rviz window
   - The first terminal allows the user to enter the (x,y) of the goal position
   - The second terminal displays the distance and velocity or the robot
