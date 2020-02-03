# ROS-projects
Some ROS exercizes in C++

Six exercizes with Turtlesim simulator, all nodes are written in C++.

1) Workspace Limits Avoidance

Implementation of an algorithm which prevent the turtle to hit the walls.
The node avoidance_node can be launched :

- individually, with the possibility to pass an x linear velocity and a z angular velocity from command line
 ( e.g. rosrun ros_ex avoidance_node or rosrun ros_ex avoidance_node 2.0 0.2 ) ,
 then launch turtlesim simulator and send a velocity command if it has not been passed to avoidance_node ;
 
- through launchfile avoidance.launch which run also the turtlesim simulator 
 ( e.g. roslaunch ros_ex avoidance.launch  or  avoidance.launch linear:=2.0 angular:=0.5 ).
 
2) Point to Point Control

Controller which moves the turtle from a given point to another desired one.
The node point2point_node also can be launched individually ot through launchfile P2P.launch
( e.g. rosrun ros_ex point2point_node   or  rosrun ros_ex point2point_node 2.0 6.0  
 or    roslaunch ros_ex P2P.launch x=2.0 y=6.0 ).
 
3) Consensus

Implementation of a discrete consensus protocol for three turtles on their (x,y) position.

roslaunch ros_ex spawn_three
roslaunch ros_ex consensus.launch

4) Exploration

Three turtles communicate each others their position and then each one visits position of the others and come back to its starting point.

roslaunch ros_ex spawn_three 
roslaunch ros_ex exploration.launch 

5) Shortest Path

Implementation of Shortest Path distributed algorithm for 4 turtles.
Each one can communicate only with the next two except for the first one, which communicates only with turtle number 1.

roslaunch ros_ex shortest.launch

6) Formation

Five turtles arrange themeselves in a given formation through a consensus algorithm.

roslaunch ros_ex spawn_five 
roslaunch ros_ex formation.launch
