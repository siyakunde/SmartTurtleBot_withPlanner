# Goal</br>

The goal of this project is to apply skill in architecting, implementing, deploying, and simulating a robotic system. The emphasis is on the planning and waypoint navigation with a controller.</br>

# Task</br>

For this assignment you are to build two systems and compare their performance.</br>

## System 1</br>
extends your system from SmartTurtleBot so that it also takes a map, a goal location, and a robot size. Then, the system should generate the series of locations the bot needs to go through to reach the goal location from the starting location, using a path connecting them.</br>
More in detail, the system should:</br>

Load the given map file into memory (using map_server).</br>
Convert map into an occupancy grid (each cell is 0 (free) or 1 (occupied)), and inflate the objects according to the robots size.</br>
Build a graph, in which each node represents a cell in the grid and the edges connect adjacent cells in the grid.</br>
Compute the shortest path from the starting location to the goal location. If there is no valid path, print an error message and exit.</br>
Develop a local planner to send the appropriate velocity commands to the robot to make it move along the waypoints computed in the previous step. It should use a “PD” controller to set the robot speed given a waypoint distance.</br>

## System 2</br>
has the same functionality and mostly the same launch file as System 1, but it uses ROS Navigation stack to perform global and local planning. It must use ROS actions. </br>