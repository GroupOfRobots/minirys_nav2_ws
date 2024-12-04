# minirys_nav2_ws

This repository contains miniRyś robot navigation system in ROS2 using the nav2 navigation software and SLAM Toolbox. Nav2 is a fully integrated navigation software that allows robots to navigate in dynamic and changing environments. 

## System Requirements
Before starting, ensure that your system meets the following requirements:
1. Ubuntu 22.04
2. ROS2 Humble
3. Nav2 
4. SLAM Toolbox

## Running the Navigation System
To run the navigation system, use the following commands:
1. Run RViz node with miniRyś robot model
```
ros2 launch minirys_nav2_bringup display.launch.py namespace:=<robot_namespace>
```
2. Run Nav2 navigation stack
```
ros2 launch minirys_nav2_bringup nav_launch.py namespace:=<robot_namespace>
```

## Running SLAM Toolbox
To run SLAM, use the following commands:
1. Run RViz node with miniRyś robot model
```
ros2 launch minirys_nav2_bringup display.launch.py namespace:=<robot_namespace>
```
2. Run SLAM Toolbox node
```
ros2 launch minirys_nav2_bringup slam_toolbox.launch.py namespace:=<robot_namespace>
```
In order to save a map you can use the map_saver tool from the ros2 map_server package
```
ros2 run nav2_map_server map_saver_cli -f <map_file_name>
```
