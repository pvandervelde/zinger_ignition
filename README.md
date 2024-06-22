# zinger_ignition

This repository contains a number of [Gazebo](http://gazebosim.org/) world definitions that can be used
for testing the Zinger in a virtual world. It is assumed
that you have at least the following ROS packages:

* [zinger_description](https://github.com/pvandervelde/zinger_description) - Contains the geometric
  description of the Zinger robot for ROS to work with.
* [zinger_viz](https://github.com/pvandervelde/zinger_viz) - Contains the launch file to start
  [rviz](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html)
  with the Zinger robot model.

## Worlds

The following world files are available in order of complexity.

### Empty World

An empty world with no obstacles. Zinger will be placed at the origin. Started
by giving the following command:

    ros2 launch zinger_ignition ignition.launch.py rviz:=true world:=empty_world

### Room with walls

A set of rooms with separating walls. Zinger will be placed in the center of the
largest room.

![Room with walls](models/room_with_walls_1/map_0.05-px.png)

Started by giving the following command:

    ros2 launch zinger_ignition ignition.launch.py rviz:=true world:=room_with_walls
