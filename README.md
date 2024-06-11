# zinger_ignition

This repository contains a number of [Gazebo](http://gazebosim.org/) world definitions that can be used
for testing the Zinger in a virtual world. It is assumed
that you have at least the following ROS packages:

* [zinger_description](https://github.com/pvandervelde/zinger_description) - Contains the geometric
  description of the Zinger robot for ROS to work with.
* [zinger_bringup](https://github.com/pvandervelde/zinger_bringup) - Contains a launch file that will
  create all the ROS nodes required for Zinger to function

## Worlds

The following world files are available in order of complexity.

### Empty World

An empty world with no obstacles. Zinger will be placed at the origin. Started
by giving the following command:

    ros2 launch zinger_ignition ignition.launch.py rviz:=true world:=empty_world
