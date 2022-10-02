# tblaze_ignition

This repository contains a number of [Gazebo](http://gazebosim.org/) world definitions that can be used
for testing the TBlaze in a virtual world. It is assumed
that you have at least the following ROS packages:

* [tblaze_description](https://github.com/pvandervelde/tblaze_description) - Contains the geometric
  description of the TBlaze robot for ROS to work with.
* [tblaze_bringup](https://github.com/pvandervelde/tblaze_bringup) - Contains a launch file that will
  create all the ROS nodes required for TBlaze to function

## Worlds

The following world files are available in order of complexity.

### Empty World

An empty world with no obstacles. TBlaze will be placed at the origin. Started
by giving the following command:

    ros2 launch tblaze_ignition ignition_launch rviz:=true world:=empty_world
