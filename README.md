Sensor emulator
==================

## Overview

This ROS package provides a simple action server to publish sensor_msgs::PointCloud2 from .ply files.

## Installation

1. Install [ROS](http://wiki.ros.org/ROS/Installation) and [PCL](https://pointclouds.org/)

2. Clone repository into your catkin workspace and compile the package

## How to use

1. **Launch node**

You need to define the topic in which the pointcloud will be published (`ambient_pointcloud_topic`), the frame_id of the emulated sensor (`frame_id`) and the folder where the .ply file is (`folder_name`).

  - Option 1:

```
roslaunch sensor_emulator sensor_emulator.launch ambient_pointcloud_topic:=/pointcloud frame_id:=camera_frame 
```

  - Option 2:
See `/sensor_emulator_node/launch/sensor_emulator_example.launch` example file and change it accordingly. 

Note that the default folder (`folder_name`) is the `/sensor_emulator_node/models` folder, but it can be changed as well.

2. **Publish action goal**

Example:

```
rostopic pub /sensor_emulator/goal sensor_emulator_msgs/SensorEmulatorActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  file_name: 'footbones'"
```

The pointcloud can be visualized using rviz. 









