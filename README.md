# unity_ros_lidar_3d
Unity package for simulating 3D Laser Scanner (LIDAR) and publishing the point cloud to ros2.

## This package implements following things
### A simple robot with 4 wheels
- A simple urdf file is created (along with xacro). This urdf file is imported to unity using urdf importer.
- A differential drive robot controller is implemented in unity. This subscribe to "cmd_vel" topic and generate commands for movement of robot in unity.

### LIDAR (3D laser Scanner)
- A 3D laser scanner sensor is implement using Ray Casting. It has parameters such as fov, angular_resolution (both horizontal and vertical), min_range, max_range etc.
- A Point Cloud publisher publish the generated point cloud to 'point_cloud' topic. It aslo publish the pose (position and orientation) of the laser_scanner to 'laser_scan_pose'.

 ### Notes
 - Please do read how unity-ros framework works. [ROS Unity Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub) [ROS Unity Integration](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/README.md)
 - User can edit xacro file and then generate urdf from it. And import new urdf into the unity to try different robot.
 - User can edit all parameters either from unity script or from panel.

## Installation
### Pre-requistes
- Read ROS-Unity integration (try few examples to get the grasp on ros-unity framework works in general). [ROS Unity Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub) [ROS Unity Integration](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/README.md)


## How to re-use this yourself in some new project
- Install [ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector) and [URDF Importer](https://github.com/Unity-Technologies/URDF-Importer) packages in your unity project. Select appropriate ros version after installing the ROS-TCP-Connector package.
- Create a ros workspace and build package [ROS-TCP-Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint) in the workspace by downloading approriate branch (ROS or ROS2) from github.
- Copy required Scripts from this project to your project. Note Scripts in this project are written for ROS2 so some error might come if you use ROS1 so you need to edit them accordingly.
