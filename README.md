# VinOdom
Visial Innertial Odometry based on Homography Decomposition

## Description
This software performs odometry computation based on homography decomposition. The software has been designed and tested in ROS2. It uses the following information to compute the odometry:
- Scene image
- IMU
- Altimeter

This software is specielly designed to work with a down-facing camera mounted in drones. This information is used to select the right solution provided by the homography decomposition, the one with a plane normal closest to (0,0,-1). 

For now, the IMU loose-coupled into the solution, just overriding the estimated visual odometry with the IMU information. Future work will consider this to better choose the righ homographt decomposition solution. 

## Execution
VinOdom integrates a simgle node that subscribe to the previous information. The node also subscribes to TF in order to compute the relative posiction of IMU and the Camera with respecto to the base frame id.

A launch file will be ready soon with detailed parameter information. For now, the following line gives a general idea of the basic parameters and execution:

`ros2 run vinodom vinodom --ros-args -p camera_topic:=/quadrotor_3/slot3/ -p imu_topic:=/quadrotor_3/imu/data -p altimeter_topic:=/quadrotor_3/slot6/scan -p base_frame:=quadrotor_3 -p show_matching:=true`

The odometry estimation is published as a nav_msgs::msg::Odometry ROS2 message in the topic given by the parameters "odom_topic", which by default is "/odom"

## Compilation
The software does not have signifcant dependencies except for ROS2. To compile it just clone the repo into your ROS2 workspace and run `colcon build`

