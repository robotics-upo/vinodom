# VinOdom
Visial Innertial Odometry based on Homography Decomposition

## Description
This software performs odometry computation based on homography decomposition. It uses the following information to compute the odometry:
- Scene image
- IMU
- Altimeter

This software is specielly designed to work with a down-facing camera mounted in drones. This information is used to select the right solution provided by the homography decomposition, the one with a plane normal closest to (0,0,-1). 

For now, the IMU loose-coupled into the solution, just overriding the estimated visual odometry with the IMU information. Future work will consider this to better choose the righ homographt decomposition solution. 

## Execution
VinOdom integrates a simgle node that subscribe to the previous information. The node also subscribes to TF in order to compute the relative posiction of IMU and the Camera with respecto to the base frame id.
