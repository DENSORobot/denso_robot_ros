Please refer to https://wiki.ros.org/denso_robot_ros.

## Setup Environment

This branch is validated on the ROS humble version.

### Dependencies
`sudo apt install ros-humble-xacro`


## Real Robot
`ros2 launch denso_robot_bringup denso_robot_bringup.launch.py robot_ip:=192.168.0.1 robot_port:=5007`

## Simulation

If you want to use gazebo. You must install gazebo by manully source build method. 
### 1. Install Gazebo
Following as [Gazebo Official Install](https://gazebosim.org/docs/garden/install_ubuntu_src)

### 2. Install Gazebo Ros Plugin
Clone the [gz_ros2_control package](https://github.com/ros-controls/gz_ros2_control)

Check out to the `ahcorde/rename/ign_to_gz` branch.

This is because the version update of the Gazebo....

### 3. Source the Gazebo ws then export the Gazebo model environment variables
`export GZ_SIM_RESOURCE_PATH=/home/xx/xx_ws/install/denso_description/share/`

! Please replace the "xx" to your real path.

### 4. Launch the Gazebo Robot & Enjoy


## Update Log

Rather than V3. This ROS2 version making the Denso robot SDK independentely, and you can use thses 3 packages(`bcap_core`, `bcap_service`, `denso_robot_core`) on any platform.