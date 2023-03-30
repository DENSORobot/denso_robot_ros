Please refer to https://wiki.ros.org/denso_robot_ros.

## Setup Environment

### 1. Install Gazebo
Following as [Gazebo Official Install](https://gazebosim.org/docs/garden/install_ubuntu)

### 2. Install the Gazebo Ros2 Packages by binary
The official Gazebo ROS packages not work now. So please follow this temporary packge.

Following as [Gazebo Ros2 temporary package](https://github.com/leledeyuan00/gazebo_ros)

### 3. Install the Moveit by binary

Following as [Moveit2 Humble Official package](https://moveit.ros.org/install-moveit2/binary/)

### 4. Clone and compile this package

`mkdir -p ~/denso_ws/src && cd ~/denso_ws/src`

`git clone https://github.com/smartrobotsdesignlab/denso_robot_ros2.git`

`cd ~/denso_ws`

`rosdep install --from-paths src --ignore-src -r -y`

`colcon build`

### 5. Source the Gazebo ws then export the Gazebo model environment variables
`export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/home/xx/denso_ws/install/denso_description/share/`

! Please replace the "xx" to your real path.

### 6. Launch the Gazebo Robot & Enjoy

![Gazebo](./docs/gazebo.png)

`ros2 launch denso_robot_bringup denso_robot_bringup.launch.py sim_gazebo:=true robot_controller:=forward_position_controller`

> Option Luanch Moveit

`ros2 launch denso_robot_bringup denso_robot_bringup.launch.py sim_gazebo:=true robot_controller:=trajectory_controller`

`ros2 launch denso_robot_moveit_config denso_robot_moveit.launch.py use_sim_time:=true`

### 6. Control Robot
You can control the robot by publish the command to `/forward_position_controller/commands`

For example:

`ros2 topic pub --once /forward_position_controller/commands std_msgs/msg/Float64MultiArray "{layout: {}, data: [0,0,1.574,0,0,0]}"`

## Real Robot
`ros2 launch denso_robot_bringup denso_robot_bringup.launch.py robot_ip:=192.168.0.1 robot_port:=5007`

## Update Log

Rather than V3. This ROS2 version making the Denso robot SDK independentely, and you can use thses 3 packages(`bcap_core`, `bcap_service`, `denso_robot_core`) on any platform.
