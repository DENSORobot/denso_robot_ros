# denso_robot_ros

```
$ cd your_ws/src
$ git clone https://github.com/Nishida-Lab/denso_robot_ros.git
$ cd ..
$ rosdep install -i --from-paths src
$ catkin build
```

## DENSO VS060

### Simulation (gazebo)

```
$ roslaunch denso_robot_bringup vs060_bringup.launch sim:=true
```

* Use STOMP for the planner

```
$ roslaunch denso_robot_bringup vs060_bringup.launch sim:=true planner:=stomp
```


## DENSO VS087

### Simulation (rviz + fake controller)

```
$ roslaunch vs087_moveit_config demo.launch
```

* Use STOMP for the planner

```
$ roslaunch vs087_moveit_config demo.launch planner:=stomp
```


### Simulation (gazebo)

```
$ roslaunch denso_robot_bringup vs087_bringup.launch sim:=true
```

* Use STOMP for the planner

```
$ roslaunch denso_robot_bringup vs087_bringup.launch sim:=true planner:=stomp
```

### Real robot

```
$ roslaunch denso_robot_bringup vs087_bringup.launch sim:=false ip_address:=xxx.xxx.xxx.xxx
```

* Use STOMP for the planner

```
$ roslaunch denso_robot_bringup vs087_bringup.launch sim:=false ip_address:=xxx.xxx.xxx.xxx planner:=stomp
```

# STOMP planner integration

```
$ cd your_ws/src
$ git clone https://github.com/ros-planning/moveit.git
$ git clone https://github.com/ros-industrial/industrial_moveit.git
$ cd industrial_moveit
$ rm -rf industrial_collision_detection
$ rm -rf constrained_ik
$ rm -rf industrial_moveit_benchmarking
$ cd ../..
$ rosdep install -i --from-paths src
$ catkin build
```