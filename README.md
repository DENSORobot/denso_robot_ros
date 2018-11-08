# denso_robot_ros

```
$ cd your_ws/src
$ git clone https://github.com/Nishida-Lab/denso_robot_ros.git
$ cd ..
$ rosdep install -iry --from-paths src
$ catkin build
```

## DENSO VS060

### Simulation (gazebo)

```
$ roslaunch denso_robot_bringup vs060_bringup.launch sim:=true
```

* Using STOMP or CHOMP for the planner (for CHOMP,  `planner:=chomp`)

```
$ roslaunch denso_robot_bringup vs060_bringup.launch sim:=true planner:=stomp
```


## DENSO VS087

### Simulation (rviz + fake controller)

```
$ roslaunch vs087_moveit_config demo.launch
```

* Using STOMP or CHOMP for the planner (for CHOMP,  `planner:=chomp`)

```
$ roslaunch vs087_moveit_config demo.launch planner:=stomp
```


### Simulation (gazebo)

```
$ roslaunch denso_robot_bringup vs087_bringup.launch sim:=true
```

* Using STOMP or CHOMP for the planner (for CHOMP,  `planner:=chomp`)

```
$ roslaunch denso_robot_bringup vs087_bringup.launch sim:=true planner:=stomp
```

### Real robot

```
$ roslaunch denso_robot_bringup vs087_bringup.launch sim:=false ip_address:=xxx.xxx.xxx.xxx
```

* Using STOMP or CHOMP for the planner (for CHOMP,  `planner:=chomp`)

```
$ roslaunch denso_robot_bringup vs087_bringup.launch sim:=false ip_address:=xxx.xxx.xxx.xxx planner:=stomp
```

# STOMP and CHOMP planner integration

```
$ cd your_ws/src
$ git clone https://github.com/Nishida-Lab/moveit.git
$ git clone https://github.com/Nishida-Lab/industrial_moveit.git
$ cd ..
$ rosdep install -iry --from-paths src
$ catkin build
```
