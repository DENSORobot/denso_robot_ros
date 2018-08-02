# denso_robot_ros

```
rosdep install -i --from-paths src
```

## VS060

### Simulation (gazebo)

```
roslaunch denso_robot_bringup vs060_bringup.launch sim:=true
```

## VS087

### Simulation (gazebo)

```
roslaunch denso_robot_bringup vs087_bringup.launch sim:=true
```

### Real robot

```
roslaunch denso_robot_bringup vs060_bringup.launch sim:=false ip_address:=xxx.xxx.xxx.xxx
```