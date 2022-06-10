# Ros Commands
This readme is a helpful document designed to log some useful ros commands for the MIRV project. When adding commands please do so in the listed format to keep this document clean and uniform.



## Common Commands
Commands that will be used frequently when working on the ROS project. 

### roscore
Starts the central roscore which manages topics, publishers, subscribers and nodes.

Usage:
```
roscore
```

Example:
```
roscore
```

### rosrun
runs a ros node from anywhere within the ros workspace

Usage:
```
rosrun <package_name> <node_name>
```

Example:
```
rosrun mirv_real drive
```

### roslaunch
Launches a configuration of ros nodes and resources. This may include simulation or real components as well as control logic. Launch files should be stored in the corresponding launch directory within their host package.

Usage:
```
roslaunch <package_name> <launch_file>.launch
```

Example:
```
roslaunch mirv_real joystick_drive.launch
```


## Debug Commands
Commands that make very useful debug tools when developing in the ROS environment.

### rostopic echo
listens to a rostopic and relays the data on the topic to the terminal. This is very usefull for seeing what data is on a topic, or if there is data on a topic. Be careful when using this command with data that cannot be text encoded.

Usage:
```
rostopic echo <topic_name>
```

Example:
```
rostopic echo joy
```
