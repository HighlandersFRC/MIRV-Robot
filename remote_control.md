# Below are the steps to remotely drive MIRV from a controller connected to a computer.

## Setup ROS variables on computer (these will need to be setup in every computer terminal):
```
export ROS_MASTER_URI=http://[agx ip]:11311
export ROS_IP=[computer ip]
```
## Setup ROS variables on AGX (needed in every AGX terminal):
```
export ROS_MASTER_URI=http://[agx ip]:11311
export ROS_IP=[computer ip]
```

## Start roscores

### On the computer:
```
roscore
```
### On the AGX:
```
roscore
```

## Launch joystick_drive on the AGX:
```
roslaunch mirv_real joystick_drive.Launch
```

## Configure joystick input on computer:
```
rosparam set joy_node/dev "/dev/input/js1"
```

## Start joystick on the same computer terminal:
```
rosrun joy joy_node
```
