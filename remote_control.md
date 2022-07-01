# Below are the steps to remotely drive MIRV from a controller connected to a computer.

## Connect to MIRV
Connect to the wifi access point on MIRV from the computer

## Setup ROS environment variables
In computer terminal:
```
export ROS_MASTER_URI=http://[agx ip]:11311
export ROS_IP=[computer ip]
rosparam set joy_node/dev "/dev/input/js1"
```

In AGX terminal:
```
export ROS_MASTER_URI=http://[agx ip]:11311
export ROS_IP=[computer ip]
```

## Launch ROS nodes
In computer terminal:
```
roslaunch mirv_real joystick_drive_remote.launch
```

In AGX terminal:
```
roslaunch mirv_real joystick_drive_mirv.launch
```

## Controls
### Driving
Left Joystick: Drive forwards / backwards

Right Joystick: Turn left / right

### Intake
A: Disable intake movement

B: Reset intake to upright position

X: Set intake to downward position and run intake wheels

Y: Bring intake up and deposit PiLit into conveyor

Left Bumper: Load PiLit from conveyor into intake, lower intake, and deposit PiLit on ground

Right Bumper: Toggle between using left or right side of the intake and conveyor
