# Documentation

## Simulation

- imu: /imu, sensor_msgs/Imu
  - 9-axis gyro, accelerometer, and magnetometer
- wheel encoders: /encoder/wheel_angles, sensor_msgs/JointState
  - angle of each wheel in radians
- camera images: /camera/rgb/image_raw, sensor_msgs/Image
- vehicle control: /cmd_vel, geometry_msgs/Twist
  - target linear/angular velocity (linear x is forward/reverse, angular z is rotation)
- gps: /fix, sensor_msgs/NavSatFix
- lidar: /velodyne_points/points, sensor_msgs/PointCloud2

sensor noise parameters (when available) are specified in [mirv_description/urdf/mirv_gazebo.urdf.xacro](mirv_description/urdf/mirv_gazebo.urdf.xacro)
