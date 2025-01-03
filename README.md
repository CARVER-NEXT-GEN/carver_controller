# carver_controller

## How to use carver_messenger.py
1. To save covariance and exit:
```bash
ros2 run carver_controller carver_messenger.py --ros-args -p mode:=save_covariance
```
2. To publish IMU data continuously:
```bash
ros2 run carver_controller carver_messenger.py 
```

## Required Package for Imu in Realsense L515
1. Install the ROS IMU plugin for visualize the IMU data on rviz.
```bash
sudo apt-get install ros-humble-rviz-imu-plugin
```
2. Install "imu filter madgwick" package for create oreintation of this imu
```bash
sudo apt install ros-humble-imu-filter-madgwick
```