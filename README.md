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