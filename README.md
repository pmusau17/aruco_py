# aruco_py

Aruco marker detection with OpenCV 4.5.3 and ROS2. 

# Getting Started 

Make sure you have [ROS2](https://docs.ros.org/en/foxy/Installation.html) Installed on your machine. Create a workspace

```
$ mkdir -p dev_ws/src
```

Clone the usb cam demo ros2 branch into the src folder alongside this repository

```
$ cd dev_ws/src
$ git clone https://github.com/ros-drivers/usb_cam -b ros2
$ git clone https://github.com/pmusau17/aruco_py.git
```

Build the package

```
$ cd .. 
$ colcon build 
```

Source the overlay

```
source install/setup.bash
```

# Running the Code 

Launch the camera

```
ros2 launch usb_cam demo_launch.py
```

Launch the marker detection node

```
ros2 run aruco_py detectVideo -t DICT_6X6_250
```

