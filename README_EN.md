# 🧭 ROS 2 Humble SLAM + Navigation Setup (Ubuntu 22.04)

[简体中文](README.md) | [English](README_EN.md)

_This is a class project for robot mapping and navigation, so there might be some specific requirements for the robot hardware and embedded code. Please check the Additional Notes section for more details._

![demo video](doc/527-ezgif.com-speed.gif)

This repository provides a comprehensive setup for SLAM and Navigation using ROS 2 Humble on Ubuntu 22.04. It includes the necessary code, configurations, and instructions to get started with robot mapping and navigation. However, please note that this setup is currently designed for **pc and raspberry pi 4b**, and raspberry's code please check the branch [**pi**](https://github.com/wexhi/Nav_ws/tree/pi)

## 🚀 Quick Start Guide

🧰 Prerequisites
✅ System

- OS: Ubuntu 22.04
- ROS 2: Humble
  
✅ Dependencies

- Install ROS 2 Humble: [ROS 2 Installation Guide](https://fishros.org.cn/forum/topic/20/%E5%B0%8F%E9%B1%BC%E7%9A%84%E4%B8%80%E9%94%AE%E5%AE%89%E8%A3%85%E7%B3%BB%E5%88%97)
  
``` bash
wget http://fishros.com/install -O fishros && . fishros
```

- Install additional dependencies:

``` bash
sudo apt update && sudo apt install -y \
    python3-colcon-common-extensions \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-turtlebot3*  # optional, for simulation
```

## 🧱 Step 1: Clone and build the code

``` bash
git clone https://github.com/wexhi/Nav_ws.git
cd Nav_ws
colcon build --symlink-install
```

## 🛠️ Step 2: Source the workspace

``` bash
source install/setup.bash
```

Optionally, add the source command to your `~/.bashrc`:

``` bash
echo "source ~/Nav_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 🗺️ Step 3: Launch the SLAM

Run the following command to start the SLAM process and visualize it in RViz:

``` bash
source install/setup.bash
ros2 launch serial_bridge slam_launch.py
```

Run the following command to control the robot using teleop:

``` bash
ros2 run serial_bridge wasdx_teleop_launch.py
```

Save the map:

``` bash
ros2 run nav2_map_server map_saver_cli -f ~/Nav_ws/src/serial_bridge/maps/<map_name>
```

## 🧭 Step 4: Launch the Navigation

Run the following command to start the navigation process and visualize it in RViz:

``` bash
ros2 launch serial_bridge nav_with_map_launch.py
```

Optionally, you can specify a custom map file:

``` bash
ros2 launch serial_bridge nav_with_map_launch.py map:=/path/to/save/map.yaml
```

## 🚀 Step 5: Navigation while Mapping

To run navigation while mapping, you can use the following command:

``` bash
ros2 launch serial_bridge nav_with_slam.launch.py
```

You can also save the map while navigating:

``` bash
ros2 run nav2_map_server map_saver_cli -f ~/Nav_ws/src/serial_bridge/maps/<map_name>
```

## 📝 Additional Notes

- Ensure that your robot is running with [**specific stm32 code**](https://github.com/wexhi/SR) and right hardware conditions.

- The robot should be equipped with a compatible LiDAR sensor for SLAM which is connected to the Raspberry Pi.

- The raspberry pi and PC should under the same network(e.g., same Wi-Fi). And modify the `~/.bashrc` file to add the following line:

``` bash
export ROS_DOMAIN_ID=0
```

or you can run the following command:

``` bash
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
source ~/.bashrc
```
