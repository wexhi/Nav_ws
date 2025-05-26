# 🧭 ROS 2 Humble SLAM + Navigation (Raspberry Pi 4B code)

Currently, this project is only tested on Raspberry Pi 4B with Ubuntu 22.04 Server and ROS 2 Humble. 

## 🚀 Quick Start Guide

### 🧰 Prerequisites

- **OS**: Ubuntu 22.04
- **ROS 2**: Humble
- **Hardware**: Raspberry Pi 4B, LD14
  
### ✅ Dependencies

Please ensure download and build the LD14 driver first:

```bash
git clone https://github.com/ldrobotSensorTeam/ldlidar_sl_ros2.git
sudo chmod 777 /dev/ttyUSB*
cd ldlidar_sl_ros2
colcon build 
echo “source ~/ldlidar_ros2_ws/install/setup.bash” >> ~/.bashrc 
source ~/.bashrc
```

More details about USB device permissions can be found in the [LD14 driver documentation](https://developer.d-robotics.cc/api/v1/static/fileData/LDROBOT_LD14_Development_Manual_CN_v1_20220530170742.0.pdf)

### 🧱 Step 1: Clone and build the code

```bash
git clone https://github.com/wexhi/Nav_ws.git
# switch branch to the pi version
git checkout pi
cd Nav_ws
colcon build
echo "source ~/Nav_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 🛠 Step 2: Run all

```bash
./run_all_pi.sh
```

Optionally, you can add the .sh script to boot, which will automatically run the communication with hardware and start the controller process.
