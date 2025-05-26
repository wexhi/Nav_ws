# 🧭 ROS 2 Humble SLAM + Navigation 设置指南（Ubuntu 22.04）

[简体中文](README.md) | [English](README_EN.md)

_这是一个用于机器人建图和导航的课堂项目，因此可能对机器人硬件和嵌入式代码有特定要求。请查阅文末的“附加说明”部分了解更多细节。_

![demo video](doc/527-ezgif.com-speed.gif)

本项目提供了在 **Ubuntu 22.04** 上使用 **ROS 2 Humble** 进行 SLAM 和导航的完整配置，包括代码、参数文件和使用说明。当前版本主要面向 **PC 与 Raspberry Pi 4B**，如需查看树莓派相关代码，请参见分支：[**pi 分支**](https://github.com/wexhi/Nav_ws/tree/pi)。

---

## 🚀 快速开始指南

### 🧰 前置准备

✅ 系统要求：

- 操作系统：Ubuntu 22.04  
- ROS 2 版本：Humble Hawksbill

✅ 依赖安装：

- 安装 ROS 2 Humble：  
  推荐使用小鱼安装器：[安装教程（中文）](https://fishros.org.cn/forum/topic/20/%E5%B0%8F%E9%B1%BC%E7%9A%84%E4%B8%80%E9%94%AE%E5%AE%89%E8%A3%85%E7%B3%BB%E5%88%97)

```bash
wget http://fishros.com/install -O fishros && . fishros
```

- 安装额外依赖：

```bash
sudo apt update && sudo apt install -y \
    python3-colcon-common-extensions \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-turtlebot3*  # 可选，适用于仿真
```

---

## 🧱 第一步：克隆与编译代码

```bash
git clone https://github.com/wexhi/Nav_ws.git
cd Nav_ws
colcon build --symlink-install
```

---

## 🛠️ 第二步：设置环境变量

```bash
source install/setup.bash
```

建议将其加入 `~/.bashrc` 以便每次自动生效：

```bash
echo "source ~/Nav_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 🗺️ 第三步：启动 SLAM 建图

运行以下命令开始 SLAM 建图，并在 RViz 中可视化：

```bash
source install/setup.bash
ros2 launch serial_bridge slam_launch.py
```

运行以下命令控制机器人（键盘遥控）：

```bash
ros2 run serial_bridge wasdx_teleop_launch.py
```

保存地图：

```bash
ros2 run nav2_map_server map_saver_cli -f ~/Nav_ws/src/serial_bridge/maps/<map_name>
```

---

## 🧭 第四步：启动导航模块

运行以下命令启动导航，并在 RViz 中查看路径规划：

```bash
ros2 launch serial_bridge nav_with_map_launch.py
```

如果你有自定义地图文件，可以这样启动：

```bash
ros2 launch serial_bridge nav_with_map_launch.py map:=/path/to/save/map.yaml
```

---

## 🚀 第五步：边建图边导航（SLAM + Navigation）

若希望在建图的同时执行导航：

```bash
ros2 launch serial_bridge nav_with_slam.launch.py
```

可以在导航过程中随时保存地图：

```bash
ros2 run nav2_map_server map_saver_cli -f ~/Nav_ws/src/serial_bridge/maps/<map_name>
```

---

## 📝 附加说明

- 请确保你的机器人运行了[**指定的 STM32 固件代码**](https://github.com/wexhi/SR)，并连接了合适的底层硬件。

- 机器人需搭载兼容的 LiDAR 雷达，用于 SLAM 建图，且与树莓派正确连接。

- 请确保你的树莓派和PC处于同一局域网内，以便进行数据传输和控制。请设置 `~/.bashrc` 文件，添加以下行：

```bash
export ROS_DOMAIN_ID=0
```

或

```bash
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
source ~/.bashrc
```
