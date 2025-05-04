#!/bin/bash

# 启动 PC 上的 slam 和 teleop 节点，在两个 tmux 窗口中运行
SESSION_NAME="ros_pc_startup"

tmux new-session -d -s $SESSION_NAME

# 窗口 1: SLAM
tmux rename-window -t $SESSION_NAME:0 'slam'
tmux send-keys -t $SESSION_NAME:0 "source ~/Nav_ws/install/setup.bash && ros2 launch serial_bridge slam_launch.py" C-m

# 窗口 2: TELEOP
tmux new-window -t $SESSION_NAME -n 'teleop'
tmux send-keys -t $SESSION_NAME:1 "source ~/Nav_ws/install/setup.bash && ros2 launch serial_bridge wasdx_teleop_launch.py" C-m

# 打开 tmux 会话
tmux attach -t $SESSION_NAME
