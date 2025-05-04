#!/bin/bash

SESSION_NAME="ros_pi_startup"

tmux new-session -d -s $SESSION_NAME

# 窗口 1: serial_bridge
tmux rename-window -t $SESSION_NAME:0 'bridge'
tmux send-keys -t $SESSION_NAME:0 "source ~/Nav_ws/install/setup.bash && ros2 launch serial_bridge serial_bridge_launch.py" C-m

# 窗口 2: serial_commander
tmux new-window -t $SESSION_NAME -n 'commander'
tmux send-keys -t $SESSION_NAME:1 "source ~/Nav_ws/install/setup.bash && ros2 launch serial_bridge serial_commander_launch.py" C-m

# 窗口 3: ldlidar
tmux new-window -t $SESSION_NAME -n 'ldlidar'
tmux send-keys -t $SESSION_NAME:2 "source ~/Nav_ws/install/setup.bash && ros2 launch ldlidar_sl_ros2 ld14.launch.py" C-m

# 附加到会话
tmux attach -t $SESSION_NAME
