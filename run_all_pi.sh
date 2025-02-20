#!/bin/bash

SESSION_NAME="ros_pi_startup"
TMUX_BIN="/usr/bin/tmux"
SETUP_SCRIPT="$HOME/Nav_ws/install/setup.bash"

# 检查会话是否已存在，避免重复启动
if ! $TMUX_BIN has-session -t $SESSION_NAME 2>/dev/null; then
    $TMUX_BIN new-session -d -s $SESSION_NAME

    # 窗口 1: bridge
    $TMUX_BIN rename-window -t $SESSION_NAME:0 'bridge'
    $TMUX_BIN send-keys -t $SESSION_NAME:0 "source $SETUP_SCRIPT && ros2 launch serial_bridge serial_bridge_launch.py" C-m

    # 窗口 2: commander
    $TMUX_BIN new-window -t $SESSION_NAME -n 'commander'
    $TMUX_BIN send-keys -t $SESSION_NAME:1 "source $SETUP_SCRIPT && ros2 launch serial_bridge serial_commander_launch.py" C-m

    # 窗口 3: ldlidar
    $TMUX_BIN new-window -t $SESSION_NAME -n 'ldlidar'
    $TMUX_BIN send-keys -t $SESSION_NAME:2 "source $SETUP_SCRIPT && ros2 launch ldlidar_sl_ros2 ld14.launch.py" C-m
fi

# 不要 attach，systemd 没有终端可供附加
# $TMUX_BIN attach -t $SESSION_NAME
