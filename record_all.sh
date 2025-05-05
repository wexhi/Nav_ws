#!/bin/bash
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
OUTPUT_DIR="$HOME/bags/bags_$TIMESTAMP"
echo "Recording all topics to folder: $OUTPUT_DIR"
ros2 bag record -a -o $OUTPUT_DIR
