#!/bin/bash
colcon build && source .venv/bin/activate && source install/local_setup.sh && ros2 run kinect_ros2_py test_node