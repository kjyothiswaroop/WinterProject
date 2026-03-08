#!/bin/bash
export ROS_DOMAIN_ID=0
source ~/unitree_ros2/setup.sh
source ~/ws/winter/install/setup.bash
exec "$@"
