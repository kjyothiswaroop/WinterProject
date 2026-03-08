#!/bin/bash
export ROS_DOMAIN_ID=1
source /opt/ros/kilted/setup.bash
source ~/ws/winter/install/setup.bash
exec "$@"
